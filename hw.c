/* 
 * JaldiMAC hw
 */

#include "jaldi.h"
#include "hw.h"

// Channel-related

/**
 * Populates the centers struct with the frequencies of the channel
 * we're currently on. This is needed since we can be using channels
 * that are either 20Mhz or 40Mhz wide. In the parlance of 802.11n,
 * which we adpot here, the two channels we bond to create a 40Mhz
 * channel are "ctl" and "ext"; "ext" is +/- one channel from "ctl".
 * 
 * Note that 25Mhz spacing is supported by hardware, allowing up to 
 * 50Mhz wide channels; this capability is not explored.
 */
void jaldi_hw_get_channel_centers(struct jaldi_channel *chan, 
		struct chan_centers *centers) {

	int8_t extoff; // +/- offset

	if(!IS_CHAN_HT40(chan)) {
		centers->ctl_center = centers->ext_center = centers->synth_center = chan->channel;
		return;
	}

	if (!IS_CHAN_HT40(chan)) {
		centers->ctl_center = centers->ext_center =
			centers->synth_center = chan->channel;
		return;
	}

	if (chan->chanmode == CHANNEL_HT40PLUS) { 
		centers->synth_center =
			chan->channel + HT40_CHANNEL_CENTER_SHIFT;
		extoff = 1;
	} else {
		centers->synth_center =
			chan->channel - HT40_CHANNEL_CENTER_SHIFT;
		extoff = -1;
	}

	centers->ctl_center =
		centers->synth_center - (extoff * HT40_CHANNEL_CENTER_SHIFT);
	/* "25 MHz spacing is supported by hw but not on upper layers" 
	 * TODO: Does this apply for jaldi, or is it just a mac80211 thing?
	 */
	centers->ext_center =
		centers->synth_center + (extoff * HT40_CHANNEL_CENTER_SHIFT);
}


// Initialization and bus

/* This function reads hardware version information from the device. It is 
 * closely based upon ath9k_hw_read_revisions in a9k/hw.c. This is essentially
 * reading directly from HW registers, so its operation is a bit opaque.
 */
static void jaldi_hw_read_versions(struct jaldi_hw *hw) {
	u32 val;
	val = REG_READ(hw, AR_SREV) & AR_SREV_ID;

	if (val == 0xFF) {
		val = REG_READ(ah, AR_SREV);
		hw->hw_version.macVersion =
			(val & AR_SREV_VERSION2) >> AR_SREV_TYPE2_S;
		hw->hw_version.macRev = MS(val, AR_SREV_REVISION2); // TODO: unclear what this is doing, needs doc
		hw->is_pciexpress = (val & AR_SREV_TYPE2_HOST_MODE) ? 0 : 1;
	} else {
		if (!AR_SREV_9100(hw))
			hw->hw_version.macVersion = MS(val, AR_SREV_VERSION);

		hw->hw_version.macRev = val & AR_SREV_REVISION;

		if (hw->hw_version.macVersion == AR_SREV_VERSION_5416_PCIE)
			hw->is_pciexpress = true;
	}
}

/* Checks to see if we have a pending interrupt in hw 
 * We use this when we're on a shared IRQ to identify our interrupts
 */
bool jaldi_hw_intrpend(struct jaldi_hw *hw) {
	u32 host_isr;

	host_isr = REG_READ(hw, AR_INTR_ASYNC_CAUSE);

	if ((host_isr & AR_INTR_MAC_IRQ) 
		&& (host_isr != AR_INTR_SPURIOUS)) { return true; }
	
	host_isr = REG_READ(hw, AR_INTR_SYNC_CAUSE);
	if ((host_isr & AR_INTR_SYNC_DEFAULT) 
		&& (host_isr != AR_INTR_SPURIOUS)) { return true; }

	return false;
}

/* wait for a register value to be set to some desired value */
bool jaldi_hw_wait(struct jaldi_hw *hw, u32 reg, u32 mask, u32 val, u32 timeout)
{
	int i;

	BUG_ON(timeout < JALDI_TIME_QUANTUM);

	for (i = 0; i < (timeout / JALDI_TIME_QUANTUM); i++) {
		if ((REG_READ(hw, reg) & mask) == val)
			return true;

		udelay(JALDI_TIME_QUANTUM);
	}

	jaldi_print(JALDI_WARN,
		  "timeout (%d us) on reg 0x%x: 0x%08x & 0x%08x != 0x%08x\n",
		  timeout, reg, REG_READ(ah, reg), mask, val);

	return false;
}

/* Time synchronization function
 * Reads 64bit TSF from hardware. Could be useful. */
#define JALDI_MAX_TSF_READ 10
u64 jaldi_hw_gettsf64(struct jaldi_hw *hw)
{
	u32 tsf_lower, tsf_upper1, tsf_upper2;
	int i;

	tsf_upper1 = REG_READ(hw, AR_TSF_U32);
	for (i = 0; i < JALDI_MAX_TSF_READ; i++) {
		tsf_lower = REG_READ(hw, AR_TSF_L32);
		tsf_upper2 = REG_READ(hw, AR_TSF_U32);
		if (tsf_upper2 == tsf_upper1)
			break;
		tsf_upper1 = tsf_upper2;
	}

	WARN_ON( i == JALDI_MAX_TSF_READ );
	return (((u64)tsf_upper1 << 32) | tsf_lower);
}

/***********
 * Power Management
 ***********/
static bool jaldi_hw_set_power_awake(struct jaldi_hw *hw, int setChip)
{
	u32 val;
	int i;

	if (setChip) {
		if ((REG_READ(hw, AR_RTC_STATUS) &
		     AR_RTC_STATUS_M) == AR_RTC_STATUS_SHUTDOWN) {
			if (jaldi_hw_set_reset_reg(hw,
					   JALDI_RESET_POWER_ON) != true) {
				return false;
			}
			if (!AR_SREV_9300_20_OR_LATER(hw))
				jaldi_hw_init_pll(hw, NULL);
		}
		if (AR_SREV_9100(hw))
			REG_SET_BIT(hw, AR_RTC_RESET,
				    AR_RTC_RESET_EN);

		REG_SET_BIT(hw, AR_RTC_FORCE_WAKE,
			    AR_RTC_FORCE_WAKE_EN);
		udelay(50);

		for (i = POWER_UP_TIME / 50; i > 0; i--) {
			val = REG_READ(hw, AR_RTC_STATUS) & AR_RTC_STATUS_M;
			if (val == AR_RTC_STATUS_ON)
				break;
			udelay(50);
			REG_SET_BIT(hw, AR_RTC_FORCE_WAKE,
				    AR_RTC_FORCE_WAKE_EN);
		}
		if (i == 0) {
			ath_print(jaldi_hw_common(hw), ATH_DBG_FATAL,
				  "Failed to wakeup in %uus\n",
				  POWER_UP_TIME / 20);
			return false;
		}
	}

	REG_CLR_BIT(hw, AR_STA_ID1, AR_STA_ID1_PWR_SAV);

	return true;
}

/* We should not be putting device to sleep, so this should not be used */
static void jaldi_set_power_sleep(struct jaldi_hw *hw, int setChip)
{
	REG_SET_BIT(hw, AR_STA_ID1, AR_STA_ID1_PWR_SAV);
	if (setChip) {
		/*
		 * Clear the RTC force wake bit to allow the
		 * mac to go to sleep.
		 */
		REG_CLR_BIT(hw, AR_RTC_FORCE_WAKE,
			    AR_RTC_FORCE_WAKE_EN);
		if (!AR_SREV_9100(hw) && !AR_SREV_9300_20_OR_LATER(hw))
			REG_WRITE(hw, AR_RC, AR_RC_AHB | AR_RC_HOSTIF);

		/* Shutdown chip. Active low */
		if (!AR_SREV_5416(hw) && !AR_SREV_9271(hw))
			REG_CLR_BIT(hw, (AR_RTC_RESET),
				    AR_RTC_RESET_EN);
	}
}

bool jaldi_hw_setpower(struct jaldi_hw *hw, enum jaldi_power_mode mode)
{
	int status = true, setChip = true;
	static const char *modes[] = {
		"AWAKE",
		"FULL-SLEEP",
		"NETWORK SLEEP",
		"UNDEFINED"
	};

	if (hw->power_mode == mode)
		return status;

	jaldi_print(JALDI_DEBUG, "%s -> %s\n",
		  modes[hw->power_mode], modes[mode]);

	switch (mode) {
	case JALDI_PM_AWAKE:
		status = jaldi_hw_set_power_awake(hw, setChip);
		break;
	case JALDI_PM_FULL_SLEEP:
		jaldi_set_power_sleep(hw, setChip);
		hw->chip_fullsleep = true;
		break;
	case JALDI_PM_NETWORK_SLEEP:
		jaldi_print(JALDI_FATAL,"Network sleep is not supported.\n");
		return false;
	default:
		jaldi_print(JALDI_FATAL,
			  "Unknown power mode %u\n", mode);
		return false;
	}
	hw->power_mode = mode;

	return status;
}

/************************************/
/* HW Attach, Detach, Init Routines */
/************************************/
static void jaldi_hw_disablepcie(struct jaldi_hw *hw)
{
	if (AR_SREV_9100(hw))
		return;

	ENABLE_REGWRITE_BUFFER(hw);

	REG_WRITE(hw, AR_PCIE_SERDES, 0x9248fc00);
	REG_WRITE(hw, AR_PCIE_SERDES, 0x24924924);
	REG_WRITE(hw, AR_PCIE_SERDES, 0x28000029);
	REG_WRITE(hw, AR_PCIE_SERDES, 0x57160824);
	REG_WRITE(hw, AR_PCIE_SERDES, 0x25980579);
	REG_WRITE(hw, AR_PCIE_SERDES, 0x00000000);
	REG_WRITE(hw, AR_PCIE_SERDES, 0x1aaabe40);
	REG_WRITE(hw, AR_PCIE_SERDES, 0xbe105554);
	REG_WRITE(hw, AR_PCIE_SERDES, 0x000e1007);

	REG_WRITE(hw, AR_PCIE_SERDES2, 0x00000000);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);
}

/* This should work for all families including legacy */
static bool jaldi_hw_chip_test(struct jaldi_hw *hw)
{
	u32 regAddr[2] = { AR_STA_ID0 };
	u32 regHold[2];
	u32 patternData[4] = { 0x55555555,
			       0xaaaaaaaa,
			       0x66666666,
			       0x99999999 };
	int i, j, loop_max;

	if (!AR_SREV_9300_20_OR_LATER(hw)) {
		loop_max = 2;
		regAddr[1] = AR_PHY_BASE + (8 << 2);
	} else
		loop_max = 1;

	for (i = 0; i < loop_max; i++) {
		u32 addr = regAddr[i];
		u32 wrData, rdData;

		regHold[i] = REG_READ(hw, addr);
		for (j = 0; j < 0x100; j++) {
			wrData = (j << 16) | j;
			REG_WRITE(hw, addr, wrData);
			rdData = REG_READ(hw, addr);
			if (rdData != wrData) {
				jaldi_print(JALDI_DBG_FATAL,
					  "address test failed "
					  "addr: 0x%08x - wr:0x%08x != "
					  "rd:0x%08x\n",
					  addr, wrData, rdData);
				return false;
			}
		}
		for (j = 0; j < 4; j++) {
			wrData = patternData[j];
			REG_WRITE(hw, addr, wrData);
			rdData = REG_READ(hw, addr);
			if (wrData != rdData) {
				jaldi_print(JALDI_DBG_FATAL,
					  "address test failed "
					  "addr: 0x%08x - wr:0x%08x != "
					  "rd:0x%08x\n",
					  addr, wrData, rdData);
				return false;
			}
		}
		REG_WRITE(ah, regAddr[i], regHold[i]);
	}
	udelay(100);

	return true;
}

static void jaldi_hw_init_config(struct jaldi_hw *hw)
{ 
	// TODO: right now, does nothing. set hw config here as needed.
}

	

static void jaldi_hw_init_defaults(struct jaldi_hw *hw)
{
	hw->hw_version.magic = AR5416_MAGIC; // TODO: Why this number?
	hw->hw_version.subvendorid = 0;

	hw->ah_flags = 0;
	if (!AR_SREV_9100(hw)) { hw->ah_flags = AH_USE_EEPROM; }

	hw->slottime = (u32) -1; // TODO: why?
	hw->globaltxtimeout = (u32) -1;
	hw->power_mode = JALDI_PM_UNDEFINED;
	// TODO: other default settings
}

static void jaldi_hw_init_macaddr {
	
	struct jaldi_softc *sc = hw->sc;
	u32 sum; 
	int i;
	u16 eeval;
	u32 EEP_MAC[] = { EEP_MAC_LSW, EEP_MAC_MID, EEP_MAC_MSW };

	sum = 0;
	for (i = 0; i < 3; i++) {
		eeval = ah->eep_ops->get_eeprom(ah, EEP_MAC[i]);
		sum += eeval;
		common->macaddr[2 * i] = eeval >> 8;
		common->macaddr[2 * i + 1] = eeval & 0xff;
	}
	if (sum == 0 || sum == 0xffff * 3)
		return -EADDRNOTAVAIL;

	return 0;
}

static inline void jaldi_hw_set_dma(struct jaldi_hw *hw)
{
	u32 regval;

	ENABLE_REGWRITE_BUFFER(hw);

	/*
	 * set AHB_MODE not to do cacheline prefetches
	*/
	if (!AR_SREV_9300_20_OR_LATER(hw)) {
		regval = REG_READ(hw, AR_AHB_MODE);
		REG_WRITE(hw, AR_AHB_MODE, regval | AR_AHB_PREFETCH_RD_EN);
	}

	/*
	 * let mac dma reads be in 128 byte chunks
	 */
	regval = REG_READ(hw, AR_TXCFG) & ~AR_TXCFG_DMASZ_MASK;
	REG_WRITE(hw, AR_TXCFG, regval | AR_TXCFG_DMASZ_128B);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);

	/*
	 * Restore TX Trigger Level to its pre-reset value.
	 * The initial value depends on whether aggregation is enabled, and is
	 * adjusted whenever underruns are detected.
	 */
	if (!AR_SREV_9300_20_OR_LATER(hw))
		REG_RMW_FIELD(hw, AR_TXCFG, AR_FTRIG, hw->tx_trig_level);

	ENABLE_REGWRITE_BUFFER(hw);

	/*
	 * let mac dma writes be in 128 byte chunks
	 */
	regval = REG_READ(hw, AR_RXCFG) & ~AR_RXCFG_DMASZ_MASK;
	REG_WRITE(hw, AR_RXCFG, regval | AR_RXCFG_DMASZ_128B);

	/*
	 * Setup receive FIFO threshold to hold off TX activities
	 */
	REG_WRITE(hw, AR_RXFIFO_CFG, 0x200);

	/*
	 * reduce the number of usable entries in PCU TXBUF to avoid
	 * wrap around issues.
	 */
	if (AR_SREV_9285(hw)) {
		/* For AR9285 the number of Fifos are reduced to half.
		 * So set the usable tx buf size also to half to
		 * avoid data/delimiter underruns
		 */
		REG_WRITE(hw, AR_PCU_TXBUF_CTRL,
			  AR_9285_PCU_TXBUF_CTRL_USABLE_SIZE);
	} else if (!AR_SREV_9271(hw)) {
		REG_WRITE(hw, AR_PCU_TXBUF_CTRL,
			  AR_PCU_TXBUF_CTRL_USABLE_SIZE);
	}

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);
}

static bool jaldi_hw_set_reset(struct jaldi_hw *hw, int type) {

}

static bool jaldi_hw_set_reset_power_on(struct jaldi_hw *hw) {
	ENABLE_REGWRITE_BUFFER(hw);

	REG_WRITE(hw, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN |
		AR_RTC_FORCE_WAKE_ON_INT);

	if (!AR_SREV_9100(hw) && !AR_SREV_9300_20_OR_LATER(hw))
		{ REG_WRITE(hw, AR_RC, AR_RC_AHB); }

	REG_WRITE(hw, AR_RTC_RESET, 0);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);

	if (!AR_SREV_9300_20_OR_LATER(hw))
		udelay(2);

	if (!AR_SREV_9100(hw) && !AR_SREV_9300_20_OR_LATER(hw))
		REG_WRITE(hw, AR_RC, 0);

	REG_WRITE(hw, AR_RTC_RESET, 1);

	if (!jaldi_hw_wait(hw,
			   AR_RTC_STATUS,
			   AR_RTC_STATUS_M,
			   AR_RTC_STATUS_ON,
			   AH_WAIT_TIMEOUT)) {
		jaldi_print(ATH_DBG_RESET,
			  "RTC not waking up\n");
		return false;
	}

	jaldi_hw_read_versions(hw);

	return jaldi_hw_set_reset(hw, JALDI_RESET_WARM);
}

static bool jaldi_hw_set_reset_reg(struct jaldi_hw *hw, u32 type) {
	REG_WRITE(ah, AR_RTC_FORCE_WAKE,
		  AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT);

	switch (type) {
	case JALDI_RESET_POWER_ON:
		return jaldi_hw_set_reset_power_on(ah);
	case JALDI_RESET_WARM:
	case JALDI_RESET_COLD:
		return jaldi_hw_set_reset(ah, type);
	default:
		return false;
	}
}



static bool jaldi_hw_reset(struct jaldi_hw *hw, struct jaldi_channel *chan,
				bool bChangeChannel) {
	/* Steps
	 * 1. Save existing hw state (chainmask, channel, etc)
	 * 2. Reset the chip.
	 * 3. Re-initialize hw with saved state
	 */

	u32 saveDefAntenna;
	u64 tsf = 0;
	 // chainmask save goes here
	struct jaldi_channel *curchan = hw->curchan;

	if(!hw->chip_fullsleep) {
		jaldi_hw_abortpcurecv(hw); // TODO
		if(!jaldi_hw_stopdmarecv(hw)) 
			{ jaldi_print(0,"Failed to stop recv dma\n"); }
	}
	

	if (!jaldi_hw_setpower(hw, JALDI_PM_AWAKE))
		return -EIO;

	if (curchan && !hw->chip_fullsleep)
		jaldi_hw_getnf(hw, curchan); // TODO
	

	// load new noise floor info if we're changing the channel
	if (bChannelChange &&
	    (hw->chip_fullsleep != true) &&
	    (hw->curchan != NULL) &&
	    (chan->channel != hw->curchan->channel) &&
	    ((chan->channelFlags & CHANNEL_ALL) ==
	     (hw->curchan->channelFlags & CHANNEL_ALL)) &&
	    !AR_SREV_9280(ah)) {

		if (jaldi_hw_channel_change(hw, chan)) { // TODO
		// loadnf seems to only be implemented for ar9003
		//	jaldi_hw_loadnf(hw, hw->curchan); 
			jaldi_hw_start_nfcal(hw);
			return 0;
		}
	}

	
	saveDefAntenna = REG_READ(hw, AR_DEF_ANTENNA);
	if (saveDefAntenna == 0)
		saveDefAntenna = 1;

	/* For chips on which RTC reset is done, save TSF before it gets cleared */
	if (AR_SREV_9280(hw) && hw->eep_ops->get_eeprom(hw, EEP_OL_PWRCTRL))
		tsf = jaldi_hw_gettsf64(ah);

	
	/* Only required on the first reset (TODO: Only run on first reset...) */
	if (AR_SREV_9271(hw)) {
		REG_WRITE(ah,
			  AR9271_RESET_POWER_DOWN_CONTROL,
			  AR9271_RADIO_RF_RST);
		udelay(50);
	}

	if (!jaldi_hw_chip_reset(hw, chan)) {
		jaldi_print(JALDI_FATAL, "Chip reset failed\n");
		return -EINVAL;
	}


	
}

static void jaldi_hw_attach_ops(struct jaldi_hw *hw)
{
	struct 	
}

static int __jaldi_hw_init(struct jaldi_hw *hw)
{
	int res = 0;

	if (hw->hw_version.devid != AR9280_DEVID_PCI 
		|| hw->hw_version.devid != AR9280_DEVID_PCIE) {
		jaldi_print(0,"This device is not supported by JaldiMAC.\n");
		// goal here is to only allow ar9280 (ubnt ns5m and similar) to work

		return
	}

	if(!jaldi_hw_set_reset_reg(hw, JALDI_RESET_POWER_ON)) {
		jaldi_print(JALDI_DEBUG_FATAL, "Couldn't reset the chip (%s, line %d)\n", __FILE__, __LINE__);
		return -EIO;
	}

}


