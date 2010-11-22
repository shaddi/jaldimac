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
		val = REG_READ(hw, AR_SREV);
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

/********************/
/* Helper Functions */
/********************/

/* Translates times to clock cycles, which are used when writing slot duration 
 * to the device. */
static u32 jaldi_hw_mac_clks(struct jaldi_hw *hw, u32 usecs)
{
	if (!hw->curchan) /* should really check for CCK instead */
		return usecs *JALDI_CLOCK_RATE_CCK;

	/* We don't use the 2Ghz band so this isn't called. 
	 * if (conf->channel->band == IEEE80211_BAND_2GHZ)
	 * 	return usecs *JALDI_CLOCK_RATE_2GHZ_OFDM;
	 */

	if (hw->caps.hw_caps & JALDI_HW_CAP_FASTCLOCK)
		return usecs * JALDI_CLOCK_FAST_RATE_5GHZ_OFDM;
	else
		return usecs * JALDI_CLOCK_RATE_5GHZ_OFDM;
}

static u32 jaldi_hw_mac_to_clks(struct jaldi_hw *hw, u32 usecs)
{
	/* probably should check a config setting somewhere rather than channel */
	if (hw->curchan !=0 &&
		IS_CHAN_HT40(hw->curchan))
		return jaldi_hw_mac_clks(hw, usecs) * 2;
	else
		return jaldi_hw_mac_clks(hw, usecs);
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
		  timeout, reg, REG_READ(hw, reg), mask, val);

	return false;
}

u32 jaldi_hw_reverse_bits(u32 val, u32 n)
{
	u32 retval;
	int i;

	for (i = 0, retval = 0; i < n; i++) {
		retval = (retval << 1) | (val & 1);
		val >>= 1;
	}
	return retval;
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

static void jaldi_hw_set_operating_mode(struct jaldi_hw *hw, int opmode)
{
	u32 val;

	val = REG_READ(hw, AR_STA_ID1);
	val &= ~(AR_STA_ID1_STA_AP | AR_STA_ID1_ADHOC);
	switch (opmode) {
	case JALDI_MASTER:
		hw->opmode = JALDI_MASTER;
		break;
	case JALDI_CLIENT:
		hw->opmode = JALDI_CLIENT;
		break;
	default:
		jaldi_print(JALDI_INFO, "Invalid opmode, ignoring\n");
	}

	// We operate in ad-hoc mode. Monitor commented out. See hw.c in a9k
	// for more details.
	val |= (AR_STA_ID1_ADHOC | AR_STA_ID1_KSRCH_MODE);
	REG_WRITE(hw, AR_STA_ID1, val);
	REG_SET_BIT(hw, AR_CFG, AR_CFG_AP_ADHOC_INDICATION);
	
	/* This is how we would set monitor mode. */
	//REG_WRITE(ah, AR_STA_ID1, val | AR_STA_ID1_KSRCH_MODE);
}

/************
 * ar9002
 ************/
void jaldi_hw_enable_async_fifo(struct jaldi_hw *hw)
{
	if (AR_SREV_9287_13_OR_LATER(hw)) {
		REG_SET_BIT(hw, AR_MAC_PCU_ASYNC_FIFO_REG3,
				AR_MAC_PCU_ASYNC_FIFO_REG3_DATAPATH_SEL);
		REG_SET_BIT(hw, AR_PHY_MODE, AR_PHY_MODE_ASYNCFIFO);
		REG_CLR_BIT(hw, AR_MAC_PCU_ASYNC_FIFO_REG3,
				AR_MAC_PCU_ASYNC_FIFO_REG3_SOFT_RESET);
		REG_SET_BIT(hw, AR_MAC_PCU_ASYNC_FIFO_REG3,
				AR_MAC_PCU_ASYNC_FIFO_REG3_SOFT_RESET);
	}
}

/*
 * If Async FIFO is enabled, the following counters change as MAC now runs
 * at 117 Mhz instead of 88/44MHz when async FIFO is disabled.
 *
 * The values below tested for ht40 2 chain.
 * Overwrite the delay/timeouts initialized in process ini.
 */
void jaldi_hw_update_async_fifo(struct jaldi_hw *hw)
{
	if (AR_SREV_9287_13_OR_LATER(hw)) {
		REG_WRITE(hw, AR_D_GBL_IFS_SIFS,
				AR_D_GBL_IFS_SIFS_ASYNC_FIFO_DUR);
		REG_WRITE(hw, AR_D_GBL_IFS_SLOT,
				AR_D_GBL_IFS_SLOT_ASYNC_FIFO_DUR);
		REG_WRITE(hw, AR_D_GBL_IFS_EIFS,
				AR_D_GBL_IFS_EIFS_ASYNC_FIFO_DUR);

		REG_WRITE(hw, AR_TIME_OUT, AR_TIME_OUT_ACK_CTS_ASYNC_FIFO_DUR);
		REG_WRITE(hw, AR_USEC, AR_USEC_ASYNC_FIFO_DUR);

		REG_SET_BIT(hw, AR_MAC_PCU_LOGIC_ANALYZER,
				AR_MAC_PCU_LOGIC_ANALYZER_DISBUG20768);
		REG_RMW_FIELD(hw, AR_AHB_MODE, AR_AHB_CUSTOM_BURST_EN,
				AR_AHB_CUSTOM_BURST_ASYNC_FIFO_VAL);
	}
}

static bool jaldi_hw_macversion_supported(struct jaldi_hw *hw) 
{
	u32 macversion = hw->hw_version.macVersion;

	switch(macversion) {
		case AR_SREV_VERSION_5416_PCI:
		case AR_SREV_VERSION_5416_PCIE:
		case AR_SREV_VERSION_9160:
		case AR_SREV_VERSION_9100:
		case AR_SREV_VERSION_9280:
		case AR_SREV_VERSION_9285:
		case AR_SREV_VERSION_9287:
		case AR_SREV_VERSION_9271:
			return true;
		default:
			break;
	}
	return false;
}


/************
 * ar5008 phy
 ************/
static void jaldi_hw_init_baseband(struct jaldi_hw *hw,
			      struct jaldi_channel *chan)
{
	u32 synthDelay;

	synthDelay = REG_READ(hw, AR_PHY_RX_DELAY) & AR_PHY_RX_DELAY_DELAY;
	if (IS_CHAN_B(chan))
		synthDelay = (4 * synthDelay) / 22;
	else
		synthDelay /= 10;

	/* Enables the phy */
	REG_WRITE(hw, AR_PHY_ACTIVE, AR_PHY_ACTIVE_EN);

	udelay(synthDelay + BASE_ACTIVATE_DELAY);
}

static void jaldi_hw_restore_chainmask(struct jaldi_hw *hw)
{
	int rx_chainmask = hw->rxchainmask;

	if ((rx_chainmask == 0x5) || (rx_chainmask == 0x3)) {
		REG_WRITE(hw, AR_PHY_RX_CHAINMASK, rx_chainmask);
		REG_WRITE(hw, AR_PHY_CAL_CHAINMASK, rx_chainmask);
	}
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
	DBG_START_MSG;
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
				jaldi_print(JALDI_FATAL,
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
				jaldi_print(JALDI_FATAL,
					  "address test failed "
					  "addr: 0x%08x - wr:0x%08x != "
					  "rd:0x%08x\n",
					  addr, wrData, rdData);
				return false;
			}
		}
		REG_WRITE(hw, regAddr[i], regHold[i]);
	}
	udelay(100);

	return true;
}

static void jaldi_hw_init_config(struct jaldi_hw *hw)
{ 
	DBG_START_MSG;	
	hw->analog_shiftreg = 1;

	if (hw->hw_version.devid != AR2427_DEVID_PCIE)
		hw->ht_enable = 1;
	else 
		hw->ht_enable = 0;

	/* From ath9k:
	 * "We need this for PCI devices only (Cardbus, PCI, miniPCI)
	 * _and_ if on non-uniprocessor systems (Multiprocessor/HT).
	 * This means we use it for all AR5416 devices, and the few
	 * minor PCI AR9280 devices out there.
	 *
	 * Serialization is required because these devices do not handle
	 * well the case of two concurrent reads/writes due to the latency
	 * involved. During one read/write another read/write can be issued
	 * on another CPU while the previous read/write may still be working
	 * on our hardware, if we hit this case the hardware poops in a loop.
	 * We prevent this by serializing reads and writes.
	 *
	 * This issue is not present on PCI-Express devices or pre-AR5416
	 * devices (legacy, 802.11abg)."
	 */
	if (num_possible_cpus() > 1)
		hw->serialize_regmode = SER_REG_MODE_AUTO;
}

void jaldi_hw_deinit(struct jaldi_hw *hw) 
{
	if (hw->dev_state < JALDI_HW_INITIALIZED) return;

	jaldi_hw_setpower(hw, JALDI_PM_FULL_SLEEP);
}

static void jaldi_hw_setslottime(struct jaldi_hw *hw, u32 us)
{
	u32 val = jaldi_hw_mac_to_clks(hw, us);
	val = min(val, (u32) 0xFFFF);
	REG_WRITE(hw, AR_D_GBL_IFS_SLOT, val);
}

static void jaldi_hw_set_ack_timeout(struct jaldi_hw *hw, u32 us)
{
	u32 val = jaldi_hw_mac_to_clks(hw, us);
	val = min(val, (u32) MS(0xFFFFFFFF, AR_TIME_OUT_ACK));
	REG_RMW_FIELD(hw, AR_TIME_OUT, AR_TIME_OUT_ACK, val);
}

static void jaldi_hw_set_cts_timeout(struct jaldi_hw *hw, u32 us)
{
	u32 val = jaldi_hw_mac_to_clks(hw, us);
	val = min(val, (u32) MS(0xFFFFFFFF, AR_TIME_OUT_CTS));
	REG_RMW_FIELD(hw, AR_TIME_OUT, AR_TIME_OUT_CTS, val);
}

static bool jaldi_hw_set_global_txtimeout(struct jaldi_hw *hw, u32 tu)
{
	if (tu > 0xFFFF) {
		jaldi_print(JALDI_DEBUG,
			  "bad global tx timeout %u\n", tu);
		hw->globaltxtimeout = (u32) -1;
		return false;
	} else {
		REG_RMW_FIELD(hw, AR_GTXTO, AR_GTXTO_TIMEOUT_LIMIT, tu);
		hw->globaltxtimeout = tu;
		return true;
	}
}

void jaldi_hw_init_global_settings(struct jaldi_hw *hw)
{
	int acktimeout;
	int slottime;
	int ifstime;

	// TODO: set default ifs and slot times
	ifstime = hw->ifstime;
	slottime = hw->slottime;
	acktimeout = slottime + ifstime; // TODO: is this what we want? may want to disable acktimeouts to handle at higher level.

	jaldi_hw_setslottime(hw, slottime);
	jaldi_hw_set_ack_timeout(hw, acktimeout);
	jaldi_hw_set_cts_timeout(hw, acktimeout);
	if (hw->globaltxtimeout != (u32) -1)
		jaldi_hw_set_global_txtimeout(hw, hw->globaltxtimeout);
}

static void jaldi_hw_init_defaults(struct jaldi_hw *hw)
{
	DBG_START_MSG;	
	hw->hw_version.magic = AR5416_MAGIC;
	hw->hw_version.subvendorid = 0;

	/* 406 platform data patch */
	//hw->hw_flags = 0;
	//if (!AR_SREV_9100(hw)) { hw->hw_flags = AH_USE_EEPROM; }

	hw->slottime = (u32) -1;
	hw->ifstime = (u32) -1;
	hw->globaltxtimeout = (u32) -1;
	hw->disable_acks = true;
	hw->disable_cs = true;
	hw->power_mode = JALDI_PM_UNDEFINED;
}

static int jaldi_hw_init_macaddr(struct jaldi_hw *hw) {
	
	struct jaldi_softc *sc = hw->sc;
	u32 sum; 
	int i;
	u16 eeval;
	u32 EEP_MAC[] = { EEP_MAC_LSW, EEP_MAC_MID, EEP_MAC_MSW };

	sum = 0;
	for (i = 0; i < 3; i++) {
		eeval = hw->eep_ops->get_eeprom(hw, EEP_MAC[i]);
		sum += eeval;
		sc->macaddr[2 * i] = eeval >> 8;
		sc->macaddr[2 * i + 1] = eeval & 0xff;
	}
	if (sum == 0 || sum == 0xffff * 3)
		return -EADDRNOTAVAIL;

	return 0;
}

static void jaldi_hw_init_pll(struct jaldi_hw *hw,
			      struct jaldi_channel *chan)
{
	DBG_START_MSG;
	u32 pll = jaldi_get_hw_ops(hw)->compute_pll_control(hw, chan);

	REG_WRITE(hw, AR_RTC_PLL_CONTROL, pll);

	/* Switch the core clock for ar9271 to 117Mhz */
	if (AR_SREV_9271(hw)) {
		udelay(500);
		REG_WRITE(hw, 0x50040, 0x304);
	}

	udelay(RTC_PLL_SETTLE_DELAY);

	REG_WRITE(hw, AR_RTC_SLEEP_CLK, AR_RTC_FORCE_DERIVED_CLK);
}

/**********************************/
/* Hw Reset and Channel Switching */
/**********************************/

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
	u32 rst_flags;
	u32 tmpReg;

	DBG_START_MSG;	
	if (AR_SREV_9100(hw)) {
		u32 val = REG_READ(hw, AR_RTC_DERIVED_CLK);
		val &= ~AR_RTC_DERIVED_CLK_PERIOD;
		val |= SM(1, AR_RTC_DERIVED_CLK_PERIOD);
		REG_WRITE(hw, AR_RTC_DERIVED_CLK, val);
		(void)REG_READ(hw, AR_RTC_DERIVED_CLK);
	}

	ENABLE_REGWRITE_BUFFER(hw);

	REG_WRITE(hw, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN |
		  AR_RTC_FORCE_WAKE_ON_INT);

	if (AR_SREV_9100(hw)) {
		rst_flags = AR_RTC_RC_MAC_WARM | AR_RTC_RC_MAC_COLD |
			AR_RTC_RC_COLD_RESET | AR_RTC_RC_WARM_RESET;
	} else {
		tmpReg = REG_READ(hw, AR_INTR_SYNC_CAUSE);
		if (tmpReg &
		    (AR_INTR_SYNC_LOCAL_TIMEOUT |
		     AR_INTR_SYNC_RADM_CPL_TIMEOUT)) {
			u32 val;
			REG_WRITE(hw, AR_INTR_SYNC_ENABLE, 0);

			val = AR_RC_HOSTIF;
			if (!AR_SREV_9300_20_OR_LATER(hw))
				val |= AR_RC_AHB;
			REG_WRITE(hw, AR_RC, val);

		} else if (!AR_SREV_9300_20_OR_LATER(hw))
			REG_WRITE(hw, AR_RC, AR_RC_AHB);

		rst_flags = AR_RTC_RC_MAC_WARM;
		if (type == JALDI_RESET_COLD)
			rst_flags |= AR_RTC_RC_MAC_COLD;
	}

	REG_WRITE(hw, AR_RTC_RC, rst_flags);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);

	udelay(50);

	REG_WRITE(hw, AR_RTC_RC, 0);
	if (!jaldi_hw_wait(hw, AR_RTC_RC, AR_RTC_RC_M, 0, JALDI_WAIT_TIMEOUT)) {
		jaldi_print(JALDI_WARN,
			  "RTC stuck in MAC reset\n");
	}

	if (!AR_SREV_9100(hw))
		REG_WRITE(hw, AR_RC, 0);

	if (AR_SREV_9100(hw))
		udelay(50);

	return true;
}

static bool jaldi_hw_set_reset_power_on(struct jaldi_hw *hw) 
{
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
			   JALDI_WAIT_TIMEOUT)) {
		jaldi_print(JALDI_DEBUG,
			  "RTC not waking up\n");
		return false;
	}

	jaldi_hw_read_versions(hw);

	return jaldi_hw_set_reset(hw, JALDI_RESET_WARM);
}

static bool jaldi_hw_set_reset_reg(struct jaldi_hw *hw, u32 type) {
	DBG_START_MSG;	
	REG_WRITE(hw, AR_RTC_FORCE_WAKE,
		  AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT);

	switch (type) {
	case JALDI_RESET_POWER_ON:
		return jaldi_hw_set_reset_power_on(hw);
	case JALDI_RESET_WARM:
	case JALDI_RESET_COLD:
		return jaldi_hw_set_reset(hw, type);
	default:
		return false;
	}
}

static bool jaldi_hw_chip_reset(struct jaldi_hw *hw,
				struct jaldi_channel *chan)
{
	DBG_START_MSG;	
	if (AR_SREV_9280(hw) && hw->eep_ops->get_eeprom(hw, EEP_OL_PWRCTRL)) {
		if (!jaldi_hw_set_reset_reg(hw, JALDI_RESET_POWER_ON))
			return false;
	} else if (!jaldi_hw_set_reset_reg(hw, JALDI_RESET_WARM))
		return false;

	if (!jaldi_hw_setpower(hw, JALDI_PM_AWAKE))
		return false;

	hw->chip_fullsleep = false;
	jaldi_hw_init_pll(hw, chan);
	jaldi_get_hw_ops(hw)->set_rfmode(hw, chan);

	return true;
}

static bool jaldi_hw_channel_change(struct jaldi_hw *hw,
				    struct jaldi_channel *chan)
{
	u32 qnum;
	int r;

	for (qnum = 0; qnum < AR_NUM_QCU; qnum++) {
		if (jaldi_hw_numtxpending(hw, qnum)) {
			jaldi_print(JALDI_ALERT,
				  "Transmit frames pending on "
				  "queue %d\n", qnum);
			return false;
		}
	}

	if (!jaldi_get_hw_ops(hw)->rfbus_req(hw)) {
		jaldi_print(JALDI_FATAL,
			  "Could not kill baseband RX\n");
		return false;
	}

	jaldi_get_hw_ops(hw)->set_channel_regs(hw, chan);

	r = jaldi_get_hw_ops(hw)->rf_set_freq(hw, chan);
	if (r) {
		jaldi_print(JALDI_FATAL,
			  "Failed to set channel\n");
		return false;
	}

	hw->eep_ops->set_txpower(hw, chan,
			     0x01ff, /* see regd.h NO_CTL in ath for details, this is DEBUG_REG_DMN */
	//		     channel->max_antenna_gain * 2,
	//		     channel->max_power * 2,
				0, 0,	// TODO: what should these values be?
			     (u32) MAX_RATE_POWER);

	jaldi_get_hw_ops(hw)->rfbus_done(hw);

	if (IS_CHAN_OFDM(chan) || IS_CHAN_HT(chan))
//		ath9k_hw_set_delta_slope(hw, chan); // TODO

	jaldi_get_hw_ops(hw)->spur_mitigate_freq(hw, chan);

	/* TODO: we're not doing calibration yet.
	if (!chan->oneTimeCalsDone)
		chan->oneTimeCalsDone = true;
	*/

	return true;
}

int jaldi_hw_reset(struct jaldi_hw *hw, struct jaldi_channel *chan,
				bool bChannelChange) {
	/* Steps
	 * 1. Save existing hw state (chainmask, channel, etc)
	 * 2. Reset the chip.
	 * 3. Re-initialize hw with saved state
	 */

	u32 saveDefAntenna;
	u32 macStaId1;
	u64 tsf = 0;
	int i, r;
	 // chainmask save goes here
	struct jaldi_channel *curchan = hw->curchan;

	DBG_START_MSG;

	if(!hw->chip_fullsleep) {
		jaldi_hw_abortpcurecv(hw); 
		if(!jaldi_hw_stopdmarecv(hw)) 
			{ jaldi_print(0,"Failed to stop recv dma\n"); }
	}

	if (!jaldi_hw_setpower(hw, JALDI_PM_AWAKE))
		return -EIO;

//	if (curchan && !hw->chip_fullsleep)
//		jaldi_hw_getnf(hw, curchan); // TODO
	
	OHAI;

	// load new noise floor info if we're changing the channel
	if (bChannelChange &&
	    (hw->chip_fullsleep != true) &&
	    (hw->curchan != NULL) &&
	    (chan->channel != hw->curchan->channel) &&
	    ((chan->channelFlags & CHANNEL_ALL) ==
	     (hw->curchan->channelFlags & CHANNEL_ALL)) &&
	    !AR_SREV_9280(hw)) {

		if (jaldi_hw_channel_change(hw, chan)) {
		// loadnf seems to only be implemented for ar9003
		//	jaldi_hw_loadnf(hw, hw->curchan); 
		//	jaldi_hw_start_nfcal(hw); // TODO
			return 0;
		}
	}

	OHAI;
	
	saveDefAntenna = REG_READ(hw, AR_DEF_ANTENNA);
	if (saveDefAntenna == 0)
		saveDefAntenna = 1;

	macStaId1 = REG_READ(hw, AR_STA_ID1);
	// & AR_STA_ID1_BASE_RATE_11B;
	// ath9k always sets this bit, we skip it.

	/* For chips on which RTC reset is done, save TSF before it gets cleared */
	if (AR_SREV_9280(hw) && hw->eep_ops->get_eeprom(hw, EEP_OL_PWRCTRL))
		tsf = jaldi_hw_gettsf64(hw);

	
	/* Only required on the first reset (TODO: Only run on first reset...) */
	if (AR_SREV_9271(hw)) {
		REG_WRITE(hw,
			  AR9271_RESET_POWER_DOWN_CONTROL,
			  AR9271_RADIO_RF_RST);
		udelay(50);
	}

	if (!jaldi_hw_chip_reset(hw, chan)) {
		jaldi_print(JALDI_FATAL, "Chip reset failed\n");
		return -EINVAL;
	}

	OHAI;

	/* Only required on the first reset */
	if (AR_SREV_9271(hw)) {
		REG_WRITE(hw,
			  AR9271_RESET_POWER_DOWN_CONTROL,
			  AR9271_GATE_MAC_CTL);
		udelay(50);
	}

	if (AR_SREV_9280_10_OR_LATER(hw))
		REG_SET_BIT(hw, AR_GPIO_INPUT_EN_VAL, AR_GPIO_JTAG_DISABLE);

	if (!AR_SREV_9300_20_OR_LATER(hw))
		jaldi_hw_enable_async_fifo(hw);

	jaldi_get_hw_ops(hw)->spur_mitigate_freq(hw, chan);
	hw->eep_ops->set_board_values(hw, chan);

	jaldi_hw_set_operating_mode(hw, hw->opmode); // initially set in main

	ENABLE_REGWRITE_BUFFER(hw);
	
	REG_WRITE(hw, AR_STA_ID0, get_unaligned_le32(hw->sc->macaddr));
	REG_WRITE(hw, AR_STA_ID1, get_unaligned_le16(hw->sc->macaddr + 4)
		  | macStaId1
		  | AR_STA_ID1_RTS_USE_DEF);
	//	  | hw->sta_id1_defaults); // we're not using this right now... may do so later.

	REG_WRITE(hw, AR_DEF_ANTENNA, saveDefAntenna);
	REG_WRITE(hw, AR_ISR, ~0);
	REG_WRITE(hw, AR_RSSI_THR, INIT_RSSI_THR);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);

	r = jaldi_get_hw_ops(hw)->rf_set_freq(hw, chan);
	if (r)
		return r;

	ENABLE_REGWRITE_BUFFER(hw);

	for (i = 0; i < AR_NUM_DCU; i++)
		REG_WRITE(hw, AR_DQCUMASK(i), 1 << i);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);

	hw->intr_txqs = 0;
	for (i = 0; i < hw->caps.total_queues; i++)
		jaldi_hw_resettxqueue(hw, i);

	//ath9k_hw_init_interrupt_masks(ah, ah->opmode); // TODO
	//ath9k_hw_init_qos(ah); //TODO

	/* Not implemented for now TODO */
//	if (hw->caps.hw_caps & ATH9K_HW_CAP_RFSILENT)
//		jaldi_enable_rfkill(ah);

	jaldi_hw_init_global_settings(hw);

	if (!AR_SREV_9300_20_OR_LATER(hw)) {
		jaldi_hw_update_async_fifo(hw); 
	}

	REG_WRITE(hw, AR_STA_ID1,
		  REG_READ(hw, AR_STA_ID1) | AR_STA_ID1_PRESERVE_SEQNUM);

	jaldi_hw_set_dma(hw);

	REG_WRITE(hw, AR_OBS, 8);

	if (hw->rx_intr_mitigation) {
		REG_RMW_FIELD(hw, AR_RIMT, AR_RIMT_LAST, 500);
		REG_RMW_FIELD(hw, AR_RIMT, AR_RIMT_FIRST, 2000);
	}

	if (hw->tx_intr_mitigation) {
		REG_RMW_FIELD(hw, AR_TIMT, AR_TIMT_LAST, 300);
		REG_RMW_FIELD(hw, AR_TIMT, AR_TIMT_FIRST, 750);
	}

	jaldi_hw_init_baseband(hw, chan);

	/* TODO: calibration code */
//	if (!ath9k_hw_init_cal(ah, chan))
//		return -EIO;

	ENABLE_REGWRITE_BUFFER(hw);

	jaldi_hw_restore_chainmask(hw);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);

	/*
	 * For big endian systems turn on swapping for descriptors
	 */
	if (AR_SREV_9100(hw)) {
		u32 mask;
		mask = REG_READ(hw, AR_CFG);
		if (mask & (AR_CFG_SWRB | AR_CFG_SWTB | AR_CFG_SWRG)) {
			jaldi_print(JALDI_DEBUG,
				"CFG Byte Swap Set 0x%x\n", mask);
		} else {
			mask =
				INIT_CONFIG_STATUS | AR_CFG_SWRB | AR_CFG_SWTB;
			REG_WRITE(hw, AR_CFG, mask);
			jaldi_print(JALDI_DEBUG,
				"Setting CFG 0x%x\n", REG_READ(hw, AR_CFG));
		}
	} else {
		if (hw->bus_ops->type == JALDI_USB) {
			/* Configure AR9271 target WLAN */
			if (AR_SREV_9271(hw))
				REG_WRITE(hw, AR_CFG, AR_CFG_SWRB | AR_CFG_SWTB);
			else
				REG_WRITE(hw, AR_CFG, AR_CFG_SWTD | AR_CFG_SWRD);
		}
#ifdef __BIG_ENDIAN
                else
			REG_WRITE(hw, AR_CFG, AR_CFG_SWTD | AR_CFG_SWRD);
#endif
	}

	if (AR_SREV_9300_20_OR_LATER(hw)) {
	//	ath9k_hw_loadnf(hw, curchan); // TODO
	//	ath9k_hw_start_nfcal(hw); // TODO
	}

	return 0;
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
			jaldi_print(JALDI_FATAL,
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
	DBG_START_MSG;
	jaldi_print(JALDI_DEBUG, "power_mode %d\n", hw->power_mode);
	if (hw->power_mode == mode) {
		OHAI;
		return status;
	}

	OHAI;

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

/*******************/
/* HW Capabilities */
/*******************/

int jaldi_hw_fill_cap_info(struct jaldi_hw *hw)
{
	DBG_START_MSG;
	struct jaldi_hw_capabilities *pCap = &hw->caps;
	u16 capField = 0, eeval;

	capField = hw->eep_ops->get_eeprom(hw, EEP_OP_CAP);

	/* Make sure at least one band is marked as supported */
	eeval = hw->eep_ops->get_eeprom(hw, EEP_OP_MODE);
	if ((eeval & (AR5416_OPFLAGS_11G | AR5416_OPFLAGS_11A)) == 0) {
		jaldi_print(JALDI_FATAL,
			  "no band has been marked as supported in EEPROM.\n");
		return -EINVAL;
	} else if ((eeval & AR5416_OPFLAGS_11G) != 0) {
		jaldi_print(JALDI_WARN, "This hardware only supports the 2GHz band.\n");	
	}

	/* Read the wireless modes we support */
	bitmap_zero(pCap->wireless_modes, JALDI_MODE_MAX);
	
	if (eeval & AR5416_OPFLAGS_11A) {
		set_bit(JALDI_MODE_11A, pCap->wireless_modes);
		if (hw->ht_enable) {
			if (!(eeval & AR5416_OPFLAGS_N_5G_HT20))
				set_bit(JALDI_MODE_11NA_HT20,
					pCap->wireless_modes);
			if (!(eeval & AR5416_OPFLAGS_N_5G_HT40)) {
				set_bit(JALDI_MODE_11NA_HT40PLUS,
					pCap->wireless_modes);
				set_bit(JALDI_MODE_11NA_HT40MINUS,
					pCap->wireless_modes);
			}
		}
	}

	if (eeval & AR5416_OPFLAGS_11G) {
		set_bit(JALDI_MODE_11G, pCap->wireless_modes);
		if (hw->ht_enable) {
			if (!(eeval & AR5416_OPFLAGS_N_2G_HT20))
				set_bit(JALDI_MODE_11NG_HT20,
					pCap->wireless_modes);
			if (!(eeval & AR5416_OPFLAGS_N_2G_HT40)) {
				set_bit(JALDI_MODE_11NG_HT40PLUS,
					pCap->wireless_modes);
				set_bit(JALDI_MODE_11NG_HT40MINUS,
					pCap->wireless_modes);
			}
		}
	}

	/* ath9k does some special handling for setting up a temp rx chainmask for
	 * ar9271 devices; pretty sure we don't need to do this for our purposes. */
	pCap->tx_chainmask = hw->eep_ops->get_eeprom(hw, EEP_TX_MASK);
	pCap->rx_chainmask = hw->eep_ops->get_eeprom(hw, EEP_RX_MASK);

	pCap->low_2ghz_chan = 2312;
	pCap->high_2ghz_chan = 2732;

	pCap->low_5ghz_chan = 4920;
	pCap->high_5ghz_chan = 6100;
	
	/* ath9k sets up crypto capabilities here (no eeprom read... just defaults 
	 * apparently), but we're omitting those here. */

	if (hw->ht_enable) /* set during hw_config_init */
		pCap->hw_caps |= JALDI_HW_CAP_HT;
	else
		pCap->hw_caps &= ~JALDI_HW_CAP_HT;

	pCap->hw_caps |= JALDI_HW_CAP_GTT;
	pCap->hw_caps |= JALDI_HW_CAP_VEOL;
	pCap->hw_caps |= JALDI_HW_CAP_BSSIDMASK;
	pCap->hw_caps &= ~JALDI_HW_CAP_MCAST_KEYSEARCH;

	if (capField & AR_EEPROM_EEPCAP_MAXQCU)
		pCap->total_queues =
			MS(capField, AR_EEPROM_EEPCAP_MAXQCU);
	else
		pCap->total_queues = JALDI_NUM_TX_QUEUES;

	pCap->hw_caps |= JALDI_HW_CAP_FASTCC;

	if (AR_SREV_9285(hw) || AR_SREV_9271(hw))
		pCap->tx_triglevel_max = MAX_TX_FIFO_THRESHOLD >> 1;
	else
		pCap->tx_triglevel_max = MAX_TX_FIFO_THRESHOLD;

	if (AR_SREV_9271(hw))
		pCap->num_gpio_pins = AR9271_NUM_GPIO;
	else if (AR_SREV_9285_10_OR_LATER(hw))
		pCap->num_gpio_pins = AR9285_NUM_GPIO;
	else if (AR_SREV_9280_10_OR_LATER(hw))
		pCap->num_gpio_pins = AR928X_NUM_GPIO;
	else
		pCap->num_gpio_pins = AR_NUM_GPIO;

	if (AR_SREV_9160_10_OR_LATER(hw) || AR_SREV_9100(hw)) {
		pCap->hw_caps |= JALDI_HW_CAP_CST;
		pCap->rts_aggr_limit = JALDI_AMPDU_LIMIT_MAX;
	} else {
		pCap->rts_aggr_limit = (8 * 1024);
	}

	pCap->hw_caps |= JALDI_HW_CAP_ENHANCEDPM;
/* This is throwing a compile error for some reason... we don't really care about rfkill for our purposes though.
#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)) && defined(CONFIG_RFKILL) || defined(CONFIG_RFKILL_MODULE)) || ((LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)) && defined(CONFIG_RFKILL_BACKPORT) || defined(CONFIG_RFKILL_BACKPORT_MODULE))
	hw->rfsilent = hw->eep_ops->get_eeprom(hw, EEP_RF_SILENT);
	if (hw->rfsilent & EEP_RFSILENT_ENABLED) {
		hw->rfkill_gpio =
			MS(hw->rfsilent, EEP_RFSILENT_GPIO_SEL);
		hw->rfkill_polarity =
			MS(hw->rfsilent, EEP_RFSILENT_POLARITY);

		pCap->hw_caps |= JALDI_HW_CAP_RFSILENT;
	}
#endif
	*/
	if (AR_SREV_9271(hw) || AR_SREV_9300_20_OR_LATER(hw))
		pCap->hw_caps |= JALDI_HW_CAP_AUTOSLEEP;
	else
		pCap->hw_caps &= ~JALDI_HW_CAP_AUTOSLEEP;

	if (AR_SREV_9280(hw) || AR_SREV_9285(hw))
		pCap->hw_caps &= ~JALDI_HW_CAP_4KB_SPLITTRANS;
	else
		pCap->hw_caps |= JALDI_HW_CAP_4KB_SPLITTRANS;
	
	/* Removed regulatory (reg_cap) init here */

	pCap->num_antcfg_5ghz =
		hw->eep_ops->get_num_ant_config(hw, JALDI_HAL_FREQ_BAND_5GHZ);
	pCap->num_antcfg_2ghz =
		hw->eep_ops->get_num_ant_config(hw, JALDI_HAL_FREQ_BAND_2GHZ);

	if (AR_SREV_9300_20_OR_LATER(hw)) {
		pCap->hw_caps |= JALDI_HW_CAP_EDMA | JALDI_HW_CAP_LDPC |
				 JALDI_HW_CAP_FASTCLOCK;
		pCap->rx_hp_qdepth = JALDI_HW_RX_HP_QDEPTH;
		pCap->rx_lp_qdepth = JALDI_HW_RX_LP_QDEPTH;
		/* We're ignoring ar9003 for now */
//		pCap->rx_status_len = sizeof(struct ar9003_rxs);
//		pCap->tx_desc_len = sizeof(struct ar9003_txc);
//		pCap->txs_len = sizeof(struct ar9003_txs);
	} else {
		pCap->tx_desc_len = sizeof(struct jaldi_desc);
		if (AR_SREV_9280_20(hw) &&
		    ((hw->eep_ops->get_eeprom(hw, EEP_MINOR_REV) <=
		      AR5416_EEP_MINOR_VER_16) ||
		     hw->eep_ops->get_eeprom(hw, EEP_FSTCLK_5G)))
			pCap->hw_caps |= JALDI_HW_CAP_FASTCLOCK;
	}

	if (AR_SREV_9300_20_OR_LATER(hw))
		pCap->hw_caps |= JALDI_HW_CAP_RAC_SUPPORTED;

	if (AR_SREV_9287_10_OR_LATER(hw) || AR_SREV_9271(hw))
		pCap->hw_caps |= JALDI_HW_CAP_SGI_20;

	return 0;
}

/* TODO: unfinished */
static int jaldi_hw_post_init(struct jaldi_hw *hw)
{
	int err = 0;
	if (!jaldi_hw_chip_test(hw)) { return -ENODEV; }

	err = jaldi_hw_eeprom_init(hw);	
	if (err != 0) {
		jaldi_print(JALDI_DEBUG, "eeprom_init failed.\n");
		return err;
	}

	jaldi_print(JALDI_INFO, "eeprom version %d, rev %d\n", 
			hw->eep_ops->get_eeprom_ver(hw),
			hw->eep_ops->get_eeprom_rev(hw));

	


	return 0;
}

static void jaldi_hw_attach_ops(struct jaldi_hw *hw)
{
	DBG_START_MSG;
	jaldi_hw_attach_phy_ops(hw);
	DBG_END_MSG;
}

static int __jaldi_hw_init(struct jaldi_hw *hw)
{
	int r = 0;

	DBG_START_MSG;	
	if (!(hw->hw_version.devid == AR9280_DEVID_PCI 
		|| hw->hw_version.devid == AR9280_DEVID_PCIE)) {
		jaldi_print(JALDI_FATAL,"This device (devid: %d) is not supported by JaldiMAC (%d or %d).\n",
				hw->hw_version.devid, AR9280_DEVID_PCI, AR9280_DEVID_PCIE);
		// goal here is to only allow ar9280 (ubnt ns5m and similar) to work

		return -EOPNOTSUPP;
	}

	if(!jaldi_hw_set_reset_reg(hw, JALDI_RESET_POWER_ON)) {
		jaldi_print(JALDI_FATAL, "Couldn't reset the chip (%s, line %d)\n", __FILE__, __LINE__);
		return -EIO;
	}

	jaldi_print(JALDI_DEBUG, "chip reset\n");
	jaldi_hw_init_defaults(hw);
	jaldi_hw_init_config(hw);
	jaldi_hw_attach_ops(hw);

	if (!jaldi_hw_setpower(hw, JALDI_PM_AWAKE)) {
		jaldi_print(JALDI_FATAL, "Couldn't wakeup chip\n");
		return -EIO;
	}
	
	if (AR_SREV_9271(hw) || AR_SREV_9100(hw))
		hw->is_pciexpress = false;

	if (hw->serialize_regmode == SER_REG_MODE_AUTO) {
		if (hw->hw_version.macVersion == AR_SREV_VERSION_5416_PCI ||
		    (AR_SREV_9280(hw) && !hw->is_pciexpress)) {
			hw->serialize_regmode =
				SER_REG_MODE_ON;
		} else {
			hw->serialize_regmode =
				SER_REG_MODE_OFF;
		}
	}

	jaldi_print(JALDI_INFO, "serialize_regmode is %d\n",
		hw->serialize_regmode);

	if (AR_SREV_9285(hw) || AR_SREV_9271(hw))
		hw->max_txtrig_level = MAX_TX_FIFO_THRESHOLD >> 1; /* 2KB bug, see hw.h */
	else
		hw->max_txtrig_level = MAX_TX_FIFO_THRESHOLD;

	if (!jaldi_hw_macversion_supported(hw)) {
		jaldi_print(JALDI_FATAL,
			  "Mac Chip Rev 0x%02x.%x is not supported by "
			  "this driver\n", hw->hw_version.macVersion,
			  hw->hw_version.macRev);
		return -EOPNOTSUPP;
	}

	hw->hw_version.phyRev = REG_READ(hw, AR_PHY_CHIP_ID);

	r = jaldi_hw_post_init(hw);
	if (r) 
		return r;

	r = jaldi_hw_fill_cap_info(hw);
	if (r)
		return r;

	hw->dev_state = JALDI_HW_INITIALIZED;

	return 0;
}

/* hw init container. checks device is supported, then runs private init */
int jaldi_hw_init(struct jaldi_hw *hw)
{
	int ret;

	DBG_START_MSG;	
	/* These are all the AR5008/AR9001/AR9002 hardware family of chipsets */
	switch (hw->hw_version.devid) {
	case AR5416_DEVID_PCI:
	case AR5416_DEVID_PCIE:
	case AR5416_AR9100_DEVID:
	case AR9160_DEVID_PCI:
	case AR9280_DEVID_PCI:
	case AR9280_DEVID_PCIE:
	case AR9285_DEVID_PCIE:
	case AR9287_DEVID_PCI:
	case AR9287_DEVID_PCIE:
	case AR2427_DEVID_PCIE:
	case AR9300_DEVID_PCIE:
		break;
	default:
		if (hw->bus_ops->type == JALDI_USB)
			break;
		jaldi_print(JALDI_FATAL,
			  "Hardware device ID 0x%04x not supported\n",
			  hw->hw_version.devid);
		return -EOPNOTSUPP;
	}

	ret = __jaldi_hw_init(hw);
	if (ret) {
		jaldi_print(JALDI_FATAL,
			  "Unable to initialize hardware; "
			  "initialization status: %d\n", ret);
		return ret;
	}

	return 0;
}

/***************/
/* MAC (RX/TX) */
/***************/

u32 jaldi_hw_gettxbuf(struct jaldi_hw *hw, u32 q)
{
	return REG_READ(hw, AR_QTXDP(q));
}

/* puts a tx_buf (pointer to dma) in specified hw queue */
void jaldi_hw_puttxbuf(struct jaldi_hw *hw, u32 q, u32 txdp)
{
	REG_WRITE(hw, AR_QTXDP(q), txdp);
}

void jaldi_hw_txstart(struct jaldi_hw *hw, u32 q)
{
	jaldi_print(JALDI_DEBUG,
		  "Enable TXE on queue: %u\n", q);
	REG_WRITE(hw, AR_Q_TXE, 1 << q);
}

u32 jaldi_hw_numtxpending(struct jaldi_hw *hw, u32 q)
{
	u32 npend;

	npend = REG_READ(hw, AR_QSTS(q)) & AR_Q_STS_PEND_FR_CNT;
	if (npend == 0) {

		if (REG_READ(hw, AR_Q_TXE) & (1 << q))
			npend = 1;
	}

	return npend;
}

/**
 * jaldi_hw_updatetxtriglevel - adjusts the frame trigger level
 *
 * @hw: hardware struct
 * @bIncTrigLevel: whether or not the frame trigger level should be updated
 * 
 * From ath9k:
 * "The frame trigger level specifies the minimum number of bytes,
 * in units of 64 bytes, that must be DMA'ed into the PCU TX FIFO
 * before the PCU will initiate sending the frame on the air. This can
 * mean we initiate transmit before a full frame is on the PCU TX FIFO.
 * Resets to 0x1 (meaning 64 bytes or a full frame, whichever occurs
 * first)
 *
 * Caution must be taken to ensure to set the frame trigger level based
 * on the DMA request size. For example if the DMA request size is set to
 * 128 bytes the trigger level cannot exceed 6 * 64 = 384. This is because
 * there need to be enough space in the tx FIFO for the requested transfer
 * size. Hence the tx FIFO will stop with 512 - 128 = 384 bytes. If we set
 * the threshold to a value beyond 6, then the transmit will hang.
 *
 * Current dual stream devices have a PCU TX FIFO size of 8 KB.
 * Current single stream devices have a PCU TX FIFO size of 4 KB, however,
 * there is a hardware issue which forces us to use 2 KB instead so the
 * frame trigger level must not exceed 2 KB for these chipsets."
 */
bool jaldi_hw_updatetxtriglevel(struct jaldi_hw *hw, bool bIncTrigLevel)
{
	u32 txcfg, curLevel, newLevel;
	enum jaldi_intr_type omask;

	if (hw->tx_trig_level >= hw->max_txtrig_level)
		return false;

	//omask = jaldi_hw_set_interrupts(hw, hw->imask & ~JALDI_INT_GLOBAL); // TODO

	txcfg = REG_READ(hw, AR_TXCFG);
	curLevel = MS(txcfg, AR_FTRIG);
	newLevel = curLevel;
	if (bIncTrigLevel) {
		if (curLevel < hw->max_txtrig_level)
			newLevel++;
	} else if (curLevel > MIN_TX_FIFO_THRESHOLD)
		newLevel--;
	if (newLevel != curLevel)
		REG_WRITE(hw, AR_TXCFG,
			  (txcfg & ~AR_FTRIG) | SM(newLevel, AR_FTRIG));

//	jaldi_hw_set_interrupts(hw, omask); // TODO

	hw->tx_trig_level = newLevel;

	return newLevel != curLevel;
}

bool jaldi_hw_set_txq_props(struct jaldi_hw *hw, int q,
			    const struct jaldi_tx_queue_info *qinfo)
{
	u32 cw;
	struct jaldi_hw_capabilities *pCap = &hw->caps;
	struct jaldi_tx_queue_info *qi;

	if (q >= pCap->total_queues) {
		jaldi_print(JALDI_DEBUG, "Set TXQ properties, "
			  "invalid queue: %u\n", q);
		return false;
	}

	qi = &hw->txq[q];
	if (qi->tqi_type == JALDI_TX_QUEUE_INACTIVE) {
		jaldi_print(JALDI_DEBUG, "Set TXQ properties, "
			  "inactive queue: %u\n", q);
		return false;
	}

	jaldi_print(JALDI_DEBUG, "Set queue properties for: %u\n", q);

	qi->tqi_ver = qinfo->tqi_ver;
	qi->tqi_subtype = qinfo->tqi_subtype;
	qi->tqi_qflags = qinfo->tqi_qflags;
	qi->tqi_priority = qinfo->tqi_priority;
	if (qinfo->tqi_aifs != JALDI_TXQ_USEDEFAULT)
		qi->tqi_aifs = min(qinfo->tqi_aifs, 255U);
	else
		qi->tqi_aifs = INIT_AIFS;
	if (qinfo->tqi_cwmin != JALDI_TXQ_USEDEFAULT) {
		cw = min(qinfo->tqi_cwmin, 1024U);
		qi->tqi_cwmin = 1;
		while (qi->tqi_cwmin < cw)
			qi->tqi_cwmin = (qi->tqi_cwmin << 1) | 1;
	} else
		qi->tqi_cwmin = qinfo->tqi_cwmin;
	if (qinfo->tqi_cwmax != JALDI_TXQ_USEDEFAULT) {
		cw = min(qinfo->tqi_cwmax, 1024U);
		qi->tqi_cwmax = 1;
		while (qi->tqi_cwmax < cw)
			qi->tqi_cwmax = (qi->tqi_cwmax << 1) | 1;
	} else
		qi->tqi_cwmax = INIT_CWMAX;

	if (qinfo->tqi_shretry != 0)
		qi->tqi_shretry = min((u32) qinfo->tqi_shretry, 15U);
	else
		qi->tqi_shretry = INIT_SH_RETRY;
	if (qinfo->tqi_lgretry != 0)
		qi->tqi_lgretry = min((u32) qinfo->tqi_lgretry, 15U);
	else
		qi->tqi_lgretry = INIT_LG_RETRY;
	qi->tqi_cbrPeriod = qinfo->tqi_cbrPeriod;
	qi->tqi_cbrOverflowLimit = qinfo->tqi_cbrOverflowLimit;
	qi->tqi_burstTime = qinfo->tqi_burstTime;
	qi->tqi_readyTime = qinfo->tqi_readyTime;

	return true;
}

/* Note to self: was going to integrate tx queue setup, release, and reset next.
 * Next need to look into the various descriptor structs and queue info structs
 * to see what I need to port over from those. Work harder, you are behind. */
int jaldi_hw_setuptxqueue(struct jaldi_hw *hw, enum jaldi_tx_queue type,
			  const struct jaldi_tx_queue_info *qinfo)
{
	struct jaldi_tx_queue_info *qi;
	struct jaldi_hw_capabilities *pCap = &hw->caps;
	int q;

	switch (type) {
	case JALDI_TX_QUEUE_DATA:
		for (q = 0; q < pCap->total_queues; q++)
			if (hw->txq[q].tqi_type ==
			    JALDI_TX_QUEUE_INACTIVE)
				break;
		if (q == pCap->total_queues) {
			jaldi_print(JALDI_FATAL,
				  "No available TX queue\n");
			return -1;
		}
		break;
	default:
		jaldi_print(JALDI_FATAL,
			  "Invalid TX queue type: %u\n", type);
		return -1;
	}

	jaldi_print(JALDI_INFO, "Setup TX queue: %u\n", q);

	qi = &hw->txq[q];
	if (qi->tqi_type != JALDI_TX_QUEUE_INACTIVE) {
		jaldi_print(JALDI_FATAL,
			  "TX queue: %u already active\n", q);
		return -1;
	}
	memset(qi, 0, sizeof(struct jaldi_tx_queue_info));
	qi->tqi_type = type;
	if (qinfo == NULL) {
		qi->tqi_qflags =
			TXQ_FLAG_TXOKINT_ENABLE
			| TXQ_FLAG_TXERRINT_ENABLE
			| TXQ_FLAG_TXDESCINT_ENABLE | TXQ_FLAG_TXURNINT_ENABLE;
		qi->tqi_aifs = INIT_AIFS;
		qi->tqi_cwmin = JALDI_TXQ_USEDEFAULT;
		qi->tqi_cwmax = INIT_CWMAX;
		qi->tqi_shretry = INIT_SH_RETRY;
		qi->tqi_lgretry = INIT_LG_RETRY;
		qi->tqi_physCompBuf = 0;
	} else {
		qi->tqi_physCompBuf = qinfo->tqi_physCompBuf;
		(void) jaldi_hw_set_txq_props(hw, q, qinfo);
	}

	return q;
}

bool jaldi_hw_releasetxqueue(struct jaldi_hw *hw, u32 q)
{
	struct jaldi_hw_capabilities *pCap = &hw->caps;
	struct jaldi_tx_queue_info *qi;

	if (q >= pCap->total_queues) {
		jaldi_print(JALDI_ALERT, "Release TXQ, "
			  "invalid queue: %u\n", q);
		return false;
	}
	qi = &hw->txq[q];
	if (qi->tqi_type == JALDI_TX_QUEUE_INACTIVE) {
		jaldi_print(JALDI_ALERT, "Release TXQ, "
			  "inactive queue: %u\n", q);
		return false;
	}

	jaldi_print(JALDI_DEBUG, "Release TX queue: %u\n", q);

	qi->tqi_type = JALDI_TX_QUEUE_INACTIVE;
	/* not sure we need this TODO
	hw->txok_interrupt_mask &= ~(1 << q);
	hw->txerr_interrupt_mask &= ~(1 << q);
	hw->txdesc_interrupt_mask &= ~(1 << q);
	hw->txeol_interrupt_mask &= ~(1 << q);
	hw->txurn_interrupt_mask &= ~(1 << q); */
	//ath9k_hw_set_txq_interrupts(ah, qi); // TODO

	return true;
}

/* Because jaldi doesn't deal with beacons or cabq a lot of this queue 
 * reset code isn't necessary. First pass through eliminated the most 
 * obvious stuff but I'm sure this could be simplified.
 */
bool jaldi_hw_resettxqueue(struct jaldi_hw *hw, u32 q)
{
	struct jaldi_hw_capabilities *pCap = &hw->caps;
	struct jaldi_channel *chan = hw->curchan;
	struct jaldi_tx_queue_info *qi;
	u32 cwMin, chanCwMin, value;

	if (q >= pCap->total_queues) {
		jaldi_print(JALDI_DEBUG, "Reset TXQ, "
			  "invalid queue: %u\n", q);
		return false;
	}

	qi = &hw->txq[q];
	if (qi->tqi_type == JALDI_TX_QUEUE_INACTIVE) {
		jaldi_print(JALDI_DEBUG, "Reset TXQ, "
			  "inactive queue: %u\n", q);
		return true;
	}

	jaldi_print(JALDI_DEBUG, "Reset TX queue: %u\n", q);

	if (qi->tqi_cwmin == JALDI_TXQ_USEDEFAULT) {
		if (chan && IS_CHAN_B(chan))
			chanCwMin = INIT_CWMIN_11B;
		else
			chanCwMin = INIT_CWMIN;

		for (cwMin = 1; cwMin < chanCwMin; cwMin = (cwMin << 1) | 1);
	} else
		cwMin = qi->tqi_cwmin;

	ENABLE_REGWRITE_BUFFER(hw);

	REG_WRITE(hw, AR_DLCL_IFS(q),
		  SM(cwMin, AR_D_LCL_IFS_CWMIN) |
		  SM(qi->tqi_cwmax, AR_D_LCL_IFS_CWMAX) |
		  SM(qi->tqi_aifs, AR_D_LCL_IFS_AIFS));

	REG_WRITE(hw, AR_DRETRY_LIMIT(q),
		  SM(INIT_SSH_RETRY, AR_D_RETRY_LIMIT_STA_SH) |
		  SM(INIT_SLG_RETRY, AR_D_RETRY_LIMIT_STA_LG) |
		  SM(qi->tqi_shretry, AR_D_RETRY_LIMIT_FR_SH));

	REG_WRITE(hw, AR_QMISC(q), AR_Q_MISC_DCU_EARLY_TERM_REQ);
	REG_WRITE(hw, AR_DMISC(q),
		  AR_D_MISC_CW_BKOFF_EN | AR_D_MISC_FRAG_WAIT_EN | 0x2);

	REGWRITE_BUFFER_FLUSH(hw);

	if (qi->tqi_cbrPeriod) {
		REG_WRITE(hw, AR_QCBRCFG(q),
			  SM(qi->tqi_cbrPeriod, AR_Q_CBRCFG_INTERVAL) |
			  SM(qi->tqi_cbrOverflowLimit, AR_Q_CBRCFG_OVF_THRESH));
		REG_WRITE(hw, AR_QMISC(q),
			  REG_READ(hw, AR_QMISC(q)) | AR_Q_MISC_FSP_CBR |
			  (qi->tqi_cbrOverflowLimit ?
			   AR_Q_MISC_CBR_EXP_CNTR_LIMIT_EN : 0));
	}

	REGWRITE_BUFFER_FLUSH(hw);

	REG_WRITE(hw, AR_DCHNTIME(q),
		  SM(qi->tqi_burstTime, AR_D_CHNTIME_DUR) |
		  (qi->tqi_burstTime ? AR_D_CHNTIME_EN : 0));

	if (qi->tqi_burstTime
	    && (qi->tqi_qflags & TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE)) {
		REG_WRITE(hw, AR_QMISC(q),
			  REG_READ(hw, AR_QMISC(q)) |
			  AR_Q_MISC_RDYTIME_EXP_POLICY);

	}

	if (qi->tqi_qflags & TXQ_FLAG_BACKOFF_DISABLE) {
		REG_WRITE(hw, AR_DMISC(q),
			  REG_READ(hw, AR_DMISC(q)) |
			  AR_D_MISC_POST_FR_BKOFF_DIS);
	}

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);

	if (qi->tqi_qflags & TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE) {
		REG_WRITE(hw, AR_DMISC(q),
			  REG_READ(hw, AR_DMISC(q)) |
			  AR_D_MISC_FRAG_BKOFF_EN);
	}

	if (qi->tqi_intFlags & JALDI_TXQ_USE_LOCKOUT_BKOFF_DIS) {
		REG_WRITE(hw, AR_DMISC(q),
			  REG_READ(hw, AR_DMISC(q)) |
			  SM(AR_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL,
			     AR_D_MISC_ARB_LOCKOUT_CNTRL) |
			  AR_D_MISC_POST_FR_BKOFF_DIS);
	}

	if (AR_SREV_9300_20_OR_LATER(hw))
		REG_WRITE(hw, AR_Q_DESC_CRCCHK, AR_Q_DESC_CRCCHK_EN);

	/* Do we use this? I think not.
	if (qi->tqi_qflags & TXQ_FLAG_TXOKINT_ENABLE)
		hw->txok_interrupt_mask |= 1 << q;
	else
		hw->txok_interrupt_mask &= ~(1 << q);
	if (qi->tqi_qflags & TXQ_FLAG_TXERRINT_ENABLE)
		hw->txerr_interrupt_mask |= 1 << q;
	else
		hw->txerr_interrupt_mask &= ~(1 << q);
	if (qi->tqi_qflags & TXQ_FLAG_TXDESCINT_ENABLE)
		hw->txdesc_interrupt_mask |= 1 << q;
	else
		hw->txdesc_interrupt_mask &= ~(1 << q);
	if (qi->tqi_qflags & TXQ_FLAG_TXEOLINT_ENABLE)
		hw->txeol_interrupt_mask |= 1 << q;
	else
		hw->txeol_interrupt_mask &= ~(1 << q);
	if (qi->tqi_qflags & TXQ_FLAG_TXURNINT_ENABLE)
		hw->txurn_interrupt_mask |= 1 << q;
	else
		hw->txurn_interrupt_mask &= ~(1 << q);
	ath9k_hw_set_txq_interrupts(hw, qi); // TODO
	*/
	return true;
}

/*
 * This can stop or re-enables RX.
 *
 * If bool is set this will kill any frame which is currently being
 * transferred between the MAC and baseband and also prevent any new
 * frames from getting started.
 */
bool jaldi_hw_setrxabort(struct jaldi_hw *hw, bool set)
{
	u32 reg;

	DBG_START_MSG;	
	if (set) {
		REG_SET_BIT(hw, AR_DIAG_SW,
			    (AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT));

		if (!jaldi_hw_wait(hw, AR_OBS_BUS_1, AR_OBS_BUS_1_RX_STATE,
				   0, JALDI_WAIT_TIMEOUT)) {
			REG_CLR_BIT(hw, AR_DIAG_SW,
				    (AR_DIAG_RX_DIS |
				     AR_DIAG_RX_ABORT));

			reg = REG_READ(hw, AR_OBS_BUS_1);
			jaldi_print(JALDI_FATAL,
				  "RX failed to go idle in 10 ms RXSM=0x%x\n",
				  reg);

			return false;
		}
	} else {
		REG_CLR_BIT(hw, AR_DIAG_SW,
			    (AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT));
	}

	return true;
}

void jaldi_hw_putrxbuf(struct jaldi_hw *hw, u32 rxdp)
{
	REG_WRITE(hw, AR_RXDP, rxdp);
}

void jaldi_hw_startpcureceive(struct jaldi_hw *hw)
{
	REG_CLR_BIT(hw, AR_DIAG_SW, (AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT));
}

void jaldi_hw_stoppcurecv(struct jaldi_hw *hw)
{
	REG_SET_BIT(hw, AR_DIAG_SW, AR_DIAG_RX_DIS);
}

void jaldi_hw_abortpcurecv(struct jaldi_hw *hw)
{
	REG_SET_BIT(hw, AR_DIAG_SW, AR_DIAG_RX_ABORT | AR_DIAG_RX_DIS);
}

bool jaldi_hw_stopdmarecv(struct jaldi_hw *hw)
{
#define RX_STOP_DMA_TIMEOUT 10000   /* usec */
#define RX_TIME_QUANTUM     100     /* usec */
	int i;

	REG_WRITE(hw, AR_CR, AR_CR_RXD);

	/* Wait for rx enable bit to go low */
	for (i = RX_STOP_DMA_TIMEOUT / JALDI_TIME_QUANTUM; i != 0; i--) {
		if ((REG_READ(hw, AR_CR) & AR_CR_RXE) == 0)
			break;
		udelay(JALDI_TIME_QUANTUM);
	}

	if (i == 0) {
		jaldi_print(JALDI_FATAL,
			  "DMA failed to stop in %d ms "
			  "AR_CR=0x%08x AR_DIAG_SW=0x%08x\n",
			  RX_STOP_DMA_TIMEOUT / 1000,
			  REG_READ(hw, AR_CR),
			  REG_READ(hw, AR_DIAG_SW));
		return false;
	} else {
		return true;
	}

#undef RX_TIME_QUANTUM
#undef RX_STOP_DMA_TIMEOUT
}

/****************/
/* MAC (ar9002) */
/****************/
static void jaldi_hw_fill_txdesc(struct jaldi_hw *hw, struct jaldi_desc *ds, u32 seglen,
				  bool is_firstseg, bool is_lastseg,
				  const struct jaldi_desc *ds0, dma_addr_t buf_addr,
				  unsigned int qcu)
{
	struct ar5416_desc *ads = AR5416DESC(ds);

	ads->ds_data = buf_addr;

	if (is_firstseg) {
		ads->ds_ctl1 |= seglen | (is_lastseg ? 0 : AR_TxMore);
	} else if (is_lastseg) {
		ads->ds_ctl0 = 0;
		ads->ds_ctl1 = seglen;
		ads->ds_ctl2 = AR5416DESC_CONST(ds0)->ds_ctl2;
		ads->ds_ctl3 = AR5416DESC_CONST(ds0)->ds_ctl3;
	} else {
		ads->ds_ctl0 = 0;
		ads->ds_ctl1 = seglen | AR_TxMore;
		ads->ds_ctl2 = 0;
		ads->ds_ctl3 = 0;
	}
	ads->ds_txstatus0 = ads->ds_txstatus1 = 0;
	ads->ds_txstatus2 = ads->ds_txstatus3 = 0;
	ads->ds_txstatus4 = ads->ds_txstatus5 = 0;
	ads->ds_txstatus6 = ads->ds_txstatus7 = 0;
	ads->ds_txstatus8 = ads->ds_txstatus9 = 0;
}

static void jaldi_hw_set11n_txdesc(struct jaldi_hw *hw, void *ds,
				    u32 pktLen, enum jaldi_pkt_type type,
				    u32 txPower, u32 flags)
{
	struct ar5416_desc *ads = AR5416DESC(ds);

	if (txPower > 63)
		txPower = 63;

	ads->ds_ctl0 = (pktLen & AR_FrameLen)
		| (flags & ATH9K_TXDESC_VMF ? AR_VirtMoreFrag : 0)
		| SM(txPower, AR_XmitPower)
		| (flags & ATH9K_TXDESC_VEOL ? AR_VEOL : 0)
		| (flags & ATH9K_TXDESC_CLRDMASK ? AR_ClrDestMask : 0)
		| (flags & ATH9K_TXDESC_INTREQ ? AR_TxIntrReq : 0);

	if (type != JALDI_PKT_TYPE_NORMAL)
		jaldi_print(JALDI_DEBUG, "uh-oh! sending a packet of type %d in set11n_txdesc, should be 0.\n", type); 
		
	ads->ds_ctl1 =
		 SM(type, AR_FrameType) 
		| (flags & ATH9K_TXDESC_NOACK ? AR_NoAck : 0)
		| (flags & ATH9K_TXDESC_EXT_ONLY ? AR_ExtOnly : 0)
		| (flags & ATH9K_TXDESC_EXT_AND_CTL ? AR_ExtAndCtl : 0);

	/* We only support unencrypted traffic for now. */
#define JALDI_KEY_TYPE_CLEAR 0
	ads->ds_ctl6 = SM(JALDI_KEY_TYPE_CLEAR, AR_EncrType);

	if (AR_SREV_9285(hw) || AR_SREV_9271(hw)) {
		ads->ds_ctl8 = 0;
		ads->ds_ctl9 = 0;
		ads->ds_ctl10 = 0;
		ads->ds_ctl11 = 0;
	}
}
