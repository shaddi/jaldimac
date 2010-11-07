/*
 * PHY-related settings.
 */

#include "jaldi.h"
#include "hw.h"

static int jaldi_hw_set_freq(struct jaldi_hw *hw, struct jaldi_channel *chan)
{
	u16 bMode, fracMode, aModeRefSel = 0;
	u32 freq, ndiv, channelSel = 0, channelFrac = 0, reg32 = 0;
	u32 refDivA = 24;

	struct chan_centers centers;
	jaldi_hw_get_channel_centers(chan, &centers);

	freq = centers.synth_center;

	reg32 = REG_READ(hw, AR_PHY_SYNTH_CONTROL);
	reg32 &= 0xc0000000;

	if (freq < 4800) {
		// we're on an unsupported 2GHz band
		jaldi_print(0, "2Ghz band is not supported.\n");
	} else {
		// we're on the 5Ghz band
		switch (hw->eep_ops->get_eeprom(hw, EEP_FRAC_N_5G)) {
			case 0:
				if ((freq % 20) == 0 ) { aModeRefSel = 3; }
				else if ((freq % 10) == 0) { aModeRefSel = 2; }
				if (aModeRefSel) { break; }
			case 1:
			default:
				aModeRefSel = 0;
				/* Enable "2G (fractional) mode for 5Mhz spaced channels */
				fracMode = 1;
				refDivA = 1;
				channelSel = CHANSEL_5G(freq);

				/* RefDivA setting */
				REG_RMW_FIELD(hw, AR_AN_SYNTH9, AR_AN_SYNTH_REFDIVA, refDivA);
		}

		if(!fracMode) {
			ndiv = (freq * (refDivaA >> aModeRefSel)) / 60;
			channelSel = ndiv & 0x1ff;
			channelFrac = (ndiv & 0xfffffe00) * 2;
			channelSel = (channelSel << 17) | channelFrac;
		}

	}

	reg32 = reg32 | 
		(bMode << 29) | // not used on 5G band
		(fracMode << 28) | 
		(aModeRefSel << 26) | 
		(channelSel);
	
	REG_WRITE(hw, AR_PHY_SYNTH_CONTROL, reg32);

	hw->curchan = chan;

	return 0;
}


/**
 * ar9002_hw_spur_mitigate - convert baseband spur frequency
 * @hw: hardware structure
 * @chan:
 *
 * For single-chip solutions. Converts to baseband spur frequency given the
 * input channel frequency and compute register settings below.
 */
static void jaldi_hw_spur_mitigate(struct jaldi_hw *hw,
				    struct jaldi_channel *chan)
{
	int bb_spur = AR_NO_SPUR;
	int freq;
	int bin, cur_bin;
	int bb_spur_off, spur_subchannel_sd;
	int spur_freq_sd;
	int spur_delta_phase;
	int denominator;
	int upper, lower, cur_vit_mask;
	int tmp, newVal;
	int i;
	int pilot_mask_reg[4] = { AR_PHY_TIMING7, AR_PHY_TIMING8,
			  AR_PHY_PILOT_MASK_01_30, AR_PHY_PILOT_MASK_31_60
	};
	int chan_mask_reg[4] = { AR_PHY_TIMING9, AR_PHY_TIMING10,
			 AR_PHY_CHANNEL_MASK_01_30, AR_PHY_CHANNEL_MASK_31_60
	};
	int inc[4] = { 0, 100, 0, 0 };
	struct chan_centers centers;

	int8_t mask_m[123];
	int8_t mask_p[123];
	int8_t mask_amt;
	int tmp_mask;
	int cur_bb_spur;
	bool is2GHz = IS_CHAN_2GHZ(chan);

	memset(&mask_m, 0, sizeof(int8_t) * 123);
	memset(&mask_p, 0, sizeof(int8_t) * 123);

	jaldi_hw_get_channel_centers(chan, &centers);
	freq = centers.synth_center;

	hw->config.spurmode = SPUR_ENABLE_EEPROM;
	for (i = 0; i < AR_EEPROM_MODAL_SPURS; i++) {
		cur_bb_spur = hw->eep_ops->get_spur_channel(hw, i, is2GHz);

		if (is2GHz)
			cur_bb_spur = (cur_bb_spur / 10) + AR_BASE_FREQ_2GHZ;
		else
			cur_bb_spur = (cur_bb_spur / 10) + AR_BASE_FREQ_5GHZ;

		if (AR_NO_SPUR == cur_bb_spur)
			break;
		cur_bb_spur = cur_bb_spur - freq;

		if (IS_CHAN_HT40(chan)) {
			if ((cur_bb_spur > -AR_SPUR_FEEQ_BOUND_HT40) &&
			    (cur_bb_spur < AR_SPUR_FEEQ_BOUND_HT40)) {
				bb_spur = cur_bb_spur;
				break;
			}
		} else if ((cur_bb_spur > -AR_SPUR_FEEQ_BOUND_HT20) &&
			   (cur_bb_spur < AR_SPUR_FEEQ_BOUND_HT20)) {
			bb_spur = cur_bb_spur;
			break;
		}
	}

	if (AR_NO_SPUR == bb_spur) {
		REG_CLR_BIT(hw, AR_PHY_FORCE_CLKEN_CCK,
			    AR_PHY_FORCE_CLKEN_CCK_MRC_MUX);
		return;
	} else {
		REG_CLR_BIT(hw, AR_PHY_FORCE_CLKEN_CCK,
			    AR_PHY_FORCE_CLKEN_CCK_MRC_MUX);
	}

	bin = bb_spur * 320;

	tmp = REG_READ(hw, AR_PHY_TIMING_CTRL4(0));

	ENABLE_REGWRITE_BUFFER(hw);

	newVal = tmp | (AR_PHY_TIMING_CTRL4_ENABLE_SPUR_RSSI |
			AR_PHY_TIMING_CTRL4_ENABLE_SPUR_FILTER |
			AR_PHY_TIMING_CTRL4_ENABLE_CHAN_MASK |
			AR_PHY_TIMING_CTRL4_ENABLE_PILOT_MASK);
	REG_WRITE(hw, AR_PHY_TIMING_CTRL4(0), newVal);

	newVal = (AR_PHY_SPUR_REG_MASK_RATE_CNTL |
		  AR_PHY_SPUR_REG_ENABLE_MASK_PPM |
		  AR_PHY_SPUR_REG_MASK_RATE_SELECT |
		  AR_PHY_SPUR_REG_ENABLE_VIT_SPUR_RSSI |
		  SM(SPUR_RSSI_THRESH, AR_PHY_SPUR_REG_SPUR_RSSI_THRESH));
	REG_WRITE(hw, AR_PHY_SPUR_REG, newVal);

	if (IS_CHAN_HT40(chan)) {
		if (bb_spur < 0) {
			spur_subchannel_sd = 1;
			bb_spur_off = bb_spur + 10;
		} else {
			spur_subchannel_sd = 0;
			bb_spur_off = bb_spur - 10;
		}
	} else {
		spur_subchannel_sd = 0;
		bb_spur_off = bb_spur;
	}

	if (IS_CHAN_HT40(chan))
		spur_delta_phase =
			((bb_spur * 262144) /
			 10) & AR_PHY_TIMING11_SPUR_DELTA_PHASE;
	else
		spur_delta_phase =
			((bb_spur * 524288) /
			 10) & AR_PHY_TIMING11_SPUR_DELTA_PHASE;

	denominator = IS_CHAN_2GHZ(chan) ? 44 : 40;
	spur_freq_sd = ((bb_spur_off * 2048) / denominator) & 0x3ff;

	newVal = (AR_PHY_TIMING11_USE_SPUR_IN_AGC |
		  SM(spur_freq_sd, AR_PHY_TIMING11_SPUR_FREQ_SD) |
		  SM(spur_delta_phase, AR_PHY_TIMING11_SPUR_DELTA_PHASE));
	REG_WRITE(hw, AR_PHY_TIMING11, newVal);

	newVal = spur_subchannel_sd << AR_PHY_SFCORR_SPUR_SUBCHNL_SD_S;
	REG_WRITE(hw, AR_PHY_SFCORR_EXT, newVal);

	cur_bin = -6000;
	upper = bin + 100;
	lower = bin - 100;

	for (i = 0; i < 4; i++) {
		int pilot_mask = 0;
		int chan_mask = 0;
		int bp = 0;
		for (bp = 0; bp < 30; bp++) {
			if ((cur_bin > lower) && (cur_bin < upper)) {
				pilot_mask = pilot_mask | 0x1 << bp;
				chan_mask = chan_mask | 0x1 << bp;
			}
			cur_bin += 100;
		}
		cur_bin += inc[i];
		REG_WRITE(hw, pilot_mask_reg[i], pilot_mask);
		REG_WRITE(hw, chan_mask_reg[i], chan_mask);
	}

	cur_vit_mask = 6100;
	upper = bin + 120;
	lower = bin - 120;

	for (i = 0; i < 123; i++) {
		if ((cur_vit_mask > lower) && (cur_vit_mask < upper)) {

			/* workaround for gcc bug #37014 */
			volatile int tmp_v = abs(cur_vit_mask - bin);

			if (tmp_v < 75)
				mask_amt = 1;
			else
				mask_amt = 0;
			if (cur_vit_mask < 0)
				mask_m[abs(cur_vit_mask / 100)] = mask_amt;
			else
				mask_p[cur_vit_mask / 100] = mask_amt;
		}
		cur_vit_mask -= 100;
	}

	tmp_mask = (mask_m[46] << 30) | (mask_m[47] << 28)
		| (mask_m[48] << 26) | (mask_m[49] << 24)
		| (mask_m[50] << 22) | (mask_m[51] << 20)
		| (mask_m[52] << 18) | (mask_m[53] << 16)
		| (mask_m[54] << 14) | (mask_m[55] << 12)
		| (mask_m[56] << 10) | (mask_m[57] << 8)
		| (mask_m[58] << 6) | (mask_m[59] << 4)
		| (mask_m[60] << 2) | (mask_m[61] << 0);
	REG_WRITE(hw, AR_PHY_BIN_MASK_1, tmp_mask);
	REG_WRITE(hw, AR_PHY_VIT_MASK2_M_46_61, tmp_mask);

	tmp_mask = (mask_m[31] << 28)
		| (mask_m[32] << 26) | (mask_m[33] << 24)
		| (mask_m[34] << 22) | (mask_m[35] << 20)
		| (mask_m[36] << 18) | (mask_m[37] << 16)
		| (mask_m[48] << 14) | (mask_m[39] << 12)
		| (mask_m[40] << 10) | (mask_m[41] << 8)
		| (mask_m[42] << 6) | (mask_m[43] << 4)
		| (mask_m[44] << 2) | (mask_m[45] << 0);
	REG_WRITE(hw, AR_PHY_BIN_MASK_2, tmp_mask);
	REG_WRITE(hw, AR_PHY_MASK2_M_31_45, tmp_mask);

	tmp_mask = (mask_m[16] << 30) | (mask_m[16] << 28)
		| (mask_m[18] << 26) | (mask_m[18] << 24)
		| (mask_m[20] << 22) | (mask_m[20] << 20)
		| (mask_m[22] << 18) | (mask_m[22] << 16)
		| (mask_m[24] << 14) | (mask_m[24] << 12)
		| (mask_m[25] << 10) | (mask_m[26] << 8)
		| (mask_m[27] << 6) | (mask_m[28] << 4)
		| (mask_m[29] << 2) | (mask_m[30] << 0);
	REG_WRITE(hw, AR_PHY_BIN_MASK_3, tmp_mask);
	REG_WRITE(hw, AR_PHY_MASK2_M_16_30, tmp_mask);

	tmp_mask = (mask_m[0] << 30) | (mask_m[1] << 28)
		| (mask_m[2] << 26) | (mask_m[3] << 24)
		| (mask_m[4] << 22) | (mask_m[5] << 20)
		| (mask_m[6] << 18) | (mask_m[7] << 16)
		| (mask_m[8] << 14) | (mask_m[9] << 12)
		| (mask_m[10] << 10) | (mask_m[11] << 8)
		| (mask_m[12] << 6) | (mask_m[13] << 4)
		| (mask_m[14] << 2) | (mask_m[15] << 0);
	REG_WRITE(hw, AR_PHY_MASK_CTL, tmp_mask);
	REG_WRITE(hw, AR_PHY_MASK2_M_00_15, tmp_mask);

	tmp_mask = (mask_p[15] << 28)
		| (mask_p[14] << 26) | (mask_p[13] << 24)
		| (mask_p[12] << 22) | (mask_p[11] << 20)
		| (mask_p[10] << 18) | (mask_p[9] << 16)
		| (mask_p[8] << 14) | (mask_p[7] << 12)
		| (mask_p[6] << 10) | (mask_p[5] << 8)
		| (mask_p[4] << 6) | (mask_p[3] << 4)
		| (mask_p[2] << 2) | (mask_p[1] << 0);
	REG_WRITE(hw, AR_PHY_BIN_MASK2_1, tmp_mask);
	REG_WRITE(hw, AR_PHY_MASK2_P_15_01, tmp_mask);

	tmp_mask = (mask_p[30] << 28)
		| (mask_p[29] << 26) | (mask_p[28] << 24)
		| (mask_p[27] << 22) | (mask_p[26] << 20)
		| (mask_p[25] << 18) | (mask_p[24] << 16)
		| (mask_p[23] << 14) | (mask_p[22] << 12)
		| (mask_p[21] << 10) | (mask_p[20] << 8)
		| (mask_p[19] << 6) | (mask_p[18] << 4)
		| (mask_p[17] << 2) | (mask_p[16] << 0);
	REG_WRITE(hw, AR_PHY_BIN_MASK2_2, tmp_mask);
	REG_WRITE(hw, AR_PHY_MASK2_P_30_16, tmp_mask);

	tmp_mask = (mask_p[45] << 28)
		| (mask_p[44] << 26) | (mask_p[43] << 24)
		| (mask_p[42] << 22) | (mask_p[41] << 20)
		| (mask_p[40] << 18) | (mask_p[39] << 16)
		| (mask_p[38] << 14) | (mask_p[37] << 12)
		| (mask_p[36] << 10) | (mask_p[35] << 8)
		| (mask_p[34] << 6) | (mask_p[33] << 4)
		| (mask_p[32] << 2) | (mask_p[31] << 0);
	REG_WRITE(hw, AR_PHY_BIN_MASK2_3, tmp_mask);
	REG_WRITE(hw, AR_PHY_MASK2_P_45_31, tmp_mask);

	tmp_mask = (mask_p[61] << 30) | (mask_p[60] << 28)
		| (mask_p[59] << 26) | (mask_p[58] << 24)
		| (mask_p[57] << 22) | (mask_p[56] << 20)
		| (mask_p[55] << 18) | (mask_p[54] << 16)
		| (mask_p[53] << 14) | (mask_p[52] << 12)
		| (mask_p[51] << 10) | (mask_p[50] << 8)
		| (mask_p[49] << 6) | (mask_p[48] << 4)
		| (mask_p[47] << 2) | (mask_p[46] << 0);
	REG_WRITE(hw, AR_PHY_BIN_MASK2_4, tmp_mask);
	REG_WRITE(hw, AR_PHY_MASK2_P_61_45, tmp_mask);

	REGWRITE_BUFFER_FLUSH(hw);
	DISABLE_REGWRITE_BUFFER(hw);
}

static void jaldi_hw_do_getnf(struct jaldi_hw *hw,
			      int16_t nfarray[NUM_NF_READINGS])
{
	int16_t nf;

	nf = MS(REG_READ(hw, AR_PHY_CCA), AR9280_PHY_MINCCA_PWR);

	if (nf & 0x100)
		nf = 0 - ((nf ^ 0x1ff) + 1);
		jaldi_print(JALDI_DEBUG,
		  "NF calibrated [ctl] [chain 0] is %d\n", nf);

	if (AR_SREV_9271(hw) && (nf >= -114))
		nf = -116;

	nfarray[0] = nf;

	if (!AR_SREV_9285(hw) && !AR_SREV_9271(hw)) {
		nf = MS(REG_READ(hw, AR_PHY_CH1_CCA),
				AR9280_PHY_CH1_MINCCA_PWR);

		if (nf & 0x100)
			nf = 0 - ((nf ^ 0x1ff) + 1);
		jaldi_print(JALDI_INFO,
			  "NF calibrated [ctl] [chain 1] is %d\n", nf);
		nfarray[1] = nf;
	}

	nf = MS(REG_READ(hw, AR_PHY_EXT_CCA), AR9280_PHY_EXT_MINCCA_PWR);
	if (nf & 0x100)
		nf = 0 - ((nf ^ 0x1ff) + 1);
	jaldi_print(JALDI_INFO,
		  "NF calibrated [ext] [chain 0] is %d\n", nf);

	if (AR_SREV_9271(hw) && (nf >= -114))
		nf = -116;

	nfarray[3] = nf;

	if (!AR_SREV_9285(hw) && !AR_SREV_9271(hw)) {
		nf = MS(REG_READ(hw, AR_PHY_CH1_EXT_CCA),
				AR9280_PHY_CH1_EXT_MINCCA_PWR);

		if (nf & 0x100)
			nf = 0 - ((nf ^ 0x1ff) + 1);
		jaldi_print(JALDI_INFO,
			  "NF calibrated [ext] [chain 1] is %d\n", nf);
		nfarray[4] = nf;
	}
}

void jaldi_hw_attach_phy_ops(struct jaldi_hw *hw)
{
	struct jaldi_hw_ops *ops = hw->ops;

	ops->rf_set_freq = jaldi_hw_set_freq;
	ops->spur_mitigate_freq = jaldi_hw_spur_mitigate;
	ops->do_getnf = jaldi_hw_do_getnf;
	
}

/*
 * Remaining phy_ops (as seen in ar9002_phy.c)
 * set_rf_regs = NULL;
 * set_rf_alloc_ext_banks = NULL;
 * set_rf_free_ext_banks = NULL;
 * rf_set_freq (implemented above)
 * spur_mitigate_freq = ar9002_hw_spur_mitigate; (not sure what this does)
 * olc_init = ar9002_olc_init
 * 	- For before AR9280, this does not seem to be used (i.e., not supported on chip)
 *	- For AR9287 and later, does something with tempsense 
 *	- For everything else, does something with gain (gain table... what is this?)
 * compute_pll_control = ar9002_hw_compute_pll_control
 	- ...
 * do_getnf = ar9002_hw_do_getnf (reads various CCA registers and sets nf to 'min pwr' on each)
 */
