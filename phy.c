/*
 * PHY-related settings.
 */

static int jaldi_hw_set_freq(struct jaldi_hw *hw, struct jaldi_channel *chan)
{
	u16 bMode, fracMode, aModeRefSel = 0;
	u32 freq, ndiv, channelSel = 0, channelFrac = 0, reg32 = 0;
	u32 refDivA = 24;

	struct chan_centers centers;
	jaldi_hw_get_channel_centers(hw, chan, &centers);

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
