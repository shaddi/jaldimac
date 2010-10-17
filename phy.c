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

