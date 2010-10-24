/* deals with hw; bare-bones implementation of ath9k_hw
 *
 * TODO:
 * - Bus operations (init/deinit)
 * - Firmware loading?
 * - PHY (channels, power, interface for manipulating hardware)
 */

#ifndef HW_H
#define HW_H

#include "reg.h"
#include "eeprom.h"

#define ATHEROS_VENDOR_ID	0x168c

#define AR5416_DEVID_PCI	0x0023
#define AR5416_DEVID_PCIE	0x0024
#define AR9160_DEVID_PCI	0x0027
#define AR9280_DEVID_PCI	0x0029
#define AR9280_DEVID_PCIE	0x002a
#define AR9285_DEVID_PCIE	0x002b
#define AR2427_DEVID_PCIE	0x002c
#define AR9287_DEVID_PCI	0x002d
#define AR9287_DEVID_PCIE	0x002e
#define AR9300_DEVID_PCIE	0x0030

#define AR5416_AR9100_DEVID	0x000b

#define	AR_SUBVENDOR_ID_NOG	0x0e11
#define AR_SUBVENDOR_ID_NEW_A	0x7065
#define AR5416_MAGIC		0x19641014

#define JALDI_AMPDU_LIMIT_MAX		(64 * 1024 - 1)
#define	JALDI_DEFAULT_NOISE_FLOOR	-95
#define JALDI_RSSI_BAD			-128
#define JALDI_WAIT_TIMEOUT		100000 /* (us) */

/* Register operation macros */
#define REG_WRITE(_jh, _reg, _val) \
	(_jh)->io_ops->write((_jh), (_val), (_reg))

#define REG_READ(_jh, _reg) \
	(_jh)->io_ops->read((_jh), (_reg))
#define ENABLE_REGWRITE_BUFFER(_ah)					\
	do {								\
		if (AR_SREV_9271(_ah))					\
			ath9k_hw_common(_ah)->ops->enable_write_buffer((_ah)); \ // TODO
	} while (0)

#define DISABLE_REGWRITE_BUFFER(_ah)					\
	do {								\
		if (AR_SREV_9271(_ah))					\
			ath9k_hw_common(_ah)->ops->disable_write_buffer((_ah)); \ // TODO
	} while (0)

#define REGWRITE_BUFFER_FLUSH(_ah)					\
	do {								\
		if (AR_SREV_9271(_ah))					\
			ath9k_hw_common(_ah)->ops->write_flush((_ah));	\ // TODO
	} while (0)

#define SM(_v, _f)  (((_v) << _f##_S) & _f)
#define MS(_v, _f)  (((_v) & _f) >> _f##_S)
// RMW: "Read modify write"
#define REG_RMW(_a, _r, _set, _clr)    \
	REG_WRITE(_a, _r, (REG_READ(_a, _r) & ~(_clr)) | (_set))
#define REG_RMW_FIELD(_a, _r, _f, _v) \
	REG_WRITE(_a, _r, \
	(REG_READ(_a, _r) & ~_f) | (((_v) << _f##_S) & _f))
#define REG_READ_FIELD(_a, _r, _f) \
	(((REG_READ(_a, _r) & _f) >> _f##_S))
#define REG_SET_BIT(_a, _r, _f) \
	REG_WRITE(_a, _r, REG_READ(_a, _r) | _f)
#define REG_CLR_BIT(_a, _r, _f) \
	REG_WRITE(_a, _r, REG_READ(_a, _r) & ~_f)

#define DO_DELAY(x) do {			\
		if ((++(x) % 64) == 0)          \
			udelay(1);		\
	} while (0)

#define REG_WRITE_ARRAY(iniarray, column, regWr) do {                   \
		int r;							\
		for (r = 0; r < ((iniarray)->ia_rows); r++) {		\
			REG_WRITE(ah, INI_RA((iniarray), (r), 0),	\
				  INI_RA((iniarray), r, (column)));	\
			DO_DELAY(regWr);				\
		}							\
	} while (0)

// it's unclear what these do: document TODO
#define SM(_v, _f)  (((_v) << _f##_S) & _f)
#define MS(_v, _f)  (((_v) & _f) >> _f##_S)
/* End register r/w macros */


// closely based upon ath9k_hw_version (a9k/hw.h)
struct jaldi_hw_version {
	u32 magic;
	u32 devid;
	u16 subvendorid;
	u32 macVersion;
	u16 macRev;
	u16 phyRev;
	u16 analog5GhzRev;
	u16 analog2GhzRev; // don't think this is needed for us...
	u16 subsysid;
};

#define HT40_CHANNEL_CENTER_SHIFT	10
// channel mode flags
#define CHANNEL_2GHZ     		0x00080
#define CHANNEL_5GHZ 			0x00100
#define CHANNEL_HT20			0x10000
#define CHANNEL_HT40PLUS		0x20000
#define CHANNEL_HT40MINUS		0x40000

/* Macros for checking chanmode */
#define IS_CHAN_HT20(_c) (((_c)->chanmode == CHANNEL_HT20)) 
#define IS_CHAN_HT40(_c) (((_c)->chanmode == CHANNEL_HT40PLUS) || \
			  ((_c)->chanmode == CHANNEL_HT40MINUS))

struct jaldi_channel {
	u16 channel; // MHz
	u16 center_freq; // MHz
	u16 hw_value; // HW-specific channel value (see hw.h, others). Used to index.
	u16 max_power;
	u32 channelFlags; // TODO: not needed?
	u32 chanmode; // 20MHz vs 40Mhz 
	/* TODO: Noise calibration data per channel may go here, as in a9k */
};

// a convenient way to refer to the freq we're on
struct chan_centers {
	u16 synth_center;
	u16 ctl_center;
	u16 ext_center;
};

// we're not currently doing power management
enum jaldi_power_mode { 
	JALDI_PM_AWAKE = 0,
	JALDI_PM_FULL_SLEEP,
	JALDI_PM_NETWORK_SLEEP,
	JALDI_PM_UNDEFINED,
};

// different types of hw reset we perform.
enum {
	JALDI_RESET_POWER_ON,
	JALDI_RESET_WARM,
	JALDI_RESET_COLD,
};

struct jaldi_bitrate {
	u16 bitrate;
	u16 hw_value;
};

struct jaldi_hw_ops {
	
	bool (*macversion_supported)(u32 macversion);

	/* PHY ops */
	int (*rf_set_freq)(struct jaldi_hw *hw,
			   struct jaldi_channel *chan);
};

struct jaldi_hw {
	struct jaldi_hw_version hw_version;
	struct jaldi_softc *sc;
	
	struct jaldi_channel *curchan;
	struct jaldi_bitrate *cur_rate;
	enum jaldi_power_mode power_mode;

	u32 intr_txqs;
	u8 txchainmask;
	u8 rxchainmask;

	/* functions to control hw */
	struct jaldi_hw_ops ops;
	struct eeprom_ops eep_ops; // This is more or less copied from ath9k
};

#endif
