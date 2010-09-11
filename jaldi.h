#ifndef JALDI_H
#define JALDI_H

#include <linux/etherdevice.h>
#include <linux/device.h>

/* ath9k includes debug and common headers here. Need to determine if this is necessary here TODO */

//irqreturn_t ath_isr(int irq, void *dev);

/* LED control */

#define ATH_LED_PIN_DEF			1
#define ATH_LED_PIN_9287		8
#define	ATH_LED_ON_DURATION_IDLE	350 	/* msec */
#define ATH_LED_OFF_DURATION_IDLE	250	/* msec */

enum ath_led_type {
	ATH_LED_RADIO,
	ATH_LED_ASSOC,
	ATH_LED_TX,
	ATH_LED_RX
};

/* ath_led definition, init/deinit of led's TODO */

struct jaldi_wiphy;
struct jaldi_rate_table;

struct jaldi_softc {
	struct device *dev;

	spinlock_t wiphy_lock; /* spinlock to protect jaldi_wiphy data */
	struct ath_wiphy *pri_wiphy;

	struct tasklet_struct intr_tq;

#endif /* JALDI_H */

