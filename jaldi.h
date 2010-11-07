/* JaldiMAC
 * GPL. 
 *
 * Written by Shaddi Hasan (shaddi@eecs.berkeley.edu)
 * Based strongly off ath9k, architecture inspired by i2400m.
 * 
 * GENERAL DRIVER ARCHITECTURE
 * 
 * THe jaldi driver is split into two major parts:
 *
 * 	- Device specific driver (so far only targets UBNT NSM5, running AR9280)
 *	- Device generic part (this part)
 *
 * Because of the ath9k heritage, this demarcation may be fuzzy at times. The 
 * device specific part of the driver handles tasks involving moving data
 * between the kernal and the device, and bus-related tasks such as probing,
 * hardware resets, and disconnecting. 
 *
 * The device generic part aims to implement a fairly normal layer, if thin, 
 * layer between Linux and the hardware. It appears as an Ethernet device for 
 * the sake of simplicity, but packets passed to this driver should follow the 
 * as-yet-to-be-determined JaldiMAC packet format. 
 */



#ifndef JALDI_H
#define JALDI_H

#include <linux/etherdevice.h>
#include <linux/device.h>
#include <linux/io.h>

#include "hw.h"
#include "debug.h"

#define SC_OP_INVALID                BIT(0)
#define SC_OP_BEACONS                BIT(1)
#define SC_OP_RXAGGR                 BIT(2)
#define SC_OP_TXAGGR                 BIT(3)
#define SC_OP_FULL_RESET             BIT(4)
#define SC_OP_PREAMBLE_SHORT         BIT(5)
#define SC_OP_PROTECT_ENABLE         BIT(6)
#define SC_OP_RXFLUSH                BIT(7)
#define SC_OP_LED_ASSOCIATED         BIT(8)
#define SC_OP_LED_ON                 BIT(9)
#define SC_OP_SCANNING               BIT(10)
#define SC_OP_TSF_RESET              BIT(11)
#define SC_OP_BT_PRIORITY_DETECTED   BIT(12)
#define SC_OP_BT_SCAN                BIT(13)

/***********/
/* RX / TX */
/***********/
#define JALDI_MAX_ANTENNA	3
#define JALDI_NUM_RXBUF		512
#define JALDI_NUM_TXBUF		512


struct jaldi_wiphy;
struct jaldi_rate_table;

enum qos_type {
	JALDI_QOS_BULK,
	JALDI_QOS_LATENCY_SENSITIVE,
	JALDI_QOS_UNDEFINED,
};

enum jaldi_freq_band {
	JALDI_2GHZ = 0,
	JALDI_5GHZ,
};

enum jaldi_packet_type {
	JALDI_CONTROL = 0,
	JALDI_DATA,
};

struct jaldi_packet {
	struct jaldi_packet *next;
	struct net_device *dev;
	struct sk_buff *skb;
	int jaldi_packet_type;
	int datalen;
	char *data;
	s64 tx_time; /* the time at which this packet should be sent */
	int qos_type;
};

/* This does not provide support for aggregates as ath9k does */
struct jaldi_buf {
	struct list_head list;

	void *bf_desc;			/* virtual addr of desc */
	dma_addr_t bf_daddr;		/* physical addr of desc */
	dma_addr_t bf_buf_addr;		/* physical addr of data buffer */
	dma_addr_t bf_dma_context;
	struct jaldi_packet *bf_mpdu; 	/* MAC protocol data unit: raw sk_buff enclosed */
	struct jaldi_txq *txq;
	struct jaldi_hw *hw;
	
	// currently unused...
	bool bf_stale;
	u16 bf_flags;
};

/**********/
/*  MAC   */
/**********/
enum jaldi_tx_queue_type {
	JALDI_WME_AC_BK = 0,	// background (lowest priority)
	JALDI_WME_AC_BE,	// best effort
	JALDI_WME_AC_VI,	// video
	JALDI_WME_AC_VO,	// voice (highest priority)
	JALDI_WME_UNSP
};

struct jaldi_tx {
	u16 seq_no;
	int hwq_map[JALDI_WME_AC_VO+1]; // the five jaldi hw queues
	spinlock_t txbuflock;
	struct list_head txbuf;
};

struct jaldi_softc {
	struct device *dev;
	struct net_device *net_dev;
	//	spinlock_t wiphy_lock; /* spinlock to protect jaldi_wiphy data */
	//	struct ath_wiphy *pri_wiphy;

	/* Hardware related */
	struct tasklet_struct intr_tq; // interrupt task queue
	struct jaldi_hw *hw; // from ath9k_hw, hw main struct
	void __iomem *mem; // see pci_iomap and lwn article
	int irq; // irq number...
	spinlock_t sc_resetlock;
	spinlock_t sc_serial_rw;

	u32 intrstatus; // keep track of reason for interrupt
	u32 sc_flags; /* TODO: investigate the usage of this */
	bool hw_ready; // flag to see if hw is ready
	u16 ps_flags; /* powersave -- not used */
	u16 curtxpow; /* tx power (.5 dBm units) */
	
	/* netdev */
	struct net_device_stats stats;
	int status;
	int rx_int_enabled;
	int tx_int_enabled;
	struct jaldi_packet *tx_queue; /* packets scheduled for sending */
	struct sk_buff *skb;
	spinlock_t sc_netdevlock;

	u8 macaddr[ETH_ALEN];

	/* tx/rx */
	struct jaldi_tx tx;

	/* ops */
	// none at softc level yet...
};

int jaldi_init_softc(u16 devid, struct jaldi_softc *sc, u16 subsysid, const struct jaldi_bus_ops *bus_ops);
int jaldi_init_device(u16 devid, struct jaldi_softc *sc, u16 subsysid, const struct jaldi_bus_ops *bus_ops);
void jaldi_deinit_device(struct jaldi_softc *sc);
int jaldi_init_interrupts(struct jaldi_softc *sc); 

irqreturn_t jaldi_isr(int irq, void *dev);

#endif /* JALDI_H */

