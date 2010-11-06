/* 
 * jaldi 
 */
 

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/types.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "jaldi.h"

MODULE_AUTHOR("Shaddi Hasan");
MODULE_LICENSE("Dual BSD/GPL");

/* Maintains a priority queue of jaldi_packets to be sent */
void jaldi_tx_enqueue(struct jaldi_softc *sc, struct jaldi_packet *pkt) {
	unsigned long flags;
	
	spin_lock_irqsave(&sc->sc_netdevlock, flags);
	if(sc->tx_queue == NULL || pkt->tx_time < sc->tx_queue->tx_time){
		/* New packet goes in front */
		pkt->next = sc->tx_queue;
		sc->tx_queue = pkt;
	} else {
		struct jaldi_packet *curr = sc->tx_queue;
		while(curr->next != NULL && pkt->tx_time < curr->next->tx_time){	
			curr = curr->next;
		}
		pkt->next = curr->next;
		curr->next = pkt;
	}
	spin_unlock_irqrestore(&sc->sc_netdevlock, flags);
}

/* Returns the next jaldi_packet to be transmitted */
struct jaldi_packet *jaldi_tx_dequeue(struct jaldi_softc *sc) {
	struct jaldi_packet *pkt;
	unsigned long flags;
	
	spin_lock_irqsave(&sc->sc_netdevlock, flags);
	pkt = sc->tx_queue;
	if (pkt != NULL) sc->tx_queue = pkt->next;
	spin_unlock_irqrestore(&sc->sc_netdevlock, flags);
	return pkt;
}

irqreturn_t jaldi_isr(int irq, void *dev)
{
	struct jaldi_softc *sc = dev;
	struct jaldi_hw *hw = sc->hw;

	if (hw->dev_state != JALDI_HW_INITIALIZED) { return IRQ_NONE; }
	
	/* shared irq, not for us */
	if(!jaldi_hw_intrpend(hw)) { return IRQ_NONE; }

	// TODO: determine how we want to handle interrupts
}

	

int jaldi_open(struct net_device *dev)
{
	memcpy(dev->dev_addr, "\0JALDI0", ETH_ALEN);
	netif_start_queue(dev);
	return 0;
}

int jaldi_release(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

/* Method stub from snull -- call from interrupt handler */
void jaldi_rx(struct net_device *dev, struct jaldi_packet *pkt)
{
	struct sk_buff *skb;
	struct jaldi_softc *sc = netdev_priv(dev);
	
	skb = dev_alloc_skb(pkt->datalen+2);
	
	if(!skb) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "jaldi_rx: low on mem - packet dropped\n");
		sc->stats.rx_dropped++;
		goto out;
	}
	
	skb_reserve(skb,2);
	memcpy(skb_put(skb,pkt->datalen),pkt->data,pkt->datalen);
	
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;
	sc->stats.rx_packets++;
	sc->stats.rx_bytes += pkt->datalen;
	netif_rx(skb);
  out:
  	return;
}

int get_jaldi_tx_from_skb(struct sk_buff *skb) {
	return 0; // TODO: decide on a packet format and implement this by reading protocol header field
}

int jaldi_pkt_type(struct jaldi_packet *pkt)
{
	// TODO : define format and check for potential control pkt
	return JALDI_DATA;
}

// TODO: should return appropriate type
int jaldi_get_qos_type_from_skb(struct sk_buff *skb) {
	return JALDI_QOS_BULK; // TODO: make this random-ish for testing
}


int jaldi_hw_ctl(struct jaldi_softc *sc, struct jaldi_packet *pkt) {
	// TODO: set the right control parameters

	return 0;
}

/* This method basically does the following:
 * - assigns skb to hw buffer
 * - sets up tx flags based on current config
 * - allocates DMA buffer
 * - 
 */
int jaldi_hw_tx(struct jaldi_softc *sc, struct jaldi_packet *pkt)
{
	struct jaldi_buf bf;
	

	// TODO: set queue number in hw
	// bf.txq = softc.txq[qnum];

	// TODO: set config tx flags

	// setup buffer
	memset(&bf, 0, sizeof(struct jaldi_buf));

	bf.bf_dma_context = dma_map_single(sc->dev, pkt->skb->data, 
						pkt->skb->len, DMA_TO_DEVICE);

	if(unlikely(dma_mapping_error(sc->dev, bf.bf_dma_context))) {
		jaldi_print(JALDI_WARN, "dma_mapping_error during TX\n");
		return -ENOMEM;
	}

	bf.bf_buf_addr = bf.bf_dma_context;

	

/* Note to self: goal here is to get an sk_buff into a jaldi_buf, which encapsulates
 all the relevant /device/ specific information: dma stuff, etc. We need to couple this
 with a jaldi_tx_control which ensures the proper tx settings go with the sk_buff and push
 this to the device. 

 In short, the jaldi_buf represents an encapsulation of data that takes care of all the
 dma stuff. The jaldi_tx_control encapsulates all the tx settings we need: channel, power, the
 hardware queue we're on (WMA_BE or WMA_BK or WMA_VO). The sk_buff should really be in a 
 jaldi_packet, which will encapsulate the driver logic: which virtual queue we're on.

 This function should do everything needed to move a jaldi_packet into dma. The tx interrupt
 handler should drain the low level queues and initiate transmission over the air.
 */

	return 0;
}

//void jaldi_timer_tx(

/* 
 * This function receives an sk_buff from the kernel, and packages it up into a
 * jaldi_packet. Other high-level tx behaviors should be handled here. This 
 * should be completely device agnostic, as device-specific stuff should be
 * handled in jaldi_hw_tx.
 */
int jaldi_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len;
	char *data;
	struct jaldi_softc *sc = netdev_priv(dev);
	struct jaldi_packet *pkt;
	
	data = skb->data;
	len = skb->len;
	
	dev->trans_start = jiffies;
	
	sc->skb = skb;
	
	/* create jaldi packet */
	pkt = kmalloc (sizeof (struct jaldi_packet), GFP_KERNEL);
	if (!pkt) {
		printk(KERN_NOTICE "jaldi: Out of memory while allocating packet\n");
		return 0;
	}
	
	pkt->dev = dev;
	pkt->next = NULL; /* XXX */
	pkt->data = skb->data;
	pkt->tx_time = get_jaldi_tx_from_skb(skb);
	pkt->qos_type = jaldi_get_qos_type_from_skb(skb);
	
	/* add jaldi packet to tx_queue */
	jaldi_tx_enqueue(sc,pkt); // should add packet to proper software queue


	/* create kernel timer for packet transmission */
/*	struct timer_list tx_timer;
	init_timer(&tx_timer);
	tx_timer.function = jaldi_timer_tx;
	tx_timer.data = (unsigned long)*pkt;
	tx_timer.expires = pkt->tx_time;*/

	/* check for type: control packet or standard */
	if( jaldi_pkt_type(pkt) == JALDI_CONTROL ) { // should check header
		jaldi_hw_ctl(sc, pkt);
	} else {
		jaldi_hw_tx(sc, pkt);
	}
	
	return 0;

}

/* Enable rx interrupts */
static void jaldi_rx_ints(struct jaldi_softc *sc, int enable)
{
	sc->rx_int_enabled = enable;

}
/* Enable tx interrupts */
static void jaldi_tx_ints(struct jaldi_softc *sc, int enable)
{
	sc->tx_int_enabled = enable;
}

/* Set up a device's packet pool. (from snull) */
void jaldi_setup_pool(struct net_device *dev)
{
	return; /* TODO: Implement this */
}

struct net_device *jaldi_dev;

void jaldi_cleanup(void)
{
	unregister_netdev(jaldi_dev);
	free_netdev(jaldi_dev);
	
	return;
}

static const struct net_device_ops jaldi_netdev_ops =
{
	.ndo_open			= jaldi_open,
	.ndo_stop			= jaldi_release,
	.ndo_start_xmit			= jaldi_tx,
//	.ndo_get_stats			= jaldi_stats,
};

/* TODO move to init.c, or call the init there */
void jaldi_init(struct net_device *dev)
{
	struct jaldi_softc *sc;

	jaldi_print(JALDI_INFO, "jaldi_init start\n");

	ether_setup(dev);

	dev->netdev_ops = &jaldi_netdev_ops;
	sc = netdev_priv(dev);

	memset(sc,0,sizeof(struct jaldi_softc));
	
	spin_lock_init(&sc->sc_netdevlock);

	sc->net_dev = dev;
	
	if(!jaldi_init_interrupts(sc)) {
		jaldi_print(JALDI_FATAL, "error initializing interrupt handlers\n");
		return; 
	}

	jaldi_rx_ints(sc,1);
	jaldi_tx_ints(sc,1);

	jaldi_setup_pool(dev);
	jaldi_print(JALDI_INFO, "jaldi_init end\n");
}
	

int jaldi_init_module(void)
{
	int result, ret = -ENOMEM;
	jaldi_print(JALDI_DEBUG, "creating netdev...\n");
	jaldi_dev = alloc_netdev(sizeof(struct jaldi_softc), "jaldi%d", jaldi_init);
	
	if (jaldi_dev == NULL) {
		printk(KERN_ERR "jaldi_dev is null");
		goto out;
	}	
	
	jaldi_print(JALDI_DEBUG, "netdev allocated.\n");
		
	ret = -ENODEV;
	result = register_netdev(jaldi_dev);
	if (result) 
		jaldi_print(JALDI_DEBUG, "error %i registering device \"%s\"\n", result, jaldi_dev->name);
	else
		ret = 0;
		
	jaldi_print(JALDI_DEBUG, "netdev registered.\n");	
	
out:
	if (ret)
		jaldi_cleanup();
	return ret;
}

module_init(jaldi_init_module);
module_exit(jaldi_cleanup);
