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

struct jaldi_priv {
	struct net_device_stats stats;
	int status;
	int rx_int_enabled;
	int tx_enabled;
	struct jaldi_packet *tx_queue; /* packets scheduled for sending */
	struct sk_buff *skb;
	spinlock_t lock;
};

struct jaldi_packet {
	struct jaldi_packet *next;
	struct net_device *dev;
	int datalen;
	//char data[ETH_DATA_LEN];
	char data[52];
	s64 tx_time; /* the time at which this packet should be sent */
};

struct jaldi_hw {
	//struct ath_hw *ahw;
	int i;
};


/* Maintains a priority queue of jaldi_packets to be sent */
void jaldi_tx_enqueue(struct net_device *dev, struct jaldi_packet *pkt) {
	unsigned long flags;
	struct jaldi_priv *priv = netdev_priv(dev);
	
	spin_lock_irqsave(&priv->lock, flags);
	if(priv->tx_queue == NULL || pkt->tx_time < priv->tx_queue->tx_time){
		/* New packet goes in front */
		pkt->next = priv->tx_queue;
		priv->tx_queue = pkt;
	} else {
		struct jaldi_packet *curr = priv->tx_queue;
		while(curr->next != NULL && pkt->tx_time < curr->next->tx_time){	
			curr = curr->next;
		}
		pkt->next = curr->next;
		curr->next = pkt;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
}

/* Returns the next jaldi_packet to be transmitted */
struct jaldi_packet *jaldi_tx_dequeue(struct net_device *dev) {
	struct jaldi_priv *priv = netdev_priv(dev);
	struct jaldi_packet *pkt;
	unsigned long flags;
	
	spin_lock_irqsave(&priv->lock, flags);
	pkt = priv->tx_queue;
	if (pkt != NULL) priv->tx_queue = pkt->next;
	spin_unlock_irqrestore(&priv->lock, flags);
	return pkt;
}

int jaldi_open(struct net_device *dev)
{
	memcpy(dev->dev_addr, "\0TIER0", ETH_ALEN);
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
	struct jaldi_priv *priv = netdev_priv(dev);
	
	skb = dev_alloc_skb(pkt->datalen+2);
	
	if(!skb) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "snull rx: low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		goto out;
	}
	
	skb_reserve(skb,2);
	memcpy(skb_put(skb,pkt->datalen),pkt->data,pkt->datalen);
	
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += pkt->datalen;
	netif_rx(skb);
  out:
  	return;
}

int get_jaldi_tx_from_skb(struct sk_buff *skb) {
	return 0; // TODO: decide on a packet format and implement this by reading protocol header field
}

void jaldi_hw_tx(char *buf, int len, struct net_device *dev)
{
	/* TODO: method stub. integrate w/ ath9k_hw */
	printk(KERN_DEBUG "jaldi_hw_tx\n");
	return;
}

//void jaldi_timer_tx(

int jaldi_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len;
	char *data;
	struct jaldi_priv *priv = netdev_priv(dev);
	struct jaldi_packet *pkt;
	
	data = skb->data;
	len = skb->len;
	
	dev->trans_start = jiffies;
	
	priv->skb = skb;
	
	/* create jaldi packet */
	pkt = kmalloc (sizeof (struct jaldi_packet), GFP_KERNEL);
	if (!pkt) {
		printk (KERN_NOTICE "Out of memory while allocating packet for delayed tx\n");
		return 0;
	}
	
	pkt->dev = dev;
	pkt->next = NULL; /* XXX */
//	pkt->data = skb->data;
	pkt->tx_time = get_jaldi_tx_from_skb(skb);
	
	/* add jaldi packet to tx_queue */
	jaldi_tx_enqueue(dev,pkt);
	
	/* create kernel timer for packet transmission */
/*	struct timer_list tx_timer;
	init_timer(&tx_timer);
	tx_timer.function = jaldi_timer_tx;
	tx_timer.data = (unsigned long)*pkt;
	tx_timer.expires = pkt->tx_time;*/
	
	jaldi_hw_tx(data,len,dev);
	
	
	return 0;

}

/* Enable rx (from snull) */
static void jaldi_rx_ints(struct net_device *dev, int enable)
{
	struct jaldi_priv *priv = netdev_priv(dev);
	priv->rx_int_enabled = enable;
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
	.ndo_open				= jaldi_open,
	.ndo_stop				= jaldi_release,
	.ndo_start_xmit			= jaldi_tx,
//	.ndo_get_stats			= jaldi_stats,
};

void jaldi_init(struct net_device *dev)
{
	printk(KERN_DEBUG "jaldi: jaldi_init start");
	struct jaldi_priv *priv;

	ether_setup(dev);

	dev->netdev_ops = &jaldi_netdev_ops;
	priv = netdev_priv(dev);

	memset(priv,0,sizeof(struct jaldi_priv));

	spin_lock_init(&priv->lock);
	
	jaldi_rx_ints(dev,1);

	jaldi_setup_pool(dev);
	printk(KERN_DEBUG "jaldi: jaldi_init end");
}
	

int jaldi_init_module(void)
{
	int result, i, ret = -ENOMEM;
	printk(KERN_DEBUG "jaldi: creating netdev...");
	jaldi_dev = alloc_netdev(sizeof(struct jaldi_priv), "tier%d", jaldi_init);
	
	if (jaldi_dev == NULL) {
		printk(KERN_ERR "jaldi_dev is null");
		goto out;
	}	
	
	printk(KERN_DEBUG "jaldi: netdev allocated.");
		
	ret = -ENODEV;
	if ((result = register_netdev(jaldi_dev))) 
	{
		printk(KERN_DEBUG "jaldi: error %i registering device \"%s\"%n", result, jaldi_dev->name);
	}
	else
	{
		ret = 0;
	}
		
	printk(KERN_DEBUG "jaldi: netdev registered.");	
	
out:
		if (ret)
			jaldi_cleanup();
		return ret;

}

module_init(jaldi_init_module);
module_exit(jaldi_cleanup);
