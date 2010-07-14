/* 
 * jaldi 
 * "It's over ath9k"
 */
 
#include <linux/config.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/module.h>

#include <linux/kernel.h> /* printk() */
#include <linux/types.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "jaldi.h"

MODULE_AUTHOR("Shaddi Hasan")
MODULE_LICENSE("Dual BSD/GPL")

struct jaldi_priv {
	struct net_device_stats stats;
	int status;
	struct jaldi_packet *tx_queue; /* packets scheduled for sending */
	struct sk_buff *skb;
	spinlock_t lock;
	
}

struct jaldi_packet {
	struct jaldi_packet *next;
	struct net_device *dev;
	int datalen;
	u8 data[ETH_DATA_LEN];
	
	s64 tx_time; /* the time at which this packet should be sent */

}



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
	netf_start_queue(dev);
	return 0;
}

int jaldi_release(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

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

void jaldi_hw_tx(char *buf, int len, struct net_device *dev)
{
	/* TODO: method stub. integrate w/ ath9k_hw */
	printk(KERN_DEBUG "jaldi_hw_tx\n");
	return;
}

int jaldi_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len;
	char *data
	struct jaldi_priv *priv = netdev_priv(dev);
	
	data = skb->data;
	len = skb->len;
	
	dev->trans_start = jiffies;
	
	priv->skb = skb;
	
	
	/* Packets need to be sent at prescribed times. If the time
	 * field of the packet is in the past, send it immediately (or drop?).
	 * Otherwise set 
	 *
	 */
	 
	
	jaldi_hw_tx(data,len,dev);
	
	return 0;

}

struct net_device *jaldi_dev;

void jaldi_cleanup(void)
{
	unregister_netdev(jaldi_dev);
	free_netdev(jaldi_dev);
	
	return;
}



void jaldi_init(struct net_device *dev)
{
	struct jaldi_priv *priv;
	
	ether_setup(dev);
	dev->open				= jaldi_open;
	dev->stop				= jaldi_release;
	dev->set_config			= jaldi_config;
	dev->hard_start_xmit	= jaldi_tx;
	dev->do_ioctl			= jaldi_ioctl;
	dev->get_stats			= jaldi_stats;
	dev->change_mtu			= jaldi_change_mtu;
	dev->rebuild_header		= jaldi_rebuild_header;
	dev->hard_header		= jaldi_header;
	dev->tx_timeout			= jaldi_tx_timeout;
	dev->watchdog_timeo		= timeout;
	
	priv = netdev_priv(dev);
	memset(priv,0,sizeof(struct jaldi_priv));
	spin_lock_init(&priv->lock);
	jaldi_rx_ints(dev,1);
	jaldi_setup_pool(dev);
}
	

int jaldi_init_module(void)
{
	int result, i, ret = -ENOMEM;
	
	jaldi_dev = alloc_netdev(sizeof(struct jaldi_priv), "tier%d", jaldi_init);
	
	if (jaldi_dev == NULL)
		goto out;
		
	
	
		
	out:
		if (ret)
			jaldi_cleanup();
		return ret;

}

module_init(jaldi_init_module);
module_exit(jaldi_cleanup);
