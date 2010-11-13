/*
 * jaldimac
 */
 
#include "jaldi.h"

/* We use the hw_value as an index into our private channel structure */

#define CHAN2G(_freq, _idx)  { \
	.channel = (_freq), \
	.center_freq = (_freq), \
	.hw_value = (_idx), \
	.max_power = 20, \
}

#define CHAN5G(_freq, _idx) { \
	.channel = (_freq), \
	.center_freq = (_freq), \
	.hw_value = (_idx), \
	.max_power = 20, \
}

/* From a9k:
 * "Some 5 GHz radios are actually tunable on XXXX-YYYY
 * on 5 MHz steps, we support the channels which we know
 * we have calibration data for all cards though to make
 * this static"
 *
 * Because we (JaldiMAC) are not doing anything with calibration yet,
 * perhaps we can leverage this capability. TODO: does 9280 support this?
 */
static struct jaldi_channel jaldi_5ghz_chantable[] = {
	/* _We_ call this UNII 1 */
	CHAN5G(5180, 14), /* Channel 36 */
	CHAN5G(5200, 15), /* Channel 40 */
	CHAN5G(5220, 16), /* Channel 44 */
	CHAN5G(5240, 17), /* Channel 48 */
	/* _We_ call this UNII 2 */
	CHAN5G(5260, 18), /* Channel 52 */
	CHAN5G(5280, 19), /* Channel 56 */
	CHAN5G(5300, 20), /* Channel 60 */
	CHAN5G(5320, 21), /* Channel 64 */
	/* _We_ call this "Middle band" */
	CHAN5G(5500, 22), /* Channel 100 */
	CHAN5G(5520, 23), /* Channel 104 */
	CHAN5G(5540, 24), /* Channel 108 */
	CHAN5G(5560, 25), /* Channel 112 */
	CHAN5G(5580, 26), /* Channel 116 */
	CHAN5G(5600, 27), /* Channel 120 */
	CHAN5G(5620, 28), /* Channel 124 */
	CHAN5G(5640, 29), /* Channel 128 */
	CHAN5G(5660, 30), /* Channel 132 */
	CHAN5G(5680, 31), /* Channel 136 */
	CHAN5G(5700, 32), /* Channel 140 */
	/* _We_ call this UNII 3 */
	CHAN5G(5745, 33), /* Channel 149 */
	CHAN5G(5765, 34), /* Channel 153 */
	CHAN5G(5785, 35), /* Channel 157 */
	CHAN5G(5805, 36), /* Channel 161 */
	CHAN5G(5825, 37), /* Channel 165 */
};

/* List of rates we can select from. 
 * TODO: What is the hardware limit for these rates? 
 */
#define RATE(_bitrate, _hw_rate) {              \
	.bitrate        = (_bitrate),                   \
	.hw_value       = (_hw_rate),                   \
}

static struct jaldi_bitrate jaldi_rates[] = {
	RATE(10, 0x1b),
	RATE(20, 0x1a),
	RATE(55, 0x19),
	RATE(110, 0x18),
	RATE(60, 0x0b),
	RATE(90, 0x0f),
	RATE(120, 0x0a),
	RATE(180, 0x0e),
	RATE(240, 0x09),
	RATE(360, 0x0d),
	RATE(480, 0x08),
	RATE(540, 0x0c),
};

static void jaldi_deinit_softc(struct jaldi_softc *sc);

static void jaldi_iowrite32(struct jaldi_hw *hw, u32 val, u32 reg_offset) {
	struct jaldi_softc *sc = hw->sc;

	if (hw->serialize_regmode == SER_REG_MODE_ON) {
		unsigned long flags;
		spin_lock_irqsave(&sc->sc_serial_rw, flags);
		iowrite32(val, sc->mem + reg_offset);
		spin_unlock_irqrestore(&sc->sc_serial_rw, flags);
	} else {
		iowrite32(val, sc->mem + reg_offset);
	}
}

static void jaldi_ioread32(struct jaldi_hw *hw, u32 reg_offset) {
	struct jaldi_softc *sc = hw->sc;
	u32 val; 

	if (hw->serialize_regmode == SER_REG_MODE_ON) {
		unsigned long flags;
		spin_lock_irqsave(&sc->sc_serial_rw, flags);
		val = ioread32(sc->mem + reg_offset);
		spin_unlock_irqrestore(&sc->sc_serial_rw, flags);
	} else {
		val = ioread32(sc->mem + reg_offset);
	}
}

static const struct jaldi_register_ops jaldi_reg_ops = {
	.read = jaldi_ioread32,
	.write = jaldi_iowrite32,
};

// TODO: finish this. 
/* Should set up DMA as well as worker thread to handle setting up queues, etc. */
int jaldi_tx_init(struct jaldi_softc *sc, int nbufs)
{
	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	spin_lock_init(&sc->tx.txbuflock);

	return 0;

}

void jaldi_tx_cleanup(struct jaldi_softc *sc)
{
	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	if (sc->tx.txdma.dd_desc_len != 0)
		jaldi_descdma_cleanup(sc, &sc->tx.txdma, &sc->tx.txbuf); 

}

void jaldi_descdma_cleanup(struct jaldi_softc *sc, struct jaldi_descdma *dd,
				struct list_head *head)
{
	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	dma_free_coherent(sc->dev, dd->dd_desc_len, dd->dd_desc,
				dd->dd_desc_paddr);

	INIT_LIST_HEAD(head);
	kfree(dd->dd_bufptr);
	memset(dd, 0, sizeof(*dd));
}
/*  From ath9k:
 *  "This function will allocate both the DMA descriptor structure, and the
 *  buffers it contains.  These are used to contain the descriptors used
 *  by the system."
*/
int jaldi_descdma_setup(struct jaldi_softc *sc, struct jaldi_descdma *dd,
		      struct list_head *head, const char *name,
		      int nbuf, int ndesc, bool is_tx)
{
#define	DS2PHYS(_dd, _ds)						\
	((_dd)->dd_desc_paddr + ((caddr_t)(_ds) - (caddr_t)(_dd)->dd_desc))
#define ATH_DESC_4KB_BOUND_CHECK(_daddr) ((((_daddr) & 0xFFF) > 0xF7F) ? 1 : 0)
#define ATH_DESC_4KB_BOUND_NUM_SKIPPED(_len) ((_len) / 4096)
	u8 *ds;
	struct jaldi_buf *bf;
	int i, bsize, error, desc_len;

	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	jaldi_print(JALDI_DEBUG, "%s DMA: %u buffers %u desc/buf\n",
		  name, nbuf, ndesc);

	INIT_LIST_HEAD(head);

	if (is_tx)
		desc_len = sc->hw->caps.tx_desc_len;
	else
		desc_len = sizeof(struct jaldi_desc);

	/* ath_desc must be a multiple of DWORDs */
	if ((desc_len % 4) != 0) {
		jaldi_print(JALDI_FATAL,
			  "jaldi_desc not DWORD aligned\n");
		BUG_ON((desc_len % 4) != 0);
		error = -ENOMEM;
		goto fail;
	}

	dd->dd_desc_len = desc_len * nbuf * ndesc;

	/*
	 * Need additional DMA memory because we can't use
	 * descriptors that cross the 4K page boundary. Assume
	 * one skipped descriptor per 4K page.
	 */
	if (!(sc->hw->caps.hw_caps & JALDI_HW_CAP_4KB_SPLITTRANS)) {
		u32 ndesc_skipped =
			ATH_DESC_4KB_BOUND_NUM_SKIPPED(dd->dd_desc_len);
		u32 dma_len;

		while (ndesc_skipped) {
			dma_len = ndesc_skipped * desc_len;
			dd->dd_desc_len += dma_len;

			ndesc_skipped = ATH_DESC_4KB_BOUND_NUM_SKIPPED(dma_len);
		}
	}

	/* allocate descriptors */
	dd->dd_desc = dma_alloc_coherent(sc->dev, dd->dd_desc_len,
					 &dd->dd_desc_paddr, GFP_KERNEL);
	if (dd->dd_desc == NULL) {
		error = -ENOMEM;
		goto fail;
	}
	ds = (u8 *) dd->dd_desc;
	jaldi_print(JALDI_INFO, "%s DMA map: %p (%u) -> %llx (%u)\n",
		  name, ds, (u32) dd->dd_desc_len,
		  ito64(dd->dd_desc_paddr), /*XXX*/(u32) dd->dd_desc_len);

	/* allocate buffers */
	bsize = sizeof(struct jaldi_buf) * nbuf;
	bf = kzalloc(bsize, GFP_KERNEL);
	if (bf == NULL) {
		error = -ENOMEM;
		goto fail2;
	}
	dd->dd_bufptr = bf;

	for (i = 0; i < nbuf; i++, bf++, ds += (desc_len * ndesc)) {
		bf->bf_desc = ds;
		bf->bf_daddr = DS2PHYS(dd, ds);

		if (!(sc->hw->caps.hw_caps &
		      JALDI_HW_CAP_4KB_SPLITTRANS)) {
			/*
			 * Skip descriptor addresses which can cause 4KB
			 * boundary crossing (addr + length) with a 32 dword
			 * descriptor fetch.
			 */
			while (ATH_DESC_4KB_BOUND_CHECK(bf->bf_daddr)) {
				BUG_ON((caddr_t) bf->bf_desc >=
				       ((caddr_t) dd->dd_desc +
					dd->dd_desc_len));

				ds += (desc_len * ndesc);
				bf->bf_desc = ds;
				bf->bf_daddr = DS2PHYS(dd, ds);
			}
		}
		list_add_tail(&bf->list, head);
		list_add_tail(&bf->list, head);
	}
	return 0;
fail2:
	dma_free_coherent(sc->dev, dd->dd_desc_len, dd->dd_desc,
			  dd->dd_desc_paddr);
fail:
	memset(dd, 0, sizeof(*dd));
	return error;
#undef ATH_DESC_4KB_BOUND_CHECK
#undef ATH_DESC_4KB_BOUND_NUM_SKIPPED
#undef DS2PHYS
}

static int jaldi_init_queues(struct jaldi_softc *sc)
{
	int i = 0;

	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	for (i = 0; i < ARRAY_SIZE(sc->tx.hwq_map); i++)
		sc->tx.hwq_map[i] = -1;

	
	if (!jaldi_tx_setup(sc, JALDI_WME_AC_BK)) { 
		jaldi_print(JALDI_FATAL,
			  "Unable to setup xmit queue for BK traffic\n");
		goto err;
	}

	if (!jaldi_tx_setup(sc, JALDI_WME_AC_BE)) {
		jaldi_print(JALDI_FATAL,
			  "Unable to setup xmit queue for BE traffic\n");
		goto err;
	}
	if (!jaldi_tx_setup(sc, JALDI_WME_AC_VI)) {
		jaldi_print(JALDI_FATAL,
			  "Unable to setup xmit queue for VI traffic\n");
		goto err;
	}
	if (!jaldi_tx_setup(sc, JALDI_WME_AC_VO)) {
		jaldi_print(JALDI_FATAL,
			  "Unable to setup xmit queue for VO traffic\n");
		goto err;
	}

	return 0;
err:
	for (i = 0; i < JALDI_NUM_TX_QUEUES; i++)
		if (JALDI_TXQ_SETUP(sc, i))
			jaldi_tx_cleanupq(sc, &sc->tx.txq[i]);

	return -EIO;
}

int jaldi_init_softc(u16 devid, struct jaldi_softc *sc, u16 subsysid, const struct jaldi_bus_ops *bus_ops)
{
	struct jaldi_hw *hw = NULL;
	int ret = 0;

	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	hw = kzalloc(sizeof(struct jaldi_hw), GFP_KERNEL);
	if (!hw) return -ENOMEM;

	hw->hw_version.devid = devid;
	hw->hw_version.subsysid = subsysid;
	sc->hw = hw;

	spin_lock_init(&sc->sc_resetlock);
	spin_lock_init(&sc->sc_netdevlock);
	spin_lock_init(&sc->sc_pm_lock);
	/* init tasklets and other locks here */

	/* ath9k reads cache line size here... may be relevant */
	ret = jaldi_hw_init(hw);
	if (ret) goto err_hw;

	ret = jaldi_init_queues(sc);
	if (ret) goto err_queues;
	
err_queues:
	jaldi_hw_deinit(hw);
err_hw:
	/* need to kill any tasklets we started */
	kfree(hw);
	sc->hw = NULL;
	jaldi_print(JALDI_FATAL,"init_device failed, ret=%d\n",ret);
	return ret;
}

int jaldi_init_device(u16 devid, struct jaldi_softc *sc, u16 subsysid, const struct jaldi_bus_ops *bus_ops)
{
	struct jaldi_hw *hw;
	int error;

	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	error = jaldi_init_softc(devid, sc, subsysid, bus_ops);
	if (error != 0)
		goto error_init;

	hw = sc->hw;

	/* Setup TX DMA */
	error = jaldi_tx_init(sc, JALDI_NUM_TXBUF); // TODO
	if (error) { goto error_tx; }

	/* Setup RX DMA */
//	error = jaldi_rx_init(sc, JALDI_NUM_RXBUF); // TODO
//	if (error) { goto error_rx; }

	/* initialize workers here if needed */

	return 0;

error_rx:
	jaldi_tx_cleanup(sc);
error_tx:
	jaldi_deinit_softc(sc);
error_init:
	jaldi_print(JALDI_FATAL, "init_device failed, error %d.\n",error);
	return error;
}

/* TODO
 * this is probably similar to ath9k_init_interrupt_masks in ath9k's hw.c
 */
int jaldi_init_interrupts(struct jaldi_softc *sc)
{
	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	return 0;
}

/*****************************/
/*     De-Initialization     */
/*****************************/

static void jaldi_deinit_softc(struct jaldi_softc *sc)
{
	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	int i = 0;

	for (i = 0; i < JALDI_NUM_TX_QUEUES; i++)
		if (JALDI_TXQ_SETUP(sc, i))
			jaldi_tx_cleanupq(sc, &sc->tx.txq[i]);

	jaldi_hw_deinit(sc->hw);

	kfree(sc->hw);
	sc->hw = NULL;
}

void jaldi_deinit_device(struct jaldi_softc *sc)
{
	jaldi_print(JALDI_DEBUG,"Entering '%s'\n", __FUNCTION__);	
	jaldi_ps_wakeup(sc); 

//	jaldi_rx_cleanup(sc); // TODO 
	jaldi_tx_cleanup(sc); 
	jaldi_deinit_softc(sc);
}

/* deinit of descdma could go here */
