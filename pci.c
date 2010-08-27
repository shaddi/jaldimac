/* 
 * JaldiMAC PCI related stuff
 */
 
#include <linux/pci.h>
#include "jaldi.h"


static int jaldi_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	void __iomem *mem;
	u8 csz;
	int ret = 0;
	
	i = pci_enable_device(pdev)
	if (i) return error; 
	
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		printk(KERN_ERR "jaldi: 32-bit DMA not available\n");
		goto err_dma;
	}
	
	ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		printk(KERN_ERR "jaldi: 32-bit DMA consistent "
			"DMA enable failed\n");
		goto err_dma;
	}

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &csz);
	if (csz == 0) {
		/*
		 * Linux 2.4.18 (at least) writes the cache line size
		 * register as a 16-bit wide register which is wrong.
		 * We must have this setup properly for rx buffer
		 * DMA to work so force a reasonable value here if it
		 * comes up zero.
		 */
		csz = L1_CACHE_BYTES / sizeof(u32);
		pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, csz);
	}
	/*
	 * The default setting of latency timer yields poor results,
	 * set it to the value used by other systems. It may be worth
	 * tweaking this setting more.
	 */
	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0xa8);

	pci_set_master(pdev);
	
	/*
	 * Disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 */
	pci_read_config_dword(pdev, 0x40, &val);
	if ((val & 0x0000ff00) != 0)
		pci_write_config_dword(pdev, 0x40, val & 0xffff00ff);
		
	ret = pci_request_region(pdev, 0, "jaldi");
	if (ret) {
		dev_err(&pdev->dev, "PCI memory region reserve error\n");
		ret = -ENODEV;
		goto err_region;
	}
	
	mem = pci_iomap(pdev, 0, 0);
	if (!mem) {
		printk(KERN_ERR "PCI memory map error\n") ;
		ret = -EIO;
		goto err_iomap;
	}
	
	// allocate memory for jaldi driver (wiphy + softc)
	
	ret = request_irq(pdev->irq, , IRQF_SHARED, "jaldi", 

err_iomap:
	pci_release_region(pdev, 0);
err_region:
	/* Nothing */
err_dma:
	pci_disable_device(pdev);
	return ret;
}


static struct pci_driver jaldi_pci_driver = {
	.name       = "jaldi",
	.id_table   = jaldi_pci_id_table,
	.probe      = jaldi_pci_probe,
	.remove     = jaldi_pci_remove,
#ifdef CONFIG_PM
	.suspend    = jaldi_pci_suspend,
	.resume     = jaldi_pci_resume,
#endif /* CONFIG_PM (what is this? XXX) */
};



int jaldi_pci_init(void)
{
	return pci_register_driver(&jaldi_pci_driver);
}

void jaldi_pci_exit(void)
{
	pci_unregister_driver(&jaldi_pci_driver);
}
