/*  
 *  great.c -  The network driver tutorial for Linux 3.5.0.
 *  Copyright(c) 2015 Jie Yan <yanjie111@gmail.com>. All rights reserved.
 */
#include <linux/module.h> // definitions of symbols and functions needed by loadable modules	
#include <linux/init.h>	// specify initialization and cleanup functions
#include <linux/pci.h>  // PCI structure defined here
#include <linux/netdevice.h> // net_device structure defined here
#include <linux/etherdevice.h> // alloc_etherdev
#include <linux/interrupt.h> // request_irq


#define GREAT_DRV_VERSION "1.0.0.0"
char great_driver_name[] = "great";
char great_driver_version[] = GREAT_DRV_VERSION;

/* hardware specified data start here */
#define VENDER_ID			0x1969
#define DEVICE_ID			0x1062
#define REG_MAC_STA_ADDR	0x1488 // register offset which saved the MAC address 
#define NUM_TX_DESC 		4 // from 4 to 1024 for different hardware
#define REG_MASTER_CTRL		0x1400 // soft reset register offset
#define SOFT_RESET_CMD		0x41	// BIT(0) | BIT(6)
/* hardware specified data end here */

#define TX_BUF_SIZE  		1536  /* should be at least MTU + 14 + 4 */
#define TOTAL_TX_BUF_SIZE	(TX_BUF_SIZE * NUM_TX_DESC)
#define TOTAL_RX_BUF_SIZE	16000  // random number. Will change later

// use "lspci -nn" to get the vender ID and device ID
static DEFINE_PCI_DEVICE_TABLE(great_pci_tbl) = {
	{PCI_DEVICE(VENDER_ID, DEVICE_ID)},
	/* required last entry */
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, great_pci_tbl);

MODULE_AUTHOR("Jie Yan  <yanjie111@gmail.com>");
MODULE_DESCRIPTION("   - for Linux version 3.5.0");
MODULE_DESCRIPTION("sample of <Linux Ethernet device driver demystified>");
MODULE_LICENSE("GPL");
MODULE_VERSION(GREAT_DRV_VERSION);

struct great_buffer {
	struct sk_buff *skb;	/* socket buffer */
	u16 length;		/* rx buffer length */
	u16 flags;		/* information of buffer */
	dma_addr_t dma;
};

/* board specific private data structure */
struct great_adapter {
	struct net_device	*netdev;
	struct pci_dev	*pdev;
	spinlock_t	    lock;
	bool 			have_msi;
	u8 __iomem		*hw_addr;  // memory mapped I/O addr (virtual address)
	unsigned long	regs_len;   // length of I/O or MMI/O region
	unsigned int	cur_tx;
	unsigned int	dirty_tx;
	unsigned char	*tx_buf[NUM_TX_DESC];
	unsigned char 	*tx_bufs;        /* Tx buffer start address (virtual address). */
	dma_addr_t 		tx_bufs_dma;      /* Tx buffer dma address (physical address) */

    struct net_device_stats stats;
	unsigned char *rx_ring;
	dma_addr_t rx_ring_dma;
	unsigned int cur_rx;
};

//interrupt routine for receive and xmit
static irqreturn_t great_interrupt (int irq, void *dev_instance) 
{
	struct net_device *netdev = (struct net_device*)dev_instance;
	struct great_adapter *adapter = netdev_priv(netdev);

	// TODO: read register to see if it is Rx interrupt or Tx interrupt

	printk(KERN_INFO "Now calls great_interrupt(). irq is %d.\n", irq);

	return IRQ_HANDLED;
}

// initialize tx ring
static void great_init_ring (struct net_device *netdev)
{
	struct great_adapter *adapter = netdev_priv(netdev);
	int i;

	adapter->cur_tx = 0;
	adapter->dirty_tx = 0;

	for (i = 0; i < NUM_TX_DESC; i++)
		adapter->tx_buf[i] = &adapter->tx_bufs[i * TX_BUF_SIZE];
        
	return;
}

static void great_chip_reset (void *ioaddr)
{
    	u32 ctrl_data = 0; // control data write to register

    	// read control register value
    	*(u32 *)&ctrl_data = readl(ioaddr + REG_MASTER_CTRL);
    	// add soft reset control bit
    	ctrl_data |= SOFT_RESET_CMD;

        /* Soft reset the chip. */
        writel(ctrl_data, ioaddr + REG_MASTER_CTRL);
        // wait 20ms for reset ready
        msleep(20);

    	printk(KERN_INFO "reset chip ready .\n");

        return;
}

// set tx DMA address. start xmit.
static void great_hw_start (struct net_device *netdev)
{
	struct great_adapter *adapter = netdev_priv(netdev);
	void *ioaddr = adapter->hw_addr;

	// reset the chip
	great_chip_reset(ioaddr);

    /* TODO: Must enable Tx/Rx before setting transfer thresholds! */

	/* TODO: tx config */

	/* TODO: rx config */

	/* TODO: init Tx buffer DMA addresses */

    /* TODO: Enable all known interrupts by setting the interrupt mask. */

	netif_start_queue (netdev);
	return;
}

// register IRQ, allocate tx buffer, initialize tx ring
static int great_open(struct net_device *netdev)
{
	int retval;
	struct great_adapter *adapter = netdev_priv(netdev);
	printk(KERN_INFO "Now calls great_open().\n");

	// enable MSI, can get IRQ 43
	adapter->have_msi = true;
	retval = pci_enable_msi(adapter->pdev);
	printk(KERN_INFO "retval is %d, adapter->pdev->irq is %d, .\n", retval,  adapter->pdev->irq);
	/* get the IRQ,  43 is correct, 17 is wrong
	 * second arg is interrupt handler
	 * third is flags, 0 means no IRQ sharing
	 */
	retval = request_irq(adapter->pdev->irq, great_interrupt, 0, netdev->name, netdev);
	if(retval)
	{
		printk(KERN_INFO "request_irq() failed. netdev->irq is %d, adapter->pdev->irq is %d, .\n", netdev->irq, adapter->pdev->irq);
		free_irq(adapter->pdev->irq, netdev);
		return retval;
	}
	printk(KERN_INFO "SUCCEED! request_irq() succeed. netdev->irq is %d, adapter->pdev->irq is %d, .\n", netdev->irq, adapter->pdev->irq);

	/* get memory for Tx buffers
	 * memory must be DMAable
	 */
	adapter->tx_bufs = pci_alloc_consistent(adapter->pdev, TOTAL_TX_BUF_SIZE, &adapter->tx_bufs_dma);
	/* get memory for Rx buffers*/
	adapter->rx_ring = pci_alloc_consistent(adapter->pdev, TOTAL_RX_BUF_SIZE, &adapter->rx_ring_dma);

	if((!adapter->tx_bufs)  || (!adapter->rx_ring))
	{
		printk(KERN_INFO "FAILED to allocate buffer for receive or xmit packets! .\n");
		free_irq(adapter->pdev->irq, netdev);

		if(adapter->tx_bufs) {
			pci_free_consistent(adapter->pdev, TOTAL_TX_BUF_SIZE, adapter->tx_bufs, adapter->tx_bufs_dma);
			adapter->tx_bufs = NULL;
		}
		if(adapter->rx_ring) {
			pci_free_consistent(adapter->pdev, TOTAL_RX_BUF_SIZE, adapter->rx_ring, adapter->rx_ring_dma);
			adapter->rx_ring = NULL;
		}
		return -ENOMEM;
	}
	printk(KERN_INFO "SUCCEED to allocate buffer for xmit packets! pci_alloc_consistent() succeed. adapter->tx_bufs is %p.\n", adapter->tx_bufs);

	// initialize the tx ring
	great_init_ring(netdev);
	// 
	great_hw_start(netdev);
	return 0;
}

static int great_close(struct net_device *netdev)
{
	printk(KERN_INFO "calls great_close().\n");
	return 0;
}

static netdev_tx_t great_xmit_frame(struct sk_buff *skb,
					  struct net_device *netdev)
{
	struct great_adapter *adapter = netdev_priv(netdev);
	unsigned int entry = adapter->cur_tx;

	skb_copy_and_csum_dev(skb, adapter->tx_buf[entry]);
	dev_kfree_skb(skb);

    entry++;
    adapter->cur_tx = entry % NUM_TX_DESC;

 	if(adapter->cur_tx == adapter->dirty_tx) {
 		netif_stop_queue(netdev);
	}

	return NETDEV_TX_OK;
}

static struct net_device_stats *great_get_stats(struct net_device *netdev)
{
	struct great_adapter *adapter = netdev_priv(netdev);

//	printk(KERN_INFO "calls great_get_stats().\n");
	return &(adapter->stats);
}

static const struct net_device_ops great_netdev_ops = {
	.ndo_open		= great_open,
	.ndo_stop		= great_close,
	.ndo_start_xmit	= great_xmit_frame,
	.ndo_get_stats		= great_get_stats,
};

static int __devinit great_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct great_adapter *adapter; // great adapter private data structure
	unsigned long mmio_start, mmio_end, mmio_len, mmio_flags; // PCI bus physical mmio address
	void *ioaddr; // virtual address of mmio_start after ioremap()

	int err = 0, i;

	printk(KERN_INFO "insmod calls great_probe(), Vender ID is 0x%x, Device ID is 0x%x.\n", great_pci_tbl->vendor, great_pci_tbl->device);

	/* enable device (incl. PCI PM wakeup and hotplug setup) */
	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "cannot enable PCI device\n");
		printk(KERN_INFO "cannot enable PCI device.\n");
		return err;
	}

	if ((pci_set_dma_mask(pdev, DMA_BIT_MASK(32)) != 0) ||
	    (pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32)) != 0)) {
		dev_err(&pdev->dev, "No usable DMA configuration,aborting\n");
		pci_disable_device(pdev);
		return err;
	}

	err = pci_request_regions(pdev, great_driver_name);
	if (err) {
		dev_err(&pdev->dev, "cannot obtain PCI resources\n");
		printk(KERN_INFO "cannot obtain PCI resources.\n");
		pci_disable_device(pdev);
		return err;
	}

	printk(KERN_INFO "Now can obtain PCI resources. pdev->irq is %d.\n", pdev->irq);
	pci_set_master(pdev);
	printk(KERN_INFO "Now pci_set_master.\n"); // no ethernet interface, no /sys/class/net/, yes at /sys/bus/pci/drivers/

	// There are a lot of PCI devices on the PCI bus. How does the kernel know great is an ethernet device?
	// allocate the memory for ethernet device. Alloccate memoryfor net_device plus private adapter memory
	netdev = alloc_etherdev(sizeof(struct great_adapter));
	if (netdev == NULL) {
		err = -ENOMEM;
		printk(KERN_INFO "cannot allocate ethernet device resources.\n");
		pci_release_regions(pdev);
		return err;
	}

//#define SET_NETDEV_DEV(net, pdev)	((net)->dev.parent = (pdev))
// set net device's parent device to PCI device
	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

// private data pointer point to net device 's private data
	adapter = netdev_priv(netdev);
// set private data's PCI device to PCI device
	adapter->pdev = pdev;
// set private data's net device to net device
	adapter->netdev = netdev;
// initialize the spinlock
	spin_lock_init (&adapter->lock);
// get mem map io address
	mmio_start = pci_resource_start(pdev, 0);
	printk(KERN_INFO "mmio_start is 0x%lx.\n", mmio_start); // "mmio_start is 0xf1000000." correct
        mmio_end = pci_resource_end(pdev, 0);
	printk(KERN_INFO "mmio_end is 0x%lx.\n", mmio_end); 
        mmio_len = pci_resource_len(pdev, 0);
	printk(KERN_INFO "mmio_len is 0x%lx.\n", mmio_len); 
        mmio_flags = pci_resource_flags(pdev, 0);
	printk(KERN_INFO "mmio_flags is 0x%lx.\n", mmio_flags); 
/*[45391.228356] mmio_start is 0xf1000000.
[45391.228359] mmio_end is 0xf103ffff.
[45391.228361] mmio_len is 0x40000.
[45391.228364] mmio_flags is 0x140204.
*/

/*$lshw -c network
 *-network
       description: Ethernet interface
       product: AR8132 Fast Ethernet
       vendor: Atheros Communications Inc.
       physical id: 0
       bus info: pci@0000:08:00.0
       logical name: eth0
       version: c0
       serial: 00:26:22:64:65:bf
       size: 100Mbit/s
       capacity: 100Mbit/s
       width: 64 bits
       clock: 33MHz
       capabilities: bus_master cap_list ethernet physical tp 10bt 10bt-fd 100bt 100bt-fd autonegotiation
       configuration: autonegotiation=on broadcast=yes driver=atl1c driverversion=1.0.1.0-NAPI duplex=full latency=0 multicast=yes port=twisted pair speed=100Mbit/s
       resources: irq:43 memory:f1000000-f103ffff ioport:2000(size=128)
*/
	/* ioremap MMI/O region */
	ioaddr = ioremap(mmio_start, mmio_len);
	if(!ioaddr) {
		printk(KERN_INFO "Could not ioremap.\n"); 
		free_netdev(netdev);
	}
	// set private data
	netdev->base_addr = (long)ioaddr;
	adapter->hw_addr = ioaddr;
	adapter->regs_len = mmio_len;
	printk(KERN_INFO "Set private data.\n");

	for(i = 0; i < 6; i++) {  /* Hardware Address */
		netdev->dev_addr[i] = readb((const volatile void*)(netdev->base_addr+REG_MAC_STA_ADDR+i));
		netdev->broadcast[i] = 0xff;
	}
	printk(KERN_INFO "Found the MAC is %pM.\n", netdev->dev_addr); // add MAC output
	netdev->hard_header_len = 14;
	// set driver name and irq
	memcpy(netdev->name, great_driver_name, sizeof(great_driver_name)); /* Device Name */
	printk(KERN_INFO "The IRQ number is %d.\n", adapter->pdev->irq); // get irq
 	netdev->netdev_ops = &great_netdev_ops;

 	/* register the network device */
	err = register_netdev(netdev);
	if (err) {
		printk(KERN_INFO "Could not register netdevice.\n");
		return err;
	}

	printk(KERN_INFO "Now init net device in probe function.\n");
	return 0;
}

static void __devexit great_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct great_adapter *adapter = netdev_priv(netdev);

	// rmmod will call it
	free_irq(adapter->pdev->irq, netdev);
	iounmap(adapter->hw_addr);
 	unregister_netdev(netdev);
	free_netdev(netdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static struct pci_driver great_driver = {
	.name     = great_driver_name,
	.id_table = great_pci_tbl,
	.probe    = great_probe,
	.remove   = __devexit_p(great_remove),
};

/*
 * great_init_module - is the first routine called when the driver is loaded.
 * static - it is not meant to be visible outside the file
 * int - return 0 = succeed; other = failed
 * __init - the token is a hint to the kernel that the given function is used 
 * only at initialization time. 
 */
static int __init great_init_module(void)
{	// There is no printf in kernel. 
	printk(KERN_INFO "Init great network driver.\n");
	return pci_register_driver(&great_driver);
}
/*
 * great_exit_module - Driver Exit Cleanup Routine. It is called just before 
 * the driver is removed from memory.
 * void - the cleanup function has no value to return.
 * __exit - it can be called only at module unload or system shutdown time.
 */
static void __exit great_exit_module(void)
{
	printk(KERN_INFO "Cleanup great network driver.\n");
	pci_unregister_driver(&great_driver);
}
// module_init macro adds a special section to the module's object code
// stating where the module's initialization function is to be found
module_init(great_init_module);

// module_exit macro enables kernel to find the cleanup function
module_exit(great_exit_module);
