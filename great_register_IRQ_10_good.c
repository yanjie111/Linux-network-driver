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
#define TRANSMIT_RING_SIZE	2 // 
/* hardware specified data end here */

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

/* transimit packet descriptor (tpd) ring */
struct great_tpd_ring {
	void *desc;		/* descriptor ring virtual address */
	dma_addr_t dma;		/* descriptor ring physical address */
	u16 size;		/* descriptor ring length in bytes */
	u16 count;		/* number of descriptors in the ring */
	u16 next_to_use; 	/* this is protectd by adapter->lock */
	atomic_t next_to_clean;
	struct great_buffer *buffer_info;
};

/* board specific private data structure */
struct great_adapter {
	struct net_device   *netdev;
	struct pci_dev      *pdev;
	spinlock_t	    lock;
	bool have_msi;

//	u32		    msg_enable; // controls the debug message level
	u8 __iomem        *hw_addr;  // memory mapped I/O addr (virtual address)
        unsigned long 	    regs_len;   // length of I/O or MMI/O region
// ring data
	struct great_tpd_ring tpd_ring[TRANSMIT_RING_SIZE]; // transmit packet ring. 2~4

};

static irqreturn_t great_interrupt (int irq, void *dev_instance) 
{
 //       struct net_device *netdev = (struct net_device*)dev_instance;
//	struct great_adapter *adapter = netdev_priv(netdev);
 //       void *ioaddr = adapter->hw_addr;
//        unsigned short isr = readw(ioaddr + ISR);

	printk(KERN_INFO "Now calls great_interrupt(). 111 irq is %d.\n", irq);

	return IRQ_HANDLED;
}

// Add register IRQ
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
 //       retval = request_irq(adapter->pdev->irq, great_interrupt, 0, netdev->name, netdev);
        retval = request_irq(adapter->pdev->irq, great_interrupt, 0, netdev->name, netdev);
        if(retval)
	{
		printk(KERN_INFO "request_irq() failed. netdev->irq is %d, adapter->pdev->irq is %d, .\n", netdev->irq, adapter->pdev->irq);
               return retval;
	}
	printk(KERN_INFO "SUCCEED! request_irq() succeed. netdev->irq is %d, adapter->pdev->irq is %d, .\n", netdev->irq, adapter->pdev->irq);
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
/*	struct atl1c_adapter *adapter = netdev_priv(netdev);
	unsigned long flags;
	u16 tpd_req = 1;
	struct atl1c_tpd_desc *tpd;
	enum atl1c_trans_queue type = atl1c_trans_normal;*/
//	printk(KERN_INFO "calls great_xmit_frame().\n");
	return NETDEV_TX_OK;
}

static struct net_device_stats *great_get_stats(struct net_device *netdev)
{
/*	struct atl1c_adapter *adapter = netdev_priv(netdev);
	struct atl1c_hw_stats  *hw_stats = &adapter->hw_stats;*/
	struct net_device_stats *net_stats = &netdev->stats;
//	printk(KERN_INFO "calls great_get_stats().\n");
	return net_stats;
}

static const struct net_device_ops great_netdev_ops = {
	.ndo_open		= great_open,
	.ndo_stop		= great_close,
	.ndo_start_xmit	= great_xmit_frame,
	.ndo_get_stats		= great_get_stats,
};

//#define AT_TX_WATCHDOG  (5 * HZ)

static int __devinit great_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct great_adapter *adapter; // great adapter private data structure
//	void __iomem *mmio_start;
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

	// read mac address: should be 00:26:22:64:65:bf 
//But the MAC read is 26:65:64:22:bf:00. 

     	for(i = 0; i < 6; i++) {  /* Hardware Address */
               netdev->dev_addr[i] = readb((const volatile void*)netdev->base_addr+REG_MAC_STA_ADDR+i);
               netdev->broadcast[i] = 0xff;
        }
	printk(KERN_INFO "Found the MAC is %pM.\n", netdev->dev_addr); // add MAC output
        netdev->hard_header_len = 14;
	// set driver name and irq
        memcpy(netdev->name, great_driver_name, sizeof(great_driver_name)); /* Device Name */
//        netdev->irq = pdev->irq;  /* Interrupt Number */
	printk(KERN_INFO "The IRQ number is %d.\n", adapter->pdev->irq); // get irq
 

	netdev->netdev_ops = &great_netdev_ops;
//	netdev->watchdog_timeo = AT_TX_WATCHDOG;
//	netdev->ethtool_ops = &great_ethtool_ops;

       /* register the network device */
	err = register_netdev(netdev);
	if (err) {
		printk(KERN_INFO "Could not register netdevice.\n");
		return err;
        }   

	printk(KERN_INFO "Now init net device in probe function.\n");
	// TODO: 
//	printk(KERN_INFO "Succeed to initialize ethernet device.\n");
	// **_hw_set_mac_addr(&adapter->hw, adapter->hw.mac_addr);
	//err = register_netdev(netdev);


	return 0;
}

static void __devexit great_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct great_adapter *adapter = netdev_priv(netdev);

	// rmmod will call it
	printk(KERN_INFO "rmmod calls great_remove...\n");
	iounmap(adapter->hw_addr);
 	unregister_netdev(netdev);
	free_netdev(netdev);
	pci_release_regions(pdev); // added
	pci_disable_device(pdev);
	printk(KERN_INFO "Disabled pci device...\n");
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
	printk(KERN_INFO "Init great network driver 333.\n");
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
	printk(KERN_INFO "Cleanup great network driver 333.\n");
	pci_unregister_driver(&great_driver);
}
// module_init macro adds a special section to the module's object code
// stating where the module's initialization function is to be found
module_init(great_init_module);

// module_exit macro enables kernel to find the cleanup function
module_exit(great_exit_module);
