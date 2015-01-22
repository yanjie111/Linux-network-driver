/*  
 *  great.c -  The network driver tutorial for Linux 3.5.0.
 *  Copyright(c) 2015 Jie Yan <yanjie111@gmail.com>. All rights reserved.
 */
#include <linux/module.h> // definitions of symbols and functions needed by loadable modules	
#include <linux/init.h>	// specify initialization and cleanup functions
/* insert code area 1 here for chapter 3.2 */

#define GREAT_DRV_VERSION "1.0.0.0"
char great_driver_name[] = "great";
char great_driver_version[] = GREAT_DRV_VERSION;

MODULE_AUTHOR("Jie Yan  <yanjie111@gmail.com>");
MODULE_DESCRIPTION("   - for Linux version 3.5.0");
MODULE_DESCRIPTION("sample of <Linux Ethernet device driver demystified>");
MODULE_LICENSE("GPL");
MODULE_VERSION(GREAT_DRV_VERSION);
/* insert code area 2 here for chapter 3.2 */

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
	/* insert code area 3 here for chapter 3.2 */
	return 0;
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
	/* insert code area 4 here for chapter 3.2 */
}
// module_init macro adds a special section to the module's object code
// stating where the module's initialization function is to be found
module_init(great_init_module);

// module_exit macro enables kernel to find the cleanup function
module_exit(great_exit_module);
