#include <linux/module.h>    /* Needed by all modules */
#include <linux/kernel.h>    /* Needed for KERN_INFO */

static int __init hello_init(void)
{
    pr_info("Hello World!\n");
    pr_info("This is the Simple Module\n");
    pr_info("Kernel Module Inserted Successfully...\n");
    return 0;
}
static void __exit hello_exit(void)
{
    pr_info("Goodbye World.\n");
    pr_info("Kernel Module Removed Successfully...\n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Author");

MODULE_DESCRIPTION("A sample driver");
MODULE_VERSION("2:1.0");