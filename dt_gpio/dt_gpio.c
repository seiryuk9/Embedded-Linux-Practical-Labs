#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/semaphore.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
 
#define GPIO_CNT_IRQ	1
#define GPIO_NAME	"gpio1_pc7_irq"
#define KEY0VALUE	0x01
#define INVAKEY	0xFF
#define KEY_NUM	1

 
// struct gpio1_pc7_irq_dev gpio1_pc7_irq;
 
// static irqreturn_t gpio1_pc7_handler(int irq, void *dev_id)
// {
// 	printk("irq has been happen\r\n");
// 	return IRQ_RETVAL(IRQ_HANDLED);
// }
struct firefly_gpio_info
{
	int firefly_gpio;
	int gpio_enable_value;
};

static int gpio1_pc7_probe(struct platform_device *pdev)
{
	int ret;
	int gpio;
	enum of_gpio_flags flag;
	struct firefly_gpio_info *gpio_info;
	struct device_node *firefly_gpio_node = pdev->dev.of_node;

	printk("Firefly GPIO Test Program Probe\n");
	gpio_info = devm_kzalloc(&pdev->dev,sizeof(struct firefly_gpio_info *), GFP_KERNEL);
	if (!gpio_info) {
		return -ENOMEM;
	}
	gpio = of_get_named_gpio_flags(firefly_gpio_node, "gpio2-a2", 0, &flag);
	if (!gpio_is_valid(gpio)) {
		printk("firefly-gpio: %d is invalid\n", gpio); return -ENODEV;
	}
	if (gpio_request(gpio, "firefly-gpio")) {
		printk("gpio %d request failed!\n", gpio);
		gpio_free(gpio);
		return -ENODEV;
	}
	gpio_info->firefly_gpio = gpio;
	gpio_info->gpio_enable_value = (flag == OF_GPIO_ACTIVE_LOW) ? 0:1;
	gpio_direction_output(gpio_info->firefly_gpio, gpio_info->gpio_enable_value);
	printk("Firefly gpio putout\n");
 
	return 0;
}
 
static int gpio1_pc7_remove(struct platform_device *pdev)
{
	int ret;
	int gpio;
	enum of_gpio_flags flag;
	struct device_node *firefly_gpio_node = pdev->dev.of_node;

	printk("Firefly GPIO Test Program Remove\n");
	gpio = of_get_named_gpio_flags(firefly_gpio_node, "gpio2-a2", 0, &flag);
	if (!gpio_is_valid(gpio)) {
		printk("firefly-gpio: %d is invalid\n", gpio); return -ENODEV;
	}
    gpio_free(gpio);

	return 0;
}

static const struct of_device_id gpio_of_match[] = 
{
	{ .compatible = "gpios,rv1106-gpio" },
	{},
};
 
MODULE_DEVICE_TABLE(of, gpio_of_match);

static struct platform_driver gpio1_pc7_driver = 
{
	.driver =
	{
        .owner = THIS_MODULE,
		.name = GPIO_NAME,
		.of_match_table = gpio_of_match,
	},
	.probe = gpio1_pc7_probe,
	.remove = gpio1_pc7_remove,
};
 
static int __init gpio1_pc7_init(void)
{
	return platform_driver_register(&gpio1_pc7_driver);
}
 
static void __exit gpio1_pc7_exit(void)
{
	platform_driver_unregister(&gpio1_pc7_driver);
}
 
module_init(gpio1_pc7_init);
module_exit(gpio1_pc7_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luckfox");
MODULE_VERSION("V1.0");