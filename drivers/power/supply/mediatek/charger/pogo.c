#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/mtk_battery.h>
#include <mt-plat/mtk_charger.h>
#include <mtk_charger_intf.h>

int dock_status = -1;
int usb_status = -1;
struct platform_device *pogo_dev;
struct work_struct irq_wq;
static struct charger_device *primary_charger;

static enum power_supply_property dock_pogo_props[] = {
	POWER_SUPPLY_PROP_charge_dock,
	POWER_SUPPLY_PROP_charge_usb,
};

static int dock_pogo_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{

    switch (psp) {
    case POWER_SUPPLY_PROP_charge_dock:
        val->intval = !gpio_get_value(dock_status);
        break;
    case POWER_SUPPLY_PROP_charge_usb:
        val->intval = !gpio_get_value(usb_status);
        break;

    default:
        return -EINVAL;
    }

    return 0;
}


static const struct power_supply_desc dock_pogo_desc = {
	.name			= "customer",
	.type			= POWER_SUPPLY_TYPE_CUST,
	.properties		= dock_pogo_props,
	.num_properties		= 2,
	.get_property		= dock_pogo_property,
	.set_property		= NULL,
	.property_is_writeable	= NULL,
};

void uevent_send(struct work_struct *work)
{
	int usb_value,dock_value;
	bool is_otg,is_pogo_en;
        char *env[] = { "dock-pogo", "status-change", NULL };
	kobject_uevent_env(&pogo_dev->dev.kobj, KOBJ_CHANGE,env);

	usb_value = !gpio_get_value(usb_status);
	dock_value = !gpio_get_value(dock_status);
	is_otg = charger_dev_is_otg(primary_charger);
	is_pogo_en = charger_dev_is_pogo_enable(primary_charger);
	printk("=============pogo_state_changed=================is_otg[%d]=========is_pogo_en[%d]========\r\n",is_otg,is_pogo_en);
	if((is_otg == 0)&&(is_pogo_en == 1))
	{
		printk("=========leon====enable pogo when power change=================\r\n");
		mdelay(1000);
		charger_dev_pogo_enable(primary_charger,true);
	}
	else
	{
		printk("=========leon====nothing when power change=================\r\n");
	}

}

static irqreturn_t dock_interrupt(int irq, void *data)
{
	printk("=============pogo_state_changed---dock=================\r\n");
	schedule_work(&irq_wq);
	return IRQ_HANDLED;
}


static irqreturn_t usb_interrupt(int irq, void *data)
{
	printk("=============pogo_state_changed--usb=================\r\n");
	schedule_work(&irq_wq);
	return IRQ_HANDLED;
}


static int dock_pogo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;


	int irq_dock;

	int irq_usb;

	pogo_dev = devm_kzalloc(dev, sizeof(struct platform_device), GFP_KERNEL);
	pogo_dev = pdev;

	dock_status = of_get_named_gpio(node, "dock_status", 0);
	if (dock_status < 0)
	{
		printk( "dock_status is not available\n");
		return -1;
	}

	usb_status = of_get_named_gpio(node, "usb_status", 0);
	if (usb_status < 0)
	{
		printk( "usb_status is not available\n");
		return -1;
	}


	node = of_find_compatible_node(NULL, NULL, "mediatek,usb_int");
	if (node) {
		mdelay(10);
		printk( "usb ===irq_node available\n");
	}
	else {
		mdelay(10);
		printk( "usb ===irq_node is not available\n");
	}
	irq_usb = irq_of_parse_and_map(node, 0);



	node = of_find_compatible_node(NULL, NULL, "mediatek,pogo_int");
	if (node) {
		printk( "pogo ===irq_node available\r\n");
	}
	else {
		printk( "pogo===irq_node is not available\r\n");
	}
	irq_dock = irq_of_parse_and_map(node, 0);

	gpio_request(dock_status,"dock_status");
	gpio_direction_input(dock_status);

	gpio_request(usb_status,"usb_status");
	gpio_direction_input(usb_status);

	INIT_WORK(&irq_wq,uevent_send);

	request_irq(irq_dock, dock_interrupt, IRQ_TYPE_EDGE_BOTH, "pogo_dock_irq", NULL);
	request_irq(irq_usb, usb_interrupt, IRQ_TYPE_EDGE_BOTH, "pogo_usb_irq", NULL);
	primary_charger = get_charger_by_name("primary_chg");
	if (!primary_charger) {
		printk("==========pogo driver: get primary charger device failed\n");
	}

  	power_supply_register(&pdev->dev, &dock_pogo_desc,NULL);
    return 0;
}


static struct of_device_id dock_pogo_table[] = {
	{ .compatible = "mediatek,dock_pogo",},
	{ },
};

static struct platform_driver dock_pogo_driver = {
	.probe = dock_pogo_probe,
	.driver = {
		.name = "dock_pogo",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dock_pogo_table),
	},
};


static __init int dock_pogo_init(void)
{
	return platform_driver_register(&dock_pogo_driver);
}
static __exit void dock_pogo_exit(void)
{
	//i2c_del_driver(&mm8013_i2c_driver);
}
module_init(dock_pogo_init);
module_exit(dock_pogo_exit);
MODULE_AUTHOR("leon");
MODULE_DESCRIPTION("dock pogo driver for levono m10");
MODULE_LICENSE("GPL v2");
