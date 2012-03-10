#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Flagg");

#define GPIO_67 67
#define GPIO_68 68
#define GPIO_69 69

// We could cache the version number.
static char
get_version(void)
{
    char version;
    char rev1, rev2, rev3;

    rev1 = gpio_get_value(GPIO_67);
    rev2 = gpio_get_value(GPIO_68);
    rev3 = gpio_get_value(GPIO_69);
    version = rev1 | (rev2 << 1) | (rev3 << 1);

    return version;
}

static int
cpld_read_proc(char *page, char **start, off_t off, int count,
                                                    int *eof, void *data)
{
    *page = get_version();
    return 1;
}

static int
create_cpld_proc_entry(void)
{
    struct proc_dir_entry *entry;

    entry = create_proc_entry("cpld_version", 0644, NULL);
    if (entry == NULL)
        return -ENOMEM;

    entry->read_proc = cpld_read_proc;

    return 0;
}

static int __init
cpld_init(void)
{
    printk("[cpld] init\n");

    if (gpio_request(GPIO_67, "gpioGPIO_67") != 0)
        printk("[cpld] gpio_request(GPIO_67) failed.\n");
    if (gpio_request(GPIO_68, "gpioGPIO_68") != 0)
        printk("[cpld] gpio_request(GPIO_68) failed.\n");
    if (gpio_request(GPIO_69, "gpioGPIO_69") != 0)
        printk("[cpld] gpio_request(GPIO_69) failed.\n");

    if (gpio_direction_input(GPIO_67) != 0)
        printk("[cpld] gpio_direction_input(GPIO_67) failed.\n");
    if (gpio_direction_input(GPIO_68) != 0)
        printk("[cpld] gpio_direction_input(GPIO_68) failed.\n");
    if (gpio_direction_input(GPIO_69) != 0)
        printk("[cpld] gpio_direction_input(GPIO_69) failed.\n");

    return create_cpld_proc_entry();
}

static void __exit
cpld_exit(void) {
    printk("[cpld] exit.\n");

    gpio_free(GPIO_67);
    gpio_free(GPIO_68);
    gpio_free(GPIO_69);

    remove_proc_entry("cpld_version", NULL);
}

module_init(cpld_init);
module_exit(cpld_exit);
