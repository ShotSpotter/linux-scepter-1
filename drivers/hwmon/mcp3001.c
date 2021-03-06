/* 
   mcp3001 driver and sysfs based interface

   misc notes:
   According to the timing diagrams, the clock starts high,
   and data is available on the trailing (rising) edge.  
   conversion takes place on the leading (falling) edge.
   Thus, CPOL=1 and CPHA=1, for SPI mode 3 (defined in platform data)
   
   10bit data, default CS/AL and MSB first.

   Correct CS handling (by the SPI core?) will be necessary.
   if powered up with CS low, it must be toggled high/low to begin comms.
   if held low past 10 bits of output, output is repeated with LSB first.
   if still held low 0's are output.
   CS can be raised to end comms. at any point.
*/

#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#define DRVNAME  "mcp3k1"

#define ADC_MUX_ENABLE_GPIO	101
#define ADC_MUX_A0_GPIO		102
#define ADC_MUX_A1_GPIO		103
#define ADC_MUX_A2_GPIO		104
#define ADC_MUX_A3_GPIO		105
#define ADC_MUX_A4_GPIO		72

struct mcp3k1 {
	struct device *hwmon_dev;
	struct mutex lock;
	struct spi_device *spi;
};

// which sub-parameter we are querying.  set by writing sysfs.
static int index;

/* forward declarations */
static int mcp3k1_attr_index(struct attribute *attr);
static ssize_t mcp3k1_show(struct device *dev, struct device_attribute *attr,
			   char *buf);
static ssize_t mcp3k1_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count);

static void __mcp3k1_store(int index) {
	gpio_set_value(ADC_MUX_A0_GPIO, ((index & (1 << 0))?1:0));
	gpio_set_value(ADC_MUX_A1_GPIO, ((index & (1 << 1))?1:0));
	gpio_set_value(ADC_MUX_A2_GPIO, ((index & (1 << 2))?1:0));
	gpio_set_value(ADC_MUX_A3_GPIO, ((index & (1 << 3))?1:0));
	gpio_set_value(ADC_MUX_A4_GPIO, ((index & (1 << 4))?1:0));
}

static ssize_t mcp3k1_store(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
	int c;
	c = sscanf(buf, "%du", &index);
	if (c > 0) {
		if (index >= 0 && index < (1 << 5)) {
			__mcp3k1_store(index);
		}
	}
	return count;
}

static DEVICE_ATTR(mcp3k1, 0666, mcp3k1_show, mcp3k1_store);

static ssize_t
name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DRVNAME);
}
static DEVICE_ATTR(name, 0444, name_show, NULL);

#define set_in(offset) \
static DEVICE_ATTR(in##offset##_input, 0440, mcp3k1_show, NULL)

set_in(00);
set_in(01);
set_in(02);
set_in(03);
set_in(04);
set_in(05);
set_in(06);
set_in(07);
set_in(08);
set_in(09);
set_in(10);
set_in(11);
set_in(12);
set_in(13);
set_in(14);
set_in(15);
set_in(16);
set_in(17);
set_in(18);
set_in(19);
set_in(20);
set_in(21);
set_in(22);
set_in(23);
set_in(24);
set_in(25);
set_in(26);
set_in(27);
set_in(28);
set_in(29);
set_in(30);
set_in(31);


#define set_in_arr(offset) &dev_attr_in##offset##_input.attr

static const struct attribute *mcp3k1_attrs[] = {
	set_in_arr(00),
	set_in_arr(01),
	set_in_arr(02),
	set_in_arr(03),
	set_in_arr(04),
	set_in_arr(05),
	set_in_arr(06),
	set_in_arr(07),
	set_in_arr(08),
	set_in_arr(09),
	set_in_arr(10),
	set_in_arr(11),
	set_in_arr(12),
	set_in_arr(13),
	set_in_arr(14),
	set_in_arr(15),
	set_in_arr(16),
	set_in_arr(17),
	set_in_arr(18),
	set_in_arr(19),
	set_in_arr(20),
	set_in_arr(21),
	set_in_arr(22),
	set_in_arr(23),
	set_in_arr(24),
	set_in_arr(25),
	set_in_arr(26),
	set_in_arr(27),
	set_in_arr(28),
	set_in_arr(29),
	set_in_arr(30),
	set_in_arr(31),
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group mcp3k1_attr_group = {
	.attrs = (struct attribute **) mcp3k1_attrs,
};


static int mcp3k1_attr_index(struct attribute *attr)
{
	int i;
	const struct attribute *p;

	for (i = 0; (p = mcp3k1_attrs[i]) != NULL; i++) {
		if (p == attr) {
			return i;
		}
	}
	return -1;
}

static ssize_t mcp3k1_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mcp3k1 *p_mcp3k1 = dev_get_drvdata(dev);
	u8 rxbuf[3];
	int val = 0, index;
	int millivolt;

        index = mcp3k1_attr_index(&attr->attr);
        if (index < 0) {
		dev_dbg(dev, "mcp3k1: invalid attribute index.\n");
		return 0;
        }

	mutex_lock(&p_mcp3k1->lock);
	__mcp3k1_store(index);
	msleep(100);
	spi_write_then_read(p_mcp3k1->spi, NULL, 0, rxbuf, 2);
	mutex_unlock(&p_mcp3k1->lock);

	val = ((rxbuf[0] & 0x1f) << 5) | ((rxbuf[1] & 0xf8) >> 3);
#if 0
	/* Return raw value */
	return sprintf(buf, "0x%0x", val);  // hack for now
#else
	millivolt = (val * 2500)/1024;
	return sprintf(buf, "%i", millivolt);
#endif
}

static int __devinit mcp3k1_probe(struct spi_device *spi)
{
	int status, err;
	struct mcp3k1 *p_mcp3k1;

	// CS == AL and MSB first are defaults, and correct for mcp3k1
	err = spi_setup(spi);
	if (err < 0) {
		return err;
	}

	p_mcp3k1 = kzalloc(sizeof *p_mcp3k1, GFP_KERNEL);
	if (!p_mcp3k1)
		return -ENOMEM;

	mutex_init(&p_mcp3k1->lock);

	p_mcp3k1->spi = spi;

	err = gpio_request(ADC_MUX_ENABLE_GPIO, "ADC Enable");
	if (err < 0) {
		return err;
	}
	err = gpio_request(ADC_MUX_A0_GPIO, "ADC A0");
	if (err < 0) {
		gpio_free(ADC_MUX_ENABLE_GPIO);
		return err;
	}
	err = gpio_request(ADC_MUX_A1_GPIO, "ADC A1");
	if (err < 0) {
		gpio_free(ADC_MUX_ENABLE_GPIO);
		gpio_free(ADC_MUX_A0_GPIO);
		return err;
	}
	err = gpio_request(ADC_MUX_A2_GPIO, "ADC A2");
	if (err < 0) {
		gpio_free(ADC_MUX_ENABLE_GPIO);
		gpio_free(ADC_MUX_A0_GPIO);
		gpio_free(ADC_MUX_A1_GPIO);
		return err;
	}
	err = gpio_request(ADC_MUX_A3_GPIO, "ADC A3");
	if (err < 0) {
		gpio_free(ADC_MUX_ENABLE_GPIO);
		gpio_free(ADC_MUX_A0_GPIO);
		gpio_free(ADC_MUX_A1_GPIO);
		gpio_free(ADC_MUX_A2_GPIO);
		return err;
	}
	err = gpio_request(ADC_MUX_A4_GPIO, "ADC A4");
	if (err < 0) {
		gpio_free(ADC_MUX_ENABLE_GPIO);
		gpio_free(ADC_MUX_A0_GPIO);
		gpio_free(ADC_MUX_A1_GPIO);
		gpio_free(ADC_MUX_A2_GPIO);
		gpio_free(ADC_MUX_A3_GPIO);
		return err;
	}
	gpio_export(ADC_MUX_ENABLE_GPIO, 0);
	gpio_direction_output(ADC_MUX_ENABLE_GPIO, 0);	/* Enable it */
	gpio_export(ADC_MUX_A0_GPIO, 0);
	gpio_direction_output(ADC_MUX_A0_GPIO, 0);
	gpio_export(ADC_MUX_A1_GPIO, 0);
	gpio_direction_output(ADC_MUX_A1_GPIO, 0);
	gpio_export(ADC_MUX_A2_GPIO, 0);
	gpio_direction_output(ADC_MUX_A2_GPIO, 0);
	gpio_export(ADC_MUX_A3_GPIO, 0);
	gpio_direction_output(ADC_MUX_A3_GPIO, 0);
	gpio_export(ADC_MUX_A4_GPIO, 0);
	gpio_direction_output(ADC_MUX_A4_GPIO, 0);

	p_mcp3k1->hwmon_dev = hwmon_device_register(&spi->dev);
	if (IS_ERR(p_mcp3k1->hwmon_dev)) {
		dev_dbg(&spi->dev, "mcp3k1: hwmon_device_register failed.\n");
		status = PTR_ERR(p_mcp3k1->hwmon_dev);
		goto hwmon_reg_failed;
	}

	dev_set_drvdata(&spi->dev, p_mcp3k1);


	status = device_create_file(&spi->dev, &dev_attr_mcp3k1);
	if (status != 0) {
		dev_dbg(&spi->dev, "mcp3k1: Device create file failed.\n");
hwmon_reg_failed:
		p_mcp3k1->spi = NULL;
		dev_set_drvdata(&spi->dev, NULL);
		kfree(p_mcp3k1);
		return status;
	}

	status = sysfs_create_group(&spi->dev.kobj, &mcp3k1_attr_group);
	if (status != 0) {
		dev_dbg(&spi->dev, "mcp3k1: sysfs create group failed.\n");
		hwmon_device_unregister(p_mcp3k1->hwmon_dev);
		p_mcp3k1->spi = NULL;
		dev_set_drvdata(&spi->dev, NULL);
		kfree(p_mcp3k1);
	}
	return status;
}

static int __devexit mcp3k1_remove(struct spi_device *spi)
{
	struct mcp3k1 *p_mcp3k1 = dev_get_drvdata(&spi->dev);

	sysfs_remove_group(&spi->dev.kobj, &mcp3k1_attr_group);
	device_remove_file(&spi->dev, &dev_attr_mcp3k1);
	hwmon_device_unregister(p_mcp3k1->hwmon_dev);
	dev_set_drvdata(&spi->dev, NULL);
	kfree(p_mcp3k1);

	return 0;
}

static struct spi_driver mcp3k1_driver = {
	.driver = {
		.name = DRVNAME,
		.owner = THIS_MODULE,
	},
	.probe    = mcp3k1_probe,
	.remove   = __devexit_p(mcp3k1_remove),
};

static int __init init_mcp3k1(void)
{
	return spi_register_driver(&mcp3k1_driver);
}

static void __exit cleanup_mcp3k1(void)
{
	spi_unregister_driver(&mcp3k1_driver);
}

module_init(init_mcp3k1);
module_exit(cleanup_mcp3k1);
MODULE_AUTHOR("HY Research");
MODULE_DESCRIPTION("driver and sysfs interface for the mcp3001");
MODULE_LICENSE("GPL");
