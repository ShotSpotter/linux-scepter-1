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

static ssize_t mcp3k1_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mcp3k1 *p_mcp3k1 = dev_get_drvdata(dev);
	u8 rxbuf[3]; 
	int val = 0;
	int millivolt;

	spi_write_then_read(p_mcp3k1->spi, NULL, 0, rxbuf, 2);

	val = ((rxbuf[0] & 0x1f) << 5) | ((rxbuf[1] & 0xf8) >> 3);
#if 0
	/* Return raw value */
	return sprintf(buf, "0x%0x", val);  // hack for now
#else
	millivolt = (val * 2500)/1024;
	return sprintf(buf, "%i", millivolt);
#endif
}

static ssize_t mcp3k1_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int c;
	c = sscanf(buf, "%du", &index);
	if (c > 0) {
		if (index >= 0 && index < (1 << 5)) {
			gpio_set_value(ADC_MUX_A0_GPIO, ((index & (1 << 0))?1:0));
			gpio_set_value(ADC_MUX_A1_GPIO, ((index & (1 << 1))?1:0));
			gpio_set_value(ADC_MUX_A2_GPIO, ((index & (1 << 2))?1:0));
			gpio_set_value(ADC_MUX_A3_GPIO, ((index & (1 << 3))?1:0));
			gpio_set_value(ADC_MUX_A4_GPIO, ((index & (1 << 4))?1:0));
		}
	}
	return count;
}


// creates dev_attr_mcp3k1
static DEVICE_ATTR(mcp3k1, 0666, mcp3k1_show, mcp3k1_store);

static int __devinit mcp3k1_probe(struct spi_device *spi)
{
	int status, err;
	struct mcp3k1 *p_mcp3k1;

	// CS == AL and MSB first are defaults, and correct for mcp3k1
	err = spi_setup(spi);
	if (err < 0)
		return err;

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
	}
	return status;
}

static int __devexit mcp3k1_remove(struct spi_device *spi)
{
	struct mcp3k1 *p_mcp3k1 = dev_get_drvdata(&spi->dev);

	device_remove_file(&spi->dev, &dev_attr_mcp3k1);
	hwmon_device_unregister(p_mcp3k1->hwmon_dev);
	dev_set_drvdata(&spi->dev, NULL);
	kfree(p_mcp3k1);

	return 0;
}

static struct spi_driver mcp3k1_driver = {
	.driver = {
		.name = "mcp3k1",
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
