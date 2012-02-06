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
#include <linux/mutex.h>
#include <linux/spi/spi.h>

#define DRVNAME  "mcp3k1"

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
  /*
  TODO: a bunch of this:
    spi_read(p_mcp3k1->spi, buf, 16);
  or, to control the mux:
    sprintf(txbuf, "%d", index);
    spi_write_then_read(p_mcp3k1->spi, const u8 *txbuf, unsigned n_tx, 
                        u8 *rxbuf, unsigned n_rx);
  or perhaps
    spi_w8r16()
  */
  return sprintf(buf, "%d", index);  // hack for now
}

static ssize_t mcp3k1_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
  return sscanf(buf, "%du", &index);
}


// creates dev_attr_mcp3k1
static DEVICE_ATTR(mcp3k1, 0666, mcp3k1_show, mcp3k1_store);

static int __devinit mcp3k1_probe(struct spi_device *spi)
{
  int status, err;
  struct mcp3k1 *p_mcp3k1;

  // CS == AL and MSB first are defaults, and correct for mcp3k1
  spi->mode = SPI_MODE_3;
  spi->bits_per_word = 10;
  err = spi_setup(spi);
  if (err < 0)
    return err;

  p_mcp3k1 = kzalloc(sizeof *p_mcp3k1, GFP_KERNEL);
  if (!p_mcp3k1)
    return -ENOMEM;

  mutex_init(&p_mcp3k1->lock);
  
  p_mcp3k1->spi = spi;
  p_mcp3k1->hwmon_dev = hwmon_device_register(&spi->dev);
  if (IS_ERR(p_mcp3k1->hwmon_dev)) {
    dev_dbg(&spi->dev, "mcp3k1: hwmon_device_register failed.\n");
    status = PTR_ERR(p_mcp3k1->hwmon_dev);
    goto hwmon_reg_failed;
  }
  dev_set_drvdata(&spi->dev, p_mcp3k1);

  status = device_create_file(&spi->dev, &dev_attr_mcp3k1);
  if (!status) {
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

static const struct spi_device_id mcp3k1_ids[] = {
  { "mcp3k1", 0 }
};
MODULE_DEVICE_TABLE(spi, mcp3k1_ids);

static struct spi_driver mcp3k1_driver = {
  .driver = {
    .name = "mcp3k1",
    .owner = THIS_MODULE,
  },
  .id_table = mcp3k1_ids,
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
