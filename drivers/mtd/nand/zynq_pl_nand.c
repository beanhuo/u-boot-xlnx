#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/of_mtd.h>
#include <linux/debugfs.h>
#include <linux/mtd/nand_bch.h>

#include <linux/poll.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>


#define NS_OUTPUT_PREFIX "[micron_pnand]"
//#define ENABLE_DEBUG

#define NS_ERR(args...) \
	do { printk(KERN_ERR NS_OUTPUT_PREFIX " error: " args); } while(0)

#define NS_INFO(args...) \
	do { printk(KERN_INFO NS_OUTPUT_PREFIX " " args); } while(0)

#ifdef CONFIG_OF
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif

#define LLD_DRIVER_NAMD "MICRON_LLD_NAND"

#define	B0					(1 << 0)
#define	B1					(1 << 1)
#define	B2					(1 << 2)
#define	B3					(1 << 3)
#define	B4					(1 << 4)
#define	B5					(1 << 5)
#define	B6					(1 << 6)
#define	B7					(1 << 7)
#define	B8					(1 << 8)
#define	B9					(1 << 9)
#define	B10					(1 << 10)
#define	B11					(1 << 11)
#define	B12					(1 << 12)
#define	B13					(1 << 13)
#define B15					(1 << 15)
//NAND_POWER_LOSS register
#define	B16					(1 << 16)
#define	B17					(1 << 17)
#define	B18					(1 << 18)
#define	B19					(1 << 19)

#define COMMAND_CR_BIT   (B10)
#define ADDRESS_CR_BIT   (B11)

#define STATUS_READY    0x40

#define CMD_STATUS	0x70

#define NAND_PULSE 		0x00
#define NAND_ADDR_LEN 		0x04
#define NAND_DATA 		0x08
#define NAND_CE_TIME 		0x0C
#define NAND_WE_TIME 		0x10
#define NAND_CLE_TIME 		0x14
#define NAND_ALE_TIME 		0x18
#define NAND_DQ_TIME 		0x1C
#define NAND_RE_TIME 		0x20
#define NAND_CYCLE_TIME 	0x24
#define NAND_STORBE_TIME 	0x28
#define NAND_CR  		0x2C
#define NAND_SR  		0x30
#define NAND_CLK 		0x34
#define NAND_DDL 		0x38
#define NAND_RD_LEN 		0x3C
#define NAND_CFG 		0x40
#define NAND_LENGTH 		0x44
#define NAND_CLK_DIVIDER	0x48
#define NAND_DDR_FIFO_COUNT	0x4c
#define NAND_SDR_FIFO_COUNT 0x50
#define NAND_VOLTAGE		0x54
#define NAND_POWER_LOSS		0x58
#define NAND_RESISTOR		0x5c
#define DEVICE_VERSION          0x78


struct lld_info {
	struct nand_chip	chip;
	struct mtd_info		mtd;
	struct mtd_partition    *parts;
	struct platform_device 	*pdev;


	int (*dev_ready)(struct mtd_info *mtd);
	u32 nr_parts;
	u8 ale;		/* address line number connected to ALE */
	u8 cle;		/* address line number connected to CLE */
	u8 width;	/* buswidth */
	u8 chip_delay;

	void __iomem		*nand_base;
	void __iomem		*fpga_regs;
	unsigned long		end_cmd_pending;
	unsigned long		end_cmd;
};
#define ENABLE_POWERLOSS_TEST 1

#ifdef ENABLE_POWERLOSS_TEST

static char *devName = "powerloss";
static unsigned int device_num = 0;
static struct class *powerloss_class;
//static struct fasync_struct *powerloss_fasync;
#define NOR_PROGRAM 1
#define NOR_ERASE 2
#define NOR_READ 3
#define NOR_FREE 4

#define START_P_POWERLOSS 5
#define START_E_POWERLOSS 6
#define START_R_POWERLOSS 7
#define START_NORMAL 8

static char currPowerlossSt = START_NORMAL;
static char initendflag = 0;
static char WERflag = 0;//write/read/erase flag 0,1,2
struct powerloss_cdev {
	struct cdev _cdev;
	struct fasync_struct *powerloss_fasync;
	int flag;
};
struct powerloss_cdev *pcdev;



void nand_PowerLoss(struct nand_chip *chip)
{
	u32 value;
	value = readl(chip->IO_ADDR_W + NAND_POWER_LOSS); 
	writel((value | B18 | B19) & (~B16) & (~B17),chip->IO_ADDR_W + NAND_POWER_LOSS); 
}

void nand_PowerOn(struct nand_chip *chip)
{

	u32 value;
	value = readl(chip->IO_ADDR_W + NAND_POWER_LOSS); 
	
	writel(((value & (~B19) & (~B18) & (~B17) & (~B16) &
		(~B7) &(~B6) & (~B5) & (~B4) & (~B3) &
		(~B2) & (~B1) & (~B0))| B16 | B17),		
		chip->IO_ADDR_W + NAND_POWER_LOSS); 
}


void nand_setvcc(struct nand_chip *chip,u32 voltage){
	u32 value;
    
	value = ((voltage * 255)/3300);
	writel(value,chip->IO_ADDR_W + NAND_VOLTAGE);
	writel(B6,chip->IO_ADDR_W + NAND_CR);
}
void nand_setvccq(struct nand_chip *chip,u32 mV){
	u32 value;
	value = ((mV * 255)/3300);
	writel(value << 16,chip->IO_ADDR_W + NAND_VOLTAGE);
	writel(B7,chip->IO_ADDR_W + NAND_CR);
}
static int powerloss_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper(fd, filp, on, &pcdev->powerloss_fasync);
	return 0;
}
static int powerloss_release(struct inode *inode,struct file *filp)
{
	powerloss_fasync(-1,filp,0);
	return 0;
}
ssize_t powerloss_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	if(size !=1 )
		return -EINVAL;
	copy_to_user(buf,&WERflag,1);
	return 1;

}
long powerloss_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	if(( cmd != START_P_POWERLOSS )&& ( cmd != START_E_POWERLOSS) &&
		( cmd != START_R_POWERLOSS) && ( cmd != START_NORMAL))
		return -EINVAL;

	switch(cmd)
		{

		case START_P_POWERLOSS:
			currPowerlossSt = START_P_POWERLOSS;
			printk("set powerloss mode: powerloss while programmimg.\n");
            break;

	    case START_R_POWERLOSS:
			currPowerlossSt = START_R_POWERLOSS;
			printk("set powerloss mode: powerloss while reading.\n");
			break;

		case START_E_POWERLOSS:
			currPowerlossSt = START_E_POWERLOSS;
			printk("set powerloss mode: powerloss while erasing.\n");
			break;


		case START_NORMAL:
			currPowerlossSt = START_NORMAL;
			printk("set powerloss mode: powerloss while noraml status.\n");
			break;
		}
	return err;
}

static const struct file_operations powerloss_cdev_fops = 
{
	.owner = THIS_MODULE,
	//.open = powerloss_open,
	//.close = powerloss_close,
	.unlocked_ioctl = powerloss_ioctl,
	.read = powerloss_read,
	.release = powerloss_release,
	.fasync = powerloss_fasync,
};

static int powerloss_cdev_init(void)
{
	int result;

	dev_t devno = MKDEV(device_num,0);

	if(device_num)
		result = register_chrdev_region(devno,1,devName);
	else
		{
		result = alloc_chrdev_region(&devno,0,1,devName);
		device_num = MAJOR(devno);
		}
	if(result < 0)
		return result;

	
	pcdev = kmalloc(sizeof(struct powerloss_cdev), GFP_KERNEL);
	if (!pcdev) {
		printk("%s:Couldn't powerloss cdev struct\n",devName);
		result = -ENOMEM;
		goto fail_malloc;
	}
	memset(pcdev, 0, sizeof(struct powerloss_cdev));
   
	cdev_init(&pcdev->_cdev,&powerloss_cdev_fops);
	pcdev->_cdev.owner = THIS_MODULE;
	pcdev->_cdev.ops = &powerloss_cdev_fops;

	cdev_add(&pcdev->_cdev,MKDEV(device_num,0),1);

	powerloss_class = class_create(THIS_MODULE, "powerloss");
	device_create(powerloss_class, NULL, MKDEV(device_num,0), NULL, "powerloss");
	initendflag = 1;
	return result;
	
fail_malloc:
	unregister_chrdev_region(devno, 1);
	return result;
	
}
static void powerloss_cdev_exit(void)
{

	device_destroy(powerloss_class, MKDEV(device_num, 0));
    class_destroy(powerloss_class);

	cdev_del(&pcdev->_cdev);
	kfree(pcdev);
	printk("====>device_nume is %d \n",device_num);
	unregister_chrdev_region(MKDEV(device_num, 0), 1);
	
}

#else
void nand_PowerLoss(struct nand_chip *chip)
{
return;
}
void nand_PowerLoss(struct nand_chip *chip)
	{
	return;
	}

void nand_setvcc(struct nand_chip *chip,double voltage)
	{
	return;
	}

void nand_setvccq(struct nand_chip *chip,double voltage)
	{
	return;
	}

#endif
static u8 chip_ready(struct nand_chip *chip)
{
	unsigned long timeo = jiffies + HZ;
	u8 ret=0;
	while((readl(chip->IO_ADDR_W + NAND_SR) & B4) == 0){
	if(time_after(jiffies, timeo)){
	ret = 1;
#ifdef ENABLE_DEBUG
	printk("\n====>nand controller ready timeout error.\n");
#endif
	break;
		}
			
	};
	return ret;
}

static void fpga_init(struct mtd_info *mtd)
{
	struct nand_chip *nc = mtd->priv;

	
    nand_setvcc(nc,3300);
	udelay(100);
	nand_setvccq(nc,3300);
		udelay(100);
	nand_PowerOn(nc);
		udelay(100);
	
	writel(0x00140001, nc->IO_ADDR_W + NAND_WE_TIME);
	writel(0x00280001, nc->IO_ADDR_W + NAND_CLE_TIME);
	writel(0x00280001, nc->IO_ADDR_W + NAND_ALE_TIME);	
	writel(0x00280001, nc->IO_ADDR_W + NAND_DQ_TIME);
	writel(0x00140001, nc->IO_ADDR_W + NAND_RE_TIME);
	writel(0x00270027, nc->IO_ADDR_W + NAND_CYCLE_TIME);
	writel(0x00000014, nc->IO_ADDR_W + NAND_STORBE_TIME);
	writel(0x00140014, nc->IO_ADDR_W + NAND_PULSE);
	
	
	writel(0x00, nc->IO_ADDR_W + NAND_CE_TIME);//CE low

	writeb(0xFF, nc->IO_ADDR_W + NAND_DATA);
    	writel(COMMAND_CR_BIT, nc->IO_ADDR_W + NAND_CR);

	writeb(0xFF, nc->IO_ADDR_W + NAND_DATA);
	writel(COMMAND_CR_BIT, nc->IO_ADDR_W + NAND_CR);
#ifdef PRINT_INFO
	printk("the device version is %04x\n",readl(nc->IO_ADDR_W + DEVICE_VERSION));	
#endif
}

static int lld_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	
	struct nand_chip *nc = mtd->priv;
	u32  config=0x00000000;
	
	if(cmd == NAND_CMD_NONE)
		return  ;
	
	if(ctrl & NAND_CLE){
		writeb(cmd, nc->IO_ADDR_W + NAND_DATA);// command
		config = COMMAND_CR_BIT;

#ifdef ENABLE_POWERLOSS_TEST

		if((cmd == 0xD0) && (initendflag == 1))
			{
	    WERflag = NOR_ERASE;
		kill_fasync(&pcdev->powerloss_fasync, SIGIO, POLL_HUP);
			}
		
#endif	
		}
	else if(ctrl & NAND_ALE){
		writeb(1, nc->IO_ADDR_W + NAND_ADDR_LEN);
		writel(cmd, nc->IO_ADDR_W + NAND_DATA);	//address
		config = ADDRESS_CR_BIT;
	 	}
	else		
		return;
	
#ifdef ENABLE_DEBUG
	printk("====>send %s: [0x%x] \n",ctrl & NAND_CLE ? "command":"address",cmd);
#endif
	writel(config, nc->IO_ADDR_W + NAND_CR);
	
	if(nc->options & NAND_BUSWIDTH_16) {
		return -ENOMEM;
	}
}

static void lld_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;
	void __iomem *io_base = chip->IO_ADDR_R;

	writel(len, chip->IO_ADDR_W + NAND_LENGTH);
	writel(B13, chip->IO_ADDR_W + NAND_CR);
       
	if(chip_ready(chip)){
#ifdef ENABLE_DEBUG
	printk("====>[%s][%d] error.\n",__FUNCTION__,__LINE__);
#endif
	return;
	}
	
#ifdef ENABLE_POWERLOSS_TEST
	if(initendflag == 1)
	 {
	WERflag = NOR_READ;
	kill_fasync(&pcdev->powerloss_fasync, SIGIO, POLL_HUP);
		}
#endif

	for(i = 0; i < len; i++)
	{
		buf[i] = (uint8_t)readl(io_base);
	}
#ifdef ENABLE_DEBUG
	printk(" \n====>read data:\n");
	for(i=0;i < len;i++)
	{
		printk("0x%x ",buf[i]);
		
		if((!((i+1)%10))&&(i))printk("\n");
	
	}
#endif
}
//static unsigned long long totalcount = 0;
static void lld_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;
	
	writel(len<<16, chip->IO_ADDR_W + NAND_LENGTH);
	
	for(i = 0; i < len; i++)
		writeb(buf[i],chip->IO_ADDR_W + NAND_DATA);
	
	writel(B12, chip->IO_ADDR_W + NAND_CR);
	
#ifdef ENABLE_POWERLOSS_TEST
   if(initendflag == 1)
   	{
	WERflag = NOR_PROGRAM;
	kill_fasync(&pcdev->powerloss_fasync, SIGIO, POLL_HUP);
	
   	}
#endif

	if(chip_ready(chip)){
#ifdef PRINT_ERROR
	printk("====>[%s][%d] error.\n",__FUNCTION__,__LINE__);
#endif
	return;
	}
}


static uint8_t lld_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem *io_base = chip->IO_ADDR_R;
	uint8_t ret;
	writel(1, chip->IO_ADDR_W + NAND_LENGTH);
	writel(B13, chip->IO_ADDR_W + NAND_CR);

	if(chip_ready(chip)){
#ifdef  PRINT_ERROR
	printk("====>[%s][%d] error.\n",__FUNCTION__,__LINE__);
#endif
	return  -ENOMEM;
	}
	ret = (uint8_t)readl(io_base);

#ifdef ENABLE_DEBUG
	printk("\n====>read byte [0x%x] \n",ret);
#endif
	return ret;
}	

/*
 * return 0 if the nand is busy, return 1 if ready
 */
static int lld_nand_device_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	return (readb(chip->IO_ADDR_W + NAND_SR) & B0) ;
}

#ifdef CONFIG_OF
static const struct of_device_id lldnand_of_mach[];
#endif
static struct platform_driver lld_driver;
struct lld_info *fnand;
struct resource *nand_res;

static int lldnand_probe(struct platform_device *pdev)
{
	int err = 0;
	int retval = -ENOMEM;
	//struct lld_info *fnand;
	struct mtd_info *mtd;
	struct nand_chip *nand_chip;
	//struct resource *nand_res;
	struct mtd_part_parser_data ppdata;
#ifdef CONFIG_OF
	const unsigned int *prop;
#endif

	fnand = devm_kzalloc(&pdev->dev, sizeof(*fnand), GFP_KERNEL);
	if(!fnand){
		dev_err(&pdev->dev, "failed to allocate device structure.\n");
		return -ENOMEM;
	}
	fnand->pdev = pdev;
	mtd = &fnand->mtd;
	mtd->name = "lld_nand";
	nand_chip = &fnand->chip;
	nand_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(nand_res == NULL) {
		err = -ENODEV;
		dev_err(&pdev->dev, "platform_get_resource for NAND failed\n");
		goto out_free_data;
	}

	nand_res = request_mem_region(nand_res->start, resource_size(nand_res), pdev->name);
	if(nand_res == NULL) {
		err = -EIO;
		dev_err(&pdev->dev, "request_mem_region for cont failed\n");
		goto out_free_data;
	}

	fnand->nand_base = ioremap(nand_res->start, resource_size(nand_res));
	if(fnand->nand_base == NULL) {
		err = -EIO;
		dev_err(&pdev->dev,"ioremap for NAND failed\n");
		goto out_release_nand_mem_region;
	}
#ifdef CONFIG_OF
	prop = of_get_property(pdev->dev.of_node, "xlnx,nand-width", NULL);
	if(prop) {
		if (be32_to_cpup(prop) == 16) {
			nand_chip->options |= NAND_BUSWIDTH_16;
		} else if (be32_to_cpup(prop) == 8) {
			nand_chip->options &= ~NAND_BUSWIDTH_16;
		} else {
			dev_info(&pdev->dev, "xlnx,nand-width not valid, using 8");
			nand_chip->options &= ~NAND_BUSWIDTH_16;
		}
	} else {
		dev_info(&pdev->dev, "xlnx,nand-width not in device tree, using 8");
		nand_chip->options &= ~NAND_BUSWIDTH_16;
	}	
#endif	
	nand_chip->priv = fnand;
	mtd->priv = nand_chip;
	mtd->owner = THIS_MODULE;

	nand_chip->IO_ADDR_R = fnand->nand_base + NAND_DATA;
	nand_chip->IO_ADDR_W = fnand->nand_base;
	nand_chip->cmd_ctrl = lld_nand_cmd_ctrl;
	nand_chip->dev_ready = lld_nand_device_ready;
	nand_chip->read_buf = lld_nand_read_buf;
	nand_chip->write_buf = lld_nand_write_buf;
	nand_chip->read_byte = lld_nand_read_byte;
	nand_chip->bbt_options |= NAND_BBT_USE_FLASH;
	nand_chip->options |= NAND_NO_SUBPAGE_WRITE;

	//platform_set_drvdata(pdev,mtd);
	platform_set_drvdata(pdev,fnand);

	fpga_init(mtd);

	retval = nand_scan_ident(mtd, 1, NULL);
	if(retval)
	{
		NS_ERR("cannot scan NAND Simulator device\n");
		if(retval > 0)
			retval = -ENXIO;
		goto no_dev;	
	}

	NS_INFO("using %u-bit/%u bytes BCH ECC\n", nand_chip->ecc_strength_ds, nand_chip->ecc_step_ds);
	nand_chip->ecc.mode = NAND_ECC_SOFT_BCH;
	nand_chip->ecc.size = nand_chip->ecc_step_ds;
	/*
	* Calculate ecc byte accroding ecc strength and ecc size.
	*/
	printk("====>nand_chip->ecc.size %d \n",nand_chip->ecc.size);
	int m= 0;
	while(1)
	{
		m++;
		if(((nand_chip->ecc.size * 8) >> m) == 0)
		{
			nand_chip->ecc.bytes = (nand_chip->ecc_strength_ds * m + 7)/8;
			pr_warn("strength_ds = %d, bytes = %d\n",nand_chip->ecc_strength_ds, nand_chip->ecc.bytes);
			break;
		}
	}
	nand_chip->ecc.strength = nand_chip->ecc.bytes * 8 / fls(8 * nand_chip->ecc.size);//bean add for 4.0
	/* second phase scan */
	retval = nand_scan_tail(mtd);
	if (retval) {
		printk("====>can't register NAND Simulator\n");
		if (retval > 0)
			retval = -ENXIO;
		goto no_dev;
	}

#ifdef CONFIG_OF
       ppdata.of_node = pdev->dev.of_node;
#endif

	err = mtd_device_parse_register(&fnand->mtd, NULL, &ppdata, NULL, 0);
	if(!err)
	{	
	powerloss_cdev_init();
	return 0;
	}	
	

	
	
out_release_nand_mem_region:
	release_mem_region(nand_res->start, resource_size(nand_res));
out_free_data:
	kfree(fnand);
	
no_dev:
	return retval;
}


static int lldnand_remove(struct platform_device *pdev)
{
#if 1
	struct lld_info *fnand = platform_get_drvdata(pdev);
	struct mtd_info *mtd = &fnand->mtd;
    struct resource *nand_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	nand_release(mtd);
	release_mem_region(nand_res->start, resource_size(nand_res));

	if(fnand->nand_base)
	iounmap(fnand->nand_base);

	powerloss_cdev_exit();
	//kfree(fnand);
#endif
	//platform_driver_unregister(&lld_driver);
	return 0;
}



static const struct of_device_id lldnand_of_mach[] = {
	{.compatible = "xlnx,ps7-fnand-1.00.a" },
	{},
};
MODULE_DEVICE_TABLE(of, lldnand_of_mach);


static struct platform_driver lld_driver = {
	.probe		= lldnand_probe,
	.remove		= lldnand_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= LLD_DRIVER_NAMD,
		.owner  = THIS_MODULE,
		.of_match_table = lldnand_of_mach, 
	},
};

#if 0
static int __init lld_nand_init(void)
{
	return platform_driver_register(&lld_driver);
}

static void __exit lld_nand_exit(void)
{
    //struct lld_info *fnand = platform_get_drvdata(&lld_driver);
	struct mtd_info *mtd = &fnand->mtd;
  //  struct resource *nand_res = platform_get_resource(&lld_driver, IORESOURCE_MEM, 0);
    nand_release(mtd);
  //  mtd_device_unregister(mtd);
	release_mem_region(nand_res->start, resource_size(nand_res));
   if(fnand->nand_base)
	iounmap(fnand->nand_base);
	
	//kfree(fnand);
	
	platform_driver_unregister(&lld_driver);
}
module_init(lld_nand_init);
module_exit(lld_nand_exit);

#endif
module_platform_driver(lld_driver);
MODULE_AUTHOR("BeanHuo@micron.com, Micorn.Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MTD nand controller driver with poserloss module for zynq zed");


