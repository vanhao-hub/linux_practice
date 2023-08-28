#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>     /* Defines functions such as gpio_request/gpio_free */    
#include <linux/platform_device.h>
#include <linux/fb.h>

#define gpio_dc		60
#define gpio_rst	61
#define LOW 		0
#define HIGH		1
#define W 320
#define H 240
#define VIDEOMEM_SIZE W * H * 2
#define LCD_W 320
#define LCD_H 240
#define WHITE       0xFFFF
#define RED         0xF800

static struct spi_device *spi_dev;
u8 data_s[] = {0x00, 0x01, 0x02};

void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);

static int lcd_io_init(void)
{
    gpio_request(gpio_dc, "pin_dc"); // set up gpio for dc pin
    gpio_request(gpio_rst, "pin_rst"); // set up gpio for rst pin
    gpio_direction_output(gpio_dc, LOW); // dc pin is output
    gpio_direction_output(gpio_rst, LOW); // rst pin is output
    pr_info("Init DC and RST pin output \n");
    return 0;
}

static void LCD_WR_REG(uint8_t byte) {
    // Command has DC low 
    gpio_set_value(gpio_dc, LOW);// dc pin
    spi_write(spi_dev, &byte, 1);
}

static void LCD_WR_DATA(uint8_t byte) {
    // Data has DC high 
    gpio_set_value(gpio_dc, HIGH);// dc pin
    spi_write(spi_dev, &byte, 1);
}

void LCD_WriteReg(u8 LCD_Reg, u8 LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA( LCD_RegValue);
}

void Lcd_WriteData_16Bit(u16 Data)
{
    u8 data_h = (u8)(Data>>8);
    u8 data_l = (u8)Data;
    // Data has DC high 
    gpio_set_value(gpio_dc, HIGH);// dc pin
	spi_write(spi_dev, &data_h, 1);
	spi_write(spi_dev, &data_l, 1);
}

void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(0x2C);
}

void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(0x2B);
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();
}

void LCD_Clear(u16 Color)
{
	unsigned int i;
	LCD_SetWindows(0,0,LCD_W-1,LCD_H-1);
	for(i=0;i<LCD_H*LCD_W;i++)
	{
		Lcd_WriteData_16Bit(Color);
	}
}

void LCD_RESET(void)
{
	gpio_set_value(gpio_rst, LOW);// rst pin
	msleep(50);
	gpio_set_value(gpio_rst, HIGH);// rst pin
	msleep(100);
}

void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);
}

void LCD_Init(void)
{
	LCD_RESET();
	LCD_WR_REG( 0xCF);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xC9);
	LCD_WR_DATA(0X30);
	LCD_WR_REG( 0xED);
	LCD_WR_DATA(0x64);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0X12);
	LCD_WR_DATA(0X81);
	LCD_WR_REG( 0xE8);
	LCD_WR_DATA(0x85);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x7A);
	LCD_WR_REG( 0xCB);
	LCD_WR_DATA(0x39);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x34);
	LCD_WR_DATA(0x02);
	LCD_WR_REG( 0xF7);
	LCD_WR_DATA(0x20);
	LCD_WR_REG( 0xEA);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_REG( 0xC0);    //Power control
	LCD_WR_DATA(0x1B);   //VRH[5:0]
	LCD_WR_REG( 0xC1);    //Power control
	LCD_WR_DATA(0x00);   //SAP[2:0];BT[3:0] 01
	LCD_WR_REG( 0xC5);    //VCM control
	LCD_WR_DATA(0x30); 	 //3F
	LCD_WR_DATA(0x30); 	 //3C
	LCD_WR_REG( 0xC7);    //VCM control2
	LCD_WR_DATA(0XB7);
	LCD_WR_REG( 0x36);    // Memory Access Control
	LCD_WR_DATA(0x08);
	LCD_WR_REG( 0x3A);
	LCD_WR_DATA(0x55);
	LCD_WR_REG( 0xB1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x1A);
	LCD_WR_REG( 0xB6);    // Display Function Control
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0xA2);
	LCD_WR_REG( 0xF2);    // 3Gamma Function Disable
	LCD_WR_DATA(0x00);
	LCD_WR_REG( 0x26);    //Gamma curve selected
	LCD_WR_DATA(0x01);
	LCD_WR_REG( 0xE0);    //Set Gamma
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x2A);
	LCD_WR_DATA(0x28);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x54);
	LCD_WR_DATA(0XA9);
	LCD_WR_DATA(0x43);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_REG( 0XE1);    //Set Gamma
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x15);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x2B);
	LCD_WR_DATA(0x56);
	LCD_WR_DATA(0x3C);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x0F);
	LCD_WR_REG( 0x2B);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG( 0x2A);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);
	LCD_WR_REG( 0x11); //Exit Sleep
	msleep(120);
	LCD_WR_REG( 0x29); //display on
	LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6)); //direction
	LCD_Clear(RED);
}

static struct spi_board_info spi_device_info = {
    .modalias = "virtual-spi-device",
    .max_speed_hz = 32000000, 
    .bus_num = 0,             
    .chip_select = 0,         
    .mode = SPI_MODE_0,       
};

int ili9341_setup(void)
{
	int i;
	struct spi_master *master;
	master = spi_busnum_to_master(spi_device_info.bus_num);
	if (!master) {
		printk(KERN_INFO "Failed to get SPI master\n");
        return -ENODEV;
    }
	spi_dev = spi_new_device(master, &spi_device_info);
	if (!spi_dev) {
		printk(KERN_INFO "Failed to create SPI device\n");
        spi_master_put(master);
        return -ENODEV;
    }
	lcd_io_init();
	LCD_Init();
	printk("ili9341_setup\n");
	return 0;
}

static struct fb_fix_screeninfo ili9341_fix  = {
		.type        = FB_TYPE_PACKED_PIXELS,
		.visual      = FB_VISUAL_TRUECOLOR,
		.accel       = FB_ACCEL_NONE,
		.line_length = W * 2,
};

static struct fb_var_screeninfo ili9341_var  = {
		.xres        = W,
		.yres        = H,
		.xres_virtual    = W,
		.yres_virtual    = H,
		.width        = W,
		.height        = H,
		.bits_per_pixel = 16,
		.red         = {11, 5, 0},
		.green         = {5, 6, 0},
		.blue         = {0, 5, 0},
		.activate     = FB_ACTIVATE_NOW,
		.vmode     = FB_VMODE_NONINTERLACED,
};
unsigned long pseudo_palette[17];

struct ili9341 {
	struct device *dev;
	struct fb_info *info;
	unsigned char *videomem;
};

static int ili9341_probe(struct platform_device *dev);
static int ili9341_remove(struct platform_device *device);
static void ili9341_update(struct fb_info *info, struct list_head *pagelist);
static int ili9341_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info);

static struct fb_ops ili9341_fbops = {
		.owner        = THIS_MODULE,
		.fb_write     = fb_sys_write,
		.fb_fillrect  = sys_fillrect,
		.fb_copyarea  = sys_copyarea,
		.fb_imageblit = sys_imageblit,
		.fb_setcolreg   = ili9341_setcolreg,
};

struct platform_driver ili9341_driver = {
		.probe = ili9341_probe,
		.remove = ili9341_remove,
		.driver = { .name = "my_fb_driver" }
};

static struct fb_deferred_io ili9341_defio = {
		.delay          = HZ / 25,
		.deferred_io    = &ili9341_update,
};
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)

static int ili9341_setcolreg(unsigned regno,
		unsigned red, unsigned green, unsigned blue,
		unsigned transp, struct fb_info *info)
{
	int ret = 1;
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 value;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			value = (red << info->var.red.offset) |
					(green << info->var.green.offset) |
					(blue << info->var.blue.offset) |
					(transp << info->var.transp.offset);

			pal[regno] = value;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}
	return ret;
}

static int  ili9341_probe(struct platform_device *dev)
{
	int ret = 0;
	struct ili9341 *item;
	struct fb_info *info;
	unsigned char  *videomemory;
	printk("ili9341_probe\n");

	item = kzalloc(sizeof(struct ili9341), GFP_KERNEL);
	if (!item) {
		printk(KERN_ALERT "unable to kzalloc for ili9341\n");
		ret = -ENOMEM;
		goto out;
	}
	item->dev = &dev->dev;
	dev_set_drvdata(&dev->dev, item);

	info = framebuffer_alloc(0, &dev->dev);
	if (!info) {
		ret = -ENOMEM;
		printk(KERN_ALERT "unable to framebuffer_alloc\n");
		goto out_item;
	}
	item->info = info;

	info->par = item;
	info->dev = &dev->dev;
	info->fbops = &ili9341_fbops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->fix = ili9341_fix;
	info->var = ili9341_var;
	info->fix.smem_len = VIDEOMEM_SIZE;
	info->pseudo_palette = &pseudo_palette;
	info->dev->kobj.name = "fb5";
	videomemory=vmalloc(info->fix.smem_len);
	if (!videomemory)
	{
		printk(KERN_ALERT "Can not allocate memory for framebuffer\n");
		ret = -ENOMEM;
		goto out_info;
	}
	info->fix.smem_start =(unsigned long)(videomemory);
	info->screen_base = (char __iomem *)info->fix.smem_start;
	item->videomem = videomemory;

	info->fbdefio = &ili9341_defio;
	fb_deferred_io_init(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		printk(KERN_ALERT "unable to register_frambuffer\n");
		goto out_pages;
	}

	if (ili9341_setup()) goto out_pages;
	return ret;

	out_pages:
	kfree(videomemory);
	out_info:
	framebuffer_release(info);
	out_item:
	kfree(item);
	out:
	return ret;
}

static int ili9341_remove(struct platform_device *device)
{
	struct fb_info *info = platform_get_drvdata(device);
	struct ili9341 *item = (struct ili9341 *)info->par;
	if (info) {
		unregister_framebuffer(info);
		framebuffer_release(info);
		kfree(item->videomem);
		kfree(item);
	}
	return 0;
}

static void ili9341_update(struct fb_info *info, struct list_head *pagelist)
{
	struct ili9341 *item = (struct ili9341 *)info->par;
	u16 *videomemory = (u16 *)item->videomem;
	int i;
	LCD_SetWindows(0,0,LCD_W-1,LCD_H-1);	
	for(i = 0; i < LCD_W * LCD_H; i++){
		Lcd_WriteData_16Bit(readw(videomemory));
		videomemory++;
	}
}

static struct platform_device *ili9341_device;
static int __init ili9341_init(void)
{
	int ret = 0;
	printk("ili9341_init\n");

	ret = platform_driver_register(&ili9341_driver);
	if (ret) {
		printk(KERN_ALERT "unable to platform_driver_register\n");

	}
	else{
		ili9341_device = platform_device_alloc("my_fb_driver", 0);
		if (ili9341_device){
			ret = platform_device_add(ili9341_device);
			printk("device added\n");
		}
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(ili9341_device);
			platform_driver_unregister(&ili9341_driver);
		}
	}

	return ret;
}
module_init(ili9341_init);

static void __exit ili9341_exit(void)
{
	platform_device_unregister(ili9341_device);
	platform_driver_unregister(&ili9341_driver);
	printk("ili9341_exit\n");
}

module_exit(ili9341_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Simple SPI Driver");