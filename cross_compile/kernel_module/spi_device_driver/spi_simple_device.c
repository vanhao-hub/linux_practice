#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>     /* Defines functions such as gpio_request/gpio_free */    

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

static void send_data(struct spi_device *spi, u8 *data, u8 size)
{
    spi_write(spi, data, size);
}

static int spi_driver_probe(struct spi_device *spi)
{
    printk(KERN_INFO "SPI driver probed\n");
    spi_dev = spi;
	spi_dev->max_speed_hz = 1000000;
	spi_setup(spi_dev);
    // send_data(spi_dev, data_s, sizeof(data_s)); => test send raw data via spi
	LCD_Init();
    printk("vanhao__ da gui tin hieu spi \n");

    return 0;
}

static int spi_driver_remove(struct spi_device *spi)
{
    printk(KERN_INFO "SPI driver removed\n");
    spi_dev = NULL;
    return 0;
}

static const struct spi_device_id simple_spi_ids[] = {
    { "simple_spi_driver", 0 }, // 
    { }
};
MODULE_DEVICE_TABLE(spi, simple_spi_ids);

static struct spi_driver simple_spi_driver = {
    .driver = {
        .name = "simple_spi_driver",
        .owner = THIS_MODULE,
    },
    .probe = spi_driver_probe,
    .remove = spi_driver_remove,
    .id_table = simple_spi_ids, // 
};
static int __init simple_spi_init(void)
{
    int ret;
	printk(KERN_INFO "[vanhao]driver start to init \n");
	lcd_io_init();
    ret = spi_register_driver(&simple_spi_driver);
    if (ret < 0) {
        printk(KERN_ERR "Failed to register SPI driver\n");
        return ret;
    }

    printk(KERN_INFO "[vanhao]Simple SPI driver loaded\n");
    return 0;
}

static void __exit simple_spi_exit(void)
{
    spi_unregister_driver(&simple_spi_driver);
	gpio_free(gpio_dc);
	gpio_free(gpio_rst);
    printk(KERN_INFO "Simple SPI driver unloaded\n");
}

module_init(simple_spi_init);
module_exit(simple_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Simple SPI Driver");