#include "sys_config.h"
#include "typesdef.h"
#include "list.h"
#include "dev.h"
#include "devid.h"
#include "osal/string.h"
#include "osal/semaphore.h"
#include "osal/mutex.h"
#include "osal/timer.h"
#include "osal/task.h"

#include "hal/uart.h"
#include "hal/i2c.h"
#include "hal/timer_device.h"
#include "hal/pwm.h"
#include "hal/capture.h"
#include "hal/dma.h"
#include "hal/netdev.h"
#include "hal/spi.h"
#include "hal/spi_nor.h"
#include "lib/ota/fw.h"
#include "lib/syscfg/syscfg.h"
#include "lib/net/ethphy/eth_mdio_bus.h"
#include "lib/net/ethphy/eth_phy.h"
#include "lib/net/ethphy/phy/ip101g.h"

#include "dev/uart/hguart_v2.h"
#include "dev/uart/hguart_v3.h"
#include "dev/i2c/hgi2c_dw.h"
#include "dev/sdio/hgsdio20_slave.h"
#include "dev/dma/dw_dmac.h"
#include "dev/gpio/hggpio_v3.h"
#include "dev/timer/hgtimer_v2.h"
#include "dev/dma/hg_m2m_dma.h"
#include "dev/usb/hgusb20_v1_dev_api.h"
//#include "dev/emac/hg_gmac_eva_v2.h"
#include "dev/wdt/hg_wdt_v1.h"
#include "dev/crc/hg_crc.h"
#include "dev/timer/hgtimer_v4.h"
#include "dev/timer/hgtimer_v5.h"
#include "dev/timer/hgtimer_v6.h"
#include "dev/pwm/hgpwm_v0.h"
#include "dev/capture/hgcapture_v0.h"
#include "dev/spi/hgspi_v1.h"
#include "dev/spi/hgspi_v2.h"
#include "dev/spi/hgspi_xip.h"
#include "dev/adc/hgadc_v0.h"
#include "dev/i2c/hgi2c.h"
#include "dev/sysaes/hg_sysaes_v2.h"
#include "dev/i2s/hgi2s_v0.h"
#include "dev/pdm/hgpdm_v0.h"
#include "dev/led/hgled_v0.h"
#include "dev/csi/hgdvp.h"
#include "dev/jpg/hgjpg.h"
#include "lib/sdhost/sdhost.h"
#include "drv_usart.h"
#include "device.h"
#include "syscfg.h"

extern const struct hgwphy_ah_cfg nphyahcfg;
extern union _dpd_ram dpd_ram;
extern const uint32 rx_imb_iq_normal[6];
extern const union hgdbgpath_cfg ndbgpathcfg;
extern const union hgrfmipi_cfg nrfmipicfg;
extern struct dma_device *m2mdma;

struct hgusb20_dev usb20_dev = {
    .usb_hw = (void *)HG_USB20_DEVICE_BASE,
    .ep_irq_num = USB20MC_IRQn,
    .dma_irq_num = USB20DMA_IRQn,

};

struct hgsdio20_slave sdioslave = {
    .hw = (void *)HG_SDIO20_SLAVE_BASE,
    .irq_num = SDIO_IRQn,
    .rst_irq_num = SDIO_RST_IRQn,
    .block_size = 64,
};

struct hg_crc crc32_module = {
    .hw = (void *)HG_CRC32_BASE,
    .irq_num = CRC_IRQn,
};

struct hg_sysaes_v2 sysaes = {
    .hw = (void *)HG_SYSAES_BASE,
    .irq_num = SYSAES_IRQn,
};

struct hguart_v2 uart0 = {
    .hw = HG_UART0_BASE,
    .irq_num = UART0_IRQn};
struct hguart_v2 uart1 = {
    .hw = HG_UART1_BASE,
    .irq_num = UART1_IRQn,
};

struct hguart_v3 uart4 = {
    .hw = HG_UART4_BASE,
    .irq_num = UART4_IRQn,
};

struct hguart_v3 uart5 = {
    .hw = HG_UART5_BASE,
    .irq_num = UART5_IRQn,
};

struct hggpio_v3 gpioa = {
    .hw = HG_GPIOA_BASE,
    .irq_num = GPIOA_IRQn,
    .pin_num = {PA_0, PA_15},
};

struct hggpio_v3 gpiob = {
    .hw = HG_GPIOB_BASE,
    .irq_num = GPIOB_IRQn,
    .pin_num = {PB_0, PB_15},
};

struct hggpio_v3 gpioc = {
    .hw = HG_GPIOC_BASE,
    .irq_num = GPIOC_IRQn,
    .pin_num = {PC_0, PC_7},
};

struct mem_dma_dev mem_dma = {
    .hw = (void *)M2M_DMA_BASE,
};

// struct hg_gmac_eva_v2 gmac = {
//     .hw      = HG_GMAC_BASE,
//     .irq_num = GMAC_IRQn,
//     .tx_buf_size = 4*1024,
//     .rx_buf_size = 8*1024,
//     .mdio_pin = PIN_GMAC_RMII_MDIO,
//     .mdc_pin  = PIN_GMAC_RMII_MDC,
// };

struct hg_wdt_v1 watchdog = {
    .hw = (void *)WDT_BASE,
    .irq_num = WDT_IRQn,
};

struct hgtimer_v4 timer0 = {
    .hw = TIMER0_BASE,
    .irq_num = TIM0_IRQn,
};

struct hgtimer_v4 timer1 = {
    .hw = TIMER1_BASE,
    .irq_num = TIM1_IRQn,
};

struct hgtimer_v4 timer2 = {
    .hw = TIMER2_BASE,
    .irq_num = TIM2_IRQn,
};

struct hgtimer_v5 led_timer0 = {
    .hw = LED_TIMER0_BASE,
    .irq_num = LEDTMR0_IRQn,
};

struct hgtimer_v6 super_timer0 = {
    .hw_comm = HG_SUPTMR_COMM_BASE,
    .hw_suptmrx = HG_SUPTMR0_BASE,
    .irq_num = SUPTMR0_IRQn,
};

struct hgpwm_v0 pwm = {
    .channel[0] = (void *)&timer0,
    .channel[1] = (void *)&timer1,
    .channel[2] = (void *)&timer2,
    .channel[3] = (void *)&super_timer0,
};

struct hgcapture_v0 capture = {
    .channel[0] = (void *)&timer0,
    .channel[1] = (void *)&timer1,
};

struct hgadc_v0 adc = {
    .hw = ADKEY_BASE,
    .irq_num = ADKEY_IRQn,
};

struct hgcqspi cqspi = {
    .hw = (void *)QSPI_BASE,
    .irq_num = QSPI_IRQn,
    .opened = 0,
};

struct hgspi_v1 spi0 = {
    .hw = SPI0_BASE,
    .irq_num = SPI0_IRQn,
};

struct hgspi_v1 spi1 = {
    .hw = SPI1_BASE,
    .irq_num = SPI1_IRQn,
};

struct hgspi_v1 spi2 = {
    .hw = SPI2_BASE,
    .irq_num = SPI2_IRQn,
};

struct hgspi_v2 spi5 = {
    .hw = SPI5_BASE,
    .irq_num = SPI5_IRQn,
};

struct hgspi_v2 spi6 = {
    .hw = SPI6_BASE,
    .irq_num = SPI6_IRQn,
};

struct hgspi_xip spi7 = {
    .hw = QSPI_BASE,
};

struct hgi2c iic1 = {
    .hw = IIC1_BASE,
    .irq_num = SPI1_IRQn,
};

struct hgi2c iic2 = {
    .hw = IIC2_BASE,
    .irq_num = SPI2_IRQn,
};

struct hgi2c iic3 = {
    .hw = IIC3_BASE,
    .irq_num = SPI3_IRQn,
};

struct hgdvp dvp = {
    .hw = DVP_BASE,
    .irq_num = DVP_IRQn,
};

struct hgjpg jpg = {
    .hw = JPG_BASE,
#ifdef FPGA_SUPPORT
    .thw = MJPEG_HUFF_MASK,
#else
    .thw = MJPEG_HUFF_BASE,
#endif
    .irq_num = JPG_IRQn,
};

struct hgsdh sdh = {
    .hw = SDHOST_BASE,
    .irq_num = SDHOST_IRQn,
};

struct hgi2s_v0 i2s0 = {
    .hw = HG_IIS0_BASE,
    .irq_num = IIS0_IRQn,
};

struct hgi2s_v0 i2s1 = {
    .hw = HG_IIS1_BASE,
    .irq_num = IIS1_IRQn,
};

struct hgpdm_v0 pdm = {
    .hw = HG_PDM_BASE,
    .irq_num = PDM_IRQn,
};

struct hgled_v0 led = {
    .hw = HG_LED_BASE,
    .irq_num = LED_IRQn,
};

// struct ethernet_mdio_bus mdio_bus0;
//
// struct ethernet_phy_device ethernet_phy0 = {
//     .addr = 0x00,
//     .drv  = &genphy_driver,
// };

#ifdef PSRAM_EN
struct spi_nor_flash flash0 = {
    .spidev = (struct spi_device *)&spi0,
    .spi_config = {12000000, SPI_CLK_MODE_0, SPI_WIRE_NORMAL_MODE, 0},
    .vendor_id = 0,
    .product_id = 0,
    .size = 0x200000,
    .block_size = 0x10000,
    .sector_size = 0x1000,
    .page_size = 256,
    .mode = SPI_NOR_NORMAL_SPI_MODE,
};
#else
struct spi_nor_flash flash0 = {
    .spidev = (struct spi_device *)&spi7,
    .spi_config = {12000000, SPI_CLK_MODE_0, SPI_WIRE_NORMAL_MODE, 0},
    .vendor_id = 0,
    .product_id = 0,
    .size = 0x200000,
    .block_size = 0x10000,
    .sector_size = 0x1000,
    .page_size = 4096,
    .mode = SPI_NOR_XIP_MODE,
};
#endif

void device_init(void)
{
    extern usart_handle_t console_handle;
    //    extern usart_handle_t console_handle0;
    hggpio_v3_attach(HG_GPIOA_DEVID, &gpioa);
    hggpio_v3_attach(HG_GPIOB_DEVID, &gpiob);
    hggpio_v3_attach(HG_GPIOC_DEVID, &gpioc);

    // hgi2s_v0_attach(HG_IIS0_DEVID, &i2s0);
    // hgi2s_v0_attach(HG_IIS1_DEVID, &i2s1);
    // hguart_v3_attach(HG_UART4_DEVID, &uart4);
    // hguart_v3_attach(HG_UART5_DEVID, &uart5);
    // hgspi_v2_attach(HG_SPI5_DEVID, &spi5);
    // hgspi_v2_attach(HG_SPI6_DEVID, &spi6);
    // hgpdm_v0_attach(HG_PDM_DEVID, &pdm);
    // hgled_v0_attach(HG_LED_DEVID, &led);
    hgtimer_v4_attach(HG_TIMER0_DEVID, &timer0);
    // hgtimer_v4_attach(HG_TIMER1_DEVID, &timer1);
    // hgtimer_v5_attach(HG_LED_TIMER0_DEVID, &led_timer0);
    // hgtimer_v6_attach(HG_SUPTMR0_DEVID, &super_timer0);
    // hgpwm_v0_attach(HG_PWM_DEVID, &pwm);
    // hgcapture_v0_attach(HG_CAPTURE_DEVID, &capture);
    // hgadc_v0_attach(HG_ADC1_DEVID, &adc1);
    hgadc_v0_attach(HG_ADC_DEVID, &adc);

    hguart_v2_attach(HG_UART0_DEVID, &uart0);
    hguart_v2_attach(HG_UART1_DEVID, &uart1);
#ifdef MACBUS_SDIO
    // hgsdio20_slave_attach(HG_SDIOSLAVE_DEVID, &sdioslave);
#endif
    hg_m2m_dma_dev_attach(HG_M2MDMA_DEVID, &mem_dma);
    m2mdma = (struct dma_device *)&mem_dma;

#ifdef ROM_FUNC_ENABLE
    /* excute again because mrom function not define func dev_register*/
    dev_register(HG_WPHY_DEVID, (struct dev_obj *)&wphy);
#endif

#if USB_EN
#if USB_HOST_EN
    hgusb20_host_attach(HG_USBDEV_DEVID, &usb20_dev);
#else
    hgusb20_dev_attach(HG_USBDEV_DEVID, &usb20_dev);
#endif
#endif

    // hguart_attach(HG_UART0_DEVID, &uart0);
#if DVP_EN
    hgdvp_attach(HG_DVP_DEVID, &dvp);
#endif

#if JPG_EN
    hgjpg_attach(HG_JPG_DEVID, &jpg);
#endif
#if SDH_EN
    hgsdh_attach(HG_SDC_HOST_DEVID, &sdh);
#endif
#if PWM_EN
    hgtimer_v4_attach(HG_TIMER0_DEVID, &timer0);
    hgtimer_v4_attach(HG_TIMER1_DEVID, &timer1);
    hgpwm_v0_attach(HG_PWM_DEVID, &pwm);
    hgcapture_v0_attach(HG_CAPTURE_DEVID, &capture);
#endif
    //    hg_gmac_v2_attach(HG_ETH_GMAC_DEVID, &gmac);

    hg_wdt_v1_attach(HG_WDTV1_DEVID, &watchdog);

    hg_crc_attach(HG_CRC_DEVID, &crc32_module);
    hg_sysaes_v2_attach(HG_SYSAES_DEVID, &sysaes);

    //    hgtimer_attach(HG_TIMER0_DEVID, &timer0);
    //    hgtimer_attach(HG_TIMER1_DEVID, &timer1);
    //    hgtimer_attach(HG_TIMER2_DEVID, &timer2);
    //    hgtimer_attach(HG_TIMER3_DEVID, &timer3);
    //    hgtimer_attach(HG_TIMER4_DEVID, &timer4);
    //    hgtimer_attach(HG_TIMER5_DEVID, &timer5);
    //    hgtimer_attach(HG_TIMER6_DEVID, &timer6);
    //    hgtimer_attach(HG_TIMER7_DEVID, &timer7);

    hgspi_v1_attach(HG_SPI0_DEVID, &spi0);
    hgspi_v1_attach(HG_SPI1_DEVID, &spi1);
    hgspi_xip_attach(HG_SPI7_DEVID, &spi7);
    //    hgspi_v1_attach(SPI3_DEVID, &spi3);
    spi_nor_attach(&flash0, HG_SPI_FLASH0_DEVID);

    //    hgi2c_attach(HG_I2C0_DEVID, &iic0);
    // hgi2c_attach(HG_I2C1_DEVID, &iic1);
    hgi2c_attach(HG_I2C2_DEVID, &iic2);
    //    hgi2c_attach(HG_I2C3_DEVID, &iic3);

    // hgcqspi_attach(HG_QSPI_DEVID, &cqspi);

    //    eth_mdio_bus_attach(HG_ETHERNET_MDIO_BUS0, &mdio_bus0);
    //    eth_phy_attach(HG_ETHERNET_PHY0, &ethernet_phy0);

    uart_open((struct uart_device *)&uart1, 115200);
    console_handle = (usart_handle_t)&uart1;

#if UART_COM_GD
    // uart_open((struct uart_device *)&uart0, 115200);
#endif
}

struct gpio_device *gpio_get(uint32 pin)
{
    if (pin >= gpioa.pin_num[0] && pin <= gpioa.pin_num[1])
    {
        return (struct gpio_device *)&gpioa;
    }
    else if (pin >= gpiob.pin_num[0] && pin <= gpiob.pin_num[1])
    {
        return (struct gpio_device *)&gpiob;
    }
    else if (pin >= gpioc.pin_num[0] && pin <= gpioc.pin_num[1])
    {
        return (struct gpio_device *)&gpioc;
    }
    return NULL;
}

int32 syscfg_info_get(struct syscfg_info *pinfo)
{
#if SYSCFG_ENABLE
    pinfo->flash1 = &flash0;
    pinfo->flash2 = &flash0;
    pinfo->size = pinfo->flash1->sector_size;
    pinfo->addr1 = pinfo->flash1->size - (2 * pinfo->size);
    pinfo->addr2 = pinfo->flash1->size - pinfo->size;
    ASSERT((pinfo->addr1 & ~(pinfo->flash1->sector_size - 1)) == pinfo->addr1);
    ASSERT((pinfo->addr2 & ~(pinfo->flash2->sector_size - 1)) == pinfo->addr2);
    ASSERT((pinfo->size >= sizeof(struct sys_config)) &&
           (pinfo->size == (pinfo->size & ~(pinfo->flash1->sector_size - 1))));
    return 0;
#else
    return -1;
#endif
}

int32 ota_fwinfo_get(struct ota_fwinfo *pinfo)
{
#if 1
    pinfo->flash0 = &flash0;
    pinfo->flash1 = &flash0;
    pinfo->addr0 = 0;
    pinfo->addr1 = 0x100000;
    return 0;
#else
    return -1;
#endif
}
