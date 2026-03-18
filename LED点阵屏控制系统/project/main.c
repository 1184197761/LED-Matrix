#include "app/tf_test/issi_driverXY.h"
#include "app/tf_test/image_shows.h"
#include "app/tf_test/i2c_test.h"
#include "app/tf_test/ISSI_Drive.h"
#include "app/tf_test/uart_protocol.h"
#include "hal/timer_device.h"
#include "hal/spi_nor.h"
#include "hal/uart.h"
#include "typesdef.h"
#include "list.h"
#include "dev.h"
#include "sysheap.h"

#define RX_BUFF_LEN 256                // 接收缓存长度
#define FRAME_0 0xFA                   // 串口协议包头0位
#define FRAME_1 0xAC                   // 控制协议包头1位
#define FRAME_2 0xAA                   // OTA协议包头1位
#define FRAME_END 0xAF                 // 串口协议包尾
#define TIMER_CLK DEFAULT_SYS_CLK / 10 // 定时器周期
#define TIME_OUT_BIT (BAUD_RATE >> 10) // 串口超时中断位
#define REFRESH_PERIOD 10              // 屏幕刷新周期
#define OTA_CMD_LEN 256                // OTA协议长度
#define FLASH_ADDR 0x100000            // 闪存起始地址
#define MAX_DIGITAL 999                // 最大显示数字
#define OTA_NOT_DATA_LEN 14            // OTA非数据指令长度

u8 uart_irq_flag = FALSE;  // 串口中断标志位
u8 timer_irq_flag = FALSE; // 定时器中断标志位

/**
 * @brief 定时器中断服务函数
 * @param data 无
 * @param irq_flag 中断标志位
 */
void timer_irq_handler(u32 data, u32 irq_flag)
{
    if (TIMER_INTR_PERIOD & irq_flag) // 若为周期中断
        timer_irq_flag = TRUE;        // 定时器中断标志置1
}

/**
 * @brief 串口中断服务函数
 * @param irq 中断标志位
 * @param irq_data 无
 * @param param1 无
 * @param param2 无
 * @return 无
 */
s32 uart_irq_handler(u32 irq, u32 irq_data, u32 param1, u32 param2)
{
    if (UART_IRQ_FLAG_TIME_OUT & irq) // 若为超时中断
        uart_irq_flag = TRUE;         // 串口中断标志置1
    return 0;
}

/**
 * @brief 加载闪存图像
 * @param flash 闪存设备
 */
void load_image(struct spi_nor_flash *flash)
{
    u8 flash_buf[OTA_CMD_LEN]; // 闪存缓存区

    for (u32 end_addr = FLASH_ADDR;; end_addr += OTA_CMD_LEN) // 遍历闪存空间
    {
        spi_nor_read(flash, end_addr, flash_buf, OTA_CMD_LEN); // 读取到闪存缓存区

        if (flash_buf[OTA_FA] != FRAME_0) // 若非OTA指令
            break;                        // 退出循环

        TYPE_CM2 cm2 = flash_buf[OTA_CM2];   // 点阵屏显示指令
        u8 frameId = flash_buf[OTA_FRAMEID]; // 图像帧序号

        (*img_addr)[cm2] = (u8 *)os_realloc((*img_addr)[cm2], IMAGE_SIZE * frameId);                // 申请对应图像内存
        (*img_len)[cm2] = frameId;                                                                  // 更新图像帧数
        os_memcpy((*img_addr)[cm2] + (frameId - 1) * IMAGE_SIZE, flash_buf + OTA_DATA, IMAGE_SIZE); // 复制数据到图像内存
    }
}

/**
 * @brief 串口控制指令处理
 * @param rx_buf 接收缓冲区
 * @param rx_tmp 接收暂存区
 * @param data 待显示数据
 */
void uart_ctrl(u8 *rx_buf, u8 *rx_tmp, u16 *data, TYPE_MODE *mode)
{
    if (rx_buf[CTRL_FA] != FRAME_0 || rx_buf[CTRL_AC] != FRAME_1 || rx_buf[CTRL_AF] != FRAME_END)
        return; // 若非控制指令

    TYPE_CM1 cm1 = rx_buf[CTRL_CM1]; // 指示灯指令
    TYPE_CM2 cm2 = rx_buf[CTRL_CM2]; // 点阵屏指令
    TYPE_CM3 cm3 = rx_buf[CTRL_CM3]; // 呼吸灯指令

    if ((rx_buf[CTRL_CRCH] << 8) + rx_buf[CTRL_CRCL] != gen_crc16(rx_buf, CTRL_CRCH))
        printf("CRC ERROR!\n"); // 若CRC校验错误

    if (rx_buf[CTRL_CM1] >= BIT_OR && rx_buf[CTRL_CM1] <= BIT_NOT) // 若指示灯指令有效
    {
        for (int bit = CTRL_WLIGHT; bit <= CTRL_RBLINK; bit++) // 遍历指示灯显示位
            if (cm1 == BIT_OR)                                 // 若为或指令
                rx_tmp[bit] |= rx_buf[bit];                    // 旧指令或上新指令
            else if (cm1 == BIT_AND)                           // 若为与指令
                rx_tmp[bit] = rx_buf[bit];                     // 新指令取代旧指令
            else if (cm1 == BIT_NOT)                           // 若为非指令
                rx_tmp[bit] ^= rx_buf[bit];                    // 旧指令异或新指令

        pilotLamp_display(rx_tmp, BRIGHTNESS); // 显示指示灯
    }

    if ((*img_len)[rx_buf[CTRL_CM2]]) // 若点阵屏指令有效
    {
        rx_tmp[CTRL_CM2] = rx_buf[CTRL_CM2]; // 保存指令

        if (cm2 == CountDown1 || cm2 == CountDown2)                 // 若为倒计时指令
            *data = (rx_buf[CTRL_DATAH] << 8) + rx_buf[CTRL_DATAL]; // 保存计时值
        *data = *data > MAX_DIGITAL ? MAX_DIGITAL : *data;          // 防止数字溢出

        if ((cm2 == Suspend && *mode != MODE_COUNTDOWN) ||
            cm2 == CountDown1 || cm2 == CountDown2)  // 若为倒计时指令或二次暂停
            *mode = MODE_COUNTDOWN;                  // 设置倒计时模式
        else if (cm2 >= BindData && cm2 <= Standby4) // 若为绑定数据或待机模式
            *mode = MODE_DIGITAL;                    // 设置数字显示模式
        else if ((*img_len)[cm2] > 1)                // 若为动态显示指令
            *mode = MODE_DYNAMIC;                    // 设置动态显示模式
        else                                         // 否则
            *mode = MODE_STATIC;                     // 设置静态显示模式

        screen_display(cm2, *data, *mode); // 显示点阵屏
    }

    if (rx_buf[CTRL_CM3]) // 若呼吸灯指令有效
    {
        rx_tmp[CTRL_CM3] = rx_buf[CTRL_CM3]; // 保存指令

        lightBar_display(TurnOff, 0);          // 熄灭呼吸灯
        if (cm3 != TurnOff)                    // 若非熄灭指令
            lightBar_display(cm3, BRIGHTNESS); // 显示呼吸灯
    }
}

/**
 * @brief 串口OTA指令处理
 * @param rx_buf 接收缓冲区
 * @param flash 闪存设备
 */
void uart_ota(u8 *rx_buf, struct spi_nor_flash *flash)
{
    if (rx_buf[OTA_FA] != FRAME_0 || rx_buf[OTA_AA] != FRAME_2)
        return; // 若非OTA指令

    TYPE_CM2 cm2 = rx_buf[OTA_CM2];                          // 点阵屏显示指令
    u16 cmdLen = (rx_buf[OTA_LENH] << 8) + rx_buf[OTA_LENL]; // 单包指令长度
    u8 frameId = rx_buf[OTA_FRAMEID];                        // 图像帧序号
    u8 dataLen = cmdLen - OTA_NOT_DATA_LEN;                  // 单包图像数据长度
    u8 ota_buf[IMAGE_SIZE];                                  // OTA暂存区
    u32 wirteAddr;                                           // 闪存写入地址
    static u8 dataSum = 0;                                   // 累计图像数据长度

    if ((rx_buf[OTA_CRCAH] << 8) + rx_buf[OTA_CRCAL] != gen_crc16(rx_buf, OTA_CRCAH))
        printf("CRCA ERROR!\n"); // CRC指令校验

    if ((rx_buf[cmdLen - 2] << 8) + rx_buf[cmdLen - 1] != gen_crc16(rx_buf, dataLen))
        printf("CRCB ERROR!\n"); // CRC数据校验

    os_memcpy(ota_buf + dataSum, rx_buf + OTA_DATA, dataLen); // 保存单包图像数据
    dataSum += dataLen;                                       // 累加图像数据长度

    if (rx_buf[OTA_PACKNUM] == rx_buf[OTA_SUBNUM]) // 若为最后一包数据
    {
        dataSum = 0; // 重置图像数据长度

        os_memcpy(rx_buf + OTA_DATA, ota_buf, IMAGE_SIZE);      // 整合完整OTA指令
        for (wirteAddr = FLASH_ADDR;; wirteAddr += OTA_CMD_LEN) // 遍历闪存空间
        {
            spi_nor_read(flash, wirteAddr, ota_buf, OTA_CM2);          // 读取到闪存缓存区
            if (ota_buf[OTA_CM2] == cm2 || ota_buf[OTA_FA] != FRAME_0) // 寻找相同指令直到空地址
                break;                                                 // 退出循环，得到写入地址
        }
        spi_nor_write(flash, wirteAddr, rx_buf, OTA_CMD_LEN); // 将OTA指令写入闪存

        (*img_addr)[cm2] = (u8 *)os_realloc((*img_addr)[cm2], IMAGE_SIZE * frameId);             // 申请对应图像内存
        (*img_len)[cm2] = frameId;                                                               // 更新图像帧数
        os_memcpy((*img_addr)[cm2] + (frameId - 1) * IMAGE_SIZE, rx_buf + OTA_DATA, IMAGE_SIZE); // 复制数据到图像内存

        display(rx_buf + OTA_DATA, FULL_WIDTH, 0); // 显示OTA图像帧
    }
}

/**
 * @brief 定时器显示更新
 * @param rx_tmp 接收暂存区
 * @param data 待显示数据
 */
void timer_updata(u8 *rx_tmp, u16 *data, TYPE_MODE mode)
{
    static u8 refresh_count = 0;      // 初始化屏幕刷新计数
    static u8 brightness = MIN_LIGHT; // 初始化灯珠亮度
    static u8 brighten = 0;           // 初始化调光方向
    TYPE_CM2 cm2 = rx_tmp[CTRL_CM2];  // 点阵屏指令
    TYPE_CM3 cm3 = rx_tmp[CTRL_CM3];  // 呼吸灯指令

    sysheap_init(SYS_HEAP_START, SYS_HEAP_SIZE, 0);

    if (brightness == MAX_LIGHT || brightness == MIN_LIGHT)          // 若亮度最大或最小
        brighten = !brighten;                                        // 反转调光方向
    brightness = brighten ? (brightness << 1) + 1 : brightness >> 1; // 亮度加倍或减半

    if (cm3 != WLight && cm3 != RLight && cm3 != TurnOff && cm3 != Max) // 若呼吸灯闪烁
        lightBar_display(cm3, brightness);                              // 更新呼吸灯

    if (rx_tmp[CTRL_RBLINK] || rx_tmp[CTRL_WBLINK]) // 若指示灯闪烁
        pilotLamp_display(rx_tmp, brightness);      // 更新指示灯

    if (!refresh_count--) // 若屏幕刷新计数结束
    {
        if (mode == MODE_COUNTDOWN && *data)        // 若为计时模式且未结束
            screen_display(Invalid, --*data, mode); // 更新屏幕计时
        else if (mode == MODE_DYNAMIC)              // 若为动图模式
            screen_display(cm2, 0, mode);           // 更新屏幕动画
        refresh_count = REFRESH_PERIOD;             // 重置屏幕刷新计数
    }
}

/**
 * @brief 主函数
 * @return 无
 */

int main()
{
    struct uart_device *uart = NULL;    // 串口设备
    struct timer_device *timer = NULL;  // 定时器设备
    struct spi_nor_flash *flash = NULL; // 闪存设备

    u8 rx_buf[RX_BUFF_LEN];        // 接收缓冲区
    u8 rx_tmp[RX_BUFF_LEN];        // 接收暂存区
    u16 data = 0;                  // 待显示数据
    TYPE_MODE mode = MODE_INVALID; // 点阵屏模式

    I2C_GPIO_Init();   // I2C端口初始化
    ISSI_Drive_Init(); // ISSi驱动初始化

    timer = (struct timer_device *)dev_get(HG_TIMER0_DEVID);          // 初始化定时器
    timer_device_open(timer, TIMER_TYPE_PERIODIC, TIMER_INTR_PERIOD); // 设置周期中断
    timer_device_start(timer, TIMER_CLK, timer_irq_handler, 0);       // 启用定时器中断

    uart = (struct uart_device *)dev_get(HG_UART0_DEVID);                // 初始化串口
    uart_open(uart, BAUD_RATE);                                          // 设置串口波特率
    uart_ioctl(uart, UART_IOCTL_CMD_SET_TIME_OUT, TIME_OUT_BIT, 1);      // 设置超时中断
    uart_ioctl(uart, UART_IOCTL_CMD_USE_DMA, TRUE, 0);                   // 启用DMA模式
    uart_gets(uart, rx_buf, RX_BUFF_LEN);                                // 设置DMA接收地址
    uart_request_irq(uart, uart_irq_handler, UART_IRQ_FLAG_TIME_OUT, 0); // 启用串口中断

    flash = (struct spi_nor_flash *)dev_get(HG_SPI_FLASH0_DEVID); // 初始化闪存
    spi_nor_open(flash);                                          // 启用闪存
    // spi_nor_block_erase(flash, FLASH_ADDR);                    // 闪存块擦除

    NVIC_SetPriority(UART0_IRQn, 1); // 设置串口中断优先级
    NVIC_SetPriority(TIM0_IRQn, 2);  // 设置定时器中断优先级

    load_image(flash); // 加载闪存图像

    while (TRUE)
    {
        if (uart_irq_flag) // 若触发串口中断
        {
            uart_irq_flag = FALSE;                   // 清除串口中断标志
            uart_gets(uart, rx_buf, RX_BUFF_LEN);    // 重置DMA起始位置
            uart_ctrl(rx_buf, rx_tmp, &data, &mode); // 串口控制指令处理
            uart_ota(rx_buf, flash);                 // 串口OTA指令处理
        }
        if (timer_irq_flag) // 若触发串定时器中断
        {
            timer_irq_flag = FALSE;            // 清除定时器中断标志
            timer_updata(rx_tmp, &data, mode); // 定时器图像更新
        }
    }
}
