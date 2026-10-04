#include "many_soft_AS5600.h"

#include "hal/time_hw.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include "core_riscv.h"

static uint32_t g_iic_delay_ticks = 1;

#define iic_delay() do { delayTicks32(g_iic_delay_ticks); } while (0)

// address
#define AS5600_write_address (0x36 << 1)
#define AS5600_read_address  ((0x36 << 1) + 1)

// regs
#define AS5600_raw_angle 0x0C
#define AS5600_status    0x0B

static inline void gpio_hi(GPIO_TypeDef* p, uint16_t pin) { p->BSHR = pin; }
static inline void gpio_lo(GPIO_TypeDef* p, uint16_t pin) { p->BCR  = pin; }
static inline uint32_t pin_index_u16(uint16_t pinMask)
{
    return (uint32_t)__builtin_ctz((uint32_t)pinMask);
}

static inline void gpio_set_cfg4(GPIO_TypeDef* p, uint16_t pinMask, uint32_t cfg4)
{
    uint32_t pin = pin_index_u16(pinMask);
    volatile uint32_t* cfg = (pin < 8) ? &p->CFGLR : &p->CFGHR;
    uint32_t shift = (pin & 7u) * 4u;
    uint32_t m = 0xFu << shift;
    uint32_t v = *cfg;
    v = (v & ~m) | ((cfg4 & 0xFu) << shift);
    *cfg = v;
}

AS5600_soft_IIC_many::AS5600_soft_IIC_many()
{
    numbers   = 0;

    // public
    online     = online_buf;
    magnet_stu = magnet_buf;
    raw_angle  = raw_buf;
    data       = data_buf;

    // private
    error    = error_buf;
    port_SDA = port_SDA_buf;
    port_SCL = port_SCL_buf;
    pin_SDA  = pin_SDA_buf;
    pin_SCL  = pin_SCL_buf;

    for (int i = 0; i < kMax; i++)
    {
        online_buf[i] = false;
        magnet_buf[i] = offline;
        raw_buf[i]    = 0;
        data_buf[i]   = 0;
        error_buf[i]  = 0;

        port_SDA_buf[i] = nullptr;
        port_SCL_buf[i] = nullptr;
        pin_SDA_buf[i]  = 0;
        pin_SCL_buf[i]  = 0;
    }
}

AS5600_soft_IIC_many::~AS5600_soft_IIC_many()
{
    // brak delete...
}

void AS5600_soft_IIC_many::enable_gpio_clock(GPIO_TypeDef* p)
{
    if      (p == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (p == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (p == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (p == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

void AS5600_soft_IIC_many::sda_mode_ipu(int i)
{
    // pull-up: set ODR=1 and CNF/MODE=1000b
    gpio_hi(port_SDA[i], pin_SDA[i]);
    gpio_set_cfg4(port_SDA[i], pin_SDA[i], 0x8u); // input pull-up/down
}

void AS5600_soft_IIC_many::sda_mode_od(int i)
{
    // output open-drain 50MHz: CNF=01, MODE=11 => 0b0111
    gpio_set_cfg4(port_SDA[i], pin_SDA[i], 0x7u);
    gpio_hi(port_SDA[i], pin_SDA[i]); // release high
}

void AS5600_soft_IIC_many::set_h(GPIO_TypeDef* const* port, const uint16_t* pin)
{
    for (int i = 0; i < numbers; i++)
        if (error[i] == 0) port[i]->BSHR = pin[i];
}

void AS5600_soft_IIC_many::set_l(GPIO_TypeDef* const* port, const uint16_t* pin)
{
    for (int i = 0; i < numbers; i++)
        if (error[i] == 0) port[i]->BCR = pin[i];
}

void AS5600_soft_IIC_many::init(GPIO_TypeDef* const* GPIO_SCL_port, const uint16_t* GPIO_SCL_pin,
                                GPIO_TypeDef* const* GPIO_SDA_port, const uint16_t* GPIO_SDA_pin,
                                int num)
{
    if (num <= 0) { numbers = 0; return; }
    if (num > kMax) num = kMax;
    numbers = num;

    // 4us delay w tickach policzone raz
    g_iic_delay_ticks = 4u * time_hw_ticks_per_us();
    if (!g_iic_delay_ticks) g_iic_delay_ticks = 1;

    for (int i = 0; i < numbers; i++)
    {
        port_SCL[i] = GPIO_SCL_port[i];
        pin_SCL[i]  = GPIO_SCL_pin[i];

        port_SDA[i] = GPIO_SDA_port[i];
        pin_SDA[i]  = GPIO_SDA_pin[i];

        magnet_stu[i] = offline;
        online[i]     = false;
        raw_angle[i]  = 0;
        data[i]       = 0;
        error[i]      = 0;

        enable_gpio_clock(port_SCL[i]);
        enable_gpio_clock(port_SDA[i]);
    }

    init_iic();
    updata_stu();
}

void AS5600_soft_IIC_many::clear_datas()
{
    for (int i = 0; i < numbers; i++)
    {
        error[i] = 0;
        data[i]  = 0;
    }
}

void AS5600_soft_IIC_many::init_iic()
{
    for (int i = 0; i < numbers; i++)
    {
        gpio_hi(port_SCL[i], pin_SCL[i]);
        gpio_hi(port_SDA[i], pin_SDA[i]);

        GPIO_InitTypeDef gi = {0};
        gi.GPIO_Speed = GPIO_Speed_50MHz;

        gi.GPIO_Mode = GPIO_Mode_Out_PP;
        gi.GPIO_Pin  = pin_SCL[i];
        GPIO_Init(port_SCL[i], &gi);

        gi.GPIO_Mode = GPIO_Mode_Out_OD;
        gi.GPIO_Pin  = pin_SDA[i];
        GPIO_Init(port_SDA[i], &gi);

        // idle high
        gpio_hi(port_SCL[i], pin_SCL[i]);
        gpio_hi(port_SDA[i], pin_SDA[i]);

        error[i] = 0;
    }
}

void AS5600_soft_IIC_many::start_iic(unsigned char ADR)
{
    iic_delay();
    set_h(port_SDA, pin_SDA);
    set_h(port_SCL, pin_SCL);
    iic_delay();
    set_l(port_SDA, pin_SDA);
    iic_delay();
    set_l(port_SCL, pin_SCL);

    write_iic((uint8_t)ADR);
}

void AS5600_soft_IIC_many::stop_iic()
{
    // ZAWSZE doprowadź do STOP/IDLE, nawet jak error[i]==1
    for (int i = 0; i < numbers; i++) {
        gpio_lo(port_SCL[i], pin_SCL[i]);
        gpio_lo(port_SDA[i], pin_SDA[i]);
    }
    iic_delay();
    for (int i = 0; i < numbers; i++) {
        gpio_hi(port_SCL[i], pin_SCL[i]);
    }
    iic_delay();
    for (int i = 0; i < numbers; i++) {
        gpio_hi(port_SDA[i], pin_SDA[i]);
    }
    iic_delay();
}

void AS5600_soft_IIC_many::write_iic(uint8_t byte)
{
    for (uint8_t m = 0x80; m; m >>= 1)
    {
        iic_delay();
        if (byte & m) set_h(port_SDA, pin_SDA);
        else          set_l(port_SDA, pin_SDA);

        set_h(port_SCL, pin_SCL);
        iic_delay();
        set_l(port_SCL, pin_SCL);
    }
    wait_ack_iic();
}

void AS5600_soft_IIC_many::read_iic(bool ack)
{
    // SDA jako INPUT_PU na czas odbioru
    for (int i = 0; i < numbers; i++)
        if (error[i] == 0) sda_mode_ipu(i);

    for (int bit = 0; bit < 8; bit++)
    {
        iic_delay();
        set_h(port_SCL, pin_SCL);
        iic_delay();

        for (int j = 0; j < numbers; j++)
        {
            data[j] <<= 1;
            if (port_SDA[j]->INDR & pin_SDA[j]) data[j] |= 0x01;
        }

        set_l(port_SCL, pin_SCL);
    }

    // wróć do OD żeby wysłać ACK/NACK
    for (int i = 0; i < numbers; i++)
        sda_mode_od(i);

    iic_delay();
    if (ack) set_l(port_SDA, pin_SDA);
    else     set_h(port_SDA, pin_SDA);

    iic_delay();
    set_h(port_SCL, pin_SCL);
    iic_delay();
    set_l(port_SCL, pin_SCL);
    iic_delay();
}

void AS5600_soft_IIC_many::wait_ack_iic()
{
    set_h(port_SDA, pin_SDA);               // puść SDA

    for (int i = 0; i < numbers; i++)       // ustaw INPUT zanim podniesiesz SCL
        if (error[i] == 0) sda_mode_ipu(i);

    iic_delay();
    set_h(port_SCL, pin_SCL);
    iic_delay();

    for (int i = 0; i < numbers; i++) {
        if (error[i] == 0) {
            if (port_SDA[i]->INDR & pin_SDA[i]) error[i] = 1; // NACK => 1
        }
    }

    set_l(port_SCL, pin_SCL);
    iic_delay();

    for (int i = 0; i < numbers; i++)       // wróć do OD
        sda_mode_od(i);
}

void AS5600_soft_IIC_many::read_reg8(uint8_t reg)
{
    if (!numbers) return;

    clear_datas();
    start_iic(AS5600_write_address);
    write_iic(reg);
    start_iic(AS5600_read_address);
    read_iic(false);
    stop_iic();
}

void AS5600_soft_IIC_many::read_reg16(uint8_t reg)
{
    if (!numbers) return;

    clear_datas();
    start_iic(AS5600_write_address);
    write_iic(reg);
    start_iic(AS5600_read_address);
    read_iic(true);
    read_iic(false);
    stop_iic();
}

void AS5600_soft_IIC_many::updata_stu()
{
    read_reg8(AS5600_status);

    for (int i = 0; i < numbers; i++)
    {
        online[i] = (error[i] == 0);

        if (error[i] != 0 || !(data[i] & 0x20)) magnet_stu[i] = offline;
        else
        {
            if      (data[i] & 0x10) magnet_stu[i] = low;
            else if (data[i] & 0x08) magnet_stu[i] = high;
            else                     magnet_stu[i] = normal;
        }
    }
}

uint8_t AS5600_soft_IIC_many::updata_angle()
{
    read_reg16(AS5600_raw_angle);
    uint8_t valid = 0u;

    for (int i = 0; i < numbers; i++)
    {
        if (error[i] == 0 && data[i] <= 4095u)
        {
            valid |= (uint8_t)(1u << i);
            raw_angle[i] = data[i];
            online[i] = true;
        }
        else
        {
            online[i] = false;
        }
    }
    return valid;
}
