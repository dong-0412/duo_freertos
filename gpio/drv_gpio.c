#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
// #include "board.h"  // 需要根据实际情况修改包含文件

#define GPIO_SWPORTA_DR     0x00
#define GPIO_SWPORTA_DDR    0x04
#define GPIO_INTEN          0x30
#define GPIO_INTTYPE_LEVEL  0x38
#define GPIO_INT_POLARITY   0x3c
#define GPIO_INTSTATUS      0x40
#define GPIO_PORTA_EOI      0x4c
#define GPIO_EXT_PORTA      0x50

#define DWAPB_GPIOA_BASE    0x03020000
#define DWAPB_GPIOE_BASE    0x05021000
#define DWAPB_GPIO_SIZE     0x1000
#define DWAPB_GPIO_PORT_NR  5
#define DWAPB_GPIO_NR       32

#define PIN_NUM(port, no)   (((((port) & 0xFu) << 8) | ((no) & 0xFFu)))
#define PIN_PORT(pin)       ((uint8_t)(((pin) >> 8) & 0xFu))
#define PIN_NO(pin)         ((uint8_t)((pin) & 0xFFu))

#define BIT(x)              (1UL << (x))

static uint32_t dwapb_gpio_base = DWAPB_GPIOA_BASE;
static uint32_t dwapb_gpio_base_e = DWAPB_GPIOE_BASE;

static struct dwapb_event
{
    void (*(hdr[DWAPB_GPIO_NR]))(void *args);
    void *args[DWAPB_GPIO_NR];
    uint8_t is_both_edge[DWAPB_GPIO_NR];
} _dwapb_events[DWAPB_GPIO_PORT_NR];

static uint32_t dwapb_read32(uint32_t addr)
{
    return *(volatile uint32_t *)(addr);
}

static void dwapb_write32(uint32_t addr, uint32_t value)
{
    *(volatile uint32_t *)(addr) = value;
}

static void dwapb_toggle_trigger(uint8_t port, uint8_t bit)
{
    uint8_t val;
    uint32_t base_addr;
    uint32_t pol;

    base_addr = (port == 4 ? dwapb_gpio_base_e : (dwapb_gpio_base + DWAPB_GPIO_SIZE * port));

    pol = dwapb_read32(base_addr + GPIO_INT_POLARITY);
    /* Just read the current value right out of the data register */
    val = (uint8_t)((dwapb_read32(base_addr + GPIO_EXT_PORTA) >> (bit)) & 1);

    if (val)
        pol &= ~BIT(bit);
    else
        pol |= BIT(bit);

    dwapb_write32(base_addr + GPIO_INT_POLARITY, pol);
}

static void dwapb_pin_mode(uint32_t pin, uint8_t mode)
{
    uint8_t bit, port;
    uint32_t base_addr;
    uint32_t reg_val;

    bit = PIN_NO(pin);
    port = PIN_PORT(pin);
    base_addr = (port == 4 ? dwapb_gpio_base_e : (dwapb_gpio_base + DWAPB_GPIO_SIZE * port));

    reg_val = dwapb_read32(base_addr + GPIO_SWPORTA_DDR);
    switch (mode)
    {
    case 0: // PIN_MODE_OUTPUT
        reg_val |= BIT(bit);
        break;
    case 1: // PIN_MODE_INPUT
        reg_val &= ~BIT(bit);
        break;
    }

    dwapb_write32(base_addr + GPIO_SWPORTA_DDR, reg_val);
}

static void dwapb_pin_write(uint32_t pin, uint8_t value)
{
    uint8_t bit, port;
    uint32_t base_addr;
    uint32_t reg_val;

    bit = PIN_NO(pin);
    port = PIN_PORT(pin);
    base_addr = (port == 4 ? dwapb_gpio_base_e : (dwapb_gpio_base + DWAPB_GPIO_SIZE * port));

    reg_val = dwapb_read32(base_addr + GPIO_SWPORTA_DR);
    reg_val = (value ? (reg_val | BIT(bit)) : (reg_val & (~BIT(bit))));
    dwapb_write32(base_addr + GPIO_SWPORTA_DR, reg_val);
}

static int32_t dwapb_pin_read(uint32_t pin)
{
    uint8_t bit, port;
    uint32_t base_addr;

    bit = PIN_NO(pin);
    port = PIN_PORT(pin);
    base_addr = (port == 4 ? dwapb_gpio_base_e : (dwapb_gpio_base + DWAPB_GPIO_SIZE * port));

    uint32_t reg_val = dwapb_read32(GPIO_EXT_PORTA + base_addr);
    return ((reg_val >> (bit)) & 1);
}

static uint32_t dwapb_pin_get(const char *name)
{
    uint32_t pin = 0;
    int port_num, pin_num = 0;
    int i, name_len;

    name_len = strlen(name);

    if ((name_len < 2) || (name_len > 3))
    {
        goto out;
    }

    if ((name[0] >= 'A') && (name[0] <= 'E'))
    {
        port_num = (int)(name[0] - 'A');
    }
    else
    {
        goto out;
    }

    for (i = 1; i < name_len; i++)
    {
        pin_num *= 10;
        pin_num += name[i] - '0';
    }

    pin = PIN_NUM(port_num, pin_num);

    return pin;
out:
    printf("xy   x:A~E  y:0~31, e.g. C24\n");
    return -1;
}

static BaseType_t dwapb_pin_attach_irq(uint32_t pin, uint8_t mode, void (*hdr)(void *args), void *args)
{
    uint8_t bit, port;
    uint32_t base_addr;
    uint32_t it_val, ip_val;

    bit = PIN_NO(pin);
    port = PIN_PORT(pin);

    base_addr = (port == 4 ? dwapb_gpio_base_e : (dwapb_gpio_base + DWAPB_GPIO_SIZE * port));

    it_val = dwapb_read32(base_addr + GPIO_INTTYPE_LEVEL);
    ip_val = dwapb_read32(base_addr + GPIO_INT_POLARITY);

    if (mode == 1 || mode == 2) // PIN_IRQ_MODE_HIGH_LEVEL || PIN_IRQ_MODE_LOW_LEVEL
    {
        BaseType_t polarity = (mode == 1); // PIN_IRQ_MODE_HIGH_LEVEL

        /* Enable level detection */
        it_val = (it_val & (~BIT(bit)));
        /* Select polarity */
        ip_val = (polarity ? (ip_val | BIT(bit)) : (ip_val & (~BIT(bit))));
    }
    else if (mode == 3) // PIN_IRQ_MODE_RISING_FALLING
    {
        /* Disable level detection */
        it_val = (it_val | BIT(bit));
        /* Select both edges */
        dwapb_toggle_trigger(port, bit);
    }
    else if (mode == 4 || mode == 5) // PIN_IRQ_MODE_RISING || PIN_IRQ_MODE_FALLING
    {
        BaseType_t rising = (mode == 4); // PIN_IRQ_MODE_RISING

        /* Disable level detection */
        it_val = (it_val | BIT(bit));
        /* Select edge */
        ip_val = (rising ? (ip_val | BIT(bit)) : (ip_val & (~BIT(bit))));
    }
    else
    {
        /* No trigger: disable everything */
        it_val = (it_val & (~BIT(bit)));
        ip_val = (ip_val & (~BIT(bit)));
    }

    dwapb_write32(base_addr + GPIO_INTTYPE_LEVEL, it_val);
    if (mode != 3) // PIN_IRQ_MODE_RISING_FALLING
        dwapb_write32(base_addr + GPIO_INT_POLARITY, ip_val);

    _dwapb_events[PIN_PORT(pin)].hdr[PIN_NO(pin)] = hdr;
    _dwapb_events[PIN_PORT(pin)].args[PIN_NO(pin)] = args;
    _dwapb_events[PIN_PORT(pin)].is_both_edge[PIN_NO(pin)] = (mode == 3); // PIN_IRQ_MODE_RISING_FALLING

    return pdPASS;
}

static BaseType_t dwapb_pin_detach_irq(uint32_t pin)
{
    _dwapb_events[PIN_PORT(pin)].hdr[PIN_NO(pin)] = NULL;
    _dwapb_events[PIN_PORT(pin)].args[PIN_NO(pin)] = NULL;
    _dwapb_events[PIN_PORT(pin)].is_both_edge[PIN_NO(pin)] = 0;

    return pdPASS;
}

static BaseType_t dwapb_pin_irq_enable(uint32_t pin, uint8_t enabled)
{
    uint8_t bit, port;
    uint32_t base_addr;

    bit = PIN_NO(pin);
    port = PIN_PORT(pin);
    base_addr = (port == 4 ? dwapb_gpio_base_e : (dwapb_gpio_base + DWAPB_GPIO_SIZE * port));

    uint32_t reg_val = dwapb_read32(base_addr + GPIO_INTEN);
    reg_val = (enabled ? (reg_val | BIT(bit)) : (reg_val & (~BIT(bit))));
    dwapb_write32(base_addr + GPIO_INTEN, reg_val);

    return pdPASS;
}

static void vGpioISRHandler(void *pvParameters)
{
    uint8_t port;
    uint32_t base_addr;
    uint32_t pending, mask;

    mask = 0;
    port = (uint32_t)pvParameters;

    base_addr = (port == 4 ? dwapb_gpio_base_e : (dwapb_gpio_base + DWAPB_GPIO_SIZE * port));
    pending = dwapb_read32(base_addr + GPIO_INTSTATUS);

    if (pending)
    {
        uint32_t bit;

        for (bit = 0; bit < DWAPB_GPIO_NR; ++bit)
        {
            if (pending & BIT(bit))
            {
                mask = (mask | (BIT(bit)));

                if (_dwapb_events[port].hdr[bit] != NULL)
                {
                    _dwapb_events[port].hdr[bit](_dwapb_events[port].args[bit]);
                }

                if (_dwapb_events[port].is_both_edge[bit]) {
                    dwapb_toggle_trigger(port, bit);
                }
            }
        }
    }

    dwapb_write32(base_addr + GPIO_PORTA_EOI, mask);
}

void vSetupGpioInterrupts(void)
{
    xTaskCreate(vGpioISRHandler, "GPIO_ISR_0", configMINIMAL_STACK_SIZE, (void *)0, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(vGpioISRHandler, "GPIO_ISR_1", configMINIMAL_STACK_SIZE, (void *)1, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(vGpioISRHandler, "GPIO_ISR_2", configMINIMAL_STACK_SIZE, (void *)2, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(vGpioISRHandler, "GPIO_ISR_3", configMINIMAL_STACK_SIZE, (void *)3, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(vGpioISRHandler, "GPIO_ISR_SYS", configMINIMAL_STACK_SIZE, (void *)4, configMAX_PRIORITIES - 1, NULL);
}

void vInitGpio(void)
{
    vSetupGpioInterrupts();
}
