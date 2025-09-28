#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// XIAO nRF54L15 has LED on P0.30 according to the wiki
#define LED0_PIN 30

void main(void)
{
    const struct device *gpio_dev;
    
    // Get GPIO device
    gpio_dev = device_get_binding("GPIO_0");
    if (!gpio_dev) {
        printk("Cannot find GPIO_0!\n");
        return;
    }
    
    // Configure LED pin as output
    gpio_pin_configure(gpio_dev, LED0_PIN, GPIO_OUTPUT_ACTIVE);
    
    printk("Starting 5Hz LED blink on XIAO nRF54L15\n");
    
    // Blink at 5Hz (toggle every 100ms = 5 complete cycles per second)
    while (1) {
        gpio_pin_toggle(gpio_dev, LED0_PIN);
        k_msleep(100);  // 100ms on, 100ms off = 5Hz
    }
}