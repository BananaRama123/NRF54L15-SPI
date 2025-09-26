// main.c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define DAC_NODE DT_NODELABEL(spim20)
#define DAC_CS_NODE DT_NODELABEL(spi_cs)

static const struct device *spi_dev = DEVICE_DT_GET(DAC_NODE);

static struct spi_cs_control cs = {
    .gpio = GPIO_DT_SPEC_GET(DAC_CS_NODE, gpios),
    .delay = 0,
};

static struct spi_config spi_cfg = {
    .frequency = 10000000U,  // 10 MHz
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
    .cs = &cs,
};

void write_dac(uint16_t code, uint8_t command)
{
    uint16_t word = ((command & 0x0F) << 12) | (code & 0x0FFF);
    uint8_t tx_buf[2] = { word >> 8, word & 0xFF };
    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf),
    };
    struct spi_buf_set tx = {
        .buffers = &tx_spi_buf,
        .count = 1,
    };

    int err = spi_write(spi_dev, &spi_cfg, &tx);
    if (err == 0) {
        printk("DAC SPI write OK: 0x%04X\n", word);
    } else {
        printk("DAC SPI write failed: %d\n", err);
    }
}

void main(void)
{
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return;
    }
    if (!device_is_ready(cs.gpio.port)) {
        printk("CS GPIO port not ready\n");
        return;
    }

    // Full scale output to DAC A
    write_dac(0x0FFF, 0x3);  // MAX5532: command 0x3 = Write & Update DAC A

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}
