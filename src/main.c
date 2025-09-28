#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#define SPI_NODE DT_NODELABEL(spi00)

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

static struct spi_config spi_cfg = {
    .frequency = 1000000U,  // 1 MHz for reliable loopback test
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
    .cs = NULL,  // No chip select for loopback test
};

int main(void)
{
    uint8_t tx_data[] = {0xAA, 0x55, 0xF0, 0x0F, 0x12, 0x34, 0x56, 0x78};
    uint8_t rx_data[sizeof(tx_data)] = {0};

    printk("\n========================================\n");
    printk("SPI Loopback Test on XIAO nRF54L15\n");
    printk("MOSI (D10/P2.02) -> MISO (D9/P2.04)\n");
    printk("========================================\n\n");

    // Check if SPI device is ready
    if (!device_is_ready(spi_dev)) {
        printk("ERROR: SPI device not ready!\n");
        return -1;
    }

    printk("SPI device initialized successfully\n");
    printk("Starting loopback test...\n\n");

    // Prepare SPI buffers
    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data),
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = sizeof(rx_data),
    };
    struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    int counter = 0;
    while (1) {
        // Clear receive buffer
        memset(rx_data, 0, sizeof(rx_data));

        // Perform SPI transfer (simultaneous send and receive)
        int err = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

        printk("Test #%d:\n", ++counter);

        if (err != 0) {
            printk("  ERROR: SPI transfer failed (err %d)\n", err);
        } else {
            // Print transmitted data
            printk("  TX: ");
            for (int i = 0; i < sizeof(tx_data); i++) {
                printk("%02X ", tx_data[i]);
            }
            printk("\n");

            // Print received data
            printk("  RX: ");
            for (int i = 0; i < sizeof(rx_data); i++) {
                printk("%02X ", rx_data[i]);
            }
            printk("\n");

            // Check if loopback is working
            bool match = true;
            for (int i = 0; i < sizeof(tx_data); i++) {
                if (tx_data[i] != rx_data[i]) {
                    match = false;
                    break;
                }
            }

            if (match) {
                printk("  Result: SUCCESS - Data matches!\n");
            } else {
                printk("  Result: MISMATCH - Check wiring\n");
            }
        }

        printk("\n");
        k_sleep(K_SECONDS(2));
    }

    return 0;
}