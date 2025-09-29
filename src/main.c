/*
 * XIAO nRF54L15 SPI Loopback Test
 * Tests SPI communication by connecting MOSI (D10) to MISO (D9)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(spi_loopback, LOG_LEVEL_INF);

/* Pin definitions matching XIAO nRF54L15 */
#define CS_NODE    DT_NODELABEL(gpio1)
#define CS_PIN     10  // P1.10 = D4

/* Test configuration */
#define TEST_PATTERN_SIZE 8
#define SPI_FREQUENCY 1000000  // Start with 1MHz for reliable operation

/* Test patterns */
static const uint8_t test_patterns[][TEST_PATTERN_SIZE] = {
    {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE},  // Pattern 1
    {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55},  // Alternating
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  // All ones
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // All zeros
    {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80},  // Walking bit
};

static uint8_t rx_buffer[TEST_PATTERN_SIZE];

/**
 * GPIO-based loopback test first (simpler, verifies physical connection)
 */
static int test_gpio_loopback(void)
{
    const struct device *gpio2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));
    
    if (!device_is_ready(gpio2)) {
        LOG_ERR("GPIO2 device not ready!");
        return -1;
    }
    
    LOG_INF("=== GPIO Loopback Test (D10->D9) ===");
    
    /* Configure D10 (P2.02) as output */
    int ret = gpio_pin_configure(gpio2, 2, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure P2.02 as output: %d", ret);
        return ret;
    }
    
    /* Configure D9 (P2.04) as input with pull-down */
    ret = gpio_pin_configure(gpio2, 4, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret < 0) {
        LOG_ERR("Failed to configure P2.04 as input: %d", ret);
        return ret;
    }
    
    /* Test high and low states */
    int errors = 0;
    
    LOG_INF("Testing GPIO HIGH...");
    gpio_pin_set(gpio2, 2, 1);
    k_msleep(10);
    int val = gpio_pin_get(gpio2, 4);
    if (val != 1) {
        LOG_ERR("  FAIL: Expected HIGH, got %s", val ? "HIGH" : "LOW");
        errors++;
    } else {
        LOG_INF("  PASS: D10=HIGH -> D9=HIGH");
    }
    
    LOG_INF("Testing GPIO LOW...");
    gpio_pin_set(gpio2, 2, 0);
    k_msleep(10);
    val = gpio_pin_get(gpio2, 4);
    if (val != 0) {
        LOG_ERR("  FAIL: Expected LOW, got %s", val ? "HIGH" : "LOW");
        errors++;
    } else {
        LOG_INF("  PASS: D10=LOW -> D9=LOW");
    }
    
    /* Rapid toggle test */
    LOG_INF("Rapid toggle test (10 cycles)...");
    int toggle_errors = 0;
    for (int i = 0; i < 10; i++) {
        int expected = i % 2;
        gpio_pin_set(gpio2, 2, expected);
        k_usleep(100);
        val = gpio_pin_get(gpio2, 4);
        if (val != expected) {
            toggle_errors++;
        }
    }
    LOG_INF("  Toggle test: %d errors in 10 cycles", toggle_errors);
    errors += toggle_errors;
    
    /* Reset pins to disconnected state */
    gpio_pin_configure(gpio2, 2, GPIO_DISCONNECTED);
    gpio_pin_configure(gpio2, 4, GPIO_DISCONNECTED);
    
    return (errors == 0) ? 0 : -1;
}

/**
 * SPI loopback test with comprehensive diagnostics
 */
static int test_spi_loopback(void)
{
    LOG_INF("=== SPI Loopback Test (MOSI->MISO) ===");
    
    /* Get SPI device directly without alias */
    const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi00));
    
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready!");
        return -1;
    }
    LOG_INF("SPI device ready (spi00)");
    
    /* Optional: Setup CS pin for manual control */
    const struct device *gpio1 = DEVICE_DT_GET(CS_NODE);
    if (device_is_ready(gpio1)) {
        gpio_pin_configure(gpio1, CS_PIN, GPIO_OUTPUT_INACTIVE);
        LOG_INF("CS pin configured (P1.10 = D4)");
    }
    
    /* SPI configuration */
    struct spi_config spi_cfg = {0};
    spi_cfg.frequency = SPI_FREQUENCY;
    spi_cfg.operation = SPI_OP_MODE_MASTER | 
                        SPI_TRANSFER_MSB |
                        SPI_WORD_SET(8) |
                        SPI_LINES_SINGLE;
    spi_cfg.slave = 0;
    
    LOG_INF("SPI Configuration:");
    LOG_INF("  Frequency: %d Hz", spi_cfg.frequency);
    LOG_INF("  Mode: 0 (CPOL=0, CPHA=0)");
    LOG_INF("  Word size: 8 bits");
    LOG_INF("  MSB first");
    
    /* Run tests with different patterns */
    int total_errors = 0;
    
    for (int pattern = 0; pattern < ARRAY_SIZE(test_patterns); pattern++) {
        LOG_INF("Testing pattern %d...", pattern + 1);
        
        /* Clear RX buffer */
        memset(rx_buffer, 0, sizeof(rx_buffer));
        
        /* Setup buffers */
        struct spi_buf tx_buf = {
            .buf = (void*)test_patterns[pattern],
            .len = TEST_PATTERN_SIZE
        };
        struct spi_buf rx_buf = {
            .buf = rx_buffer,
            .len = TEST_PATTERN_SIZE
        };
        struct spi_buf_set tx_set = {
            .buffers = &tx_buf,
            .count = 1
        };
        struct spi_buf_set rx_set = {
            .buffers = &rx_buf,
            .count = 1
        };
        
        /* Manual CS control if available */
        if (device_is_ready(gpio1)) {
            gpio_pin_set(gpio1, CS_PIN, 0);  // Assert CS
            k_usleep(10);
        }
        
        /* Perform SPI transaction */
        int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
        
        if (device_is_ready(gpio1)) {
            k_usleep(10);
            gpio_pin_set(gpio1, CS_PIN, 1);  // Deassert CS
        }
        
        if (ret < 0) {
            LOG_ERR("SPI transaction failed: %d", ret);
            total_errors++;
            continue;
        }
        
        /* Check results */
        LOG_HEXDUMP_DBG(test_patterns[pattern], TEST_PATTERN_SIZE, "TX");
        LOG_HEXDUMP_DBG(rx_buffer, TEST_PATTERN_SIZE, "RX");
        
        int match = memcmp(test_patterns[pattern], rx_buffer, TEST_PATTERN_SIZE);
        if (match == 0) {
            LOG_INF("  ✓ Pattern %d PASSED", pattern + 1);
        } else {
            LOG_ERR("  ✗ Pattern %d FAILED - Data mismatch", pattern + 1);
            total_errors++;
            
            /* Show detailed mismatch */
            for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
                if (test_patterns[pattern][i] != rx_buffer[i]) {
                    LOG_ERR("    Byte %d: TX=0x%02X, RX=0x%02X", 
                           i, test_patterns[pattern][i], rx_buffer[i]);
                }
            }
        }
        
        k_msleep(100);  // Short delay between patterns
    }
    
    return (total_errors == 0) ? 0 : -1;
}

/**
 * Diagnostic information
 */
static void print_diagnostics(void)
{
    LOG_INF("========================================");
    LOG_INF("XIAO nRF54L15 SPI Loopback Diagnostics");
    LOG_INF("========================================");
    LOG_INF("Pin Configuration:");
    LOG_INF("  SCK:  P2.01 (D8)  - Clock signal");
    LOG_INF("  MOSI: P2.02 (D10) - Master Out");
    LOG_INF("  MISO: P2.04 (D9)  - Master In");
    LOG_INF("  CS:   P1.10 (D4)  - Chip Select (optional)");
    LOG_INF("");
    LOG_INF("Required Connection:");
    LOG_INF("  Connect D10 to D9 with a jumper wire");
    LOG_INF("");
    LOG_INF("Troubleshooting:");
    LOG_INF("  - If GPIO test fails: Check jumper connection");
    LOG_INF("  - If SPI test fails but GPIO passes:");
    LOG_INF("    * Check device tree overlay is loaded");
    LOG_INF("    * Verify no pin conflicts with UART");
    LOG_INF("    * Try lower SPI frequency");
    LOG_INF("========================================\n");
}

int main(void)
{
    LOG_INF("Starting XIAO nRF54L15 SPI Loopback Test");
    print_diagnostics();
    
    /* Small delay for user to read instructions */
    LOG_INF("Starting tests in 3 seconds...");
    k_sleep(K_SECONDS(3));
    
    /* Run GPIO test first (simpler, verifies connection) */
    LOG_INF("\n--- Phase 1: GPIO Test ---");
    int gpio_result = test_gpio_loopback();
    if (gpio_result < 0) {
        LOG_ERR("GPIO loopback failed! Check D10->D9 connection");
        LOG_ERR("Cannot proceed with SPI test");
        return -1;
    }
    LOG_INF("GPIO test PASSED\n");
    
    /* Run SPI test */
    LOG_INF("--- Phase 2: SPI Test ---");
    int spi_result = test_spi_loopback();
    
    /* Summary */
    LOG_INF("\n========================================");
    LOG_INF("Test Summary:");
    LOG_INF("  GPIO Loopback: %s", gpio_result == 0 ? "PASS ✓" : "FAIL ✗");
    LOG_INF("  SPI Loopback:  %s", spi_result == 0 ? "PASS ✓" : "FAIL ✗");
    
    if (spi_result == 0) {
        LOG_INF("\nSPI communication is working correctly!");
        LOG_INF("You can now proceed with SCL3300 integration.");
    } else {
        LOG_INF("\nSPI communication failed.");
        LOG_INF("Please check the troubleshooting guide above.");
    }
    LOG_INF("========================================");
    
    /* Keep running for monitoring */
    while (1) {
        k_sleep(K_SECONDS(10));
    }
    
    return 0;
}