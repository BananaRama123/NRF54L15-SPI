/*
 * NRF54L15 SPI Loopback Test
 * Tests SPI communication by connecting MOSI (P2.02) to MISO (P2.04)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_test, LOG_LEVEL_INF);

#define SPI_NODE DT_ALIAS(spi_test)

// Check if SPI device exists in device tree
#if !DT_NODE_EXISTS(SPI_NODE)
#error "SPI device not found in device tree"
#endif

// Test data patterns
#define TEST_DATA_SIZE 16
static uint8_t tx_buffer[TEST_DATA_SIZE];
static uint8_t rx_buffer[TEST_DATA_SIZE];

// SPI configuration
static struct spi_config spi_cfg = {
    .frequency = 1000000,  // 1 MHz
    .operation = SPI_OP_MODE_MASTER | 
                 SPI_TRANSFER_MSB |
                 SPI_WORD_SET(8) |
                 SPI_LINES_SINGLE,
    .slave = 0,
    .cs = {NULL},  // We'll manage CS manually or not use it for loopback
};

// Simple CS control if needed (optional)
#if DT_NODE_HAS_PROP(SPI_NODE, cs_gpios)
static struct spi_cs_control cs_ctrl = {
    .gpio = GPIO_DT_SPEC_GET(SPI_NODE, cs_gpios),
    .delay = 0,
};
#endif

/**
 * Initialize test data patterns
 */
static void init_test_data(void)
{
    // Pattern 1: Sequential bytes
    for (int i = 0; i < TEST_DATA_SIZE; i++) {
        tx_buffer[i] = i;
    }
}

/**
 * Print buffer contents for debugging
 */
static void print_buffer(const char *name, uint8_t *buf, size_t len)
{
    LOG_INF("%s:", name);
    for (int i = 0; i < len; i++) {
        printk(" %02x", buf[i]);
        if ((i + 1) % 8 == 0) {
            printk("\n");
        }
    }
    if (len % 8 != 0) {
        printk("\n");
    }
}

/**
 * Perform SPI loopback test
 */
static int spi_loopback_test(const struct device *spi_dev)
{
    int ret;
    
    // Clear RX buffer
    memset(rx_buffer, 0, sizeof(rx_buffer));
    
    // Set up SPI buffer sets
    struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = TEST_DATA_SIZE
    };
    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = TEST_DATA_SIZE
    };
    
    struct spi_buf_set tx_set = {
        .buffers = &tx_buf,
        .count = 1
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1
    };
    
    LOG_INF("Starting SPI transceive...");
    print_buffer("TX", tx_buffer, TEST_DATA_SIZE);
    
    // Perform SPI transceive (simultaneous TX and RX)
    ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d", ret);
        return ret;
    }
    
    LOG_INF("SPI transceive complete");
    print_buffer("RX", rx_buffer, TEST_DATA_SIZE);
    
    // Compare TX and RX buffers
    bool match = true;
    for (int i = 0; i < TEST_DATA_SIZE; i++) {
        if (tx_buffer[i] != rx_buffer[i]) {
            match = false;
            LOG_ERR("Mismatch at index %d: TX=0x%02x, RX=0x%02x",
                   i, tx_buffer[i], rx_buffer[i]);
        }
    }
    
    if (match) {
        LOG_INF("✓ Loopback test PASSED - TX and RX data match!");
    } else {
        LOG_ERR("✗ Loopback test FAILED - Data mismatch");
    }
    
    return match ? 0 : -1;
}

/**
 * Test different SPI patterns
 */
static void run_pattern_tests(const struct device *spi_dev)
{
    LOG_INF("=== Test 1: Sequential Pattern ===");
    for (int i = 0; i < TEST_DATA_SIZE; i++) {
        tx_buffer[i] = i;
    }
    spi_loopback_test(spi_dev);
    
    k_sleep(K_MSEC(100));
    
    LOG_INF("=== Test 2: Alternating Pattern (0xAA/0x55) ===");
    for (int i = 0; i < TEST_DATA_SIZE; i++) {
        tx_buffer[i] = (i % 2) ? 0xAA : 0x55;
    }
    spi_loopback_test(spi_dev);
    
    k_sleep(K_MSEC(100));
    
    LOG_INF("=== Test 3: All Ones (0xFF) ===");
    memset(tx_buffer, 0xFF, TEST_DATA_SIZE);
    spi_loopback_test(spi_dev);
    
    k_sleep(K_MSEC(100));
    
    LOG_INF("=== Test 4: All Zeros (0x00) ===");
    memset(tx_buffer, 0x00, TEST_DATA_SIZE);
    spi_loopback_test(spi_dev);
    
    k_sleep(K_MSEC(100));
    
    LOG_INF("=== Test 5: Random Pattern ===");
    for (int i = 0; i < TEST_DATA_SIZE; i++) {
        tx_buffer[i] = (uint8_t)(k_cycle_get_32() & 0xFF);
    }
    spi_loopback_test(spi_dev);
}

int main(void)
{
    LOG_INF("NRF54L15 SPI Loopback Test Starting");
    LOG_INF("Connect MOSI (P2.02/D10) to MISO (P2.04/D9)");
    
    // Get SPI device
    const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -1;
    }
    LOG_INF("SPI device ready");
    
    // Set up CS control if available
    #if DT_NODE_HAS_PROP(SPI_NODE, cs_gpios)
    if (!gpio_is_ready_dt(&cs_ctrl.gpio)) {
        LOG_ERR("CS GPIO not ready");
        return -1;
    }
    
    // Configure CS pin as output, initially high (inactive)
    gpio_pin_configure_dt(&cs_ctrl.gpio, GPIO_OUTPUT_INACTIVE);
    spi_cfg.cs = &cs_ctrl;
    LOG_INF("CS pin configured");
    #else
    LOG_INF("No CS pin configured (loopback mode)");
    #endif
    
    // Initialize test data
    init_test_data();
    
    LOG_INF("Starting continuous loopback tests...");
    LOG_INF("========================================");
    
    int test_count = 0;
    while (1) {
        LOG_INF("Test iteration #%d", ++test_count);
        
        // Run various pattern tests
        run_pattern_tests(spi_dev);
        
        LOG_INF("========================================");
        LOG_INF("Waiting 2 seconds before next iteration...\n");
        k_sleep(K_SECONDS(2));
    }
    
    return 0;
}