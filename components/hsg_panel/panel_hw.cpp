#include "panel_hw.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#if defined(BOARD_OLIMEX_POE)

static const char *TAG = "panel_hw";

static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static SemaphoreHandle_t s_draw_done;
static bool s_ready;

static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                          esp_lcd_panel_io_event_data_t *edata,
                                          void *user_ctx)
{
    (void)io;
    (void)edata;
    (void)user_ctx;
    BaseType_t hp = pdFALSE;
    if (s_draw_done) {
        xSemaphoreGiveFromISR(s_draw_done, &hp);
    }
    return hp == pdTRUE;
}

/* Extra power/gamma setup many ST7789 modules need beyond esp_lcd's minimal init. */
static void st7789_vendor_init(esp_lcd_panel_io_handle_t io)
{
    const struct {
        uint8_t cmd;
        uint8_t data[16];
        uint8_t len;
        uint16_t delay_ms;
    } cmds[] = {
        {0xB2, {0x0C, 0x0C, 0x00, 0x33, 0x33}, 5, 0},
        {0xB7, {0x35}, 1, 0},
        {0xBB, {0x19}, 1, 0},
        {0xC0, {0x2C}, 1, 0},
        {0xC2, {0x01}, 1, 0},
        {0xC3, {0x12}, 1, 0},
        {0xC4, {0x20}, 1, 0},
        {0xC6, {0x0F}, 1, 0},
        {0xD0, {0xA4, 0xA1}, 2, 0},
        {0xE0, {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23}, 14, 0},
        {0xE1, {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}, 14, 0},
        {LCD_CMD_NORON, {0}, 0, 10},
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); ++i) {
        esp_lcd_panel_io_tx_param(io, cmds[i].cmd,
                                  cmds[i].len ? cmds[i].data : NULL,
                                  cmds[i].len);
        if (cmds[i].delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }
}

esp_err_t panel_hw_draw_bitmap(int x0, int y0, int x1, int y1, const void *color)
{
    if (!s_panel || !color || x1 <= x0 || y1 <= y0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Drain any stale completion token before queueing. */
    if (s_draw_done) {
        while (xSemaphoreTake(s_draw_done, 0) == pdTRUE) {
        }
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, x0, y0, x1, y1, color);
    if (err != ESP_OK) {
        return err;
    }

    if (s_draw_done &&
        xSemaphoreTake(s_draw_done, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "SPI draw wait timeout");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void fill_bars(esp_lcd_panel_handle_t panel)
{
    (void)panel;
    uint16_t line[320];
    const uint16_t colors[3] = {0xF800, 0x07E0, 0x001F};
    for (int band = 0; band < 3; ++band) {
        for (int i = 0; i < 320; ++i) {
            line[i] = colors[band];
        }
        int y0 = band * 80;
        for (int y = y0; y < y0 + 80 && y < 240; ++y) {
            if (panel_hw_draw_bitmap(0, y, 320, y + 1, line) != ESP_OK) {
                ESP_LOGE(TAG, "fill_bars failed at y=%d", y);
                return;
            }
        }
    }
}

esp_err_t panel_hw_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = PANEL_TFT_SCLK_GPIO;
    buscfg.mosi_io_num = PANEL_TFT_MOSI_GPIO;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 320 * 40 * sizeof(uint16_t) + 8;

    esp_err_t err = spi_bus_initialize(PANEL_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * 7-pin ST7789 modules tie CS low on the PCB. With CS hard-wired, the
     * clock idle state must be high → SPI mode 3 (or 2). Mode 0 leaves the
     * panel blank while backlight still works.
     */
    s_draw_done = xSemaphoreCreateBinary();
    if (!s_draw_done) {
        ESP_LOGE(TAG, "draw semaphore alloc failed");
        return ESP_ERR_NO_MEM;
    }

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = PANEL_TFT_CS_GPIO;
    io_config.dc_gpio_num = PANEL_TFT_DC_GPIO;
    io_config.spi_mode = 3;
    io_config.pclk_hz = 20 * 1000 * 1000;
    io_config.trans_queue_depth = 4;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.on_color_trans_done = on_color_trans_done;

    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)PANEL_SPI_HOST, &io_config, &s_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel_io_spi failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = PANEL_TFT_RST_GPIO;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
    panel_config.data_endian = LCD_RGB_DATA_ENDIAN_LITTLE;
    panel_config.bits_per_pixel = 16;

    err = esp_lcd_new_panel_st7789(s_io, &panel_config, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "new_panel_st7789 failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    st7789_vendor_init(s_io);
    /* Invert OFF so authored RGB565 greens/blues display correctly (invert made green→purple). */
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, false));
    /* ST7789 native GRAM is 240x320; swap_xy gives logical 320x240 landscape. */
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, PANEL_TFT_X_GAP, PANEL_TFT_Y_GAP));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    vTaskDelay(pdMS_TO_TICKS(20));

#if PANEL_TFT_BL_GPIO >= 0
    {
        gpio_config_t bl = {};
        bl.pin_bit_mask = (1ULL << PANEL_TFT_BL_GPIO);
        bl.mode = GPIO_MODE_OUTPUT;
        gpio_config(&bl);
        gpio_set_level((gpio_num_t)PANEL_TFT_BL_GPIO, 1);
    }
#endif

    fill_bars(s_panel);
    vTaskDelay(pdMS_TO_TICKS(300));

    s_ready = true;
    ESP_LOGI(TAG, "ST7789 ready SPI2 mode=3 320x240 SCLK=%d MOSI=%d DC=%d RST=%d gap=(%d,%d)",
             PANEL_TFT_SCLK_GPIO, PANEL_TFT_MOSI_GPIO, PANEL_TFT_DC_GPIO, PANEL_TFT_RST_GPIO,
             PANEL_TFT_X_GAP, PANEL_TFT_Y_GAP);
    return ESP_OK;
}

esp_lcd_panel_handle_t panel_hw_panel(void)
{
    return s_panel;
}

bool panel_hw_ready(void)
{
    return s_ready;
}

#else

esp_err_t panel_hw_init(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_lcd_panel_handle_t panel_hw_panel(void) { return nullptr; }
bool panel_hw_ready(void) { return false; }
esp_err_t panel_hw_draw_bitmap(int x0, int y0, int x1, int y1, const void *color)
{
    (void)x0; (void)y0; (void)x1; (void)y1; (void)color;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif
