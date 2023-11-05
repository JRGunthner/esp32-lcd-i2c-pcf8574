#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"

#include "smbus.h"
#include "i2c-lcd1602.h"

#define TAG "app"

#define I2C_MASTER_SCL GPIO_NUM_2
#define I2C_MASTER_SDA GPIO_NUM_1
#define LCD1602_I2C_ADDRESS 0x3f

// LCD1602
// #define LCD_NUM_ROWS            2
// #define LCD_NUM_COLUMNS         32
// #define LCD_NUM_VISIBLE_COLUMNS 16

// LCD2004
#define LCD_NUM_ROWS               4
#define LCD_NUM_COLUMNS            80
#define LCD_NUM_VISIBLE_COLUMNS    20

// Undefine USE_STDIN if no stdin is available (e.g. no USB UART) - a fixed delay will occur instead of a wait for a
// keypress.
#define USE_STDIN 1

#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0  // disabled
#define I2C_MASTER_RX_BUF_LEN 0  // disabled
#define I2C_MASTER_FREQ_HZ    100000
#define I2C_MASTER_SDA_IO     I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO     I2C_MASTER_SCL

static void i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = I2C_MASTER_SDA_IO,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_io_num         = I2C_MASTER_SCL_IO,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = I2C_MASTER_FREQ_HZ
    };

    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_LEN, I2C_MASTER_TX_BUF_LEN, 0);
}

// uart_rx_one_char_block() causes a watchdog trigger, so use the non-blocking
// uart_rx_one_char() and delay briefly to reset the watchdog.
static uint8_t aguardar_uart_rx(void) {
    uint8_t c = 0;

#ifdef USE_STDIN
    while (!c) {
        STATUS s = uart_rx_one_char(&c);
        if (s == OK) {
            printf("%c", c);
        }
        vTaskDelay(1);
    }
#else
    vTaskDelay(1000 / portTICK_RATE_MS);
#endif
    return c;
}

void vLcdTask(void* pvParameter) {
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = LCD1602_I2C_ADDRESS;

    // Inicializa SMBus
    smbus_info_t* smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    // Inicializa o LCD com backlight desligado
    i2c_lcd1602_info_t* lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true, LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));

    ESP_LOGI(TAG, "desliga backlight");
    aguardar_uart_rx();
    i2c_lcd1602_set_backlight(lcd_info, false);

    ESP_LOGI(TAG, "Ligar backlight");
    aguardar_uart_rx();
    i2c_lcd1602_set_backlight(lcd_info, true);

    ESP_LOGI(TAG, "Ligar cursor");
    aguardar_uart_rx();
    i2c_lcd1602_set_cursor(lcd_info, true);

    ESP_LOGI(TAG, "Escreve 'A' em 0,0");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_char(lcd_info, 'A');

    ESP_LOGI(TAG, "Escreve 'B' em 8,0");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 8, 0);
    i2c_lcd1602_write_char(lcd_info, 'B');

    ESP_LOGI(TAG, "Escreve 'C' em 15,1");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 15, 1);
    i2c_lcd1602_write_char(lcd_info, 'C');

    ESP_LOGI(TAG, "Move o cursor para 0,1 e pisca. O cursor deve estar ligado");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_set_blink(lcd_info, true);

    ESP_LOGI(TAG, "Escreve 'DE' e move o cursor para tras no 'D'");
    aguardar_uart_rx();
    i2c_lcd1602_write_char(lcd_info, 'D');
    i2c_lcd1602_set_right_to_left(lcd_info);
    i2c_lcd1602_write_char(lcd_info, 'E');
    i2c_lcd1602_set_left_to_right(lcd_info);

    ESP_LOGI(TAG, "Desabilita display");
    aguardar_uart_rx();
    i2c_lcd1602_set_display(lcd_info, false);

    ESP_LOGI(TAG, "Escreve 'F' em 7,1 (display desabilitado)");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 7, 1);
    i2c_lcd1602_write_char(lcd_info, 'F');

    ESP_LOGI(TAG, "Habilita display");
    aguardar_uart_rx();
    i2c_lcd1602_set_display(lcd_info, true);

    ESP_LOGI(TAG, "Desabilita cursor piscante");
    aguardar_uart_rx();
    i2c_lcd1602_set_blink(lcd_info, false);

    ESP_LOGI(TAG, "Desabilita cursor");
    aguardar_uart_rx();
    i2c_lcd1602_set_cursor(lcd_info, false);

    ESP_LOGI(TAG, "Escreve todos caracteres alfanumericos apartir de 0,0");
    aguardar_uart_rx();
    i2c_lcd1602_home(lcd_info);
    i2c_lcd1602_write_string(lcd_info, "abcdefghijklmnopqrst" \
                                       "uvwxyz0123456789.,-+" \
                                       "ABCDEFGHIJKLMNOPQRST" \
                                       "UVWXYZ");

    ESP_LOGI(TAG, "Escreve 'Aqui tem agua pro chimarrao!' em 0,1");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_write_string(lcd_info, "   Aqui tem agua");
    i2c_lcd1602_move_cursor(lcd_info, 0, 2);
    i2c_lcd1602_write_string(lcd_info, "   pro chimarrao!");

    ESP_LOGI(TAG, "Rola o display para esquerda, 8 casas, devagar");
    aguardar_uart_rx();
    for (int i = 0; i < 8; ++i) {
        i2c_lcd1602_scroll_display_left(lcd_info);
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Rola o display para direita, 8 casas, imediatamente");
    aguardar_uart_rx();
    for (int i = 0; i < 8; ++i) {
        i2c_lcd1602_scroll_display_right(lcd_info);
    }

    ESP_LOGI(TAG, "Move para 8,0 e mostra o cursor");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 8, 0);
    i2c_lcd1602_set_cursor(lcd_info, true);

    ESP_LOGI(TAG, "Move o cursor 5 casas para a direita");
    aguardar_uart_rx();
    for (int i = 0; i < 5; ++i) {
        i2c_lcd1602_move_cursor_right(lcd_info);
    }

    ESP_LOGI(TAG, "Move o cursor 3 casas para a esquerda");
    aguardar_uart_rx();
    for (int i = 0; i < 3; ++i) {
        i2c_lcd1602_move_cursor_left(lcd_info);
    }

    ESP_LOGI(TAG, "Habilita auto-scroll e escreve '>>>>>'");
    aguardar_uart_rx();
    i2c_lcd1602_set_auto_scroll(lcd_info, true);
    for (int i = 0; i < 5; ++i) {
        i2c_lcd1602_write_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Muda endereco do contador para decrementar (escreve da direita para esquerda) e escreve '<<<<<'");
    aguardar_uart_rx();
    i2c_lcd1602_set_right_to_left(lcd_info);
    for (int i = 0; i < 5; ++i) {
        i2c_lcd1602_write_char(lcd_info, '<');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Desabilita auto-scroll e esceve '+++++'");
    aguardar_uart_rx();
    i2c_lcd1602_set_auto_scroll(lcd_info, false);
    for (int i = 0; i < 5; ++i) {
        i2c_lcd1602_write_char(lcd_info, '+');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Ajusta escrita de esquerda para direita e escreve '>>>>>'");
    aguardar_uart_rx();
    i2c_lcd1602_set_left_to_right(lcd_info);
    for (int i = 0; i < 5; ++i) {
        i2c_lcd1602_write_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Limpa display e o desabilita");
    aguardar_uart_rx();
    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_set_cursor(lcd_info, false);

    ESP_LOGI(TAG, "Cria caracteres especiais");
    aguardar_uart_rx();
    uint8_t bell[8] = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
    uint8_t note[8] = {0x2, 0x3, 0x2, 0xe, 0x1e, 0xc, 0x0};
    uint8_t clock[8] = {0x0, 0xe, 0x15, 0x17, 0x11, 0xe, 0x0};
    uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
    uint8_t duck[8] = {0x0, 0xc, 0x1d, 0xf, 0xf, 0x6, 0x0};
    uint8_t check[8] = {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0};
    uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
    uint8_t retarrow[8] = {0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4};
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_0, bell);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_1, note);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_2, clock);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_3, heart);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_4, duck);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_5, check);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_6, cross);
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_7, retarrow);

    // Após definir caracteres customizados, o endereço DDRAM deve ser definido por home() ou movendo o cursor
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_0);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_1);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_2);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_3);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_4);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_5);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_6);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_CUSTOM_7);

    ESP_LOGI(TAG, "Escreve os caracteres especiais");
    aguardar_uart_rx();
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_ALPHA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_BETA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_THETA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_PI);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_OMEGA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_SIGMA);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_INFINITY);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DEGREE);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_ARROW_LEFT);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_ARROW_RIGHT);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_SQUARE);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DOT);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_DIVIDE);
    i2c_lcd1602_write_char(lcd_info, I2C_LCD1602_CHARACTER_BLOCK);

    ESP_LOGI(TAG, "Escreve todos os caracteres (loop)");
    aguardar_uart_rx();
    i2c_lcd1602_clear(lcd_info);
    i2c_lcd1602_set_cursor(lcd_info, true);
    uint8_t c = 0;
    uint8_t col = 0;
    uint8_t row = 0;
    while (1) {
        i2c_lcd1602_write_char(lcd_info, c);
        vTaskDelay(100 / portTICK_RATE_MS);
        ESP_LOGD(TAG, "col %d, row %d, char 0x%02x", col, row, c);
        ++c;
        ++col;
        if (col >= LCD_NUM_VISIBLE_COLUMNS) {
            ++row;
            if (row >= LCD_NUM_ROWS) {
                row = 0;
            }
            col = 0;
            i2c_lcd1602_move_cursor(lcd_info, col, row);
        }
    }

    vTaskDelete(NULL);
}

void app_main() {
    xTaskCreate(&vLcdTask, "vLcdTask", 4096, NULL, 0, NULL);
}
