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
#include "lcd-i2c.h"

#define TAG "MAIN"

#define I2C_MASTER_SCL   GPIO_NUM_2
#define I2C_MASTER_SDA   GPIO_NUM_1
#define LCD_I2C_ENDERECO 0x3f

// LCD1602
// #define LCD_NUM_LINHAS  2
// #define LCD_NUM_COLUNAS 16
// #define LCD_NUM_CELULAS 32

// LCD2004
#define LCD_NUM_LINHAS  4
#define LCD_NUM_COLUNAS 20
#define LCD_NUM_CELULAS 80

// Utiliza stdin para aguardar o usuario pressionar uma tecla
// Se desabilitado, aguarda 1 segundo
#define USE_STDIN 1

#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0  // Desabilitado
#define I2C_MASTER_RX_BUF_LEN 0  // Desabilitado
#define I2C_MASTER_FREQ_HZ    100000
#define I2C_MASTER_SDA_IO     I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO     I2C_MASTER_SCL

static void i2c_master_init(void) {
    uint8_t i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode               = I2C_MODE_MASTER,
                         .sda_io_num         = I2C_MASTER_SDA_IO,
                         .scl_io_num         = I2C_MASTER_SCL_IO,
                         .sda_pullup_en      = GPIO_PULLUP_ENABLE,
                         .scl_pullup_en      = GPIO_PULLUP_ENABLE,
                         .master.clk_speed   = I2C_MASTER_FREQ_HZ
    };

    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_LEN, I2C_MASTER_TX_BUF_LEN, 0);
}

static uint8_t aguardar_uart_rx(void) {
    uint8_t c = 0;

#ifdef USE_STDIN
    while (!c) {
        STATUS s = uart_rx_one_char(&c);
        if (s == OK)
            printf("%c", c);
        vTaskDelay(1);
    }
#else
    vTaskDelay(1000 / portTICK_RATE_MS);
#endif  // USE_STDIN
    return c;
}

void vLcdTask(void* pvParameter) {
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = LCD_I2C_ENDERECO;

    // Inicializa SMBus
    smbus_t* smbus = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus, 1000 / portTICK_RATE_MS));

    // Inicializa o LCD com backlight desligado
    lcd_i2c_t* lcd_info = lcd_i2c_malloc();
    ESP_ERROR_CHECK(lcd_i2c_init(lcd_info, smbus, true, LCD_NUM_LINHAS, LCD_NUM_CELULAS, LCD_NUM_COLUNAS));

    ESP_ERROR_CHECK(lcd_i2c_init_config(lcd_info));

    ESP_LOGI(TAG, "Escreve 'Aqui tem agua pro chimarrao!' em 1,4");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 1, 4);
    lcd_i2c_printf(lcd_info, "Aqui tem agua");
    lcd_i2c_mover_cursor(lcd_info, 2, 4);
    lcd_i2c_printf(lcd_info, "pro chimarrao!");

    ESP_LOGI(TAG, "Limpa display e desabilita cursor");
    aguardar_uart_rx();
    lcd_i2c_limpar_display(lcd_info);
    lcd_i2c_config_cursor(lcd_info, false);

    ESP_LOGI(TAG, "desliga backlight");
    aguardar_uart_rx();
    lcd_i2c_backlight(lcd_info, false);

    ESP_LOGI(TAG, "Ligar backlight");
    aguardar_uart_rx();
    lcd_i2c_backlight(lcd_info, true);

    ESP_LOGI(TAG, "Ligar cursor");
    aguardar_uart_rx();
    lcd_i2c_config_cursor(lcd_info, true);

    ESP_LOGI(TAG, "Escreve 'A' em 0,0");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 0, 0);
    lcd_i2c_print_char(lcd_info, 'A');

    ESP_LOGI(TAG, "Escreve 'B' em 0,8");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 0, 8);
    lcd_i2c_print_char(lcd_info, 'B');

    ESP_LOGI(TAG, "Escreve 'C' em 1,15");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 1, 15);
    lcd_i2c_print_char(lcd_info, 'C');

    ESP_LOGI(TAG, "Move o cursor para 1,0 e pisca. O cursor deve estar ligado");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 1, 0);
    lcd_i2c_config_cursor_piscante(lcd_info, true);

    ESP_LOGI(TAG, "Escreve 'DE' e move o cursor para tras no 'D'");
    aguardar_uart_rx();
    lcd_i2c_print_char(lcd_info, 'D');
    lcd_i2c_direita_para_esquerda(lcd_info);
    lcd_i2c_print_char(lcd_info, 'E');
    lcd_i2c_esquerda_para_direita(lcd_info);

    ESP_LOGI(TAG, "Desabilita display");
    aguardar_uart_rx();
    lcd_i2c_habilita_display(lcd_info, false);

    ESP_LOGI(TAG, "Escreve 'F' em 1,7 (display desabilitado)");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 1, 7);
    lcd_i2c_print_char(lcd_info, 'F');

    ESP_LOGI(TAG, "Habilita display");
    aguardar_uart_rx();
    lcd_i2c_habilita_display(lcd_info, true);

    ESP_LOGI(TAG, "Desabilita cursor piscante");
    aguardar_uart_rx();
    lcd_i2c_config_cursor_piscante(lcd_info, false);

    ESP_LOGI(TAG, "Desabilita cursor");
    aguardar_uart_rx();
    lcd_i2c_config_cursor(lcd_info, false);

    ESP_LOGI(TAG, "Escreve todos caracteres alfanumericos apartir de 0,0");
    aguardar_uart_rx();
    lcd_i2c_retornar_inicio(lcd_info);
    lcd_i2c_printf(lcd_info,
                   "abcdefghijklmnopqrst"
                   "uvwxyz0123456789.,-+"
                   "ABCDEFGHIJKLMNOPQRST"
                   "UVWXYZ");

    ESP_LOGI(TAG, "Rola o display para esquerda, 8 casas, devagar");
    aguardar_uart_rx();
    for (int i = 0; i < 8; ++i) {
        lcd_i2c_rolagem_esquerda(lcd_info);
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Rola o display para direita, 8 casas, imediatamente");
    aguardar_uart_rx();
    for (int i = 0; i < 8; ++i) {
        lcd_i2c_rolagem_direita(lcd_info);
    }

    ESP_LOGI(TAG, "Move para 0,8 e mostra o cursor");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 0, 8);
    lcd_i2c_config_cursor(lcd_info, true);

    ESP_LOGI(TAG, "Move o cursor 5 casas para a direita");
    aguardar_uart_rx();
    for (int i = 0; i < 5; ++i) {
        lcd_i2c_mover_cursor_direita(lcd_info);
    }

    ESP_LOGI(TAG, "Move o cursor 3 casas para a esquerda");
    aguardar_uart_rx();
    for (int i = 0; i < 3; ++i) {
        lcd_i2c_mover_cursor_esquerda(lcd_info);
    }

    ESP_LOGI(TAG, "Habilita auto-scroll e escreve '>>>>>'");
    aguardar_uart_rx();
    lcd_i2c_rolagem_automatica(lcd_info, true);
    for (int i = 0; i < 5; ++i) {
        lcd_i2c_print_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Muda endereco do contador para decrementar (escreve da direita para esquerda) e escreve '<<<<<'");
    aguardar_uart_rx();
    lcd_i2c_direita_para_esquerda(lcd_info);
    for (int i = 0; i < 5; ++i) {
        lcd_i2c_print_char(lcd_info, '<');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Desabilita auto-scroll e esceve '+++++'");
    aguardar_uart_rx();
    lcd_i2c_rolagem_automatica(lcd_info, false);
    for (int i = 0; i < 5; ++i) {
        lcd_i2c_print_char(lcd_info, '+');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Ajusta escrita de esquerda para direita e escreve '>>>>>'");
    aguardar_uart_rx();
    lcd_i2c_esquerda_para_direita(lcd_info);
    for (int i = 0; i < 5; ++i) {
        lcd_i2c_print_char(lcd_info, '>');
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Limpa display e desabilita cursor");
    aguardar_uart_rx();
    lcd_i2c_limpar_display(lcd_info);
    lcd_i2c_config_cursor(lcd_info, false);

    ESP_LOGI(TAG, "Cria caracteres especiais");
    aguardar_uart_rx();
    uint8_t bell[8]     = {0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04};
    uint8_t note[8]     = {0x02, 0x03, 0x02, 0x0e, 0x1e, 0x0c, 0x00};
    uint8_t clock[8]    = {0x00, 0x0e, 0x15, 0x17, 0x11, 0x0e, 0x00};
    uint8_t heart[8]    = {0x00, 0x0a, 0x1f, 0x1f, 0x0e, 0x04, 0x00};
    uint8_t duck[8]     = {0x00, 0x0c, 0x1d, 0x0f, 0x0f, 0x06, 0x00};
    uint8_t check[8]    = {0x00, 0x01, 0x03, 0x16, 0x1c, 0x08, 0x00};
    uint8_t cross[8]    = {0x00, 0x1b, 0x0e, 0x04, 0x0e, 0x1b, 0x00};
    uint8_t retarrow[8] = {0x01, 0x01, 0x05, 0x09, 0x1f, 0x08, 0x04};
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_00, bell);
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_01, note);
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_02, clock);
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_03, heart);
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_04, duck);
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_05, check);
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_06, cross);
    lcd_i2c_caracter_personalizado(lcd_info, CHAR_CUSTOM_INDICE_07, retarrow);

    // Após definir caracteres customizados, o endereço DDRAM deve ser definido por home() ou movendo o cursor
    lcd_i2c_mover_cursor(lcd_info, 0, 0);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_00);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_01);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_02);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_03);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_04);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_05);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_06);
    lcd_i2c_print_char(lcd_info, CHAR_CUSTOM_07);

    ESP_LOGI(TAG, "Escreve os caracteres especiais");
    aguardar_uart_rx();
    lcd_i2c_mover_cursor(lcd_info, 1, 0);
    lcd_i2c_print_char(lcd_info, CHAR_ALFA_MIN);
    lcd_i2c_print_char(lcd_info, CHAR_BETA_MIN);
    lcd_i2c_print_char(lcd_info, CHAR_THETA_MIN);
    lcd_i2c_print_char(lcd_info, CHAR_PI_MIN);
    lcd_i2c_print_char(lcd_info, CHAR_OMEGA_MAI);
    lcd_i2c_print_char(lcd_info, CHAR_SIGMA_MAI);
    lcd_i2c_print_char(lcd_info, CHAR_INFINITO);
    lcd_i2c_print_char(lcd_info, CHAR_GRAU);
    lcd_i2c_print_char(lcd_info, CHAR_SETA_ESQUERDA);
    lcd_i2c_print_char(lcd_info, CHAR_SETA_DIREITA);
    lcd_i2c_print_char(lcd_info, CHAR_2_SUBS);
    lcd_i2c_print_char(lcd_info, CHAR_PONTO_CENTRO);
    lcd_i2c_print_char(lcd_info, CHAR_DIVISAO);
    lcd_i2c_print_char(lcd_info, CHAR_BLOCO);

    ESP_LOGI(TAG, "Escreve todos os caracteres (loop)");
    aguardar_uart_rx();
    lcd_i2c_limpar_display(lcd_info);
    lcd_i2c_config_cursor(lcd_info, true);
    uint8_t c = 0;
    uint8_t coluna = 0;
    uint8_t linha = 0;
    while (1) {
        lcd_i2c_print_char(lcd_info, c);
        vTaskDelay(100 / portTICK_RATE_MS);
        ESP_LOGD(TAG, "coluna %d, linha %d, char 0x%02x", coluna, linha, c);
        ++c;
        ++coluna;
        if (coluna >= LCD_NUM_COLUNAS) {
            ++linha;
            if (linha >= LCD_NUM_LINHAS)
                linha = 0;
            coluna = 0;
            lcd_i2c_mover_cursor(lcd_info, linha, coluna);
        }
    }

    vTaskDelete(NULL);
}

void app_main() {
    xTaskCreate(&vLcdTask, "vLcdTask", 4096, NULL, 0, NULL);
}
