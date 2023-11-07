#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "rom/uart.h"

#include "lcd-i2c.h"

#include "lcd.h"

#define TAG "MAIN"

// Utiliza stdin para aguardar o usuario pressionar uma tecla
// Se desabilitado, aguarda 1 segundo
#define USE_STDIN 1

static uint8_t aguardar_uart_rx(void) {
    uint8_t c = 0;

#ifdef USE_STDIN
    while (!c) {
        ETS_STATUS s = uart_rx_one_char(&c);
        if (s == ETS_OK)
            printf("%c", c);
        vTaskDelay(1);
    }
#else
    vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif  // USE_STDIN
    return c;
}

void vLcdTask(void* pvParameter) {
    while (1) {
        lcd_cursor(0, 0);
        lcd_printf("Mas bah!");
        lcd_cursor(1, 0);
        lcd_printf("Aqui tem agua");
        lcd_cursor(2, 0);
        lcd_printf("pro chimarrao!");
        lcd_cursor(3, 8);
        lcd_printf("Tche!");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main() {
    lcd_init();
    xTaskCreate(&vLcdTask, "vLcdTask", 4096, NULL, 0, NULL);
}
