#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "lcd-i2c.h"

#define TAG "LCD_I2C"

// Delays (em ms)
#define DELAY_POWER_ON               50000  // espera ao menos 40us após VCC subir para 2,7V
#define DELAY_INIT_1                 4500   // espera ao menos 4,1ms (fig 24, pg 46)
#define DELAY_INIT_2                 4500   // espera ao menos 4,1ms (fig 24, pg 46)
#define DELAY_INIT_3                 120    // espera ao menos 100us (fig 24, pg 46)

#define DELAY_LIMPAR_DISPLAY         2000
#define DELAY_RETORNA_INICIO         2000

#define DELAY_HABILITA_LARGURA_PULSO 1   // o pulso de ativação deve ter pelo menos 450ns de largura
#define DELAY_HABILITA_AJUSTE_PULSO  50  // o comando requer > 37us para resolver (tabela 6 no datasheet)

// Comandos
#define CMD_LIMPAR_DISPLAY    0x01
#define CMD_RETORNAR_INICIO   0x02
#define CMD_MODO_CONFIG       0x04
#define CMD_CONTROLE_DISPLAY  0x08
#define CMD_SHIFT             0x10
#define CMD_CONFIGURAR        0x20
#define CMD_CONFIG_CGRAM_ADDR 0x40
#define CMD_CONFIG_DDRAM_ADDR 0x80

// CMD_MODO_CONFIG flags
#define FLAG_INCREMENTA 0x02
#define FLAG_DECREMENTA 0x00
#define FLAG_SHIFT_ON   0x01
#define FLAG_SHIFT_OFF  0x00

// CMD_CONTROLE_DISPLAY flags
#define FLAG_DISPLAY_ON  0x04
#define FLAG_DISPLAY_OFF 0x00
#define FLAG_CURSOR_ON   0x02
#define FLAG_CURSOR_OFF  0x00
#define FLAG_BLINK_ON    0x01
#define FLAG_BLINK_OFF   0x00

// CMD_SHIFT flags
#define FLAG_MOVER_DISPLAY       0x08
#define FLAG_MOVER_CURSOR        0x00
#define FLAG_MOVER_PARA_ESQUERDA 0x04
#define FLAG_MOVER_PARA_DIREITA  0x00

// CMD_CONFIGURAR flags
#define FLAG_MODO_8BIT 0x10
#define FLAG_MODO_4BIT 0x00
#define FLAG_2_LINHAS  0x08
#define FLAG_1_LINHA   0x00
#define FLAG_DOTS_5X10 0x04
#define FLAG_DOTS_5X8  0x00

// Control flags
#define FLAG_BACKLIGHT_LIGA 0b00001000  // liga backlight (desabilitado se display apagado)
#define FLAG_BACKLIGHT_DESL 0b00000000  // desliga backlight
#define FLAG_PINO_EN        0b00000100
#define FLAG_LEITURA        0b00000010  // leitura (escrita se display apagado)
#define FLAG_ESCRITA        0b00000000  // escrita
#define FLAG_RS_DADO        0b00000001  // dado (comando se display apagado)
#define FLAG_RS_CMD         0b00000000  // comando

// Verifica inicialização
static bool lcd_i2c_confirmar_init(const lcd_i2c_t* lcd_i2c) {
    bool ok = false;
    if (lcd_i2c != NULL) {
        if (lcd_i2c->init) {
            ok = true;
        } else {
            ESP_LOGE(TAG, "lcd_i2c nao foi inicializado");
        }
    } else {
        ESP_LOGE(TAG, "lcd_i2c=NULL");
    }
    return ok;
}

// Define ou limpa a flag especificada dependendo da condição
static uint8_t lcd_i2c_config_flag(uint8_t flags, bool condicao, uint8_t flag) {
    if (condicao) {
        flags |= flag;
    } else {
        flags &= ~flag;
    }
    return flags;
}

// Envia dados para o expansor de I/O
static esp_err_t lcd_i2c_pcf8574_enviar(const lcd_i2c_t* lcd_i2c, uint8_t data) {
    // backlight flag must be included with every write to maintain backlight state
    ESP_LOGD(TAG, "lcd_i2c_pcf8574_enviar 0x%02x", data | lcd_i2c->backlight);
    return smbus_enviar_byte(lcd_i2c->smbus, data | lcd_i2c->backlight);
}

// IMPORTANTE - para que o display fique "em sincronia" é importante
// que os erros não interrompam a sequência de 2 x nibble.

// Sincroziza o PFC8574 com o LCD, causando uma borda descendente no pino EN
static esp_err_t lcd_i2c_pcf8574_ajusta_clock(const lcd_i2c_t* lcd_i2c, uint8_t data) {
    esp_err_t err1 = lcd_i2c_pcf8574_enviar(lcd_i2c, data | FLAG_PINO_EN);
    ets_delay_us(DELAY_HABILITA_LARGURA_PULSO);
    esp_err_t err2 = lcd_i2c_pcf8574_enviar(lcd_i2c, data & ~FLAG_PINO_EN);
    ets_delay_us(DELAY_HABILITA_AJUSTE_PULSO);
    return err1 ? err1 : err2;
}

// Envia um nibble alto para o controlador LCD
static esp_err_t lcd_i2c_pcf8574_enviar_nibble(const lcd_i2c_t* lcd_i2c, uint8_t data) {
    ESP_LOGD(TAG, "lcd_i2c_pcf8574_enviar_nibble 0x%02x", data);
    esp_err_t err1 = lcd_i2c_pcf8574_enviar(lcd_i2c, data);
    esp_err_t err2 = lcd_i2c_pcf8574_ajusta_clock(lcd_i2c, data);
    return err1 ? err1 : err2;
}

// Envia um comando para o I2C
static esp_err_t lcd_i2c_enviar(const lcd_i2c_t* lcd_i2c, uint8_t value, uint8_t register_select_flag) {
    ESP_LOGD(TAG, "lcd_i2c_enviar 0x%02x | 0x%02x", value, register_select_flag);
    esp_err_t err1 = lcd_i2c_pcf8574_enviar_nibble(lcd_i2c, (value & 0xf0) | register_select_flag);
    esp_err_t err2 = lcd_i2c_pcf8574_enviar_nibble(lcd_i2c, ((value & 0x0f) << 4) | register_select_flag);
    return err1 ? err1 : err2;
}

// Envia um comando para o LCD
static esp_err_t lcd_i2c_enviar_cmd(const lcd_i2c_t* lcd_i2c, uint8_t command) {
    ESP_LOGD(TAG, "lcd_i2c_enviar_cmd 0x%02x", command);
    return lcd_i2c_enviar(lcd_i2c, command, FLAG_RS_CMD);
}

// Envia um dado para o I2C
static esp_err_t lcd_i2c_enviar_dado(const lcd_i2c_t* lcd_i2c, uint8_t data) {
    ESP_LOGD(TAG, "lcd_i2c_enviar_dado 0x%02x", data);
    return lcd_i2c_enviar(lcd_i2c, data, FLAG_RS_DADO);
}

lcd_i2c_t* lcd_i2c_malloc(void) {
    lcd_i2c_t* lcd_i2c = malloc(sizeof(*lcd_i2c));
    if (lcd_i2c != NULL) {
        memset(lcd_i2c, 0, sizeof(*lcd_i2c));
        ESP_LOGD(TAG, "malloc lcd_i2c_t %p", lcd_i2c);
    } else {
        ESP_LOGE(TAG, "malloc lcd_i2c_t falhou");
    }
    return lcd_i2c;
}

void lcd_i2c_free(lcd_i2c_t** lcd_i2c) {
    if (lcd_i2c != NULL && (*lcd_i2c != NULL)) {
        ESP_LOGD(TAG, "free lcd_i2c_t %p", *lcd_i2c);
        free(*lcd_i2c);
        *lcd_i2c = NULL;
    } else {
        ESP_LOGE(TAG, "free lcd_i2c_t falhou");
    }
}

// Procedimento de inicialização na pagina 45/46 do datasheet do HD44780.
esp_err_t lcd_i2c_init(lcd_i2c_t* lcd_i2c, smbus_t* smbus, bool backlight, uint8_t num_linhas,
                       uint8_t num_celulas, uint8_t num_colunas) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c != NULL) {
        lcd_i2c->smbus = smbus;
        lcd_i2c->backlight = backlight ? FLAG_BACKLIGHT_LIGA : FLAG_BACKLIGHT_DESL;
        lcd_i2c->num_linhas = num_linhas;
        lcd_i2c->num_celulas = num_celulas;
        lcd_i2c->num_colunas = num_colunas;

        // display ligado, sem cursor, sem cursor piscante
        lcd_i2c->display_control_flags = FLAG_DISPLAY_ON | FLAG_CURSOR_OFF | FLAG_BLINK_OFF;

        // justificado à esquerda, texto da esquerda para a direita
        lcd_i2c->entry_mode_flags = FLAG_INCREMENTA | FLAG_SHIFT_OFF;

        lcd_i2c->init = true;

        // Espera 40ms após VCC subir acima de 2,7V antes de enviar comandos.
        ets_delay_us(DELAY_POWER_ON);

        ret = lcd_i2c_init_config(lcd_i2c);
    } else {
        ESP_LOGE(TAG, "lcd_i2c=NULL");
        ret = ESP_FAIL;
    }
    return ret;
}

esp_err_t lcd_i2c_init_config(const lcd_i2c_t* lcd_i2c) {
    esp_err_t first_err = ESP_OK;
    esp_err_t last_err = ESP_FAIL;

    // Liga o LCD através do PCF8574 - pinos RS e RW em 0
    if ((last_err = lcd_i2c_pcf8574_enviar(lcd_i2c, 0)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_pcf8574_enviar 1 falhou: %d", last_err);
    }
    ets_delay_us(1000);

    // Seleciona modo de 8 bits (pg 46, fig 24)
    // TODO: fazer um for para repetir 3 vezes
    if ((last_err = lcd_i2c_pcf8574_enviar_nibble(lcd_i2c, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_pcf8574_enviar_nibble 1 falhou: %d", last_err);
    }
    ets_delay_us(DELAY_INIT_1);

    // Repete comando anterior
    if ((last_err = lcd_i2c_pcf8574_enviar_nibble(lcd_i2c, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_pcf8574_enviar_nibble 2 failed: %d", last_err);
    }
    ets_delay_us(DELAY_INIT_2);

    // repete comando anterior
    if ((last_err = lcd_i2c_pcf8574_enviar_nibble(lcd_i2c, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_pcf8574_enviar_nibble 3 failed: %d", last_err);
    }
    ets_delay_us(DELAY_INIT_3);

    // Seleciona modo de 4 bits
    if ((last_err = lcd_i2c_pcf8574_enviar_nibble(lcd_i2c, 0x02 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_pcf8574_enviar_nibble 4 failed: %d", last_err);
    }

    // Agora já podemos usar as funções command()/write()

    last_err = lcd_i2c_enviar_cmd(lcd_i2c, CMD_CONFIGURAR | FLAG_MODO_4BIT | FLAG_2_LINHAS | FLAG_DOTS_5X8);
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_enviar_cmd 1 falhou: %d", last_err);
    }

    last_err = lcd_i2c_enviar_cmd(lcd_i2c, CMD_CONTROLE_DISPLAY | lcd_i2c->display_control_flags);
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_enviar_cmd 2 falhou: %d", last_err);
    }

    last_err = lcd_i2c_limpar_display(lcd_i2c);
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_limpar_display falhou: %d", last_err);
    }

    last_err = lcd_i2c_enviar_cmd(lcd_i2c, CMD_MODO_CONFIG | lcd_i2c->entry_mode_flags);
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_enviar_cmd 3 falhou: %d", last_err);
    }

    last_err = lcd_i2c_retornar_inicio(lcd_i2c);
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: lcd_i2c_retornar_inicio failed: %d", last_err);
    }

    return first_err;
}

esp_err_t lcd_i2c_limpar_display(const lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_LIMPAR_DISPLAY);
        if (ret == ESP_OK)
            ets_delay_us(DELAY_LIMPAR_DISPLAY);
    }
    return ret;
}

esp_err_t lcd_i2c_retornar_inicio(const lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_RETORNAR_INICIO);
        if (ret == ESP_OK)
            ets_delay_us(DELAY_RETORNA_INICIO);
    }
    return ret;
}

esp_err_t lcd_i2c_mover_cursor(const lcd_i2c_t* lcd_i2c, uint8_t row, uint8_t col) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        const int inicio_linha[] = {0x00, 0x40, 0x14, 0x54};
        if (row > lcd_i2c->num_linhas)
            row = lcd_i2c->num_linhas - 1;
        if (col > lcd_i2c->num_celulas)
            col = lcd_i2c->num_celulas - 1;
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_CONFIG_DDRAM_ADDR | (col + inicio_linha[row]));
    }
    return ret;
}

esp_err_t lcd_i2c_backlight(lcd_i2c_t* lcd_i2c, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        lcd_i2c->backlight = lcd_i2c_config_flag(lcd_i2c->backlight, enable, FLAG_BACKLIGHT_LIGA);
        ret = lcd_i2c_pcf8574_enviar(lcd_i2c, 0);
    }
    return ret;
}

esp_err_t lcd_i2c_habilita_display(lcd_i2c_t* lcd_i2c, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        lcd_i2c->display_control_flags =
            lcd_i2c_config_flag(lcd_i2c->display_control_flags, enable, FLAG_DISPLAY_ON);
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_CONTROLE_DISPLAY | lcd_i2c->display_control_flags);
    }
    return ret;
}

esp_err_t lcd_i2c_config_cursor(lcd_i2c_t* lcd_i2c, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        lcd_i2c->display_control_flags =
            lcd_i2c_config_flag(lcd_i2c->display_control_flags, enable, FLAG_CURSOR_ON);
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_CONTROLE_DISPLAY | lcd_i2c->display_control_flags);
    }
    return ret;
}

esp_err_t lcd_i2c_config_cursor_piscante(lcd_i2c_t* lcd_i2c, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        lcd_i2c->display_control_flags =
            lcd_i2c_config_flag(lcd_i2c->display_control_flags, enable, FLAG_BLINK_ON);
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_CONTROLE_DISPLAY | lcd_i2c->display_control_flags);
    }
    return ret;
}

esp_err_t lcd_i2c_esquerda_para_direita(lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        lcd_i2c->entry_mode_flags |= FLAG_INCREMENTA;
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_MODO_CONFIG | lcd_i2c->entry_mode_flags);
    }
    return ret;
}

esp_err_t lcd_i2c_direita_para_esquerda(lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        lcd_i2c->entry_mode_flags &= ~FLAG_INCREMENTA;
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_MODO_CONFIG | lcd_i2c->entry_mode_flags);
    }
    return ret;
}

esp_err_t lcd_i2c_rolagem_automatica(lcd_i2c_t* lcd_i2c, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        lcd_i2c->entry_mode_flags =
            lcd_i2c_config_flag(lcd_i2c->entry_mode_flags, enable, FLAG_SHIFT_ON);
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_MODO_CONFIG | lcd_i2c->entry_mode_flags);
    }
    return ret;
}

esp_err_t lcd_i2c_rolagem_esquerda(const lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        // RAM não é alterada
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_SHIFT | FLAG_MOVER_DISPLAY | FLAG_MOVER_PARA_ESQUERDA);
    }
    return ret;
}

esp_err_t lcd_i2c_rolagem_direita(const lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        // RAM não é alterada
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_SHIFT | FLAG_MOVER_DISPLAY | FLAG_MOVER_PARA_DIREITA);
    }
    return ret;
}

esp_err_t lcd_i2c_mover_cursor_esquerda(const lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        // RAM não é alterada. Direção do deslocamento é invertida.
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_SHIFT | FLAG_MOVER_CURSOR | FLAG_MOVER_PARA_DIREITA);
    }
    return ret;
}

esp_err_t lcd_i2c_mover_cursor_direita(const lcd_i2c_t* lcd_i2c) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        // RAM não é alterada. Direção do deslocamento é invertida.
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_SHIFT | FLAG_MOVER_CURSOR | FLAG_MOVER_PARA_ESQUERDA);
    }
    return ret;
}

esp_err_t lcd_i2c_caracter_personalizado(const lcd_i2c_t* lcd_i2c, i2c_lcd1602_custom_index_t index,
                                         const uint8_t pixelmap[]) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        // Somente os 8 primeiros índices podem ser usados para caracteres personalizados
        index &= 0x07;
        ret = lcd_i2c_enviar_cmd(lcd_i2c, CMD_CONFIG_CGRAM_ADDR | (index << 3));
        for (int i = 0; ret == ESP_OK && i < 8; ++i) {
            ret = lcd_i2c_enviar_dado(lcd_i2c, pixelmap[i]);
        }
    }
    return ret;
}

esp_err_t lcd_i2c_print_char(const lcd_i2c_t* lcd_i2c, uint8_t chr) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        ret = lcd_i2c_enviar_dado(lcd_i2c, chr);
    }
    return ret;
}

esp_err_t lcd_i2c_printf(const lcd_i2c_t* lcd_i2c, const char* string) {
    esp_err_t ret = ESP_FAIL;
    if (lcd_i2c_confirmar_init(lcd_i2c)) {
        ESP_LOGI(TAG, "lcd_i2c_printf: %s", string);
        ret = ESP_OK;
        for (int i = 0; ret == ESP_OK && string[i]; ++i) {
            ret = lcd_i2c_enviar_dado(lcd_i2c, string[i]);
        }
    }
    return ret;
}
