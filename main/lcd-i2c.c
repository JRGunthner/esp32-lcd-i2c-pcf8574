/**
 * @brief O controlador LCD é o HD44780 que opera via barramento paralelo de 8 ou 4bits.
 *
 * O LCD está conectado a um expansor de I/O PCF8574A através do barramento I2C.
 * Apenas os quatro bits superiores estão conectados às linhas de dados do controlador.
 * Os quatro bits inferiores são usados como linhas de controle:
 *
 *   - B7: data bit 3
 *   - B6: data bit 2
 *   - B5: data bit 1
 *   - B4: data bit 0
 *   - B3: backlight (BL): off = 0, on = 1
 *   - B2: enable (EN): mudar de 1 para 0 para sincronizar os dados no controlador
 *   - B1: read/write (RW): escrita = 0, leitura = 1
 *   - B0: register select (RS): comando = 0, dado = 1
 *
 * Portanto, para enviar um byte de comando são necessárias as seguintes operações:
 *
 *   // Primeiro nibble:
 *   val = command & 0xf0              // extarai parte alta do byte
 *   val |= 0x04                       // RS = 0 (comando), RW = 0 (escrita), EN = 1
 *   i2c_write_byte(i2c_address, val)
 *   sleep(2ms)
 *   val &= 0xfb                       // EN = 0
 *   i2c_write_byte(i2c_address, val)
 *
 *   // Segundo nibble:
 *   val = command & 0x0f              // extrai parte baixa do byte
 *   val |= 0x04                       // RS = 0 (command), RW = 0 (escrita), EN = 1
 *   i2c_write_byte(i2c_address, val)
 *   sleep(2ms)
 *   val &= 0xfb                       // EN = 0
 *   i2c_write_byte(i2c_address, val)
 *
 * O envio de um byte de dados é muito semelhante, exceto que RS = 1 (dados)
 *
 * Quando o controlador é ligado, ele é configurado para:
 *
 *  - display apagado
 *  - interface de 8 bits, 1 linha, 5x8 pontos por caractere
 *  - incremento de cursor
 *  - sem deslocamento
 *
 * O controlador deve ser configurado para operação de 4 bits antes que a
 * comunicação adequada possa começar.
 * A sequência de inicialização para operação de 4 bits é:
 *
 *   0. delay > 15ms após Vcc subir para 4,5V, ou > 40ms após Vcc subir para 2,7V
 *   1. enviar nibble 0x03     // seleciona interface de 8 bits
 *   2. delay > 4,1ms
 *   3. enviar nibble 0x03     // seleciona interface de 8 bits novamente
 *   4. delay > 100us
 *   5. enviar comando 0x32    // seleciona interface de 4 bits
 *   6. enviar comando 0x28    // seleciona 2 linhas e 5x7(8?) pontos por caractere
 *   7. enviar comando 0x0c    // display ligado, cursor desligado
 *   8. enviar comando 0x06    // move cursor para direita quando escreve, sem deslocamento
 *   9. enviar comando 0x80    // move cursor para posição inicial (linha 1, coluna 1)
 */

#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "lcd-i2c.h"

#define TAG "LCD_I2C"

// Delays (em ms)
#define DELAY_POWER_ON 50000 // espera ao menos 40us após VCC subir para 2,7V
#define DELAY_INIT_1   4500  // espera ao menos 4.1ms (fig 24, pg 46)
#define DELAY_INIT_2   4500  // espera ao menos 4.1ms (fig 24, pg 46)
#define DELAY_INIT_3   120   // espera ao menos 100us (fig 24, pg 46)

#define DELAY_CLEAR_DISPLAY 2000
#define DELAY_RETURN_HOME   2000

#define DELAY_ENABLE_PULSE_WIDTH  1  // o pulso de ativação deve ter pelo menos 450ns de largura
#define DELAY_ENABLE_PULSE_SETTLE 50 // o comando requer > 37us para resolver (tabela 6 no datasheet)

// Comandos
#define COMMAND_CLEAR_DISPLAY   0x01
#define COMMAND_RETURN_HOME     0x02
#define COMMAND_ENTRY_MODE_SET  0x04
#define COMMAND_DISPLAY_CONTROL 0x08
#define COMMAND_SHIFT           0x10
#define COMMAND_FUNCTION_SET    0x20
#define COMMAND_SET_CGRAM_ADDR  0x40
#define COMMAND_SET_DDRAM_ADDR  0x80

// COMMAND_ENTRY_MODE_SET flags
#define FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT 0x02
#define FLAG_ENTRY_MODE_SET_ENTRY_DECREMENT 0x00
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON  0x01
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF 0x00

// COMMAND_DISPLAY_CONTROL flags
#define FLAG_DISPLAY_CONTROL_DISPLAY_ON  0x04
#define FLAG_DISPLAY_CONTROL_DISPLAY_OFF 0x00
#define FLAG_DISPLAY_CONTROL_CURSOR_ON   0x02
#define FLAG_DISPLAY_CONTROL_CURSOR_OFF  0x00
#define FLAG_DISPLAY_CONTROL_BLINK_ON    0x01
#define FLAG_DISPLAY_CONTROL_BLINK_OFF   0x00

// COMMAND_SHIFT flags
#define FLAG_SHIFT_MOVE_DISPLAY 0x08
#define FLAG_SHIFT_MOVE_CURSOR  0x00
#define FLAG_SHIFT_MOVE_LEFT    0x04
#define FLAG_SHIFT_MOVE_RIGHT   0x00

// COMMAND_FUNCTION_SET flags
#define FLAG_FUNCTION_SET_MODE_8BIT 0x10
#define FLAG_FUNCTION_SET_MODE_4BIT 0x00
#define FLAG_FUNCTION_SET_LINES_2   0x08
#define FLAG_FUNCTION_SET_LINES_1   0x00
#define FLAG_FUNCTION_SET_DOTS_5X10 0x04
#define FLAG_FUNCTION_SET_DOTS_5X8  0x00

// Control flags
#define FLAG_BACKLIGHT_ON  0b00001000  // liga backlight (desabilitado se display apagado)
#define FLAG_BACKLIGHT_OFF 0b00000000  // desliga backlight
#define FLAG_ENABLE        0b00000100
#define FLAG_READ          0b00000010  // leitura (escrita se display apagado)
#define FLAG_WRITE         0b00000000  // escrita
#define FLAG_RS_DATA       0b00000001  // dado (comando se display apagado)
#define FLAG_RS_COMMAND    0b00000000  // comando

// Verifica inicialização
static bool _is_init(const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    bool ok = false;
    if (i2c_lcd1602_info != NULL) {
        if (i2c_lcd1602_info->init) {
            ok = true;
        } else {
            ESP_LOGE(TAG, "i2c_lcd1602_info nao foi inicializado");
        }
    } else {
        ESP_LOGE(TAG, "i2c_lcd1602_info eh NULL");
    }
    return ok;
}

// Define ou limpa a flag especificada dependendo da condição
static uint8_t _set_or_clear(uint8_t flags, bool condition, uint8_t flag) {
    if (condition) {
        flags |= flag;
    } else {
        flags &= ~flag;
    }
    return flags;
}

// Envia dados para o expansor de I/O
static esp_err_t _write_to_expander(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t data) {
    // backlight flag must be included with every write to maintain backlight state
    ESP_LOGD(TAG, "_write_to_expander 0x%02x", data | i2c_lcd1602_info->backlight_flag);
    return smbus_enviar_byte(i2c_lcd1602_info->smbus_info, data | i2c_lcd1602_info->backlight_flag);
}

// IMPORTANTE - para que o display fique "em sincronia" é importante
// que os erros não interrompam a sequência de 2 x nibble.

// Dados do clock do expansor para o LCD, causando uma borda descendente em Ativar
static esp_err_t _strobe_enable(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t data) {
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data | FLAG_ENABLE);
    ets_delay_us(DELAY_ENABLE_PULSE_WIDTH);
    esp_err_t err2 = _write_to_expander(i2c_lcd1602_info, data & ~FLAG_ENABLE);
    ets_delay_us(DELAY_ENABLE_PULSE_SETTLE);
    return err1 ? err1 : err2;
}

// Envia um nibble para o controlador LCD
static esp_err_t _write_top_nibble(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t data) {
    ESP_LOGD(TAG, "_write_top_nibble 0x%02x", data);
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data);
    esp_err_t err2 = _strobe_enable(i2c_lcd1602_info, data);
    return err1 ? err1 : err2;
}

// Envia um comando para o I2C
static esp_err_t _write(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t value, uint8_t register_select_flag) {
    ESP_LOGD(TAG, "_write 0x%02x | 0x%02x", value, register_select_flag);
    esp_err_t err1 = _write_top_nibble(i2c_lcd1602_info, (value & 0xf0) | register_select_flag);
    esp_err_t err2 = _write_top_nibble(i2c_lcd1602_info, ((value & 0x0f) << 4) | register_select_flag);
    return err1 ? err1 : err2;
}

// Envia um comando para o LCD
static esp_err_t _write_command(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t command) {
    ESP_LOGD(TAG, "_write_command 0x%02x", command);
    return _write(i2c_lcd1602_info, command, FLAG_RS_COMMAND);
}

// Envia um dado para o I2C
static esp_err_t _write_data(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t data) {
    ESP_LOGD(TAG, "_write_data 0x%02x", data);
    return _write(i2c_lcd1602_info, data, FLAG_RS_DATA);
}

i2c_lcd1602_info_t* i2c_lcd1602_malloc(void) {
    i2c_lcd1602_info_t* i2c_lcd1602_info = malloc(sizeof(*i2c_lcd1602_info));
    if (i2c_lcd1602_info != NULL) {
        memset(i2c_lcd1602_info, 0, sizeof(*i2c_lcd1602_info));
        ESP_LOGD(TAG, "malloc i2c_lcd1602_info_t %p", i2c_lcd1602_info);
    } else {
        ESP_LOGE(TAG, "malloc i2c_lcd1602_info_t falhou");
    }
    return i2c_lcd1602_info;
}

void i2c_lcd1602_free(i2c_lcd1602_info_t** i2c_lcd1602_info) {
    if (i2c_lcd1602_info != NULL && (*i2c_lcd1602_info != NULL)) {
        ESP_LOGD(TAG, "free i2c_lcd1602_info_t %p", *i2c_lcd1602_info);
        free(*i2c_lcd1602_info);
        *i2c_lcd1602_info = NULL;
    } else {
        ESP_LOGE(TAG, "free i2c_lcd1602_info_t falhou");
    }
}

esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t* i2c_lcd1602_info,
                           smbus_info_t* smbus_info, bool backlight,
                           uint8_t num_rows, uint8_t num_columns,
                           uint8_t num_visible_columns) {
    esp_err_t ret = ESP_FAIL;
    if (i2c_lcd1602_info != NULL) {
        i2c_lcd1602_info->smbus_info = smbus_info;
        i2c_lcd1602_info->backlight_flag =
            backlight ? FLAG_BACKLIGHT_ON : FLAG_BACKLIGHT_OFF;
        i2c_lcd1602_info->num_rows = num_rows;
        i2c_lcd1602_info->num_columns = num_columns;
        i2c_lcd1602_info->num_visible_columns = num_visible_columns;

        // display ligado, sem cursor, sem cursor piscante
        i2c_lcd1602_info->display_control_flags =
            FLAG_DISPLAY_CONTROL_DISPLAY_ON | FLAG_DISPLAY_CONTROL_CURSOR_OFF |
            FLAG_DISPLAY_CONTROL_BLINK_OFF;

        // justificado à esquerda, texto da esquerda para a direita
        i2c_lcd1602_info->entry_mode_flags =
            FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT |
            FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF;

        i2c_lcd1602_info->init = true;

        // Procedimento de inicialização na pagina 45/46 do datasheet do HD44780.

        // Espera 40ms após VCC subir acima de 2,7V antes de enviar comandos.
        ets_delay_us(DELAY_POWER_ON);

        ret = i2c_lcd1602_reset(i2c_lcd1602_info);
    } else {
        ESP_LOGE(TAG, "i2c_lcd1602_info eh NULL");
        ret = ESP_FAIL;
    }
    return ret;
}

esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t first_err = ESP_OK;
    esp_err_t last_err = ESP_FAIL;

    // Liga o LCD através do PCF8574 - pinos RS e RW em 0
    if ((last_err = _write_to_expander(i2c_lcd1602_info, 0)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_to_expander 1 falhou: %d", last_err);
    }
    ets_delay_us(1000);

    // Seleciona modo de 8 bits (pg 46, fig 24)
    // TODO: fazer um for para repetir 3 vezes
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 1 falhou: %d", last_err);
    }
    ets_delay_us(DELAY_INIT_1);

    // Repete comando anterior
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 2 failed: %d", last_err);
    }
    ets_delay_us(DELAY_INIT_2);

    // repete comando anterior
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 3 failed: %d", last_err);
    }
    ets_delay_us(DELAY_INIT_3);

    // Seleciona modo de 4 bits
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x02 << 4)) != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 4 failed: %d", last_err);
    }

    // Agora já podemos usar as funções command()/write()

    last_err = _write_command(
        i2c_lcd1602_info,
        COMMAND_FUNCTION_SET |
        FLAG_FUNCTION_SET_MODE_4BIT |
        FLAG_FUNCTION_SET_LINES_2 |
        FLAG_FUNCTION_SET_DOTS_5X8
    );
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 1 falhou: %d", last_err);
    }

    last_err = _write_command(
        i2c_lcd1602_info,
        COMMAND_DISPLAY_CONTROL |
        i2c_lcd1602_info->display_control_flags
    );
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 2 falhou: %d", last_err);
    }

    last_err = i2c_lcd1602_clear(i2c_lcd1602_info);
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_clear falhou: %d", last_err);
    }

    last_err = _write_command(
        i2c_lcd1602_info,
        COMMAND_ENTRY_MODE_SET |
        i2c_lcd1602_info->entry_mode_flags
    );
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 3 falhou: %d", last_err);
    }

    last_err = i2c_lcd1602_home(i2c_lcd1602_info);
    if (last_err != ESP_OK) {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_home failed: %d", last_err);
    }

    return first_err;
}

esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        ret = _write_command(i2c_lcd1602_info, COMMAND_CLEAR_DISPLAY);
        if (ret == ESP_OK)
            ets_delay_us(DELAY_CLEAR_DISPLAY);
    }
    return ret;
}

esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        ret = _write_command(i2c_lcd1602_info, COMMAND_RETURN_HOME);
        if (ret == ESP_OK)
            ets_delay_us(DELAY_RETURN_HOME);
    }
    return ret;
}

esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t col, uint8_t row) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        const int inicio_linha[] = {0x00, 0x40, 0x14, 0x54};
        if (row > i2c_lcd1602_info->num_rows)
            row = i2c_lcd1602_info->num_rows - 1;
        if (col > i2c_lcd1602_info->num_columns)
            col = i2c_lcd1602_info->num_columns - 1;
        ret = _write_command(i2c_lcd1602_info, COMMAND_SET_DDRAM_ADDR | (col + inicio_linha[row]));
    }
    return ret;
}

esp_err_t i2c_lcd1602_set_backlight(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        i2c_lcd1602_info->backlight_flag = _set_or_clear(
            i2c_lcd1602_info->backlight_flag, enable, FLAG_BACKLIGHT_ON);
        ret = _write_to_expander(i2c_lcd1602_info, 0);
    }
    return ret;
}

esp_err_t i2c_lcd1602_set_display(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        i2c_lcd1602_info->display_control_flags =
            _set_or_clear(
                i2c_lcd1602_info->display_control_flags,
                enable,
                FLAG_DISPLAY_CONTROL_DISPLAY_ON
            );
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags);
    }
    return ret;
}

esp_err_t i2c_lcd1602_set_cursor(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        i2c_lcd1602_info->display_control_flags =
            _set_or_clear(
                i2c_lcd1602_info->display_control_flags,
                enable,
                FLAG_DISPLAY_CONTROL_CURSOR_ON
            );
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags);
    }
    return ret;
}

esp_err_t i2c_lcd1602_set_blink(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        i2c_lcd1602_info->display_control_flags =
            _set_or_clear(
                i2c_lcd1602_info->display_control_flags,
                enable,
                FLAG_DISPLAY_CONTROL_BLINK_ON
            );
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags);
    }
    return ret;
}

esp_err_t i2c_lcd1602_set_left_to_right(i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        i2c_lcd1602_info->entry_mode_flags |= FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT;
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags);
    }
    return ret;
}

esp_err_t i2c_lcd1602_set_right_to_left(i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        i2c_lcd1602_info->entry_mode_flags &=
            ~FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT;
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags);
    }
    return ret;
}

esp_err_t i2c_lcd1602_set_auto_scroll(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        i2c_lcd1602_info->entry_mode_flags =
            _set_or_clear(
                i2c_lcd1602_info->entry_mode_flags,
                enable,
                FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON
            );
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags);
    }
    return ret;
}

esp_err_t i2c_lcd1602_scroll_display_left(
    const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        // RAM não é alterada
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_SHIFT |
            FLAG_SHIFT_MOVE_DISPLAY |
            FLAG_SHIFT_MOVE_LEFT
        );
    }
    return ret;
}

esp_err_t i2c_lcd1602_scroll_display_right(
    const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        // RAM não é alterada
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_SHIFT |
            FLAG_SHIFT_MOVE_DISPLAY |
            FLAG_SHIFT_MOVE_RIGHT
        );
    }
    return ret;
}

esp_err_t i2c_lcd1602_move_cursor_left(
    const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        // RAM não é alterada. Direção do deslocamento é invertida.
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_SHIFT |
            FLAG_SHIFT_MOVE_CURSOR |
            FLAG_SHIFT_MOVE_RIGHT
        );
    }
    return ret;
}

esp_err_t i2c_lcd1602_move_cursor_right(
    const i2c_lcd1602_info_t* i2c_lcd1602_info) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        // RAM não é alterada. Direção do deslocamento é invertida.
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_SHIFT |
            FLAG_SHIFT_MOVE_CURSOR |
            FLAG_SHIFT_MOVE_LEFT
        );
    }
    return ret;
}

esp_err_t i2c_lcd1602_define_char(const i2c_lcd1602_info_t* i2c_lcd1602_info,
                                  i2c_lcd1602_custom_index_t index,
                                  const uint8_t pixelmap[]) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        // Somente os 8 primeiros índices podem ser usados para caracteres personalizados
        index &= 0x07;
        ret = _write_command(
            i2c_lcd1602_info,
            COMMAND_SET_CGRAM_ADDR |
            (index << 3)
        );
        for (int i = 0; ret == ESP_OK && i < 8; ++i) {
            ret = _write_data(i2c_lcd1602_info, pixelmap[i]);
        }
    }
    return ret;
}

esp_err_t i2c_lcd1602_write_char(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t chr) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        ret = _write_data(i2c_lcd1602_info, chr);
    }
    return ret;
}

esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t* i2c_lcd1602_info, const char* string) {
    esp_err_t ret = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info)) {
        ESP_LOGI(TAG, "i2c_lcd1602_write_string: %s", string);
        ret = ESP_OK;
        for (int i = 0; ret == ESP_OK && string[i]; ++i) {
            ret = _write_data(i2c_lcd1602_info, string[i]);
        }
    }
    return ret;
}
