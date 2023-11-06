#ifndef __I2C_LCD1602_H__
#define __I2C_LCD1602_H__

#include <stdbool.h>
#include "smbus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool init;                      // True se o display foi inicializado
    smbus_info_t* smbus_info;       // Ponteiro para a estrutura smbus_info
    uint8_t backlight_flag;         // Backlight habilitado (1), desabilitado (0)
    uint8_t num_rows;               // Nuúmero de linhas
    uint8_t num_columns;            // Número de células totais, incluindo as não visíveis
    uint8_t num_visible_columns;    // Número de colunas
    uint8_t display_control_flags;  // Atividade atual do controle do display
    uint8_t entry_mode_flags;       // modo de entrada ativo atual
} i2c_lcd1602_info_t;

// Caracteres especiais customizados para o código ROM A00
// Use o segundo conjunto (0bxxxx1xxx) para evitar colocar o caractere nulo dentro de uma string
#define I2C_LCD1602_CHARACTER_CUSTOM_0 0b00001000 // Caractere customizado no índice 0
#define I2C_LCD1602_CHARACTER_CUSTOM_1 0b00001001 // Caractere customizado no índice 1
#define I2C_LCD1602_CHARACTER_CUSTOM_2 0b00001010 // Caractere customizado no índice 2
#define I2C_LCD1602_CHARACTER_CUSTOM_3 0b00001011 // Caractere customizado no índice 3
#define I2C_LCD1602_CHARACTER_CUSTOM_4 0b00001100 // Caractere customizado no índice 4
#define I2C_LCD1602_CHARACTER_CUSTOM_5 0b00001101 // Caractere customizado no índice 5
#define I2C_LCD1602_CHARACTER_CUSTOM_6 0b00001110 // Caractere customizado no índice 6
#define I2C_LCD1602_CHARACTER_CUSTOM_7 0b00001111 // Caractere customizado no índice 7

#define I2C_LCD1602_CHARACTER_ALPHA    0b11100000 // Alpha minúsculo
#define I2C_LCD1602_CHARACTER_BETA     0b11100010 // Beta minúsculo
#define I2C_LCD1602_CHARACTER_THETA    0b11110010 // Theta minúsculo
#define I2C_LCD1602_CHARACTER_PI       0b11110111 // Pi minúsculo
#define I2C_LCD1602_CHARACTER_OMEGA    0b11110100 // Omega maiúsculo
#define I2C_LCD1602_CHARACTER_SIGMA    0b11110110 // Sigma maiúsculo
#define I2C_LCD1602_CHARACTER_INFINITY 0b11110011 // Infinito
#define I2C_LCD1602_CHARACTER_DEGREE   0b11011111 // Grau (°)
#define I2C_LCD1602_CHARACTER_ARROW_RIGHT 0b01111110 // Seta para a direita
#define I2C_LCD1602_CHARACTER_ARROW_LEFT  0b01111111 // Seta para a esquerda
#define I2C_LCD1602_CHARACTER_SQUARE   0b11011011 // 2 pequeno
#define I2C_LCD1602_CHARACTER_DOT      0b10100101 // ponto no centro
#define I2C_LCD1602_CHARACTER_DIVIDE   0b11111101 // Divisão
#define I2C_LCD1602_CHARACTER_BLOCK    0b11111111 // Bloco 5x8 preenchido

// Caracteres customizados
typedef enum {
    I2C_LCD1602_INDEX_CUSTOM_0 = 0,
    I2C_LCD1602_INDEX_CUSTOM_1,
    I2C_LCD1602_INDEX_CUSTOM_2,
    I2C_LCD1602_INDEX_CUSTOM_3,
    I2C_LCD1602_INDEX_CUSTOM_4,
    I2C_LCD1602_INDEX_CUSTOM_5,
    I2C_LCD1602_INDEX_CUSTOM_6,
    I2C_LCD1602_INDEX_CUSTOM_7,
} i2c_lcd1602_custom_index_t;

#define I2C_LCD1602_ERROR_CHECK(x)                                          \
    do {                                                                    \
        esp_err_t rc = (x);                                                 \
        if (rc != ESP_OK) {                                                 \
            ESP_LOGW(TAG, "I2C erro %d em %s:%d", rc, __FILE__, __LINE__); \
        }                                                                   \
    } while (0);

/**
 * @brief Construtor de uma nova instância de I2C-LCD info.
 *        A nova instância deve ser inicializada antes de chamar outras funções.
 * @return Ponteiro para a nova instância de I2C-LCD info, ou NULL se não puder ser criada
 */
i2c_lcd1602_info_t* i2c_lcd1602_malloc(void);

/**
 * @brief Delete an existing I2C-LCD1602 info instance.
 *
 * @param[in,out] tsl2561_info Pointer to I2C-LCD1602 info instance that will be
 * freed and set to NULL.
 */
void i2c_lcd1602_free(i2c_lcd1602_info_t** tsl2561_info);

/**
 * @brief Initialise a I2C-LCD1602 info instance with the specified SMBus
 * information.
 *
 * @param[in] i2c_lcd1602_info Pointer to I2C-LCD1602 info instance.
 * @param[in] smbus_info Pointer to SMBus info instance.
 * @param[in] backlight Initial backlight state.
 * @param[in] num_rows Maximum number of supported rows for this device. Typical
 * values include 2 (1602) or 4 (2004).
 * @param[in] num_columns Maximum number of supported columns for this device.
 * Typical values include 40 (1602, 2004).
 * @param[in] num_visible_columns Number of columns visible at any one time.
 * Typical values include 16 (1602) or 20 (2004).
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t* i2c_lcd1602_info,
                           smbus_info_t* smbus_info, bool backlight,
                           uint8_t num_rows, uint8_t num_columns,
                           uint8_t num_visible_columns);

/**
 * @brief Reset the display. Custom characters will be cleared.
 *
 * @param[in] i2c_lcd1602_info Pointer to I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Clears entire display (clears DDRAM) and returns cursor to home
 * position. DDRAM content is cleared, CGRAM content is not changed.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move cursor to home position. Also resets any display shift that may
 * have occurred. DDRAM content is not changed. CGRAM content is not changed.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move cursor to specified column and row position. This is where a new
 * character will appear.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @param[in] col Zero-based horizontal index of intended cursor position.
 * Column 0 is the left column.
 * @param[in] row Zero-based vertical index of intended cursor position. Row 0
 * is the top row.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t col, uint8_t row);

/**
 * @brief Enable or disable the LED backlight.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @param[in] enable True to enable, false to disable.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_set_backlight(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Enable or disable the display. When disabled, the backlight is not
 * affected, but any contents of the DDRAM is not displayed, nor is the cursor.
 * The display is "blank". Re-enabling the display does not affect the contents
 * of DDRAM or the state or position of the cursor.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @param[in] enable True to enable, false to disable.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_set_display(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Enable or disable display of the underline cursor.
 *        If enabled, this visually indicates where the next character written
 * to the display will appear. It may be enabled alongside the blinking cursor,
 * however the visual result is inelegant.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @param[in] enable True to enable, false to disable.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_set_cursor(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Enable or disable display of the blinking block cursor.
 *        If enabled, this visually indicates where the next character written
 * to the display will appear. It may be enabled alongside the underline cursor,
 * however the visual result is inelegant.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @param[in] enable True to enable, false to disable.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_set_blink(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Set cursor movement direction following each character write to
 * produce left-to-right text.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_set_left_to_right(i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Set cursor movement direction following each character write to
 * produce right-to-left text.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_set_right_to_left(i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Enable or disable auto-scroll of display.
 *        When enabled, the display will scroll as characters are written to
 * maintain the cursor position on-screen. Left-to-right text will appear to be
 * right-justified from the cursor position. When disabled, the display will not
 * scroll and the cursor will move on-screen. Left-to-right text will appear to
 * be left-justified from the cursor position.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @param[in] enable True to enable, false to disable.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_set_auto_scroll(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Scroll the display one position to the left. On-screen text will
 * appear to move to the right.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_scroll_display_left(
    const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Scroll the display one position to the right. On-screen text will
 * appear to move to the left.
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_scroll_display_right(
    const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move the cursor one position to the left, even if it is invisible.
 *        This affects where the next character written to the display will
 * appear.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_move_cursor_left(
    const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move the cursor one position to the right, even if it is invisible.
 *        This affects where the next character written to the display will
 * appear.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_move_cursor_right(
    const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Define a custom character from an array of pixel data.
 *
 *        There are eight possible custom characters, and the zero-based index
 * is used to select a character to define. Any existing character definition at
 * that index will be lost. Characters are 5 pixels wide and 8 pixels tall. The
 * pixelmap array consists of up to eight bytes, each byte representing the
 * pixel states per row. The first byte represents the top row. The eighth byte
 * is often left as zero (to leave space for the underline cursor). For each
 * row, the lowest five bits represent pixels that are to be illuminated. The
 * least significant bit represents the right-most pixel. Empty rows will be
 * zero.
 *
 *        NOTE: After calling this function, the DDRAM will not be selected and
 * the cursor position will be undefined. Therefore it is important that the
 * DDRAM address is set following this function, if text is to be written to the
 * display. This can be performed with a call to i2c_lcd1602_home() or
 * i2c_lcd1602_move_cursor().
 *
 *        Custom characters are written using the I2C_LCD1602_CHARACTER_CUSTOM_X
 * definitions.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @param[in] index Zero-based index of the character to define. Only values 0-7
 * are valid.
 * @param[in] pixelmap An 8-byte array defining the pixel map for the new
 * character definition.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_define_char(const i2c_lcd1602_info_t* i2c_lcd1602_info,
                                  i2c_lcd1602_custom_index_t index,
                                  const uint8_t pixelmap[]);

/**
 * @brief Write a single character to the display at the current position of the
 * cursor. Depending on the active mode, the cursor may move left or right, or
 * the display may shift left or right. Custom characters can be written using
 * the I2C_LCD1602_CHARACTER_CUSTOM_X definitions.
 *
 *        The display is I2C_LCD1602_NUM_COLUMNS wide, and upon encountering the
 * end of the first line, the cursor will automatically move to the beginning of
 * the second line.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_write_char(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t chr);

/**
 * @brief Write a string of characters to the display, starting at the current
 * position of the cursor. Depending on the active mode, the cursor may move
 * left or right, or the display may shift left or right, after each character
 * is written.
 *
 *        The display is I2C_LCD1602_NUM_COLUMNS wide, and upon encountering the
 * end of the first line, the cursor will automatically move to the beginning of
 * the second line.
 *
 * @param[in] i2c_lcd1602_info Pointer to initialised I2C-LCD1602 info instance.
 * @return ESP_OK if successful, otherwise an error constant.
 */
esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t* i2c_lcd1602_info, const char* string);

#ifdef __cplusplus
}
#endif

#endif  // __I2C_LCD1602_H__
