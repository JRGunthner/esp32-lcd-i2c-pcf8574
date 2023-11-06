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
 * @brief Destrutor de uma instância de I2C-LCD info.
 * @param[in,out] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info que
 *                                 será destruída e definida como NULL.
 */
void i2c_lcd1602_free(i2c_lcd1602_info_t** tsl2561_info);

/**
 * @brief Inicializa uma instância de I2C-LCD info com as informações SMBus.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] smbus_info Ponteiro para a instância de SMBus info.
 * @param[in] backlight Estado inicial do backlight.
 * @param[in] num_rows Número máximo de linhas suportadas para este dispositivo.
 *                     Valores típicos incluem 2 (1602) ou 4 (2004).
 * @param[in] num_columns Número máximo de colunas suportadas para este dispositivo.
 *                        Valores típicos incluem 32 (1602) ou 80 (2004).
 * @param[in] num_visible_columns Número de colunas visíveis em qualquer momento.
 *                               Valores típicos incluem 16 (1602) ou 20 (2004).
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t* i2c_lcd1602_info,
                           smbus_info_t* smbus_info, bool backlight,
                           uint8_t num_rows, uint8_t num_columns,
                           uint8_t num_visible_columns);

/**
 * @brief Reset LCD. Os caracteres customizados serão apagados.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Limpa todo o display (limpa DDRAM) e retorna o cursor para a posição inicial.
 * O conteúdo do DGRAM é apaga, o conteúdo do CGRAM não é alterado.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move o cursor para a posição inicial. Também redefine qualquer deslocamento de exibição que possa ter ocorrido.
 * Os conteúdos do DGRAM e do CGRAM não são alterados.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move o cursor para uma coluna e linha específicas.
 * Isso é onde um novo caractere aparecerá.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] col Índice da coluna para mover o cursor. A coluna 0 é a coluna mais à esquerda.
 * @param[in] row Índice da linha para mover o cursor. A linha 0 é a linha superior.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t col, uint8_t row);

/**
 * @brief Habilita ou desabilita o backlight do display.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] enable True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_set_backlight(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Ativa ou desativa a exibição.
 * Quando desabilitada, a luz de fundo não é afetada, mas nenhum conteúdo
 * da DDRAM é exibido, nem o cursor. O display estará "em branco".
 * A reativação da exibição não afeta o conteúdo da DDRAM ou o estado ou posição do cursor.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] enable True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_set_display(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Habilita ou desabilita a exibição do cursor sublinhado.
 * Se ativado, indica visualmente onde o próximo caractere escrito no display aparecerá.
 * Pode ser habilitado junto com o cursor piscando.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] enable True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_set_cursor(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Ativa ou desativa a exibição do cursor de bloco piscante.
 * Se ativado, indica visualmente onde o próximo caractere escrito no display aparecerá.
 * Pode ser habilitado junto com o cursor sublinhado.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] enable True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_set_blink(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * 
 * @brief Define a direção do movimento do cursor após a escrita de cada caractere.
 * O texto será escrito da esquerda para a direita.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_set_left_to_right(i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Define a direção do movimento do cursor após a escrita de cada caractere.
 * O texto será escrito da direita para a esquerda.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_set_right_to_left(i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Habilita ou desabilita a rolagem automática da tela.
 * Quando ativado, o display irá rolar conforme os caracteres são escritos para
 * manter a posição do cursor na tela.
 * O texto da esquerda para a direita aparecerá justificado à direita da posição do cursor.
 * Quando desativado, o display não rolará e o cursor se moverá na tela.
 * O texto da direita para a esquerda aparecerá justificado à esquerda da posição do cursor.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] enable True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_set_auto_scroll(i2c_lcd1602_info_t* i2c_lcd1602_info, bool enable);

/**
 * @brief Rola o display uma posição para a esquerda.
 * O texto na tela parecerá se mover para a direita.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_scroll_display_left(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Rola o display uma posição para a direita.
 * O texto na tela parecerá se mover para a esquerda.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_scroll_display_right(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move o cursor uma posição para a esquerda, mesmo que esteja invisível.
 * Isso afeta onde o próximo caractere escrito no display aparecerá.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_move_cursor_left(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Move o cursor uma posição para a direita, mesmo que esteja invisível.
 * Isso afeta onde o próximo caractere escrito no display aparecerá.
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_move_cursor_right(const i2c_lcd1602_info_t* i2c_lcd1602_info);

/**
 * @brief Define um caractere personalizado a partir de um mapa de pixels.
 * Existem oito caracteres personalizados possíveis e o índice baseado em zero é
 * usado para selecionar um caractere a ser definido. Qualquer definição de caractere
 * existente nesse índice será perdida. Os caracteres têm 5 pixels de largura e
 * 8 pixels de altura. A matriz pixelmap consiste em até oito bytes, cada byte
 * representando os estados dos pixels por linha. O primeiro byte representa a linha
 * superior. O oitavo byte geralmente é deixado como zero (para deixar espaço para o
 * cursor sublinhado). Para cada linha, os cinco bits mais baixos representam pixels
 * que serão iluminados. O bit menos significativo representa o pixel mais à direita.
 * As linhas vazias serão iguais a zero.
 *
 * Obs.: Após chamar esta função, a DDRAM não será selecionada e a posição do cursor
 * ficará indefinida. Portanto, é importante que o endereço DDRAM seja definido seguindo
 * esta função, se for necessário gravar texto no display. Isso pode ser realizado com
 * uma chamada para i2c_lcd1602_home() ou i2c_lcd1602_move_cursor().
 *
 * Os caracteres personalizados são gravados usando as definições I2C_LCD1602_CHARACTER_CUSTOM_X.
 *
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @param[in] index Índice baseado em zero do caractere a ser definido. Somente os valores 0-7 são válidos.
 * @param[in] pixelmap Uma matriz de 8 bytes definindo o mapa de pixels para a nova definição de caractere.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_define_char(const i2c_lcd1602_info_t* i2c_lcd1602_info,
                                  i2c_lcd1602_custom_index_t index,
                                  const uint8_t pixelmap[]);

/**
 * @brief Escreva um único caractere no display na posição atual do cursor.
 * Dependendo do modo ativo, o cursor pode mover-se para a esquerda ou para
 * a direita, ou o ecrã pode deslocar-se para a esquerda ou para a direita.
 * Caracteres personalizados podem ser gravados usando as
 * definições I2C_LCD1602_CHARACTER_CUSTOM_X.
 *
 * A exibição tem largura I2C_LCD1602_NUM_COLUMNS e, ao encontrar o final
 * da primeira linha, o cursor se moverá automaticamente para o início
 * da segunda linha.
 * 
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_write_char(const i2c_lcd1602_info_t* i2c_lcd1602_info, uint8_t chr);

/**
 * @brief Escreve uma sequência de caracteres no display, começando na posição atual do cursor.
 * Dependendo do modo ativo, o cursor pode se mover para a esquerda ou para a direita,
 * ou o display pode se deslocar para a esquerda ou para a direita, após cada caracter ser escrito.
 *
 * A exibição tem largura I2C_LCD1602_NUM_COLUMNS e, ao encontrar o final
 * da primeira linha, o cursor se moverá automaticamente para o início
 * da segunda linha.
 *
 * @param[in] i2c_lcd1602_info Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t* i2c_lcd1602_info, const char* string);

#ifdef __cplusplus
}
#endif

#endif  // __I2C_LCD1602_H__
