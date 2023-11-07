#ifndef __I2C_LCD1602_H__
#define __I2C_LCD1602_H__

#include <stdbool.h>
#include "smbus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool init;             // True se o display foi inicializado
    smbus_t* smbus;        // Ponteiro para a estrutura smbus
    uint8_t backlight;     // Backlight habilitado (1), desabilitado (0)
    uint8_t num_linhas;    // Número de linhas
    uint8_t num_colunas;   // Número de colunas
    uint8_t num_celulas;   // Número de células totais, incluindo as não visíveis
    uint8_t display_ctrl;  // Atividade atual do controle do display
    uint8_t modo_entrada;  // Modo de entrada ativo atual
} lcd_i2c_t;

// Caracteres especiais customizados para o código ROM A00
// Use o segundo conjunto (0bxxxx1xxx) para evitar colocar o caractere nulo dentro de uma string
#define CHAR_CUSTOM_00     0b00001000  // Caractere customizado no índice 0
#define CHAR_CUSTOM_01     0b00001001  // Caractere customizado no índice 1
#define CHAR_CUSTOM_02     0b00001010  // Caractere customizado no índice 2
#define CHAR_CUSTOM_03     0b00001011  // Caractere customizado no índice 3
#define CHAR_CUSTOM_04     0b00001100  // Caractere customizado no índice 4
#define CHAR_CUSTOM_05     0b00001101  // Caractere customizado no índice 5
#define CHAR_CUSTOM_06     0b00001110  // Caractere customizado no índice 6
#define CHAR_CUSTOM_07     0b00001111  // Caractere customizado no índice 7

#define CHAR_ALFA_MIN      0b11100000  // Alfa minúsculo
#define CHAR_BETA_MIN      0b11100010  // Beta minúsculo
#define CHAR_THETA_MIN     0b11110010  // Theta minúsculo
#define CHAR_PI_MIN        0b11110111  // Pi minúsculo
#define CHAR_OMEGA_MAI     0b11110100  // Omega maiúsculo
#define CHAR_SIGMA_MAI     0b11110110  // Sigma maiúsculo
#define CHAR_INFINITO      0b11110011  // Infinito
#define CHAR_GRAU          0b11011111  // Grau (°)
#define CHAR_SETA_DIREITA  0b01111110  // Seta para a direita
#define CHAR_SETA_ESQUERDA 0b01111111  // Seta para a esquerda
#define CHAR_2_SUBS        0b11011011  // 2 subscrito
#define CHAR_PONTO_CENTRO  0b10100101  // ponto no centro
#define CHAR_DIVISAO       0b11111101  // Divisão
#define CHAR_BLOCO         0b11111111  // Bloco 5x8 preenchido

// Caracteres customizados
typedef enum {
    CHAR_CUSTOM_INDICE_00 = 0,
    CHAR_CUSTOM_INDICE_01,
    CHAR_CUSTOM_INDICE_02,
    CHAR_CUSTOM_INDICE_03,
    CHAR_CUSTOM_INDICE_04,
    CHAR_CUSTOM_INDICE_05,
    CHAR_CUSTOM_INDICE_06,
    CHAR_CUSTOM_INDICE_07
} lcd_i2c_char_custom_indice_t;

/**
 * @brief Construtor de uma nova instância de I2C-LCD info.
 *        A nova instância deve ser inicializada antes de chamar outras funções.
 * @return Ponteiro para a nova instância de I2C-LCD info, ou NULL se não puder ser criada
 */
lcd_i2c_t* lcd_i2c_malloc(void);

/**
 * @brief Destrutor de uma instância de I2C-LCD info.
 * @param[in,out] lcd_i2c Ponteiro para a instância de I2C-LCD info que
 *                        será destruída e definida como NULL.
 */
void lcd_i2c_free(lcd_i2c_t** lcd_i2c);

/**
 * @brief Inicializa uma instância de I2C-LCD info com as informações SMBus.
 * @param[in] lcd_i2c     Ponteiro para a instância de I2C-LCD info.
 * @param[in] smbus       Ponteiro para a instância de SMBus info.
 * @param[in] backlight   Estado inicial do backlight.
 * @param[in] num_linhas  Número máximo de linhas suportadas para este dispositivo.
 *                        Valores típicos incluem 2 (1602) ou 4 (2004).
 * @param[in] num_celulas Número máximo de colunas suportadas para este dispositivo.
 *                        Valores típicos incluem 32 (1602) ou 80 (2004).
 * @param[in] num_colunas Número de colunas visíveis em qualquer momento.
 *                        Valores típicos incluem 16 (1602) ou 20 (2004).
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_init(lcd_i2c_t* lcd_i2c, smbus_t* smbus, bool backlight, uint8_t num_linhas, uint8_t num_celulas,
                       uint8_t num_colunas);

/**
 * @brief Reset LCD. Os caracteres customizados serão apagados.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_init_config(const lcd_i2c_t* lcd_i2c);

/**
 * @brief Limpa todo o display (limpa DDRAM) e retorna o cursor para a posição inicial.
 * O conteúdo do DGRAM é apaga, o conteúdo do CGRAM não é alterado.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_limpar_display(const lcd_i2c_t* lcd_i2c);

/**
 * @brief Move o cursor para a posição inicial. Também redefine qualquer deslocamento de exibição que possa ter
 * ocorrido. Os conteúdos do DGRAM e do CGRAM não são alterados.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_retornar_inicio(const lcd_i2c_t* lcd_i2c);

/**
 * @brief Move o cursor para uma coluna e linha específicas.
 * Isso é onde um novo caractere aparecerá.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @param[in] coluna  Índice da coluna para mover o cursor. A coluna 0 é a coluna mais à esquerda.
 * @param[in] linha   Índice da linha para mover o cursor. A linha 0 é a linha superior.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_mover_cursor(const lcd_i2c_t* lcd_i2c, uint8_t linha, uint8_t coluna);

/**
 * @brief Habilita ou desabilita o backlight do display.
 * @param[in] lcd_i2c  Ponteiro para a instância de I2C-LCD info.
 * @param[in] habilita True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_backlight(lcd_i2c_t* lcd_i2c, bool habilita);

/**
 * @brief Ativa ou desativa a exibição.
 * Quando desabilitada, a luz de fundo não é afetada, mas nenhum conteúdo
 * da DDRAM é exibido, nem o cursor. O display estará "em branco".
 * A reativação da exibição não afeta o conteúdo da DDRAM ou o estado ou posição do cursor.
 * @param[in] lcd_i2c  Ponteiro para a instância de I2C-LCD info.
 * @param[in] habilita True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_habilita_display(lcd_i2c_t* lcd_i2c, bool habilita);

/**
 * @brief Habilita ou desabilita a exibição do cursor sublinhado.
 * Se ativado, indica visualmente onde o próximo caractere escrito no display aparecerá.
 * Pode ser habilitado junto com o cursor piscando.
 * @param[in] lcd_i2c  Ponteiro para a instância de I2C-LCD info.
 * @param[in] habilita True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_config_cursor(lcd_i2c_t* lcd_i2c, bool habilita);

/**
 * @brief Ativa ou desativa a exibição do cursor de bloco piscante.
 * Se ativado, indica visualmente onde o próximo caractere escrito no display aparecerá.
 * Pode ser habilitado junto com o cursor sublinhado.
 * @param[in] lcd_i2c  Ponteiro para a instância de I2C-LCD info.
 * @param[in] habilita True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_config_cursor_piscante(lcd_i2c_t* lcd_i2c, bool habilita);

/**
 *
 * @brief Define a direção do movimento do cursor após a escrita de cada caractere.
 * O texto será escrito da esquerda para a direita.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_esquerda_para_direita(lcd_i2c_t* lcd_i2c);

/**
 * @brief Define a direção do movimento do cursor após a escrita de cada caractere.
 * O texto será escrito da direita para a esquerda.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_direita_para_esquerda(lcd_i2c_t* lcd_i2c);

/**
 * @brief Habilita ou desabilita a rolagem automática da tela.
 * Quando ativado, o display irá rolar conforme os caracteres são escritos para
 * manter a posição do cursor na tela.
 * O texto da esquerda para a direita aparecerá justificado à direita da posição do cursor.
 * Quando desativado, o display não rolará e o cursor se moverá na tela.
 * O texto da direita para a esquerda aparecerá justificado à esquerda da posição do cursor.
 * @param[in] lcd_i2c  Ponteiro para a instância de I2C-LCD info.
 * @param[in] habilita True para habilitar, false para desabilitar.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_rolagem_automatica(lcd_i2c_t* lcd_i2c, bool habilita);

/**
 * @brief Rola o display uma posição para a esquerda.
 * O texto na tela parecerá se mover para a direita.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_rolagem_esquerda(const lcd_i2c_t* lcd_i2c);

/**
 * @brief Rola o display uma posição para a direita.
 * O texto na tela parecerá se mover para a esquerda.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_rolagem_direita(const lcd_i2c_t* lcd_i2c);

/**
 * @brief Move o cursor uma posição para a esquerda, mesmo que esteja invisível.
 * Isso afeta onde o próximo caractere escrito no display aparecerá.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_mover_cursor_esquerda(const lcd_i2c_t* lcd_i2c);

/**
 * @brief Move o cursor uma posição para a direita, mesmo que esteja invisível.
 * Isso afeta onde o próximo caractere escrito no display aparecerá.
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_mover_cursor_direita(const lcd_i2c_t* lcd_i2c);

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
 * uma chamada para lcd_i2c_retornar_inicio() ou lcd_i2c_mover_cursor().
 *
 * Os caracteres personalizados são gravados usando as definições I2C_LCD1602_CHARACTER_CUSTOM_X.
 *
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @param[in] index Índice baseado em zero do caractere a ser definido. Somente os valores 0-7 são válidos.
 * @param[in] pixelmap Uma matriz de 8 bytes definindo o mapa de pixels para a nova definição de caractere.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_caracter_personalizado(const lcd_i2c_t* lcd_i2c, lcd_i2c_char_custom_indice_t index,
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
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_print_char(const lcd_i2c_t* lcd_i2c, uint8_t chr);

/**
 * @brief Escreve uma sequência de caracteres no display, começando na posição atual do cursor.
 * Dependendo do modo ativo, o cursor pode se mover para a esquerda ou para a direita,
 * ou o display pode se deslocar para a esquerda ou para a direita, após cada caracter ser escrito.
 *
 * A exibição tem largura I2C_LCD1602_NUM_COLUMNS e, ao encontrar o final
 * da primeira linha, o cursor se moverá automaticamente para o início
 * da segunda linha.
 *
 * @param[in] lcd_i2c Ponteiro para a instância de I2C-LCD info.
 * @return ESP_OK se bem sucedido, caso contrário, uma constante de erro.
 */
esp_err_t lcd_i2c_printf(const lcd_i2c_t* lcd_i2c, const char* string);

#ifdef __cplusplus
}
#endif

#endif  // __I2C_LCD1602_H__
