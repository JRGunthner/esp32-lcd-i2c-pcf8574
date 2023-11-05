#ifndef __SMBUS_H__
#define __SMBUS_H__

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SMBUS_DEFAULT_TIMEOUT (1000 / portTICK_RATE_MS)

// Endereco do dispositivo escravo de 7 ou 10 bits.
typedef uint16_t i2c_address_t;

// Estrutura contendo informações relacionadas ao protocolo SMBus.
typedef struct {
    bool init;              // Verdadeiro se a estrutura foi inicializada
    i2c_port_t i2c_port;    // Número da porta I2C
    i2c_address_t address;  // Endereço I2C do dispositivo escravo
    portBASE_TYPE timeout;  // Número de ticks para timeout
} smbus_info_t;

/**
 * @brief Constroi uma nova instância de SMBus info.
 *        A nova instância deve ser inicializada antes de chamar outras funções.
 * @return Ponteiro para a nova instância de SMBus info, ou NULL se não puder ser criada.
 */
smbus_info_t* smbus_malloc(void);

/**
 * @brief Deleta uma instância SMBus info existente.
 * @param[in,out] smbus_info Ponteiro para a instância SMBus info que será deletada e definida como NULL.
 */
void smbus_free(smbus_info_t** smbus_info);

/**
 * @brief Inicializa uma instância SMBus info com as informações I2C especificadas.
 *        O timeout I2C é definido como aproximadamente 1 segundo.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] i2c_port   Porta I2C associada a esta instância SMBus.
 * @param[in] address    Endereço do dispositivo escravo I2C.
 */
esp_err_t smbus_init(smbus_info_t* smbus_info, i2c_port_t i2c_port, i2c_address_t address);

/**
 * @brief Ajusta o timeout I2C.
 *        As transações I2C que não são concluídas dentro deste período são consideradas um erro.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] timeout    Número de ticks para esperar até que a transação seja considerada um erro.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_set_timeout(smbus_info_t* smbus_info, portBASE_TYPE timeout);

/**
 * @brief Envia um bit para um dispositivo escravo.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] bit        Bit a ser enviado.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_enviar_bit(const smbus_info_t* smbus_info, bool bit);

/**
 * @brief Envia um byte para um dispositivo escravo.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] data       Byte a ser enviado.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_enviar_byte(const smbus_info_t* smbus_info, uint8_t data);

/**
 * @brief Recebe um array de bytes de um dispositivo escravo.
 * @param[in]  smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[out] data       Dados recebidos do dispositivo escravo.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_receber_byte(const smbus_info_t* smbus_info, uint8_t* data);

/**
 * @brief Envia um byte para um dispositivo escravo com um código de comando.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] command    Código de comando específico do dispositivo.
 * @param[in] data       Byte a ser enviado.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_enviar_cmd_byte(const smbus_info_t* smbus_info, uint8_t command, uint8_t data);

/**
 * @brief Envia uma word (2 bytes) para um dispositivo escravo com um código de comando.
 *        O byte menos significativo é transmitido primeiro.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] command    Código de comando específico do dispositivo.
 * @param[in] data       Word a ser enviada.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_enviar_cmd_word(const smbus_info_t* smbus_info, uint8_t command, uint16_t data);

/**
 * @brief Lê um array de bytes de um dispositivo escravo com um código de comando.
 * @param[in]  smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in]  command    Código de comando específico do dispositivo.
 * @param[out] data       Dados recebidos do dispositivo escravo.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_receber_cmd_byte(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data);

/**
 * @brief Lê um array de words (2 bytes) de um dispositivo escravo com um código de comando.
 *        O byte menos significativo é colocado no primeiro local do array.
 * @param[in]  smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in]  command    Código de comando específico do dispositivo.
 * @param[out] data       Dados recebidos do dispositivo escravo.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_receber_cmd_word(const smbus_info_t* smbus_info, uint8_t command, uint16_t* data);

/**
 * 
 * @brief Escreve até 255 bytes em um dispositivo escravo com um código de comando.
 *        Esta função usa uma contagem de bytes para negociar o comprimento da transação.
 *        O primeiro byte no array de dados é transmitido primeiro.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] command    Código de comando específico do dispositivo.
 * @param[in] data       Dados a serem enviados.
 * @param[in] len        Número de bytes para enviar ao dispositivo escravo.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_enviar_cmd_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, uint8_t len);

/**
 * @brief Lê até 255 bytes de um dispositivo escravo com um código de comando.
 *        Esta função usa uma contagem de bytes para negociar o comprimento da transação.
 *        O primeiro byte recebido é colocado no primeiro local do array.
 * @param[in]  smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in]  command    Código de comando específico do dispositivo.
 * @param[out] data       Dados recebidos do dispositivo escravo.
 * @param[in]  len        Tamanho do array de dados, e número de bytes realmente recebidos.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_receber_cmd_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, uint8_t* len);

/**
 * @brief Escreve um array de bytes em um dispositivo escravo com um código de comando.
 *        Nenhuma contagem de bytes é usada - a transação dura o tempo que o mestre exigir.
 *        O primeiro byte na matriz de dados é transmitido primeiro.
 *        Esta operação não é definida pela especificação SMBus.
 * @param[in] smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in] command    Código de comando específico do dispositivo.
 * @param[in] data       Dados a serem enviados.
 * @param[in] len        Número de bytes para enviar ao dispositivo escravo.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_i2c_enviar_cmd_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, size_t len);

/**
 * @brief Lê bytes de um dispositivo escravo com um código de comando (formato combiado).
 *        Nenhuma contagem de bytes é usada - a transação dura o tempo que o mestre exigir.
 *        O primeiro byte recebido é colocado no primeiro local do array.
 *        Esta operação não é definida pela especificação SMBus.
 * @param[in]  smbus_info Ponteiro para a instância SMBus info inicializada.
 * @param[in]  command    Código de comando específico do dispositivo.
 * @param[out] data       Dados recebidos do dispositivo escravo.
 * @param[in]  len        Tamanho do array de dados. Se o escravo não fornecer bytes suficientes, ESP_ERR_TIMEOUT será retornado.
 * @return ESP_OK se bem sucedido, ESP_FAIL ou ESP_ERR_* se ocorreu um erro.
 */
esp_err_t smbus_i2c_receber_cmd_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif  // __SMBUS_H__
