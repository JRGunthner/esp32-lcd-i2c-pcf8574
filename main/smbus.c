/**
 * @brief Implementação do Protocoloo SMBus para o ESP32.
 * O SMBus é um Protocoloo de comunicação serial de baixa velocidade, baseado no Protocoloo I2C.
 * O diagrama do Protocoloo SMBus é representado como uma cadeia de símbolos "piped",
 * usando os seguintes símbolos:
 *
 *  - ADDR  : o endereço I2C de 7 bits de um barramento escravo.
 *  - S     : a condição START enviada por um barramento mestre.
 *  - Sr    : a condição REPEATED START enviada por um mestre.
 *  - P     : a condição STOP enviada por um mestre.
 *  - Wr    : bit 0 do byte de endereço indicando uma operação de escrita. O valor é 0.
 *  - Rd    : bit 0 do byte de endereço indicando uma operação de leitura. O valor é 1.
 *  - R/W   : bit 0 do byte de endereço, indicando uma operação de leitura ou escrita.
 *  - A     : bit ACKnowledge (ACK) enviado por um mestre.
 *  - N     : bit Not ACKnowledge (NACK) enviado por um mestre.
 *  - As    : bit ACKnowledge (ACK) enviado por um escravo.
 *  - Ns    : bit Not ACKnowledge (NACK) enviado por um escravo.
 *  - DATA  : byte de dados enviado por um mestre.
 *  - DATAs : byte de dados enviado por um escravo.
 */

// TODO: adicionar verificação de erro adequada em torno de todas as funções I2C

#include <stddef.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "smbus.h"

static const char* TAG = "SMBUS";

#define WRITE_BIT     I2C_MASTER_WRITE
#define READ_BIT      I2C_MASTER_READ
#define ACK_CHECK     true
#define NO_ACK_CHECK  false
#define ACK_VALUE     0
#define NACK_VALUE    1
#define MAX_BLOCK_LEN 255
// #define MEASURE         // permitir medição e relatórios da duração da transação I2C

static bool smbus_confirmar_init(const smbus_t* smbus) {
    bool ret = false;
    if (smbus != NULL) {
        if (smbus->init) {
            ret = true;
        } else {
            ESP_LOGE(TAG, "smbus nao inicializado");
        }
    } else {
        ESP_LOGE(TAG, "smbus= NULL");
    }
    return ret;
}

static esp_err_t smbus_verificar_falha_i2c(esp_err_t erro) {
    switch (erro) {
        case ESP_OK: {
            break;
        }
        case ESP_ERR_INVALID_ARG: {
            ESP_LOGE(TAG, "ERRO: parametro I2C");
            break;
        }
        case ESP_FAIL: {
            // Enviando comando de erro, escravo não ACK a transferência.
            ESP_LOGE(TAG, "I2C sem slave ACK");
            break;
        }
        case ESP_ERR_INVALID_STATE: {
            // I2C driver não instalado ou não no modo mestre.
            ESP_LOGE(TAG, "I2C driver nao instalado ou nao no modo mestre");
            break;
        }
        case ESP_ERR_TIMEOUT: {
            // Tempo limite de operação porque o barramento está ocupado.
            ESP_LOGE(TAG, "I2C timeout");
            break;
        }
        default: {
            ESP_LOGE(TAG, "ERRO I2C=%d", erro);
            break;
        }
    }
    return erro;
}

static esp_err_t smbus_protocolo_enviar_bytes(const smbus_t* smbus, uint8_t command, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | (DATA | As){*len} | P]

    esp_err_t err = ESP_FAIL;

    if (smbus_confirmar_init(smbus) && data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, command, ACK_CHECK);
        i2c_master_write(cmd, data, len, ACK_CHECK);
        i2c_master_stop(cmd);
#ifdef MEASURE
        uint64_t start_time = esp_timer_get_time();
#endif
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, cmd, smbus->timeout));
#ifdef MEASURE
        ESP_LOGI(TAG, "smbus_protocolo_enviar_bytes: i2c_master_cmd_begin took %" PRIu64 " us",
                 esp_timer_get_time() - start_time);
#endif
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

static esp_err_t smbus_protocolo_receber_bytes(const smbus_t* smbus, uint8_t command, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | (DATAs | A){*len-1} | DATAs | N | P]

    esp_err_t err = ESP_FAIL;

    if (smbus_confirmar_init(smbus) && data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, command, ACK_CHECK);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus->address << 1 | READ_BIT, ACK_CHECK);

        if (len > 1)
            i2c_master_read(cmd, data, len - 1, ACK_VALUE);

        i2c_master_read_byte(cmd, &data[len - 1], NACK_VALUE);
        i2c_master_stop(cmd);
#ifdef MEASURE
        uint64_t start_time = esp_timer_get_time();
#endif
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, cmd, smbus->timeout));
#ifdef MEASURE
        ESP_LOGI(TAG, "smbus_protocolo_receber_bytes: i2c_master_cmd_begin took %" PRIu64 " us",
                 esp_timer_get_time() - start_time);
#endif
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

smbus_t* smbus_malloc(void) {
    smbus_t* smbus = malloc(sizeof(*smbus));
    if (smbus != NULL) {
        memset(smbus, 0, sizeof(*smbus));
        ESP_LOGD(TAG, "malloc smbus_t %p", smbus);
    } else {
        ESP_LOGE(TAG, "malloc smbus_t falhou");
    }
    return smbus;
}

void smbus_free(smbus_t** smbus) {
    if (smbus != NULL && (*smbus != NULL)) {
        ESP_LOGD(TAG, "free smbus_t %p", *smbus);
        free(*smbus);
        *smbus = NULL;
    } else {
        ESP_LOGE(TAG, "free smbus_t falhou");
    }
}

esp_err_t smbus_init(smbus_t* smbus, i2c_port_t i2c_port, i2c_address_t address) {
    if (smbus != NULL) {
        smbus->i2c_port = i2c_port;
        smbus->address = address;
        smbus->timeout = SMBUS_TIMEOUT;
        smbus->init = true;
    } else {
        ESP_LOGE(TAG, "smbus eh NULL");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t smbus_set_timeout(smbus_t* smbus, portBASE_TYPE timeout) {
    esp_err_t err = ESP_FAIL;
    if (smbus_confirmar_init(smbus)) {
        smbus->timeout = timeout;
        err = ESP_OK;
    }
    return err;
}

esp_err_t smbus_enviar_bit(const smbus_t* smbus, bool bit) {
    // Protocolo:
    // [S | ADDR | R/W | As | P]

    esp_err_t err = ESP_FAIL;

    if (smbus_confirmar_init(smbus)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus->address << 1 | bit, ACK_CHECK);
        i2c_master_stop(cmd);
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, cmd, smbus->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t smbus_enviar_byte(const smbus_t* smbus, uint8_t data) {
    // Protocolo:
    // [S | ADDR | Wr | As | DATA | As | P]

    esp_err_t err = ESP_FAIL;

    if (smbus_confirmar_init(smbus)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, data, ACK_CHECK);
        i2c_master_stop(cmd);
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, cmd, smbus->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t smbus_receber_byte(const smbus_t* smbus, uint8_t* data) {
    // Protocolo:
    // [S | ADDR | Rd | As | DATAs | N | P]

    esp_err_t err = ESP_FAIL;

    if (smbus_confirmar_init(smbus)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus->address << 1 | READ_BIT, ACK_CHECK);
        i2c_master_read_byte(cmd, data, NACK_VALUE);
        i2c_master_stop(cmd);
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, cmd, smbus->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t smbus_enviar_cmd_byte(const smbus_t* smbus, uint8_t cmd, uint8_t data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | DATA | As | P]
    return smbus_protocolo_enviar_bytes(smbus, cmd, &data, 1);
}

esp_err_t smbus_enviar_cmd_word(const smbus_t* smbus, uint8_t cmd, uint16_t data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | DATA-LOW | As | DATA-HIGH | As | P]
    uint8_t temp[2] = {data & 0xff, (data >> 8) & 0xff};
    return smbus_protocolo_enviar_bytes(smbus, cmd, temp, 2);
}

esp_err_t smbus_receber_cmd_byte(const smbus_t* smbus, uint8_t cmd, uint8_t* data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | DATA | N | P]
    return smbus_protocolo_receber_bytes(smbus, cmd, data, 1);
}

esp_err_t smbus_receber_cmd_word(const smbus_t* smbus, uint8_t cmd, uint16_t* data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | DATA-LOW | A | DATA-HIGH | N | P]
    esp_err_t err = ESP_FAIL;

    uint8_t temp[2] = {0};
    if (data) {
        err = smbus_protocolo_receber_bytes(smbus, cmd, temp, 2);
        if (err == ESP_OK) {
            *data = (temp[1] << 8) + temp[0];
        } else {
            *data = 0;
        }
    }
    return err;
}

esp_err_t smbus_enviar_cmd_block(const smbus_t* smbus, uint8_t cmd, uint8_t* data, uint8_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | LEN | As | DATA-1 | As | DATA-2 | As ... | DATA-LEN | As | P]

    esp_err_t err = ESP_FAIL;

    if (smbus_confirmar_init(smbus) && data) {
        i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
        i2c_master_start(i2c_cmd);
        i2c_master_write_byte(i2c_cmd, smbus->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(i2c_cmd, cmd, ACK_CHECK);
        i2c_master_write_byte(i2c_cmd, len, ACK_CHECK);
        for (size_t i = 0; i < len; ++i) {
            i2c_master_write_byte(i2c_cmd, data[i], ACK_CHECK);
        }
        i2c_master_stop(i2c_cmd);
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, i2c_cmd, smbus->timeout));
        i2c_cmd_link_delete(i2c_cmd);
    }
    return err;
}

esp_err_t smbus_receber_cmd_block(const smbus_t* smbus, uint8_t cmd, uint8_t* data, uint8_t* len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | LENs | A | DATA-1 | A | DATA-2 | A ... | DATA-LEN | N
    // | P]

    esp_err_t err = ESP_FAIL;

    if (smbus_confirmar_init(smbus) && data && len) {
        i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
        i2c_master_start(i2c_cmd);
        i2c_master_write_byte(i2c_cmd, smbus->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(i2c_cmd, cmd, ACK_CHECK);
        i2c_master_start(i2c_cmd);
        i2c_master_write_byte(i2c_cmd, smbus->address << 1 | READ_BIT, ACK_CHECK);

        uint8_t slave_len = 0;
        i2c_master_read_byte(i2c_cmd, &slave_len, ACK_VALUE);
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, i2c_cmd, smbus->timeout));
        i2c_cmd_link_delete(i2c_cmd);

        if (err != ESP_OK) {
            *len = 0;
            return err;
        }

        if (slave_len > *len) {
            ESP_LOGW(TAG, "Tamanho do bloco de dados do escravo %d excede o tamanho do buffer %d bytes", slave_len,
                     *len);
            slave_len = *len;
        }

        i2c_cmd = i2c_cmd_link_create();
        for (size_t i = 0; i < slave_len - 1; ++i) {
            i2c_master_read_byte(i2c_cmd, &data[i], ACK_VALUE);
        }
        i2c_master_read_byte(i2c_cmd, &data[slave_len - 1], NACK_VALUE);
        i2c_master_stop(i2c_cmd);
        err = smbus_verificar_falha_i2c(i2c_master_cmd_begin(smbus->i2c_port, i2c_cmd, smbus->timeout));
        i2c_cmd_link_delete(i2c_cmd);

        if (err == ESP_OK) {
            *len = slave_len;
        } else {
            *len = 0;
        }
    }
    return err;
}

esp_err_t smbus_i2c_enviar_cmd_block(const smbus_t* smbus, uint8_t cmd, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | (DATA | As){*len} | P]
    return smbus_protocolo_enviar_bytes(smbus, cmd, data, len);
}

esp_err_t smbus_i2c_receber_cmd_block(const smbus_t* smbus, uint8_t cmd, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | (DATAs | A){*len-1} | DATAs | N | P]
    return smbus_protocolo_receber_bytes(smbus, cmd, data, len);
}
