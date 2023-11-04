/**
 * @file smbus.c
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
#define ACK_VALUE     0x0
#define NACK_VALUE    0x1
#define MAX_BLOCK_LEN 255  // SMBus v3.0 increases this from 32 to 255
// #define MEASURE         // enable measurement and reporting of I2C transaction duration

static bool _is_init(const smbus_info_t* smbus_info) {
    bool ok = false;
    if (smbus_info != NULL) {
        if (smbus_info->init) {
            ok = true;
        } else {
            ESP_LOGE(TAG, "smbus_info nao inicializado");
        }
    } else {
        ESP_LOGE(TAG, "smbus_info= NULL");
    }
    return ok;
}

static esp_err_t _check_i2c_error(esp_err_t err) {
    switch (err) {
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
            ESP_LOGE(TAG, "ERRO I2C: %d", err);
            break;
        }
    }
    return err;
}

esp_err_t _write_bytes(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | (DATA | As){*len} | P]

    esp_err_t err = ESP_FAIL;

    if (_is_init(smbus_info) && data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, command, ACK_CHECK);
        i2c_master_write(cmd, data, len, ACK_CHECK);
        i2c_master_stop(cmd);
#ifdef MEASURE
        uint64_t start_time = esp_timer_get_time();
#endif
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
#ifdef MEASURE
        ESP_LOGI(TAG, "_write_bytes: i2c_master_cmd_begin took %" PRIu64 " us", esp_timer_get_time() - start_time);
#endif
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t _read_bytes(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | (DATAs | A){*len-1} | DATAs | N | P]

    esp_err_t err = ESP_FAIL;

    if (_is_init(smbus_info) && data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, command, ACK_CHECK);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | READ_BIT, ACK_CHECK);

        if (len > 1)
            i2c_master_read(cmd, data, len - 1, ACK_VALUE);

        i2c_master_read_byte(cmd, &data[len - 1], NACK_VALUE);
        i2c_master_stop(cmd);
#ifdef MEASURE
        uint64_t start_time = esp_timer_get_time();
#endif
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
#ifdef MEASURE
        ESP_LOGI(TAG, "_read_bytes: i2c_master_cmd_begin took %" PRIu64 " us", esp_timer_get_time() - start_time);
#endif
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

// Public API

smbus_info_t* smbus_malloc(void) {
    smbus_info_t* smbus_info = malloc(sizeof(*smbus_info));
    if (smbus_info != NULL) {
        memset(smbus_info, 0, sizeof(*smbus_info));
        ESP_LOGD(TAG, "malloc smbus_info_t %p", smbus_info);
    } else {
        ESP_LOGE(TAG, "malloc smbus_info_t failed");
    }
    return smbus_info;
}

void smbus_free(smbus_info_t** smbus_info) {
    if (smbus_info != NULL && (*smbus_info != NULL)) {
        ESP_LOGD(TAG, "free smbus_info_t %p", *smbus_info);
        free(*smbus_info);
        *smbus_info = NULL;
    } else {
        ESP_LOGE(TAG, "free smbus_info_t failed");
    }
}

esp_err_t smbus_init(smbus_info_t* smbus_info, i2c_port_t i2c_port, i2c_address_t address) {
    if (smbus_info != NULL) {
        smbus_info->i2c_port = i2c_port;
        smbus_info->address = address;
        smbus_info->timeout = SMBUS_DEFAULT_TIMEOUT;
        smbus_info->init = true;
    } else {
        ESP_LOGE(TAG, "smbus_info is NULL");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t smbus_set_timeout(smbus_info_t* smbus_info, portBASE_TYPE timeout) {
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info)) {
        smbus_info->timeout = timeout;
        err = ESP_OK;
    }
    return err;
}

esp_err_t smbus_quick(const smbus_info_t* smbus_info, bool bit) {
    // Protocolo:
    // [S | ADDR | R/W | As | P]

    esp_err_t err = ESP_FAIL;

    if (_is_init(smbus_info)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | bit, ACK_CHECK);
        i2c_master_stop(cmd);
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t smbus_send_byte(const smbus_info_t* smbus_info, uint8_t data) {
    // Protocolo:
    // [S | ADDR | Wr | As | DATA | As | P]

    esp_err_t err = ESP_FAIL;

    if (_is_init(smbus_info)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, data, ACK_CHECK);
        i2c_master_stop(cmd);
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t smbus_receive_byte(const smbus_info_t* smbus_info, uint8_t* data) {
    // Protocolo:
    // [S | ADDR | Rd | As | DATAs | N | P]

    esp_err_t err = ESP_FAIL;

    if (_is_init(smbus_info)) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | READ_BIT, ACK_CHECK);
        i2c_master_read_byte(cmd, data, NACK_VALUE);
        i2c_master_stop(cmd);
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t smbus_write_byte(const smbus_info_t* smbus_info, uint8_t command, uint8_t data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | DATA | As | P]
    return _write_bytes(smbus_info, command, &data, 1);
}

esp_err_t smbus_write_word(const smbus_info_t* smbus_info, uint8_t command, uint16_t data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | DATA-LOW | As | DATA-HIGH | As | P]
    uint8_t temp[2] = {data & 0xff, (data >> 8) & 0xff};
    return _write_bytes(smbus_info, command, temp, 2);
}

esp_err_t smbus_read_byte(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | DATA | N | P]
    return _read_bytes(smbus_info, command, data, 1);
}

esp_err_t smbus_read_word(const smbus_info_t* smbus_info, uint8_t command, uint16_t* data) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | DATA-LOW | A | DATA-HIGH | N | P]
    esp_err_t err = ESP_FAIL;
    uint8_t temp[2] = {0};
    if (data) {
        err = _read_bytes(smbus_info, command, temp, 2);
        if (err == ESP_OK) {
            *data = (temp[1] << 8) + temp[0];
        } else {
            *data = 0;
        }
    }
    return err;
}

esp_err_t smbus_write_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, uint8_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | LEN | As | DATA-1 | As | DATA-2 | As ... | DATA-LEN | As | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info) && data) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, command, ACK_CHECK);
        i2c_master_write_byte(cmd, len, ACK_CHECK);
        for (size_t i = 0; i < len; ++i) {
            i2c_master_write_byte(cmd, data[i], ACK_CHECK);
        }
        i2c_master_stop(cmd);
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

esp_err_t smbus_read_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, uint8_t* len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | LENs | A | DATA-1 | A | DATA-2 | A ... | DATA-LEN | N
    // | P]
    esp_err_t err = ESP_FAIL;

    if (_is_init(smbus_info) && data && len) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, command, ACK_CHECK);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | READ_BIT, ACK_CHECK);
        uint8_t slave_len = 0;
        i2c_master_read_byte(cmd, &slave_len, ACK_VALUE);
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
        i2c_cmd_link_delete(cmd);

        if (err != ESP_OK) {
            *len = 0;
            return err;
        }

        if (slave_len > *len) {
            ESP_LOGW(TAG, "slave data length %d exceeds data len %d bytes", slave_len, *len);
            slave_len = *len;
        }

        cmd = i2c_cmd_link_create();
        for (size_t i = 0; i < slave_len - 1; ++i) {
            i2c_master_read_byte(cmd, &data[i], ACK_VALUE);
        }
        i2c_master_read_byte(cmd, &data[slave_len - 1], NACK_VALUE);
        i2c_master_stop(cmd);
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            *len = slave_len;
        } else {
            *len = 0;
        }
    }
    return err;
}

esp_err_t smbus_i2c_write_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | (DATA | As){*len} | P]
    return _write_bytes(smbus_info, command, data, len);
}

esp_err_t smbus_i2c_read_block(const smbus_info_t* smbus_info, uint8_t command, uint8_t* data, size_t len) {
    // Protocolo:
    // [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | (DATAs | A){*len-1} | DATAs | N | P]
    return _read_bytes(smbus_info, command, data, len);
}
