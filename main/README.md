O controlador LCD é o HD44780 que opera via barramento paralelo de 8 ou 4bits.

O LCD está conectado a um expansor de I/O PCF8574A através do barramento I2C.
Apenas os quatro bits superiores estão conectados às linhas de dados do controlador.
Os quatro bits inferiores são usados como linhas de controle:

   - B7: data bit 3
   - B6: data bit 2
   - B5: data bit 1
   - B4: data bit 0
   - B3: backlight (BL): off = 0, on = 1
   - B2: enable (EN): mudar de 1 para 0 para sincronizar os dados no controlador
   - B1: read/write (RW): escrita = 0, leitura = 1
   - B0: register select (RS): comando = 0, dado = 1

Portanto, para enviar um byte de comando são necessárias as seguintes operações:

   // Primeiro nibble:
   val = command & 0xf0              // extarai parte alta do byte
   val |= 0x04                       // RS = 0 (comando), RW = 0 (escrita), EN = 1
   i2c_write_byte(i2c_address, val)
   sleep(2ms)
   val &= 0xfb                       // EN = 0
   i2c_write_byte(i2c_address, val)

   // Segundo nibble:
   val = command & 0x0f              // extrai parte baixa do byte
   val |= 0x04                       // RS = 0 (command), RW = 0 (escrita), EN = 1
   i2c_write_byte(i2c_address, val)
   sleep(2ms)
   val &= 0xfb                       // EN = 0
   i2c_write_byte(i2c_address, val)

O envio de um byte de dados é muito semelhante, exceto que RS = 1 (dados)

Quando o controlador é ligado, ele é configurado para:

  - display apagado
  - interface de 8 bits, 1 linha, 5x8 pontos por caractere
  - incremento de cursor
  - sem deslocamento

O controlador deve ser configurado para operação de 4 bits antes que a comunicação adequada possa começar.
A sequência de inicialização para operação de 4 bits é:

   0. delay > 15ms após Vcc subir para 4,5V, ou > 40ms após Vcc subir para 2,7V
   1. enviar nibble 0x03     // seleciona interface de 8 bits
   2. delay > 4,1ms
   3. enviar nibble 0x03     // seleciona interface de 8 bits novamente
   4. delay > 100us
   5. enviar comando 0x32    // seleciona interface de 4 bits
   6. enviar comando 0x28    // seleciona 2 linhas e 5x7(8?) pontos por caractere
   7. enviar comando 0x0c    // display ligado, cursor desligado
   8. enviar comando 0x06    // move cursor para direita quando escreve, sem deslocamento
   9. enviar comando 0x80    // move cursor para posição inicial (linha 1, coluna 1)
