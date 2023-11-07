#ifndef __LCD_H__
#define __LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

void lcd_init(void);
void lcd_teste(void);

void lcd_cursor(uint8_t linha, uint8_t coluna);
void lcd_print_char(char c);
void lcd_printf(char* s);
void lcd_limpar(void);
void lcd_habilita_cursor(bool habilita);
void lcd_cursor_piscar(bool habilita);
void lcd_backlight(bool habilita);
void lcd_direita_para_esquerda(void);
void lcd_esquerda_para_direita(void);
void lcd_habilita_display(bool habilita);
void lcd_reset(void);
void lcd_rolagem_esquerda(uint8_t casas, uint16_t transicao);
void lcd_rolagem_direita(uint8_t casas, uint16_t transicao);
void lcd_rolagem_automatica(bool habilita);
void lcd_mover_cursor_direita(uint8_t casas);
void lcd_mover_cursor_esquerda(uint8_t casas);

// TODO: implementar
void lcd_caracter_personalizado(uint8_t indice, uint8_t* caracter);

#ifdef __cplusplus
}
#endif

#endif // __LCD_H__
