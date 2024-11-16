/*
 * draw_clock.h
 *
 *  Created on: Sep 12, 2024
 *      Author: eml
 */

#ifndef INC_DRAW_CLOCK_H_
#define INC_DRAW_CLOCK_H_

#include <stdint.h>

#define BUF_SIZE 256 // 全画像のバッファ
#define HAND_BUF_SIZE 9 // 時計の針の描画バッファ, 奇数にすること

#define __MAP60(i) (((uint16_t)(i) << 8) / 60) // 0-60 => 0-256へmap
#define __MAP12(i) (((uint16_t)(i) << 8) / 12) // 0-12 => 0-256へmap

typedef struct {
	uint8_t hour, min, sec;
	volatile uint32_t buf[BUF_SIZE]; // 途中演算用バッファ
	volatile uint32_t *pixel; // 実際に描画しているピクセル値データ (main側で配列を登録する)
} clock_pixel;

void clock_pixel_init(clock_pixel *cp, uint32_t *_pixel);
void update_clock_pixel(clock_pixel *cp);

#endif /* INC_DRAW_CLOCK_H_ */
