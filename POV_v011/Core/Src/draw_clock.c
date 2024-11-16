/*
 * draw_clock.c
 *
 *  Created on: Sep 12, 2024
 *      Author: eml
 */


#include "draw_clock.h"

const uint32_t clock_frame[BUF_SIZE] = {
#include "frame_pixel_data.csv"
		}; // 時計のフレーム画像、動かない
const uint32_t hour_hand[HAND_BUF_SIZE] = {
#include "hour_pixel_data.csv"
		};
const uint32_t min_hand[HAND_BUF_SIZE] = {
#include "minute_pixel_data.csv"
		};
const uint32_t sec_hand[HAND_BUF_SIZE] = {
#include "second_pixel_data.csv"
		};

void clock_pixel_init(clock_pixel *cp, uint32_t *_pixel) {
	cp->hour = 0;
	cp->min = 0;
	cp->sec = 0;
	cp->pixel = _pixel;

	return;
}

void update_clock_pixel(clock_pixel *cp) {
	uint16_t hour_0 = __MAP12(cp->hour) + BUF_SIZE - (HAND_BUF_SIZE >> 1),
			min_0 = __MAP60(cp->min) + BUF_SIZE - (HAND_BUF_SIZE >> 1),
			sec_0 = __MAP60(cp->sec) + BUF_SIZE - (HAND_BUF_SIZE >> 1);
	uint16_t hour_i, min_i, sec_i;

	// 時針を分針に合わせて少しずつ動かす設定。いらないならコメントアウト
	hour_0 += min_0 / 12;

	// フレームを描画
	for (uint16_t i = 0; i < BUF_SIZE; i++) {
 		cp->buf[i] = clock_frame[i];
	}

	// 針のを重ねて描写
	for (uint16_t i = 0; i < HAND_BUF_SIZE; i++) {
		// 針の描写位置インデックスを生成
		hour_i = (hour_0 + i) % BUF_SIZE;
		min_i = (min_0 + i) % BUF_SIZE;
		sec_i = (sec_0 + i) % BUF_SIZE;

		// 描画バッファに重ねる
		cp->buf[hour_i] |= hour_hand[i];
		cp->buf[min_i] |= min_hand[i];
		cp->buf[sec_i] |= sec_hand[i];
	}

	// 実際に描画する
	for (uint16_t i = 0; i < BUF_SIZE; i++) {
		cp->pixel[i] = cp->buf[i];
	}

	return;
}
