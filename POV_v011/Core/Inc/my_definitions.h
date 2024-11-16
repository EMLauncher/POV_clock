/*
 * my_definitions.h
 *
 *  Created on: Sep 3, 2024
 *      Author: eml
 */

#ifndef INC_MY_DEFINITIONS_H_
#define INC_MY_DEFINITIONS_H_

#define DIG0_msk (1UL << 8) // PA8
#define DIG1_msk (1UL << 11) // PA11
#define DIG2_msk (1UL << 12) // PA12
#define DIG3_msk (1UL << 6) // PB6
#define LED8BIT_msk (255UL) // PA0..7

#define THETA_RESL 256 // divide 360 degrees by it
#define THETA_FIX_Pos 22 // q31 (2^31==4pi) to THETA_RESL==2pi

/*
 * memo
 * q31 theta: 2^31==4pi[rad]
 * q15 omega: 2^15==256pi[rad/s]
 * Ts=0.2msなのでomegaTsの計算は
 * omega * (256pi / 2^15) * (2^31 / 4pi) * 0.0002[s] = omega * 2^22 / 10000 = omega * 2^18 / 625
 */
// #define PLL_KP 3072 // Kp=6.0[(rad/s) / (rad)]
#define PLL_KP 4096 // Kp=8.0[(rad/s) / (rad)]
#define PLL_KP_DECIMAL 15 // Kpに合わせて調整
/*
 * 1000,15 OK 収束安定
 * 2000,15 OK 収束安定
 * 4000,15 OK 収束安定
 * 4500,15 OK 収束安定 <= とりあえずこれ使う
 * 5000,15 OK 収束やや不安定
 * 6554,15 OK 収束不安定
 * 10000,15 NG 収束しない
 */
#define PLL_TSTI 13107 // とりあえずテキトー
/*
 * 1000 OK 収束かなり遅い
 * 2000 OK 収束遅い
 * 6554==0.2 OK 収束遅い
 * 10000 OK
 * 13107==0.4 OK 収束速い <= とりあえずこれ使う
 * 16554==0.5 OK 若干揺らぎあり
 * 22938==0.7
 * 25214==0.8 OK なんか定常誤差ありそう
 */
#define OMEGA_MAX 7680 // 60pi[rad/s]
#define OMEGA_MIN 256 // 2pi[rad/s]
// #define __CALC_OMEGATS(w) ((((int32_t)(w) << 16) / 625L) << 3) // 200us
// #define __CALC_OMEGATS(w) ((((int32_t)(w) << 16) / 625L) << 2) // 100us
// #define __CALC_OMEGATS(w) ((((int32_t)(w) << 16) / 625L) << 1) // 50us
#define __CALC_OMEGATS(w) (((int32_t)(w) << 16) / 625L) // 25us
#define Q15_2PI (1U << 14) // 2^14==2pi
#define Q31_2PI (1UL << 30) // 2^30==2pi
#define Q31_4PI 0x7FFFFFFF // 2^31==4pi 厳密には2^31-1

#define NO_SW_FLAG 0x00
#define MIN_INC_FLAG 0x01
#define MIN_DEC_FLAG 0x02
#define HOUR_INC_FLAG 0x03
#define HOUR_DEC_FLAG 0x04

#endif /* INC_MY_DEFINITIONS_H_ */
