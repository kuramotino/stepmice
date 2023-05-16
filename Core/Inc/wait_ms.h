/*
 * wait_ms.h
 *
 *  Created on: Sep 21, 2022
 *      Author: Ryu
 */

#ifndef INC_WAIT_MS_H_
#define INC_WAIT_MS_H_

extern void pl_timer_init(void);
extern void pl_timer_count(void);
extern void wait_ms(uint32_t wait_time);
extern void TimReset(void);
extern float Counter(void);

#endif /* INC_WAIT_MS_H_ */
