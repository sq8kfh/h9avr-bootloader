/*
 * h9avr-bootloader
 *
 * Created by SQ8KFH on 2020-07-02.
 *
 * Copyright (C) 2020 Kamil Palkowski. All rights reserved.
 */

#ifndef _CAN_H_
#define _CAN_H_

#include <avr/io.h>
#include <avr/eeprom.h>

#include "h9msg.h"

extern uint16_t ee_node_id EEMEM;
extern volatile uint16_t can_node_id;

void CAN_init(void);

void CAN_put_msg_blocking(h9msg_t *cm);
uint8_t CAN_get_msg_blocking(h9msg_t *cm);

#endif //_CAN_H_
