#ifndef _COMMON_H_
#define _COMMON_H_

#include <avr/io.h>
#include <avr/eeprom.h>

#include "h9msg.h"

extern uint16_t ee_node_id EEMEM;
extern volatile uint16_t can_node_id;

void read_node_id(void);

void init_common_CAN(void);
void CAN_put_msg(h9msg_t *cm);
void CAN_init_new_msg(h9msg_t *mes);
void set_CAN_id(uint8_t priority, uint8_t type, uint16_t destination_id, uint16_t source_id);
void set_CAN_id_mask(uint8_t priority, uint8_t type, uint16_t destination_id, uint16_t source_id);

#endif //_COMMON_H_
