#ifndef CAN_H
#define CAN_H

#include "h9msg.h"

#define CAN_RX_BUF_SIZE 16
#define CAN_RX_BUF_INDEX_MASK 0x0F

typedef struct {
	uint8_t canidt1;
	uint8_t canidt2;
	uint8_t canidt3;
	uint8_t canidt4;
	uint8_t cancdmob;
	uint8_t data[8];
} can_buf_t;

typedef struct can_std_registries {
	uint16_t node_id;
} can_std_registries_t;

extern can_std_registries_t can_std_reg;

void CAN_init(uint16_t node_id);
void CAN_reinit(uint16_t node_id);
void CAN_send_turned_on_broadcast();
void CAN_set_mob(uint8_t priority, uint8_t priority_mask, uint8_t type, uint8_t type_mask, uint16_t destination_id, uint16_t destination_id_mask, uint16_t source_id, uint16_t source_id_mask);

void CAN_put_msg(h9msg_t *cm);
uint8_t CAN_get_msg(h9msg_t*cm);
void CAN_init_new_msg(h9msg_t *mes);
void CAN_init_response_msg(const h9msg_t *req, h9msg_t *res);

#endif /*CAN_H*/
