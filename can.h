#ifndef CAN_H
#define CAN_H

#include <avr/io.h>

#include "h9msg.h"

void CAN_init(uint16_t node_id, void (*write_node_id_fun)(uint16_t));
void CAN_send_turned_on_broadcast();

void CAN_set_mob_for_remote_node(uint16_t remote_node_id);

void CAN_put_msg(h9msg_t *cm);
uint8_t CAN_get_msg(h9msg_t*cm);

void CAN_init_new_msg(h9msg_t *mes);
void CAN_init_response_msg(const h9msg_t *req, h9msg_t *res);

#endif /*CAN_H*/
