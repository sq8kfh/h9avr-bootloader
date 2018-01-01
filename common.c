#include "common.h"

volatile uint16_t can_node_id;
uint16_t ee_node_id EEMEM = 0;

void read_node_id(void) {
    uint16_t node_id = eeprom_read_word(&ee_node_id);
    if (node_id > 0 && node_id < H9_BROADCAST_ID) {
        can_node_id = node_id & ((1<<H9_SOURCE_ID_BIT_LENGTH)-1);
    }
    else {
        can_node_id = 0;
    }
}

void init_common_CAN(void) {
    read_node_id();

    CANGCON = ( 1 << SWRES );   // Software reset
    CANTCON = 0x00;             // CAN timing prescaler set to 0;

    #if F_CPU == 4000000UL
        CANBT1 = 0x06;
        CANBT2 = 0x04;
        CANBT3 = 0x13;
    #elif F_CPU == 16000000UL
        CANBT1 = 0x1e;
        CANBT2 = 0x04;
        CANBT3 = 0x13;
    #else
        #error "Please specify F_CPU"
    #endif

    for ( int8_t mob=0; mob<6; mob++ ) {
        CANPAGE = ( mob << MOBNB0 ); // Selects Message Object 0-5
        CANCDMOB = 0x00;             // Disable mob
        CANSTMOB = 0x00;             // Clear mob status register;
    }

    // 1st msg filter
    CANPAGE = 0x01 << MOBNB0; //select mob 1 for unicast
    set_CAN_id(0, 0, can_node_id, 0);
    set_CAN_id_mask(0, 0, (1<<H9_DESTINATION_ID_BIT_LENGTH)-1, 0);
    CANIDM4 |= 1 << IDEMSK; // set filter
    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only
}


void CAN_put_msg(h9msg_t *cm) {
    CANPAGE = 0 << MOBNB0;      		// Select MOb0 for transmission
    while ( CANEN2 & ( 1 << ENMOB0 ) ); // Wait for MOb 0 to be free
    CANSTMOB = 0x00;       				// Clear mob status register

    set_CAN_id(cm->priority, cm->type, cm->destination_id, cm->source_id);

    uint8_t idx = 0;
    for (; idx < 8; ++idx)
        CANMSG = cm->data[idx];

    CANCDMOB = (1 << CONMOB0) | (1 << IDE) | (cm->dlc & 0x0f);
}


void CAN_init_new_msg(h9msg_t *mes) {
    mes->priority = H9_PRIORITY_LOW;
    mes->source_id = can_node_id;
    mes->dlc = 0;
}


void set_CAN_id(uint8_t priority, uint8_t type, uint16_t destination_id, uint16_t source_id) {
    CANIDT1 = ((priority << 7) & 0x80) | ((type >> 1) & 0x7f);
    CANIDT2 = ((type << 7) & 0x80) | ((H9_RESERVED_VALUE << 5) & 0x60) | ((destination_id >> 4) & 0x1f);
    CANIDT3 = ((destination_id << 4) & 0xf0) | ((source_id >> 5) & 0x0f);
    CANIDT4 = ((source_id << 3) & 0xf8);
}


void set_CAN_id_mask(uint8_t priority, uint8_t type, uint16_t destination_id, uint16_t source_id) {
    CANIDM1 = ((priority << 7) & 0x80) | ((type >> 1) & 0x7f);
    CANIDM2 = ((type << 7) & 0x80) /*| ((H9_RESERVED_VALUE << 5) & 0x60)*/ | ((destination_id >> 4) & 0x1f);
    CANIDM3 = ((destination_id << 4) & 0xf0) | ((source_id >> 5) & 0x0f);
    CANIDM4 = ((source_id << 3) & 0xf8);
}