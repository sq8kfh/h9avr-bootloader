#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>

#include "h9msg.h"
#include "common.h"


void init_bootloader_CAN(void) {
    init_common_CAN();

    // 1st msg filter
    CANPAGE = 0x01 << MOBNB0;
    set_CAN_id(0, H9_TYPE_GROUP_0, can_node_id, 0);
    set_CAN_id_mask(0, H9_TYPE_GROUP_MASK, (1<<H9_DESTINATION_ID_BIT_LENGTH)-1, 0);
    CANIDM4 |= 1 << IDEMSK;
    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob, 29-bit only

	CANGCON = 1<<ENASTB;
}


void CAN_get_msg(h9msg_t *cm) {
    CANPAGE = 0x01 << MOBNB0;
    while (!(CANSTMOB & (1 << RXOK)));

    uint8_t canidt1 = CANIDT1;
    uint8_t canidt2 = CANIDT2;
    uint8_t canidt3 = CANIDT3;
    uint8_t canidt4 = CANIDT4;
    uint8_t cancdmob = CANCDMOB & 0x1f;

    for (uint8_t i = 0; i < 8; ++i) {
        cm->data[i] = CANMSG;
    }

    cm->priority = (canidt1 >> 7) & 0x01;
    cm->type = ((canidt1 << 1) & 0xfe) | ((canidt2 >> 7) & 0x01);
    cm->destination_id = ((canidt2 << 4) & 0x1f0) | ((canidt3 >> 4) & 0x0f);
    cm->source_id = ((canidt3 << 5) & 0x1e0) | ((canidt4 >> 3) & 0x1f);

    cm->dlc = cancdmob & 0x0f;

    CANCDMOB = (1<<CONMOB1) | (1<<IDE); //rx mob
    CANSTMOB = 0x00;
}


void write_page(uint16_t page, uint16_t dst_id) {
    uint16_t bytes_remain = SPM_PAGESIZE;
    page = page * SPM_PAGESIZE;

    boot_page_erase_safe(page);
    while (1) {
        h9msg_t cm;
        CAN_get_msg(&cm);
        if (cm.source_id == dst_id && cm.type == H9_TYPE_PAGE_FILL && cm.dlc == 8) {
            uint16_t w = cm.data[1] << 8;
            w |= cm.data[0];

            boot_page_fill_safe(page + (SPM_PAGESIZE-bytes_remain), w);
            bytes_remain -= 2;

            w = cm.data[3] << 8;
            w |= cm.data[2];

            boot_page_fill_safe(page + (SPM_PAGESIZE-bytes_remain), w);
            bytes_remain -= 2;

            w = cm.data[5] << 8;
            w |= cm.data[4];

            boot_page_fill_safe(page + (SPM_PAGESIZE-bytes_remain), w);
            bytes_remain -= 2;

            w = cm.data[7] << 8;
            w |= cm.data[6];

            boot_page_fill_safe(page + (SPM_PAGESIZE-bytes_remain), w);
            bytes_remain -= 2;

            h9msg_t cm_res;
            CAN_init_new_msg(&cm_res);
            cm_res.destination_id = dst_id;

            if (bytes_remain == 0) {
                cm_res.type = H9_TYPE_PAGE_WRITED;
                cm_res.dlc = 2;
                cm_res.data[0] = (page >> 8) & 0xff;
                cm_res.data[1] = (page) & 0xff;

                boot_page_write_safe(page);
                boot_spm_busy_wait();
                boot_rww_enable();

                CAN_put_msg(&cm_res);
                break;
            }
            else {
                cm_res.type = H9_TYPE_PAGE_FILL_NEXT;
                cm_res.dlc = 2;
                cm_res.data[0] = (bytes_remain >> 8) & 0xff;
                cm_res.data[1] = (bytes_remain) & 0xff;
                CAN_put_msg(&cm_res);
            }
        }
        else if (cm.source_id == dst_id && (cm.type & H9_TYPE_GROUP_MASK) == H9_TYPE_GROUP_0) {
            h9msg_t cm_res;
            CAN_init_new_msg(&cm_res);

            cm_res.destination_id = dst_id;
            cm_res.type = H9_TYPE_PAGE_FILL_BREAK;

            CAN_put_msg(&cm_res);
            break;
        }
    }
}

int main(void) {
	DDRB = 0xff;
	DDRC = 0xff;
	DDRD = 0xff;
	DDRE = 0xff;
    cli();
    MCUCR |= (1<<IVCE);
	MCUCR |= (1<<IVSEL);
    cli();
	PORTC = (PORTC & 0x0C) | (0xaa & 0xF3);
	PORTD = (PORTD & 0xFC) | ((0xaa>>2) & 0x03);
	/* Replace with your application code */
	
	init_bootloader_CAN();
	
	h9msg_t cm;
	CAN_init_new_msg(&cm);

	cm.type = H9_TYPE_ENTER_INTO_BOOTLOADER;
	cm.destination_id = 0x1ff;
	cm.dlc = 0;
	CAN_put_msg(&cm);
	
	while (1) {
        h9msg_t cm;
        CAN_get_msg(&cm);
        if (cm.type == H9_TYPE_PAGE_START && cm.dlc == 2) {
            uint16_t page = cm.data[0] << 8 | cm.data[1];

            h9msg_t cm_res;
            CAN_init_new_msg(&cm_res);

            cm_res.destination_id = cm.source_id;
            cm_res.type = H9_TYPE_PAGE_FILL_NEXT;
            cm_res.dlc = 2;
            cm_res.data[0] = (SPM_PAGESIZE >> 8) & 0xff;
            cm_res.data[1] = (SPM_PAGESIZE) & 0xff;

            CAN_put_msg(&cm_res);

            write_page(page, cm.source_id);
        }
        if (cm.type == H9_TYPE_QUIT_BOOTLOADER && cm.dlc == 0) {
            MCUCR &= ~(1<<IVSEL);
            asm volatile ("jmp	0x0000");
        }

	}
}


