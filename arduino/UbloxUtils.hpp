/*
 * UbloxUtils.hpp
 *
 *  Created on: 17.10.2014
 *      Author: christian
 */

#ifndef UBLOXUTILS_HPP_
#define UBLOXUTILS_HPP_

void ubloxSetHeader(uint8_t * message, uint8_t message_class,
		uint8_t message_type, uint16_t length) {
	message[0] = 0xB5;
	message[1] = 0x62;
	message[2] = message_class;
	message[3] = message_type;
	message[4] = length;
	message[5] = length >> 8;
}

void ubloxSetShort(uint8_t * message, uint32_t position, uint16_t value) {
	message[position] = (uint8_t) value;
	message[position + 1] = (uint8_t) (value >> 8);
}

void ubloxSetInt(uint8_t * message, uint32_t position, uint32_t value){
	message[position] = (uint8_t) value;
	message[position + 1] = (uint8_t) (value >> 8);
	message[position + 2] = (uint8_t) (value >> 16);
	message[position + 3] = (uint8_t) (value >> 24);
}

void ubloxSetChecksum(uint8_t * message, uint32_t length) {
	uint8_t chk_A = 0;
	uint8_t chk_B = 0;

	for (int i = 2; i < length; i++) {
		chk_A += message[i];
		chk_B += chk_A;
	}
	message[length] = chk_A;
	message[length + 1] = chk_B;
}



#endif /* UBLOXUTILS_HPP_ */
