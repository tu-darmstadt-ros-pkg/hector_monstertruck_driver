/*
 * SerialUtil.hpp
 *
 *  Created on: 16.10.2014
 *      Author: christian
 */

#ifndef SERIALUTIL_HPP_
#define SERIALUTIL_HPP_

#define DEBUG false

void serialPrint(char * str) {
	if (!DEBUG) {
		return;
	}
	Serial.println(str);
}

template<class T>
void serialPrint(char * str, T value) {
	if (!DEBUG) {
		return;
	}
	Serial.print(str);
	Serial.println(value);
}

template<class T>
void serialPrint(char * str, T value, int format) {
	if (!DEBUG) {
		return;
	}
	Serial.print(str);
	Serial.println(value, format);
}

template<class T>
void serialPrintFormated( T value, int format) {
	if (!DEBUG) {
		return;
	}
	Serial.print(value, format);
	Serial.print(":");
}

#endif /* SERIALUTIL_HPP_ */
