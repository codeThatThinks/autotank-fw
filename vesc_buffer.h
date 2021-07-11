/*
	Copyright 2016 Benjamin Vedder  benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VESC_BUFFER_H
#define VESC_BUFFER_H


#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>


void buffer_append_int16(uint8_t *buffer, int16_t number, size_t *index);
void buffer_append_uint16(uint8_t *buffer, uint16_t number, size_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t number, size_t *index);
void buffer_append_uint32(uint8_t *buffer, uint32_t number, size_t *index);
void buffer_append_float16(uint8_t *buffer, float number, float scale, size_t *index);
void buffer_append_float32(uint8_t *buffer, float number, float scale, size_t *index);
void buffer_append_float32_auto(uint8_t *buffer, float number, size_t *index);
uint8_t buffer_get_uint8(const uint8_t *buffer, size_t *index, size_t len);
int16_t buffer_get_int16(const uint8_t *buffer, size_t *index, size_t len);
uint16_t buffer_get_uint16(const uint8_t *buffer, size_t *index, size_t len);
int32_t buffer_get_int32(const uint8_t *buffer, size_t *index, size_t len);
uint32_t buffer_get_uint32(const uint8_t *buffer, size_t *index, size_t len);
float buffer_get_float16(const uint8_t *buffer, float scale, size_t *index, size_t len);
float buffer_get_float32(const uint8_t *buffer, float scale, size_t *index, size_t len);
float buffer_get_float32_auto(const uint8_t *buffer, size_t *index, size_t len);
String buffer_get_string(const uint8_t *buffer, size_t *index, size_t len);


#endif // VESC_BUFFER_H
