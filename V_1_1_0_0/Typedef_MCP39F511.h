/*********************************************************************
 *
 *       Typedef
 *
 *********************************************************************
 * FileName:        Typedef.h
 * Revision:        1.0.0 (First issue)
 * Date:            04/05/2019
 *
 * Dependencies:    Arduino.h
 *
 * Arduino Board:   Arduino Uno, Arduino Mega 2560, Fishino Uno, Fishino Mega 2560       
 *
 * Company:         Futura Group srl
 *                  www.Futurashop.it
 *                  www.open-electronics.org
 *
 * Developer:       Destro Matteo
 *
 * Support:         info@open-electronics.org
 * 
 * Software License Agreement
 *
 * Copyright (c) 2016, Futura Group srl 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************/

#ifndef _TYPEDEF_H_INCLUDED__
#define _TYPEDEF_H_INCLUDED__

#include "Arduino.h"

typedef union {
    float	fl;
    uint8_t   Byte[8];
    struct {
        uint32_t	Long0;
        uint32_t	Long1;
    };
    struct {
        uint16_t	Word0;
        uint16_t	Word1;
        uint16_t	Word2;
        uint16_t	Word3;
    };
    struct {
        uint8_t	    Byte0;
        uint8_t	    Byte1;
        uint8_t	    Byte2;
        uint8_t	    Byte3;
        uint8_t	    Byte4;
        uint8_t	    Byte5;
        uint8_t	    Byte6;
        uint8_t	    Byte7;
    };
} t_LONG_LONG;
        
typedef union {
	long	l;
	uint32_t  uDWord;
    uint8_t   Byte[4];
	struct {
		uint16_t	Word0;
		uint16_t	Word1;
	};
	struct {
		uint8_t	Byte0;
		uint8_t	Byte1;
		uint8_t	Byte2;
		uint8_t	Byte3;
	};
} t_LONG;

typedef union {
	int		    i;
	uint16_t	uWord;
	struct {
		uint8_t	Byte0;
		uint8_t	Byte1;
	};
} t_SHORT;

typedef union {
	uint8_t	b;
	struct {
		uint8_t bit0	:1;
		uint8_t bit1	:1;
		uint8_t bit2	:1;
		uint8_t bit3	:1;
		uint8_t bit4	:1;
		uint8_t bit5	:1;
		uint8_t bit6	:1;
		uint8_t bit7	:1;
	};
} t_CHARUNION;

#endif