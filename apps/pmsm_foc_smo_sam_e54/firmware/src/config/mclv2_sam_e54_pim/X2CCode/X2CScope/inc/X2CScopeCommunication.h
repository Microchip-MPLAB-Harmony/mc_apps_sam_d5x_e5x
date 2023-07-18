/*
 * Copyright (c) 2013, Linz Center of Mechatronics GmbH (LCM)
 * All rights reserved.
 */
/*
 * This file is licensed according to the BSD 3-clause license as follows:
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the "Linz Center of Mechatronics GmbH" and "LCM" nor
 *       the names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL "Linz Center of Mechatronics GmbH" BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * This file is part of X2C.
 */
#ifndef X2CSCOPECOMMUNICATION_H
#define	X2CSCOPECOMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include "stdbool.h"
#include "definitions.h"

typedef void (*SERIAL_SEND)( uint8_t data );
typedef uint8_t (*SERIAL_RECEIVE)( void );
typedef uint8_t (*SERIAL_DATA_AVAILABLE)( void );
typedef uint8_t (*SERIAL_SEND_READY)( void );

void X2CScope_Initialise(void);
void X2CScope_HookUARTFunctions(SERIAL_SEND sendAPI, SERIAL_RECEIVE receiveAPI, \
                                SERIAL_DATA_AVAILABLE dataAvailableAPI, SERIAL_SEND_READY sendReadyAPI );

void sendSerial(uint8_t data);
uint8_t receiveSerial(void);
uint8_t isReceiveDataAvailable(void);
uint8_t isSendReady(void);

#ifdef __cplusplus
}
#endif

#endif	/* X2CSCOPECOMMUNICATION_H */