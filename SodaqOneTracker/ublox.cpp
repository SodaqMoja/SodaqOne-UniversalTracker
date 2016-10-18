/**
 * Copyright 2016 Willem Eradus
 * minor change (ifdef DEBUG in db_printf) by sodaq
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **/

#include "ublox.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

//#define DEBUG

void UBlox::db_printf(const char *message,...) {
#ifdef DEBUG
    static char buffer[128];
    va_list args;
    va_start(args, message);
    vsnprintf(buffer,128, message, args);
    SerialUSB.print(buffer);
    va_end(args);
#endif
}

UBlox::UBlox ():
    Wire_(Wire) {
    address_ = 0x42;
    //
    Id_ = AckedId_ = plLength_ = 0;
     p_ = NULL;
#ifdef ARDUINO_SODAQ_LORAONE
    pinMode(GPS_ENABLE, OUTPUT);
    pinMode(GPS_TIMEPULSE, INPUT);
#endif
}

UBlox::UBlox (TwoWire& aWire,uint8_t address):
    Wire_(aWire) {
    address_ = address;
    //
    Id_ = AckedId_ = plLength_ = 0;
    p_ = NULL;
    //
#if defined(ARDUINO_SODAQ_LORAONE) || defined(ARDUINO_SODAQ_ONE)
    pinMode(GPS_ENABLE, OUTPUT);
    pinMode(GPS_TIMEPULSE, INPUT);
#endif
}

void UBlox::enable () {
    this->reset();
#if defined(ARDUINO_SODAQ_LORAONE) || defined(ARDUINO_SODAQ_ONE)
    digitalWrite(GPS_ENABLE, 1);
    db_printf("uBlox enabled\n");
#endif
}

void UBlox::disable () {
    this->reset();
#if defined(ARDUINO_SODAQ_LORAONE) || defined(ARDUINO_SODAQ_ONE)
    digitalWrite(GPS_ENABLE, 0);
    db_printf("uBlox disabled\n");
#endif
}

void UBlox::flush() {
    this->reset();
    uint16_t bytes = this->available();
    if (bytes) {
        Wire.requestFrom(address_, bytes);
        do {
            (void) Wire.read();
        } while (--bytes);
    }
}

void UBlox::reset() {
    state_ = 0;
}

bool UBlox::exists() const
{
    Wire.beginTransmission(address_);
    return (Wire.endTransmission() == 0);
}
/*
 UBX Packet Structure
 A basic UBX Packet looks as follows:
 • Every Message starts with 2 Bytes: 0xB5 0x62
 • A 1 Byte Class Field follows. The Class defines the basic subset of the message
 • A 1 Byte ID Field defines the message that is to follow
 • A 2 Byte Length Field is following. Length is defined as being the length of the payload, only. 
    It does not include Sync Chars, Length Field, Class, ID or CRC fields. 
    The number format of the length field is an unsigned 16-Bit integer in Little Endian Format.
 • The Payload is a variable length field.
 • CK_A and CK_B is a 16 Bit checksum whose calculation is defined below.
 
 • The checksum algorithm used is the 8-Bit Fletcher Algorithm, which is used in the TCP standard (RFC 1145). This algorithm works as follows:
 • Buffer[N] contains the data over which the checksum is to be calculated.
 • The two CK_ values are 8-Bit unsigned integers, only!
 */

int UBlox::process(uint8_t c) {
    static uint8_t ck_a,ck_b;
    if (state_ == 0 && c == 0xb5)
        state_ = 1;
    else if (state_ == 1) {
        if (c == 0x62)
            state_ = 2;
        else
            state_ = 0;
    }
    else if (state_ == 2) {
        ck_a = c; ck_b = c;
        state_ = 3;
        Id_ = (uint16_t) c << 8;
    }
    else if (state_ == 3) {
        ck_a += c; ck_b += ck_a;
        state_ = 4;
        Id_ |= (uint16_t) c;
        // db_printf("Id=%4.4x\n",Id);
    }
    else if (state_ == 4) {
        ck_a += c; ck_b += ck_a;
        plLength_ = c;
        state_ = 5;
    }
    else if (state_ == 5) {
        ck_a += c; ck_b += ck_a;
        plLength_ |= (uint16_t) c << 8;
        state_ = 6;
        if (Id_ == 0x500 || Id_ == 0x501) // ACK/NAK buffer
            p_ = (uint8_t *) &AckedId_;
        else {
            payLoad_.length = plLength_;
            p_ = payLoad_.buffer;
        }
    }
    else if (state_ == 6) {
        ck_a += c; ck_b += ck_a;
        *p_++ = c;
        if (--plLength_ == 0) {
            state_ = 7;
        }
    }
    else if (state_ == 7) {
        if (c == ck_a)
            state_=8;
        else
            state_ = 0;
    }
    else if (state_ == 8) {
        state_ = 0;
        if (ck_b == c) {
            if (Id_ == 0x0501 || Id_ == 0x500) {
                AckedId_ = (AckedId_ >> 8) | (AckedId_ << 8);  // Change Endianess
                // db_printf("Id=%4.4x Ack=%4.4x\n",Id,AckedId);
            }
            return Id_;
        }
    }
    return (0-state_);
}

int UBlox::available() {
    Wire_.beginTransmission(address_);
    Wire_.write((uint8_t)0xfd);
    Wire_.endTransmission(false);
    Wire_.requestFrom(address_, 2);//
    uint16_t bytes = (uint16_t) Wire_.read() << 8;
    bytes |= Wire_.read();
    return bytes;
}

void UBlox::GetPeriodic() {
    int bytes = this->available();
    if (bytes)
        return this->GetPeriodic(bytes);
}

void UBlox::dispatchMessage(int id) {
    if (id > 0 && id != 0x0501 && id != 0x0500) { // Valid and not equal to ACK or NAK
    // Process UBX packet
        switch (id) {
        case 0x0107:
            if (funcNavPvt) {
                if (payLoad_.length == 84 || payLoad_.length == 92) // ublox-7 == 84 ublox-8 == 92
                    this->funcNavPvt(
                            (NavigationPositionVelocityTimeSolution*) payLoad_.buffer);
                else
                    db_printf("Oops %d %d\n",payLoad_.length,sizeof(NavigationPositionVelocityTimeSolution));
            }
            break;
        default:
            //
            break;
        }
    }
}

void UBlox::GetPeriodic (int bytes) {
    this->db_printf("%d bytes available\n",bytes);
    if (bytes) {
        do {
            uint8_t read;       // Holds actual read
            if (bytes >= 128)
                read = Wire_.requestFrom(address_, 128);
            else
                read = Wire_.requestFrom(address_, bytes);
            bytes -= read;
            while (Wire_.available()) {
                uint8_t c = Wire.read();
                int id = this->process(c);  // id < 0 state of state machine, id > 0 valid UBX packet found, id == 0 no match for UBX
                this->dispatchMessage(id);
            }
        } while (bytes);
    }
}

int UBlox::send(uint8_t *buffer,int n) {
    uint8_t ck_a,ck_b;
    ck_a = ck_b = 0;
    //
    for (int i=0;i<n;i++) {
        ck_a += buffer[i];
        ck_b += ck_a;
    }
    //
    Wire_.beginTransmission(address_);
    Wire_.write(0xb5);
    Wire_.write(0x62);
    Wire_.write(buffer, n);
    Wire_.write(ck_a);
    Wire_.write(ck_b);
    return Wire_.endTransmission();
}

void UBlox::sendraw() {
    (void) this->send(payLoad_.buffer,payLoad_.length);
    int id = this->wait();
    this->db_printf("UBlox::sendraw() == %4.4x\n",id);
}

void UBlox::CfgMsg(uint16_t Msg,uint8_t rate) {
    uint8_t buffer[7];
    //
    buffer[0] = 0x06;
    buffer[1] = 0x01;
    buffer[2] = 0x03;
    buffer[3] = 0;
    buffer[4] = (Msg >> 8) & 0xff;
    buffer[5] = Msg & 0xff;
    buffer[6] = rate;
    // Push message on Wire
    (void) this->send(buffer,7);
    this->wait();
}

int UBlox::setTimePulseParameters (TimePulseParameters *Tpp) {
    // Warning this overwrites the receive buffer !!!
    payLoad_.buffer[0] = 0x06;
    payLoad_.buffer[1] = 0x31;
    payLoad_.buffer[2] = 32;
    payLoad_.buffer[3] = 0;
    memcpy(&payLoad_.buffer[4],(uint8_t*)Tpp,32);
    // Push message on Wire
    (void) this->send(payLoad_.buffer,36);
    return this->wait();
}

bool UBlox::getTimePulseParameters(uint8_t tpIdx,TimePulseParameters* tpp) {
    uint8_t buffer[5];
    //
    buffer[0] = 0x06;
    buffer[1] = 0x31;
    buffer[2] = 1;
    buffer[3] = 0;
    buffer[4] = tpIdx;
    // Push message on Wire
    (void) this->send(buffer,5);
    // wait for response
    return this->wait(0x0631,sizeof(TimePulseParameters),tpp);
}

int UBlox::setPortConfigurationDDC (PortConfigurationDDC *pcd) {
    // Warning this overwrites the receive buffer !!!
    payLoad_.buffer[0] = 0x06;
    payLoad_.buffer[1] = 0x00;
    payLoad_.buffer[2] = 20;
    payLoad_.buffer[3] = 0;
    memcpy(&payLoad_.buffer[4],(uint8_t*)pcd,20);
    // Push message on Wire
    (void) this->send(payLoad_.buffer,24);
    return this->wait();
}

bool UBlox::getPortConfigurationDDC(PortConfigurationDDC* pcd) {
    uint8_t buffer[4];
    //
    buffer[0] = 0x06;
    buffer[1] = 0x00;
    buffer[2] = 0;
    buffer[3] = 0;
    // Push message on Wire
    (void) this->send(buffer,4);
    // wait for response
    return this->wait(0x0600,sizeof(PortConfigurationDDC),pcd);
}

int UBlox::wait() {
    uint32_t s = millis(),elapsed;
    uint16_t bytes;
    int16_t id = 0;
    // Wait 50 ms for response
    while ((elapsed = millis()-s) < 50 && (bytes = this->available()) == 0 );
    if (bytes) {
        do {
            uint8_t read;                    // Holds actual read
            if (bytes >= 128)
                read = Wire_.requestFrom(address_, 128);
            else
                read = Wire_.requestFrom(address_, bytes);
            bytes -= read;
            while (Wire_.available()) {
                uint8_t c = Wire.read();
                id = this->process(c);  
            }
        } while (bytes);
    }
    return id;
}

bool UBlox::wait(uint16_t rid,int reqLength,void *d) {
    uint32_t s = millis(),elapsed;
    uint16_t bytes;
    bool found = false;
    // Wait 50 ms for response
    while ((elapsed = millis()-s) < 50 && (bytes = this->available()) == 0 );
    if (bytes) {
        do {
            uint8_t read;                    // Holds actual read
            if (bytes >= 128)
                read = Wire_.requestFrom(address_, 128);
            else
                read = Wire_.requestFrom(address_, bytes);
            bytes -= read;
            while (Wire_.available()) {
                uint8_t c = Wire.read();
                int pid = this->process(c);  // id < 0 state of state machine, id > 0 valid UBX packet found, id == 0 no match for UBX
                if (pid == rid) {            // procesed id == requested id
                    if (payLoad_.length == reqLength) {
                        memcpy(d,payLoad_.buffer,reqLength);
                        found = true;
                    }
                    else
                        db_printf("Oops %d %d\n",payLoad_.length,reqLength);
                }
                else if (pid > 0)
                	this->dispatchMessage(pid);
            }
        } while (bytes);
        //
    }
    return found;
}
