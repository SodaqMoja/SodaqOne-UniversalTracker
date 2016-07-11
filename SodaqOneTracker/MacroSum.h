/*
Copyright (c) 2016, Alex Tsamakos

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MACROSUM_H_
#define MACROSUM_H_

#include "MacroOverload.h"

#define SUM(...) VFUNC(SUM, __VA_ARGS__)
#define SUM1(_1) (_1)
#define SUM2(_1,_2) (_1+_2)
#define SUM3(_1,_2,_3) (_1+_2+_3)
#define SUM4(_1,_2,_3,_4) (_1+_2+_3+_4)
#define SUM5(_1,_2,_3,_4,_5) (_1+_2+_3+_4+_5)
#define SUM6(_1,_2,_3,_4,_5,_6) (_1+_2+_3+_4+_5+_6)
#define SUM7(_1,_2,_3,_4,_5,_6,_7) (_1+_2+_3+_4+_5+_6+_7)
#define SUM8(_1,_2,_3,_4,_5,_6,_7,_8) (_1+_2+_3+_4+_5+_6+_7+_8)
#define SUM9(_1,_2,_3,_4,_5,_6,_7,_8,_9) (_1+_2+_3+_4+_5+_6+_7+_8+_9)
#define SUM10(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10) (_1+_2+_3+_4+_5+_6+_7+_8+_9+_10)
#define SUM11(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11) (_1+_2+_3+_4+_5+_6+_7+_8+_9+_10+_11)
#define SUM12(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12) (_1+_2+_3+_4+_5+_6+_7+_8+_9+_10+_11+_12)
#define SUM13(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13) (_1+_2+_3+_4+_5+_6+_7+_8+_9+_10+_11+_12+_13)
#define SUM14(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14) (_1+_2+_3+_4+_5+_6+_7+_8+_9+_10+_11+_12+_13+_14)
#define SUM15(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15) (_1+_2+_3+_4+_5+_6+_7+_8+_9+_10+_11+_12+_13+_14+_15)
#define SUM16(_1,_2,_3,_4,_5,_6,_7,_8,_9,_10,_11,_12,_13,_14,_15,_16) (_1+_2+_3+_4+_5+_6+_7+_8+_9+_10+_11+_12+_13+_14+_15+_16)

#endif /* MACROSUM_H_ */
