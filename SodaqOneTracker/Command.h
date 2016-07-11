/*
Copyright (c) 2016, SODAQ
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

#ifndef COMMAND_H_
#define COMMAND_H_

#include <stdint.h>
#include <Arduino.h>

struct Command
{
public:
  const char *name;
  const char *cmd_prefix;
  void (*exec_func)(const Command *a, const char *line);
  void (*show_func)(const Command *a, Stream * stream);
  void *value;
  size_t param_size;

public:
  static bool execCommand(const Command args[], uint8_t nr, const char *line);
  static int findCommand(const Command args[], uint8_t nr, const char *line);

  static void show_name(const Command *s, Stream * stream);
  static void set_string(const Command *a, const char *line);
  static void show_string(const Command *a, Stream * stream);
  static void set_uint8(const Command *a, const char *line);
  static void show_uint8(const Command *a, Stream * stream);
  static void set_uint16(const Command *a, const char *line);
  static void show_uint16(const Command *a, Stream * stream);
  static void set_uint32(const Command *a, const char *line);
  static void show_uint32(const Command *a, Stream * stream);
};

#endif /* COMMAND_H_ */
