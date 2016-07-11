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

#include <stdint.h>
#include <Arduino.h>
#include "Command.h"

int Command::findCommand(const Command args[], uint8_t nr, const char *line)
{
  for (uint8_t i = 0; i < nr; ++i) {
    const Command *a = &args[i];
    if (strncasecmp(line, a->cmd_prefix, strlen(a->cmd_prefix)) == 0) {
      return i;
    }
  }
  return -1;
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
bool Command::execCommand(const Command args[], uint8_t nr, const char * line)
{
  int ix = findCommand(args, nr, line);
  if (ix < 0) {
    return false;
  }
  const Command *s = &args[ix];
  if (s->exec_func) {
    s->exec_func(s, line + strlen(s->cmd_prefix));
  }
  return true;
}

void Command::set_string(const Command *s, const char *line)
{
  char *ptr = (char *)s->value;
  if (ptr) {
    strncpy(ptr, line, s->param_size);
    ptr[s->param_size - 1] = '\0';
  }
}

void Command::set_uint8(const Command *s, const char *line)
{
  uint8_t *ptr = (uint8_t *)s->value;
  if (ptr) {
    *ptr = strtoul(line, NULL, 0);
  }
}

void Command::set_uint16(const Command *s, const char *line)
{
  uint16_t *ptr = (uint16_t *)s->value;
  if (ptr) {
    *ptr = strtoul(line, NULL, 0);
  }
}

void Command::set_uint32(const Command *s, const char *line)
{
  uint32_t *ptr = (uint32_t *)s->value;
  if (ptr) {
    *ptr = strtoul(line, NULL, 0);
  }
}

void Command::show_name(const Command *s, Stream * stream)
{
  // reinterpret_cast<const __FlashStringHelper *>(ptr)
  stream->print("  ");
  stream->print(s->name);
  stream->print(" (");
  stream->print(s->cmd_prefix);
  stream->print("): ");
}

void Command::show_string(const Command *s, Stream * stream)
{
  char *ptr = (char *)s->value;
  show_name(s, stream);
  if (ptr) {
    stream->println(ptr);
  } else {
    stream->println();
  }
}

void Command::show_uint8(const Command *s, Stream * stream)
{
  uint8_t *ptr = (uint8_t *)s->value;
  if (ptr) {
    show_name(s, stream);
    stream->println(*ptr);
  }
}

void Command::show_uint16(const Command *s, Stream * stream)
{
  uint16_t *ptr = (uint16_t *)s->value;
  if (ptr) {
    show_name(s, stream);
    stream->println(*ptr);
  }
}

void Command::show_uint32(const Command *s, Stream * stream)
{
  uint32_t *ptr = (uint32_t *)s->value;
  if (ptr) {
    show_name(s, stream);
    stream->println(*ptr);
  }
}
