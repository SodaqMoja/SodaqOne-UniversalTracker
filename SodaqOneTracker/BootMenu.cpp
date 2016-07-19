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

#include "BootMenu.h"

#include "Command.h"
#include "Config.h"

#define BOOT_MENU_TIMEOUT  (30 * 1000)

static VoidCallbackMethodPtr resetDevAddrOrEUItoHWEUICallback;

static bool isTimedOut(uint32_t ts)
{
    return (long)(millis() - ts) >= 0;
}

static void commitSettings(const Command* a, const char* line)
{
    params.commit(true);
}

static void resetDevAddrOrEUItoHWEUI(const Command* a, const char* line)
{
    if (resetDevAddrOrEUItoHWEUICallback) {
        resetDevAddrOrEUItoHWEUICallback();
    }
}

void setResetDevAddrOrEUItoHWEUICallback(VoidCallbackMethodPtr callback)
{
    resetDevAddrOrEUItoHWEUICallback = callback;
}

static const Command args[] = {
    { "Reset DevAddr / DevEUI to the Hardware EUI", "EUI", resetDevAddrOrEUItoHWEUI, Command::show_string },
    { "Commit Settings", "CS", commitSettings, Command::show_string }
};

static void showMyCommands(Stream* stream)
{
    size_t nr_cmnds = sizeof(args) / sizeof(args[0]);
    if (nr_cmnds == 0) {
        return;
    }
    stream->println("\r\nCommands:");
    for (size_t i = 0; i < nr_cmnds; ++i) {
        const Command* a = &args[i];
        if (a->show_func) {
            a->show_func(a, stream);
        }
    }
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
static bool execCommand(const char* line)
{
    return Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
}

static int readLine(Stream* stream, char line[], size_t size, uint32_t& ts_max)
{
    int c;
    size_t len = 0;
    bool seenCR = false;
    uint32_t ts_waitLF = 0;
    while (!isTimedOut(ts_max)) {
        if (seenCR) {
            c = stream->peek();
            // ts_waitLF is guaranteed to be non-zero
            if ((c == -1 && isTimedOut(ts_waitLF)) || (c != -1 && c != '\n')) {
                goto end;
            }
            // Only \n should fall through
        }

        c = stream->read();
        // Ignore NUL bytes too
        if (c <= 0) {
            continue;
        }

        // There is input, so extend the timeout
        ts_max = millis() + BOOT_MENU_TIMEOUT;

        seenCR = c == '\r';
        if (c == '\r') {
            stream->write((char)c);
            ts_waitLF = millis() + 50; // Wait another .05 sec for an optional LF
        }
        else if (c == '\n') {
            stream->write((char)c);
            goto end;
        }
        else if (c == 0x08 || c == 0x7f) {
            // Erase the last character
            if (len > 0) {
                stream->write("\b \b");
                --len;
            }
        }
        else {
            // Any "normal" character is stored in the line buffer
            if (len < size - 1) {
                if (c >= ' ' && c < 0x7f) {
                    stream->write((char)c);
                    line[len++] = c;
                }
            }
        }
    }
    // Timed out. Ignore the input.
    line[0] = '\0';
    return -1;

end:
    line[len] = '\0';
    return len;
}

static void showCommandPrompt(Stream* stream)
{
    showMyCommands(stream);
    ConfigParams::showConfig(stream);
    stream->print("Enter command: ");
}

void showBootMenu(Stream& stream)
{
    char buffer[200 + 1];
    int size;
    uint32_t ts_max = millis() + BOOT_MENU_TIMEOUT;
    bool needPrompt;
    bool seenCommand;
    uint8_t nrPrompts;

    needPrompt = true;
    nrPrompts = 0;

    while (!isTimedOut(ts_max) && nrPrompts < 250) {
        if (needPrompt) {
            showCommandPrompt(&stream);
            needPrompt = false;
            ++nrPrompts;
        }

        size = -1;
        while (stream.available() && stream.peek() == 0) {
            stream.read(); // Ignore initial NUL bytes on input
        }

        if (stream.available()) {
            size = readLine(&stream, buffer, sizeof(buffer), ts_max);
        }

        if (size < 0) {
            continue;
        }

        needPrompt = true;
        if (size == 0) {
            continue;
        }

        if (strcasecmp(buffer, "ok") == 0) {
            break;
        }

        seenCommand = false;

        // Is this a command for us?
        if (!seenCommand && execCommand(buffer)) {
            seenCommand = true;
        }
        // Is this a command for config?
        if (!seenCommand && params.execCommand(buffer)) {
            seenCommand = true;
        }

        if (seenCommand) {
            ts_max = millis() + BOOT_MENU_TIMEOUT;
        }
    }
    stream.println();
}
