/*
 * Based on: SQ_StartupCommands.cpp by Kees Bakker
 */

#include "BootMenu.h"

#include "Command.h"
#include "Config.h"

#define BOOT_MENU_TIMEOUT  (30 * 1000)

static bool isTimedOut(uint32_t ts)
{
    return (long)(millis() - ts) >= 0;
}

static void commitSettings(const Command* a, const char* line)
{
    params.commit();
}

static const Command args[] = {
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
