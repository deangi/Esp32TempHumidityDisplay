#include "ConsoleInput.h"
//===================================================================
// Arduino console input class - allows users to type a line of
// input characters terminated by CR (Return) chr(13) key.
//
// Dean Gienger, Claude, March 19, 2026
//===================================================================

ConsoleInput::ConsoleInput(Stream* serial, int maxLen, bool ignoreNonPrintable)
    : _serial(serial),
      _maxLen(maxLen),
      _pos(0),
      _lineReady(false),
      _ignoreNonPrintable(ignoreNonPrintable)
{
    // +1 for null terminator
    _buf = new char[_maxLen + 1];
    _buf[0] = '\0';
}

ConsoleInput::~ConsoleInput() {
    delete[] _buf;
}

void ConsoleInput::poll() {
    // If a line is waiting to be consumed, don't overwrite it yet.
    if (_lineReady) return;

    while (_serial->available()) {
        char c = (char)_serial->read();

        // Backspace: chr(8) = BS, chr(127) = DEL
        if (c == '\b' || c == 127) {
            if (_pos > 0) {
                _pos--;
                _buf[_pos] = '\0';
                // Erase character on terminal: BS, space, BS
                _serial->print('\b');
                _serial->print(' ');
                _serial->print('\b');
            }
            continue;
        }

        // Line terminator: chr(13) = CR (ignore accompanying LF)
        if (c == '\r') {
            _buf[_pos] = '\0';
            _serial->println(); // move terminal to next line
            _lineReady = true;
            return;           // stop reading; wait for consumer
        }

        // Ignore LF (it often follows CR on some hosts)
        if (c == '\n') {
            continue;
        }

        // Printable range: 0x20 (space) through 0x7E (~)
        bool isPrintable = (c >= 0x20 && c <= 0x7E);

        if (!isPrintable && _ignoreNonPrintable) {
            continue;
        }

        // Accumulate (guard against overflow — force line if full)
        if (_pos < _maxLen) {
            _buf[_pos++] = c;
            _buf[_pos]   = '\0';
            if (isPrintable) {
                _serial->print(c); // echo
            }
        } else {
            // Buffer full: treat as implicit line terminator
            _buf[_pos] = '\0';
            _serial->println();
            _lineReady = true;
            return;
        }
    }
}

const char* ConsoleInput::getLine() {
    if (!_lineReady) return nullptr;

    // Caller gets the pointer now; reset state for next line on next poll().
    _lineReady = false;
    _pos       = 0;
    // Return the buffer as-is; it remains valid until the next line is completed.
    return _buf;
}
