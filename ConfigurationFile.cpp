#include "ConfigurationFile.h"

ConfigurationFile::ConfigurationFile(const char* filename)
    : _filename(filename)
{
}

bool ConfigurationFile::get(const char* key, char* outbuf, int maxlen) const
{
    outbuf[0] = '\0';

    File finp = SPIFFS.open(_filename, FILE_READ);
    if (!finp) {
        Serial.print("ConfigurationFile: cannot open ");
        Serial.println(_filename);
        return false;
    }

    // Build the search prefix: key + '='
    char prefix[64];
    snprintf(prefix, sizeof(prefix), "%s=", key);
    int prefixLen = strlen(prefix);

    char buf[128];
    bool found = false;
    while (readln(finp, buf, sizeof(buf) - 1)) {
        if (buf[0] == '#') continue;          // skip comment lines
        if (strncmp(buf, prefix, prefixLen) == 0) {
            // strncpy may not NUL-terminate if the source fills the buffer.
            // Bound the copy at maxlen-1 and force NUL at the last index so
            // we never write past the caller's buffer (the previous version
            // wrote outbuf[maxlen], one byte past the end).
            strncpy(outbuf, buf + prefixLen, maxlen - 1);
            outbuf[maxlen - 1] = '\0';
            Serial.print("Config ");
            Serial.print(prefix);
            Serial.println(outbuf);
            found = true;
            break;
        }
    }
    finp.close();
    return found;
}

String ConfigurationFile::get(const char* key) const
{
    char buf[128] = "";
    get(key, buf, sizeof(buf) - 1);
    return String(buf);
}

bool ConfigurationFile::readln(File& finp, char* buf, int maxlen)
{
    int  len = 0;
    bool eof = false;

    buf[0] = '\0';
    while (len < maxlen) {
        if (!finp.available()) { eof = true; break; }
        char c = (char)finp.read();
        if (c < 0)  { eof = true; break; }
        if (c == 13) continue;   // ignore CR
        if (c == 10) break;      // LF = end of line
        buf[len++] = c;
    }
    buf[len] = '\0';
    return !eof;
}
