#include "DelimitedStringParser.h"

//-------------------------------------------------------------------
DelimitedStringParser::DelimitedStringParser(char delimiter)
    : _delimiter(delimiter)
{
}

//-------------------------------------------------------------------
// parse() - tokenize input applying quoting and variable substitution
//-------------------------------------------------------------------
void DelimitedStringParser::parse(const String& input)
{
    _tokens.clear();

    String current;
    int i   = 0;
    int len = (int)input.length();

    while (i < len) {
        char c = input[i];

        if (c == '\'') {
            // ---- Single quotes: completely literal ----------------
            // Nothing inside is interpreted — not $VAR, not the
            // delimiter, not backslash.  Mirrors POSIX sh behaviour.
            i++; // consume opening '
            while (i < len && input[i] != '\'') {
                current += input[i++];
            }
            if (i < len) i++; // consume closing '
        }
        else if (c == '"') {
            // ---- Double quotes: substitution, no splitting --------
            // Variable references are expanded; the delimiter is
            // treated as an ordinary character inside the quotes.
            i++; // consume opening "
            String dqContent;
            while (i < len && input[i] != '"') {
                dqContent += input[i++];
            }
            if (i < len) i++; // consume closing "
            current += expandVars(dqContent);
        }
        else if (c == '$') {
            // ---- Variable substitution in unquoted text ----------
            i++; // consume '$'
            String varName = readVarName(input, i);
            current += getVar(varName);
        }
        else if (c == _delimiter) {
            // ---- Delimiter: end of current token -----------------
            // Shell behaviour: consecutive delimiters are collapsed;
            // empty tokens are not produced.
            if (current.length() > 0) {
                _tokens.push_back(current);
                current = "";
            }
            i++;
        }
        else {
            // ---- Ordinary character ------------------------------
            current += c;
            i++;
        }
    }

    // Flush the last token (no trailing delimiter required)
    if (current.length() > 0) {
        _tokens.push_back(current);
    }
}

//-------------------------------------------------------------------
int DelimitedStringParser::count() const
{
    return (int)_tokens.size();
}

String DelimitedStringParser::getToken(int index) const
{
    if (index < 0 || index >= (int)_tokens.size()) return String();
    return _tokens[index];
}

String DelimitedStringParser::operator[](int index) const
{
    return getToken(index);
}

//-------------------------------------------------------------------
// Variable management
//-------------------------------------------------------------------
void DelimitedStringParser::setVar(const String& name, const String& value)
{
    for (int i = 0; i < (int)_varNames.size(); i++) {
        if (_varNames[i] == name) {
            _varValues[i] = value;  // update existing
            return;
        }
    }
    _varNames.push_back(name);
    _varValues.push_back(value);
}

String DelimitedStringParser::getVar(const String& name) const
{
    for (int i = 0; i < (int)_varNames.size(); i++) {
        if (_varNames[i] == name) return _varValues[i];
    }
    return String(); // unknown variable → empty string (POSIX default)
}

bool DelimitedStringParser::hasVar(const String& name) const
{
    for (int i = 0; i < (int)_varNames.size(); i++) {
        if (_varNames[i] == name) return true;
    }
    return false;
}

void DelimitedStringParser::removeVar(const String& name)
{
    for (int i = 0; i < (int)_varNames.size(); i++) {
        if (_varNames[i] == name) {
            _varNames.erase(_varNames.begin() + i);
            _varValues.erase(_varValues.begin() + i);
            return;
        }
    }
}

void DelimitedStringParser::clearVars()
{
    _varNames.clear();
    _varValues.clear();
}

//-------------------------------------------------------------------
// Private helpers
//-------------------------------------------------------------------

// Read one variable name from s starting at pos.
// Supports:  $NAME  (alphanumeric + '_', stops at first non-matching char)
//            ${NAME}  (explicit braces, any chars until '}')
String DelimitedStringParser::readVarName(const String& s, int& pos) const
{
    String name;
    int len = (int)s.length();

    if (pos < len && s[pos] == '{') {
        pos++; // consume '{'
        while (pos < len && s[pos] != '}') {
            name += s[pos++];
        }
        if (pos < len) pos++; // consume '}'
    } else {
        while (pos < len && (isalnum((unsigned char)s[pos]) || s[pos] == '_')) {
            name += s[pos++];
        }
    }
    return name;
}

// Expand all $VAR / ${VAR} references within s.
// Single-quote sections are not present here (caller already handled them).
String DelimitedStringParser::expandVars(const String& s) const
{
    String result;
    int i   = 0;
    int len = (int)s.length();

    while (i < len) {
        if (s[i] == '$') {
            i++; // consume '$'
            String varName = readVarName(s, i);
            result += getVar(varName);
        } else {
            result += s[i++];
        }
    }
    return result;
}
