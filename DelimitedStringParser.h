#pragma once
#include <Arduino.h>
#include <vector>

//===================================================================
// DelimitedStringParser - shell-style tokenizer with variable substitution
//
// Splits an input string on a configurable delimiter character.
// Quoting rules follow POSIX shell conventions:
//
//   'single quotes'  - everything inside is literal; no substitution,
//                      no escape processing; the delimiter is ignored.
//
//   "double quotes"  - variable substitution ($VAR / ${VAR}) is
//                      performed inside, but the delimiter is ignored
//                      so the content becomes part of one token.
//
//   $VAR / ${VAR}    - replaced by the stored variable value.
//                      Unknown variables expand to an empty string.
//
// Consecutive unquoted delimiters are treated as one (shell behaviour):
//   "a  b  c" with ' ' delimiter → ["a", "b", "c"]
//
// Usage:
//   DelimitedStringParser p(' ');
//   p.setVar("HOST", "esp32.local");
//   p.parse("connect $HOST 8080");
//   // p[0]="connect"  p[1]="esp32.local"  p[2]="8080"
//
//   Quoting rules (POSIX shell)
//   ┌─────────────────┬───────────────────┬─────────────────────────┐
//   │    Construct    │ Delimiter splits? │     $VAR expanded?      │
//   ├─────────────────┼───────────────────┼─────────────────────────┤
//   │ 'single quotes' │ No                │ No — completely literal │
//   ├─────────────────┼───────────────────┼─────────────────────────┤
//   │ "double quotes" │ No                │ Yes                     │
//   ├─────────────────┼───────────────────┼─────────────────────────┤
//   │ Unquoted text   │ Yes               │ Yes                     │
//   └─────────────────┴───────────────────┴─────────────────────────┘

//   ---
//   Token access
//   DelimitedStringParser p(' ');
//   p.parse("ls -la /tmp");
//   p.count();      // 3
//   p[0];           // "ls"
//   p[1];           // "-la"
//   p[2];           // "/tmp"

//   Variable substitution
//   p.setVar("HOST", "esp32.local");
//   p.setVar("PORT", "8080");
//   p.parse("connect $HOST ${PORT}");
//   // → ["connect", "esp32.local", "8080"]

//   Mixed quoting
//   p.setVar("DIR", "/var/log");
//   p.parse("cat \"$DIR/app.log\" '$DIR/raw.log'");
//   // → ["cat", "/var/log/app.log", "$DIR/raw.log"]
//   //               ^^^^ expanded     ^^^^ literal

//   Variable management
//   p.setVar("X", "hello");
//   p.hasVar("X");       // true
//   p.getVar("X");       // "hello"
//   p.removeVar("X");
//   p.clearVars();
//
//===================================================================
class DelimitedStringParser {
public:
    // delimiter: character that separates tokens (e.g. ' ', ',', ':')
    explicit DelimitedStringParser(char delimiter = ' ');

    // Parse input string into tokens with variable substitution applied.
    // Clears any tokens from a previous call.
    void parse(const String& input);

    // Number of tokens from the last parse()
    int count() const;

    // Access a token by zero-based index.  Returns empty String if out of range.
    String getToken(int index) const;
    String operator[](int index) const;

    // --- Substitution variable management ---

    // Add a new variable or update an existing one.
    void setVar(const String& name, const String& value);

    // Retrieve a variable's value.  Returns empty String if not found.
    String getVar(const String& name) const;

    // Returns true if the variable exists.
    bool hasVar(const String& name) const;

    // Remove a single variable (no-op if not found).
    void removeVar(const String& name);

    // Remove all variables.
    void clearVars();

private:
    char                _delimiter;
    std::vector<String> _tokens;
    std::vector<String> _varNames;
    std::vector<String> _varValues;

    // Expand $VAR / ${VAR} references within s (used inside double quotes
    // and in unquoted text).
    String expandVars(const String& s) const;

    // Read one variable name from s starting at pos, advance pos past it.
    // Handles both bare $NAME and ${NAME} forms.
    String readVarName(const String& s, int& pos) const;
};
