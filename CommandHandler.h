#pragma once
#include <Arduino.h>
#include <vector>
#include "DelimitedStringParser.h"

//===================================================================
// CommandHandler - command dispatch with registered handlers
//
// Uses DelimitedStringParser to tokenise input, then dispatches
// args[0] to a registered handler function.
//
// Error model
// -----------
//   execute()      returns an int error code.
//   lastError()    returns the code from the most recent execute().
//   lastErrorMsg() returns a human-readable string for that code.
//
//   Built-in codes (≤ 0):
//     CMD_OK           (0)  success
//     CMD_ERR_EMPTY   (-1)  input was empty / whitespace only
//     CMD_ERR_UNKNOWN (-2)  no handler registered for that command name
//
//   Handlers may return any positive int for command-specific errors
//   and should populate errMsg when they do so.
//
// Handler signature
// -----------------
//   int myHandler(const DelimitedStringParser& args, String& errMsg)
//   {
//       // args[0] = command name, args[1..] = arguments
//       if (args.count() < 2) { errMsg = "usage: myCmd <arg>"; return 1; }
//       ...
//       return CMD_OK;
//   }
//
// Usage
// -----
//   CommandHandler cmd(' ');
//   cmd.addCommand("ls",  lsHandler,  "list files");
//   cmd.addCommand("cat", catHandler, "print file");
//
//   cmd.parser().setVar("ROOT", "/spiffs");
//
//   int rc = cmd.execute("cat $ROOT/config.ini");
//   if (rc != CMD_OK)
//       Serial.println(cmd.lastErrorMsg());
// Here's a quick integration sketch for how it plugs into WorldClock.ino:
//
//   #include "CommandHandler.h"
//
//   CommandHandler cmd(' ');
//
//   // Handler example
//   int lsHandler(const DelimitedStringParser& args, String& errMsg) {
//       listDir(SPIFFS, "/", 9);
//       return CMD_OK;
//   }

//   int catHandler(const DelimitedStringParser& args, String& errMsg) {
//       if (args.count() < 2) { errMsg = "usage: cat <file>"; return 1; }
//       readFile(SPIFFS, args[1].c_str());
//       return CMD_OK;
//   }

//   // In setup():
//   cmd.addCommand("ls",  lsHandler,  "list files");
//   cmd.addCommand("cat", catHandler, "print file contents");
//   cmd.parser().setVar("CFG", "/config.ini");

//   // In cmdHandler() / loop():
//   int rc = cmd.execute(inputLine);
//   if (rc != CMD_OK)
//       zprintf("error %d: %s\r\n", rc, cmd.lastErrorMsg().c_str());

//   Design notes:
//   - addCommand is idempotent — calling it again on the same name updates the handler and description in place
//   - execute() runs _parser.parse() internally, so variables set via cmd.parser().setVar(...) are automatically available
//    in all commands
//   - The handler receives const DelimitedStringParser& so it can check args.count(), use args[1], args[2], etc.; args[0]
//   is always the command name
//   - Error state is always overwritten on each execute() call, so lastError()/lastErrorMsg() always reflect the most
//   recent result
//===================================================================

#define CMD_OK           0
#define CMD_ERR_EMPTY   -1
#define CMD_ERR_UNKNOWN -2

// Every command handler must match this signature.
//   args   - parser after tokenising the input; args[0] is the command name.
//   errMsg - set this to a descriptive message on failure.
//   return - CMD_OK (0) on success, non-zero on failure.
typedef int (*CommandHandlerFunc)(const DelimitedStringParser& args, String& errMsg);

//-------------------------------------------------------------------
class CommandHandler {
public:
    // delimiter is forwarded to the internal DelimitedStringParser.
    explicit CommandHandler(char delimiter = ' ');

    // --- Command registration ---

    // Register handler for name.  Replaces any existing entry silently.
    // description is optional; used by help/query methods.
    void addCommand(const String& name,
                    CommandHandlerFunc handler,
                    const String& description = "");

    // Remove a command by name (no-op if not found).
    void removeCommand(const String& name);

    // Returns true if a handler is registered for name.
    bool hasCommand(const String& name) const;

    // Returns the description for a command (empty String if not found).
    String getDescription(const String& name) const;

    // Number of registered commands.
    int commandCount() const;

    // Name of the command at zero-based index idx (empty String if out of range).
    String getCommandName(int idx) const;

    // --- Execution ---

    // Parse input and dispatch to the matching handler.
    // Returns CMD_OK, CMD_ERR_EMPTY, CMD_ERR_UNKNOWN, or a handler error code.
    int execute(const String& input);

    // --- Last-result accessors ---

    int    lastError()    const { return _lastError; }
    String lastErrorMsg() const { return _lastErrorMsg; }

    // --- Parser access ---

    // Direct access to the internal parser, e.g. to set substitution variables
    // before calling execute():   cmd.parser().setVar("X","val");
    DelimitedStringParser&       parser()       { return _parser; }
    const DelimitedStringParser& parser() const { return _parser; }

private:
    struct CommandEntry {
        String             name;
        String             description;
        CommandHandlerFunc handler;
    };

    DelimitedStringParser     _parser;
    std::vector<CommandEntry> _commands;
    int                       _lastError;
    String                    _lastErrorMsg;

    // Internal lookup — returns nullptr when not found.
    const CommandEntry* findEntry(const String& name) const;
          CommandEntry* findEntry(const String& name);
};
