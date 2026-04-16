#include "CommandHandler.h"

//-------------------------------------------------------------------
CommandHandler::CommandHandler(char delimiter)
    : _parser(delimiter), _lastError(CMD_OK)
{
}

//-------------------------------------------------------------------
// Command registration
//-------------------------------------------------------------------

void CommandHandler::addCommand(const String& name,
                                CommandHandlerFunc handler,
                                const String& description)
{
    // Replace if the command already exists
    CommandEntry* e = findEntry(name);
    if (e) {
        e->handler     = handler;
        e->description = description;
        return;
    }
    CommandEntry entry;
    entry.name        = name;
    entry.handler     = handler;
    entry.description = description;
    _commands.push_back(entry);
}

void CommandHandler::removeCommand(const String& name)
{
    for (int i = 0; i < (int)_commands.size(); i++) {
        if (_commands[i].name == name) {
            _commands.erase(_commands.begin() + i);
            return;
        }
    }
}

bool CommandHandler::hasCommand(const String& name) const
{
    return findEntry(name) != nullptr;
}

String CommandHandler::getDescription(const String& name) const
{
    const CommandEntry* e = findEntry(name);
    return e ? e->description : String();
}

int CommandHandler::commandCount() const
{
    return (int)_commands.size();
}

String CommandHandler::getCommandName(int idx) const
{
    if (idx < 0 || idx >= (int)_commands.size()) return String();
    return _commands[idx].name;
}

//-------------------------------------------------------------------
// Execution
//-------------------------------------------------------------------

int CommandHandler::execute(const String& input)
{
    _lastError    = CMD_OK;
    _lastErrorMsg = "";

    _parser.parse(input);

    if (_parser.count() == 0) {
        _lastError    = CMD_ERR_EMPTY;
        _lastErrorMsg = "empty input";
        return _lastError;
    }

    String cmdName = _parser[0];
    const CommandEntry* e = findEntry(cmdName);
    if (!e) {
        _lastError    = CMD_ERR_UNKNOWN;
        _lastErrorMsg = "unknown command: " + cmdName;
        return _lastError;
    }

    _lastError = e->handler(_parser, _lastErrorMsg);
    return _lastError;
}

//-------------------------------------------------------------------
// Private helpers
//-------------------------------------------------------------------

const CommandHandler::CommandEntry*
CommandHandler::findEntry(const String& name) const
{
    for (int i = 0; i < (int)_commands.size(); i++) {
        if (_commands[i].name == name) return &_commands[i];
    }
    return nullptr;
}

CommandHandler::CommandEntry*
CommandHandler::findEntry(const String& name)
{
    for (int i = 0; i < (int)_commands.size(); i++) {
        if (_commands[i].name == name) return &_commands[i];
    }
    return nullptr;
}
