#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include "consoleargument.h"
#include "logger.h"

using namespace PoseEstimation;

std::map<std::string, ConsoleArgument*> ConsoleArgument::_allArgs = std::map<std::string, ConsoleArgument*>();
std::map<std::string, std::vector<ConsoleArgument*> > ConsoleArgument::_categorized = std::map<std::string, std::vector<ConsoleArgument*> >();
std::map<std::string, std::string> ConsoleArgument::_categories = std::map<std::string, std::string>();

ConsoleArgument::ConsoleArgument(const std::string &category, const std::string &name, const SupportedValue &value, const std::string &description)
{
    _name = name;
    _description = description;
    _category = category;
    _value = value;
    std::string id = _parseName();
    if (_allArgs.find(id) != _allArgs.end())
    {
        _allArgs[id]->_value = value;
        Logger::debug(boost::format("Value of argument \"%s\" has been updated.") % name);
    }
    else
        _allArgs[id] = this;
    if (_categories.find(category) == _categories.end())
    {
        _categories[category] = "";
        _categorized[category] = std::vector<ConsoleArgument*>();
    }
    _categorized[category].push_back(this);
}

std::string &ConsoleArgument::name()
{
    return _name;
}

std::string &ConsoleArgument::description()
{
    return _description;
}

std::string &ConsoleArgument::category()
{
    return _category;
}

std::string ConsoleArgument::_parseName() const
{
    return (boost::format("--%s_%s") % _category % _name).str();
}

void ConsoleArgument::setValue(const SupportedValue &value)
{
    _value = value;
}

std::string type_name(PoseEstimation::SupportedValue &v)
{
    if (v.type() == typeid(int))
        return "int";
    if (v.type() == typeid(char))
        return "char";
    if (v.type() == typeid(float))
        return "float";
    if (v.type() == typeid(bool))
        return "bool";
    return "unknown";
}

/**
 * @brief Prints all registered arguments with their descriptions and default values
 * to stdout.
 */
void ConsoleArgument::displayAll()
{
    for (std::map<std::string, std::vector<ConsoleArgument*> >::iterator vit = _categorized.begin(); vit != _categorized.end(); ++vit)
    {
        std::cout << vit->first;
        if (!_categories[vit->first].empty())
            std::cout << " (" << _categories[vit->first] << ")";
        std::cout << std::endl;
        for (std::vector<ConsoleArgument*>::iterator it = vit->second.begin(); it != vit->second.end(); ++it)
        {
            ConsoleArgument *arg = *it;
            std::cout << std::left << std::setw(26) << std::setfill(' ') << arg->_parseName()
                      << std::left << std::setw(58) << std::setfill(' ') << arg->description()
                      << " ([" << type_name(arg->_value) << "] "
                      <<  boost::lexical_cast<std::string>(arg->_value) << ")" << std::endl;
        }
    }
}

/**
 * @brief Parses all registered arguments from the command-line.
 * @param argc Number of command-line strings.
 * @param argv Array of command-line strings.
 */
void ConsoleArgument::parseAll(int argc, char *argv[])
{
    for (std::map<std::string, ConsoleArgument*>::iterator it = _allArgs.begin(); it != _allArgs.end(); ++it)
    {
        it->second->_parse(argc, argv);
    }
}

int ConsoleArgument::_parse(int argc, char *argv[])
{
    if (_value.type() == typeid(int))
        return _parse_helper<int>(argc, argv);
    if (_value.type() == typeid(char))
        return _parse_helper<char>(argc, argv);
    if (_value.type() == typeid(float))
        return _parse_helper<float>(argc, argv);
    if (_value.type() == typeid(bool))
    {
        if (pcl::console::find_switch(argc, argv, ("--" + _name).c_str()))
            _value = true;
        else
            _value = false;
        return 1;
    }
}

void ConsoleArgument::_defineCategory(const std::string &name, const std::string &description)
{
    _categories[name] = description;
    if (_categorized.find(name) == _categorized.end())
        _categorized[name] = std::vector<ConsoleArgument*>();
}

ConsoleArgumentCategory::ConsoleArgumentCategory(const std::string &name, const std::string &description)
{
    ConsoleArgument::_defineCategory(name, description);
}
