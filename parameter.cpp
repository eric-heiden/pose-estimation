#include <fstream>

#include <boost/algorithm/string/join.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include "json.hpp"

#include "parameter.h"
#include "logger.h"

using namespace PoseEstimation;
using Json = nlohmann::json;

std::map<std::string, Parameter*> Parameter::_allArgs = std::map<std::string, Parameter*>();
std::map<std::string, std::vector<Parameter*> > Parameter::_categorized = std::map<std::string, std::vector<Parameter*> >();
std::map<std::string, std::string> Parameter::_categories = std::map<std::string, std::string>();

Parameter::Parameter(const std::string &category, const std::string &name, const SupportedValue &value, const std::string &description)
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
        _categorized[category] = std::vector<Parameter*>();
    }
    _categorized[category].push_back(this);
}

std::string &Parameter::name()
{
    return _name;
}

std::string &Parameter::description()
{
    return _description;
}

std::string &Parameter::category()
{
    return _category;
}

std::string Parameter::_parseName() const
{
    return (boost::format("--%s_%s") % _category % _name).str();
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
    if (v.type() == typeid(std::string))
        return "string";
    return "unknown";
}

void Parameter::_display()
{
    std::cout << std::left << std::setw(26) << std::setfill(' ') << _parseName()
              << std::left << std::setw(58) << std::setfill(' ') << description()
              << " ([" << type_name(_value) << "] "
              <<  boost::lexical_cast<std::string>(_value) << ")" << std::endl;
}

/**
 * @brief Prints all registered arguments with their descriptions and default values
 * to stdout.
 */
void Parameter::displayAll()
{
    for (auto &&vit : _categorized)
    {
        std::cout << vit.first;
        if (!_categories[vit.first].empty())
            std::cout << " (" << _categories[vit.first] << ")";
        std::cout << std::endl;
        for (Parameter *arg : vit.second)
        {
            arg->_display();
        }
    }
}

/**
 * @brief Parses all registered arguments from the command-line.
 * @param argc Number of command-line strings.
 * @param argv Array of command-line strings.
 */
void Parameter::parseAll(int argc, char *argv[])
{
    for (std::map<std::string, Parameter*>::iterator it = _allArgs.begin(); it != _allArgs.end(); ++it)
    {
        it->second->_parse(argc, argv);
    }
}

void _set_json_arg_value(Json &jarg, SupportedValue &value)
{
    if (value.type() == typeid(int))
        jarg["value"] = boost::get<int>(value);
    else if (value.type() == typeid(char))
        jarg["value"] = boost::get<char>(value);
    else if (value.type() == typeid(float))
        jarg["value"] = boost::get<float>(value);
    else if (value.type() == typeid(std::string))
        jarg["value"] = boost::get<std::string>(value);
    else if (value.type() == typeid(bool))
        jarg["value"] = boost::get<bool>(value);
}

bool Parameter::saveAll(const std::string &filename)
{
    Json j;
    for (auto &&vit : _categorized)
    {
        // create category
        std::string category = vit.first;
        j[category]["description"] = _categories[category];
        j[category]["parameters"] = std::vector<Json>();
        for (Parameter *arg : vit.second)
        {
            Json jarg;
            jarg["name"] = arg->name();
            _set_json_arg_value(jarg, arg->_value);
            jarg["description"] = arg->description();
            j[category]["parameters"].push_back(jarg);
        }
    }

    std::ofstream fout(filename);
    if (!fout)
    {
        Logger::error(boost::format("Could not save parameters to \"%s\".") % filename);
        return false;
    }

    std::string r = j.dump(4);
    fout << r;

    return true;
}

bool Parameter::loadAll(const std::string &filename)
{
    return false;
}

int Parameter::_parse(int argc, char *argv[])
{
    if (_value.type() == typeid(int))
        return _parse_helper<int>(argc, argv);
    if (_value.type() == typeid(char))
        return _parse_helper<char>(argc, argv);
    if (_value.type() == typeid(float))
        return _parse_helper<float>(argc, argv);
    if (_value.type() == typeid(std::string))
        return _parse_helper<std::string>(argc, argv);
    if (_value.type() == typeid(bool))
    {
        if (pcl::console::find_switch(argc, argv, ("--" + _name).c_str()))
            _value = true;
        else
            _value = false;
        return 1;
    }
}

void Parameter::_defineCategory(const std::string &name, const std::string &description)
{
    _categories[name] = description;
    if (_categorized.find(name) == _categorized.end())
        _categorized[name] = std::vector<Parameter*>();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ParameterCategory::ParameterCategory(const std::string &name, const std::string &description)
{
    Parameter::_defineCategory(name, description);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

EnumParameter::EnumParameter(const std::string &category, const std::string &name, std::initializer_list<std::string> value, const std::string &description)
    : Parameter(category, name, Enum::define(value), description)
{

}

EnumParameter::EnumParameter(const std::string &category, const std::string &name, Enum &value, const std::string &description)
    : Parameter(category, name, value, description)
{

}

void EnumParameter::_display()
{
    Enum val = boost::get<Enum>(_value);
    std::cout << std::left << std::setw(26) << std::setfill(' ') << _parseName()
              << std::left << std::setw(58) << std::setfill(' ') << description()
              << " ([Enum of " << boost::algorithm::join(val.names(), "|") << "] "
              << val.valueName() << ")" << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Enum::Enum()
{
    value = 0;
}

std::string Enum::valueName() const
{
    if (value >= _map.size())
    {
        throw std::exception();
    }

    return _map.left.at(value);
}

bool Enum::get(std::string name, int &id) const
{
    if (_map.right.find(name) == _map.right.end())
        return false;

    id = _map.right.at(name);
    return true;
}

bool Enum::get(int id, std::string &name) const
{
    if (_map.left.find(id) == _map.left.end())
        return false;

    name = _map.left.at(id);
    return true;
}

size_t Enum::size() const
{
    return _map.size();
}

std::vector<std::string> Enum::names() const
{
    std::vector<std::string> ns;
    for (auto name: _map.right)
        ns.push_back(name.first);

    return ns;
}

Enum Enum::define(std::initializer_list<std::string> _names)
{
    Enum e;
    int i = 0;
    for (auto name : _names)
    {
        e._map.insert(boost::bimap<int, std::string>::value_type(i, name));
    }

    return e;
}
