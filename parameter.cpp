#include <fstream>

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

void Parameter::setValue(const SupportedValue &value)
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
    if (v.type() == typeid(std::string))
        return "string";
    return "unknown";
}

/**
 * @brief Prints all registered arguments with their descriptions and default values
 * to stdout.
 */
void Parameter::displayAll()
{
    for (std::map<std::string, std::vector<Parameter*> >::iterator vit = _categorized.begin(); vit != _categorized.end(); ++vit)
    {
        std::cout << vit->first;
        if (!_categories[vit->first].empty())
            std::cout << " (" << _categories[vit->first] << ")";
        std::cout << std::endl;
        for (std::vector<Parameter*>::iterator it = vit->second.begin(); it != vit->second.end(); ++it)
        {
            Parameter *arg = *it;
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
    for (std::map<std::string, std::vector<Parameter*> >::iterator vit = _categorized.begin(); vit != _categorized.end(); ++vit)
    {
        // create category
        std::string category = vit->first;
        j[category]["description"] = _categories[category];
        j[category]["parameters"] = Json::array;
        for (std::vector<Parameter*>::iterator it = vit->second.begin(); it != vit->second.end(); ++it)
        {
            Parameter *arg = *it;
            Json jarg;
            jarg["name"] = arg->name();
            _set_json_arg_value(jarg, arg->_value);
            jarg["description"] = arg->description();
            j[category]["parameters"].push_back(jarg);
        }
    }
    std::string r = j.dump(4);
    std::cout << r << std::endl;

    std::ofstream fout(filename);
    if (!fout)
    {
        Logger::error(boost::format("Could not save parameters to \"%s\".") % filename);
        return false;
    }

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

ParameterCategory::ParameterCategory(const std::string &name, const std::string &description)
{
    Parameter::_defineCategory(name, description);
}
