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
std::map<PipelineModuleType::Type, std::vector<std::string> > Parameter::_modules = std::map<PipelineModuleType::Type, std::vector<std::string> >();

Parameter::Parameter(const std::string &category, const std::string &name,
                     const SupportedValue &value, const std::string &description,
                     std::initializer_list<std::shared_ptr<ParameterConstraint> > constraints)
    : _name(name), _description(description), _category(category), _value(value), _constraints(constraints)
{
    const std::string id = parseName();

    if (!isValid())
    {
        Logger::warning(boost::format("Parameter %1% has been initialized with invalid parameter %2%")
                        % id % _value);
    }

    if (_allArgs.find(id) != _allArgs.end())
    {
        _allArgs[id]->_value = value;
        Logger::debug(boost::format("Value of argument \"%1%\" has been updated to [%2%] %3%.")
                      % id % _type_name(value) % value);
        return;
    }
    else
        _allArgs[id] = this;

    if (_categories.find(category) == _categories.end())
    {
        _categories[category] = "";
        _categorized[category] = std::vector<Parameter*>();
        if (_modules.find(PipelineModuleType::Miscellaneous) == _modules.end())
            _modules[PipelineModuleType::Miscellaneous] = std::vector<std::string>();
        _modules[PipelineModuleType::Miscellaneous].push_back(category);
    }
    _categorized[category].push_back(this);
    //Logger::debug(boost::format("Parameter %s has been initialized.") % id);
}

std::string &Parameter::name()
{
    return _name;
}

std::string &Parameter::description()
{
    return _description;
}

ParameterCategory Parameter::category()
{
    return ParameterCategory(_category);
}

double Parameter::numericalValue() const
{
    if (_value.type() == typeid(int))
        return (double)boost::get<int>(_value);
    if (_value.type() == typeid(float))
        return (double)boost::get<float>(_value);
    if (_value.type() == typeid(bool))
        return (double)(int)boost::get<bool>(_value);
    return std::nan("Type is not numerical");
}

bool Parameter::setNumericalValue(double value)
{
    if (_value.type() == typeid(int))
    {
        _value = (int)round(value);
        return true;
    }
    if (_value.type() == typeid(float))
    {
        _value = (float)value;
        return true;
    }
    if (_value.type() == typeid(bool))
    {
        _value = (bool)(int)round(value);
        return true;
    }
    return false;
}

std::vector<std::shared_ptr<ParameterConstraint> > &Parameter::constraints()
{
    return _constraints;
}

bool Parameter::isValid()
{
    for (auto &&constraint : _constraints)
    {
        if (!constraint->isFulfilled(this))
        {
            Logger::warning(boost::format("Constraint %s %s is not satisfied.")
                          % parseName() % constraint->str());
            return false;
        }
    }
    return true;
}

double Parameter::lowerBound(double defaultValue) const
{
    bool found = false;
    double lb;
    for (auto constraint : _constraints)
    {
        if (constraint->type() == ParameterConstraintType::GreaterThanOrEqual
                || constraint->type() == ParameterConstraintType::GreaterThan)
        {
            if (!found)
            {
                found = true;
                lb = constraint->resolveNumericalValue();
            }
            else
                lb = std::max(lb, constraint->resolveNumericalValue());
        }
    }

    return found ? lb : defaultValue;
}

double Parameter::upperBound(double defaultValue) const
{
    bool found = false;
    double ub;
    for (auto constraint : _constraints)
    {
        if (constraint->type() == ParameterConstraintType::LessThanOrEqual
                || constraint->type() == ParameterConstraintType::LessThan)
        {
            if (!found)
            {
                found = true;
                ub = constraint->resolveNumericalValue();
            }
            else
                ub = std::min(ub, constraint->resolveNumericalValue());
        }
    }

    return found ? ub : defaultValue;
}

std::string Parameter::parseName() const
{
    return (boost::format("%s_%s") % _category % _name).str();
}

bool Parameter::isNumber() const
{
    return _value.type() == typeid(int)
        || _value.type() == typeid(float)
        || _value.type() == typeid(bool);
}

std::string Parameter::_type_name(const PoseEstimation::SupportedValue &v)
{
    if (v.type() == typeid(int))
        return "int";
    if (v.type() == typeid(float))
        return "float";
    if (v.type() == typeid(bool))
        return "bool";
    if (v.type() == typeid(std::string))
        return "string";
    if (v.type() == typeid(Enum))
        return "enum";
    return "unknown";
}

void Parameter::_display(int indent)
{
    while (indent-- > 0)
        std::cout << '\t';

    std::cout << std::left << std::setw(26) << std::setfill(' ') << parseName()
              << std::left << std::setw(58) << std::setfill(' ') << description()
              << " ([" << _type_name(_value) << "] "
              <<  boost::lexical_cast<std::string>(_value) << ")";

    if (!_constraints.empty())
    {
        std::cout << " constraints: ";
        size_t i = 0;
        for (auto &constraint : _constraints)
        {
            std::cout << "(" << constraint->str() << ")";
            if (++i < _constraints.size())
                std::cout << ", ";
        }
    }

    std::cout << std::endl;
}

/**
 * @brief Prints all registered arguments with their descriptions and default values
 * to stdout.
 */
void Parameter::displayAll()
{
    for (auto &&mit : _modules)
    {
        std::string moduleName = PipelineModuleType::str(mit.first);
        std::cout << moduleName << std::endl;

        for (std::string &category : mit.second)
        {
            std::cout << '\t' << category;
            if (!_categories[category].empty())
                std::cout << " (" << _categories[category] << ")";
            std::cout << std::endl;
            for (Parameter *arg : _categorized[category])
            {
                arg->_display(2);
            }
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
    for (auto it = _allArgs.begin(); it != _allArgs.end(); ++it)
    {
        it->second->_parse(argc, argv);
    }
}

Parameter *Parameter::get(std::string parseName)
{
    if (_allArgs.find(parseName) == _allArgs.end())
        return NULL;
    return _allArgs[parseName];
}

std::vector<Parameter *> Parameter::getAll(const std::string &category)
{
    if (_categorized.find(category) == _categorized.end())
    {
        Logger::warning(boost::format("Category \"%s\" was not found.") % category);
        return std::vector<Parameter*>();
    }
    return _categorized[category];
}

std::vector<Parameter *> Parameter::getAll(PipelineModuleType::Type moduleType)
{
    std::vector<Parameter*> r;
    if (_modules.find(moduleType) == _modules.end())
    {
        Logger::warning(boost::format("No parameters for module type %1% were found.") % moduleType);
        return r;
    }

    for (std::string &category : _modules[moduleType])
    {
        std::vector<Parameter*> parameters = getAll(category);
        r.insert(r.end(), parameters.begin(), parameters.end());
    }

    return r;
}


void _set_json_arg_value(Json &jarg, SupportedValue &value)
{
    if (value.type() == typeid(int))
        jarg["value"] = boost::get<int>(value);
    else if (value.type() == typeid(float))
        jarg["value"] = boost::get<float>(value);
    else if (value.type() == typeid(std::string))
        jarg["value"] = boost::get<std::string>(value);
    else if (value.type() == typeid(bool))
        jarg["value"] = boost::get<bool>(value);
    else if (value.type() == typeid(Enum))
        jarg["value"] = boost::get<Enum>(value).valueName();
}

bool Parameter::saveAll(const std::string &filename)
{
    Json j;
    j["configuration"] = std::vector<Json>();
    for (auto &&mit : _modules)
    {
        std::string moduleName = PipelineModuleType::str(mit.first);
        Json mj;
        mj["module"] = moduleName;
        mj["categories"] = std::vector<Json>();
        for (std::string &category : mit.second)
        {
            // create category
            Json cj;
            cj["description"] = _categories[category];
            cj["parameters"] = std::vector<Json>();
            cj["name"] = category;
            for (Parameter *arg : _categorized[category])
            {
                Json aj;
                aj["name"] = arg->name();
                _set_json_arg_value(aj, arg->_value);
                std::string desc = arg->description();
                if (!arg->constraints().empty())
                {
                    desc += ", constraints: ";
                    size_t i = 0;
                    for (auto &constraint : arg->constraints())
                    {
                        desc += "(" + constraint->str() + ")";
                        if (++i < arg->constraints().size())
                            desc += ", ";
                    }
                }
                if (arg->unconvertedValue().type() == typeid(Enum))
                {
                    desc += ", possible values: " + boost::algorithm::join(arg->value<Enum>().names(), "|");
                }
                aj["description"] = desc;
                cj["parameters"].push_back(aj);
            }
            mj["categories"].push_back(cj);
        }
        j["configuration"].push_back(mj);
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
    std::ifstream fin(filename);
    if (!fin)
    {
        Logger::error(boost::format("Could not load parameters from \"%s\".") % filename);
        return false;
    }

    Json j(fin);
    if (j.empty())
    {
        Logger::warning(boost::format("Configuration file \"%s\" is empty.") % filename);
        return true;
    }

    for (auto &module : j["configuration"])
    {
        Logger::debug(boost::format("Found module %s.") % module["module"]);
        auto mtype = PipelineModuleType::parse(module["module"].get<std::string>());
        for (auto &category : module["categories"])
        {
            // create category / module if it doesn't exist
            _defineCategory(category["name"].get<std::string>(), category["description"].get<std::string>(), mtype);

            for (auto &parameter : category["parameters"])
            {
                // handle Enum parameter
                std::string id = (boost::format("%s_%s") % category["name"].get<std::string>() % parameter["name"].get<std::string>()).str();

                Parameter *p = Parameter::get(id);
                if (!p)
                {
                    Logger::debug("not found");
                    continue;
                }

                if (p->_value.type() == typeid(Enum))
                    p->value<Enum>().set(parameter["value"].get<std::string>());
                else if (p->_value.type() == typeid(int))
                    p->_value = (int)parameter["value"];
                else if (p->_value.type() == typeid(float))
                    p->_value = (float)parameter["value"];
                else if (p->_value.type() == typeid(std::string))
                    p->_value = parameter["value"].get<std::string>();
                else if (p->_value.type() == typeid(bool))
                    p->_value = (bool)parameter["value"];
            }
        }
    }

    Logger::log(boost::format("Configuration has been read successfully from \"%s\".") % filename);
}

int Parameter::_parse(int argc, char *argv[])
{
    //TODO parse enum parameters from CLI
    if (_value.type() == typeid(int))
        return _parse_helper<int>(argc, argv);
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

void Parameter::_defineCategory(const std::string &name, const std::string &description,
                                PipelineModuleType::Type moduleType)
{
    Logger::debug(boost::format("Defining parameter category %s...") % name);
    if (_categories.find(name) == _categories.end() || _categories[name].empty())
        _categories[name] = description;
    if (_modules.find(moduleType) == _modules.end())
        _modules[moduleType] = std::vector<std::string>();

    if (_categorized.find(name) == _categorized.end())
        _categorized[name] = std::vector<Parameter*>();

    if (_modules.find(PipelineModuleType::Miscellaneous) != _modules.end()
            && std::find(_modules[PipelineModuleType::Miscellaneous].begin(),
                         _modules[PipelineModuleType::Miscellaneous].end(),
                         name) != _modules[PipelineModuleType::Miscellaneous].end())
    {
        // category already exists, reassign it to the correct module type
        std::vector<std::string> &cats = _modules[PipelineModuleType::Miscellaneous];
        auto it = std::find(cats.begin(), cats.end(), name);
        if (it != cats.end())
            cats.erase(it);
    }
    else if (std::find(_modules[moduleType].begin(),
                       _modules[moduleType].end(),
                       name) == _modules[moduleType].end())
    {
        _modules[moduleType].push_back(name);
        Logger::debug(boost::format("Category %s has been defined.") % name);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ParameterCategory::ParameterCategory(const std::string &name, const std::string &description,
                                     PipelineModuleType::Type moduleType)
    : _name(name)
{
    if (name != "empty")
        Parameter::_defineCategory(name, description, moduleType);
}

ParameterCategory::ParameterCategory(const ParameterCategory &category)
    : _name(category._name)
{
}

std::vector<Parameter*> ParameterCategory::parameters() const
{
    return Parameter::getAll(_name);
}

std::string ParameterCategory::name() const
{
    return _name;
}

std::string ParameterCategory::description() const
{
    return Parameter::_categories[_name];
}

ParameterCategory *_emptyCategory = nullptr;
ParameterCategory& ParameterCategory::EmptyCategory()
{
    if (!_emptyCategory)
        _emptyCategory = new ParameterCategory("empty", "");
    return *_emptyCategory;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

EnumParameter::EnumParameter(const std::string &category, const std::string &name,
                             std::initializer_list<std::string> value, const std::string &description,
                             std::initializer_list<std::shared_ptr<ParameterConstraint> > constraints)
    : Parameter(category, name, Enum::define(value), description, constraints)
{
}

EnumParameter::EnumParameter(const std::string &category, const std::string &name,
                             Enum &value, const std::string &description,
                             std::initializer_list<std::shared_ptr<ParameterConstraint> > constraints)
    : Parameter(category, name, value, description, constraints)
{
}

bool EnumParameter::setValue(const std::string &value)
{
    int id;
    if ((boost::get<Enum>(_value)).get(value, id))
    {
        (boost::get<Enum>(_value)).value = id;
        return true;
    }
    return false;
}

bool EnumParameter::setValue(const std::initializer_list<std::string> &value)
{
    _value = Enum::define(value);
    return true;
}

bool EnumParameter::setValue(const Enum &value)
{
    _value = value;
    return true;
}

void EnumParameter::_display(int indent)
{
    while (indent-- > 0)
        std::cout << '\t';

    Enum val = boost::get<Enum>(_value);
    std::cout << std::left << std::setw(26) << std::setfill(' ') << parseName()
              << std::left << std::setw(58) << std::setfill(' ') << description()
              << " ([" << boost::algorithm::join(val.names(), "|") << "] "
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

bool Enum::set(const std::string &name)
{
    int v;
    if (get(name, v))
        value = v;
}

bool Enum::set(int id)
{
    value = id;
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
        e._map.insert(boost::bimap<int, std::string>::value_type(i++, name));
    }

    return e;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ParameterConstraint::ParameterConstraint(ParameterConstraintType::Type t)
    : _type(t)
{

}

ParameterConstraintType::Type ParameterConstraint::type() const
{
    return _type;
}

bool ParameterConstraint::_basicFulfillmentTest(double value, Parameter *parameter) const
{
    switch (_type)
    {
        case ParameterConstraintType::GreaterThan:
            return parameter->numericalValue() > value;
        case ParameterConstraintType::GreaterThanOrEqual:
            return parameter->numericalValue() >= value;
        case ParameterConstraintType::LessThan:
            return parameter->numericalValue() < value;
        case ParameterConstraintType::LessThanOrEqual:
            return parameter->numericalValue() <= value;
        case ParameterConstraintType::Equal:
            return std::abs(parameter->numericalValue() - value) <= std::numeric_limits<float>::epsilon();
        case ParameterConstraintType::NotEqual:
            return std::abs(parameter->numericalValue() - value) > std::numeric_limits<float>::epsilon();
    }
    return false;
}


ConstantConstraint::ConstantConstraint(ParameterConstraintType::Type t, double constant)
    : ParameterConstraint(t), _constant(constant)
{

}

bool ConstantConstraint::isFulfilled(Parameter *parameter) const
{
    return _basicFulfillmentTest(_constant, parameter);
}

std::string ConstantConstraint::str() const
{
    std::string _str = (boost::format("%s %d") % ParameterConstraintType::str(_type) % _constant).str();
    return _str;
}

double ConstantConstraint::resolveNumericalValue() const
{
    return _constant;
}


VariableConstraint::VariableConstraint(ParameterConstraintType::Type t, const std::string &parameterName)
    : ParameterConstraint(t), _parameterName(parameterName)
{

}

bool VariableConstraint::isFulfilled(Parameter *parameter) const
{
    Parameter *p = Parameter::get(_parameterName);
    if (!p)
    {
        Logger::warning(boost::format("Parameter \"%s\" could not be found for constraint satisfaction test.")
                        % _parameterName);
        return false;
    }

    return _basicFulfillmentTest(p->numericalValue(), parameter);
}

std::string VariableConstraint::str() const
{
    std::string _str = (boost::format("%s %s") % ParameterConstraintType::str(_type) % _parameterName).str();
    return _str;
}

double VariableConstraint::resolveNumericalValue() const
{
    return Parameter::get(_parameterName)->numericalValue();
}


std::string ParameterConstraintType::str(ParameterConstraintType::Type t)
{
    static std::string names[] = { ">", "<", ">=", "<=", "=", "!=" };
    return names[(size_t)t];
}
