#pragma once

#include <map>
#include <vector>
#include <ostream>
#include <initializer_list>

#include <boost/variant.hpp>
#include <boost/bimap.hpp>

#include <pcl/console/parse.h>

#include "pipelinemoduletype.hpp"

namespace PoseEstimation
{
    /**
     * @brief Abstracts an element in a set of discrete values.
     */
    class Enum //TODO: Make generic? Now only string values ("names") are supported.
    {
        friend class EnumParameter;
    public:
        int value;
        std::string valueName() const;

        bool get(std::string name, int &id) const;
        bool get(int id, std::string &name) const;

        bool set(const std::string &name);
        bool set(int id);

        size_t size() const;
        std::vector<std::string> names() const;

        static Enum define(std::initializer_list<std::string> _names);

        friend std::ostream& operator<<(std::ostream &os, Enum const &e)
        {
            return os << e.valueName();
        }

    private:
        Enum();
        boost::bimap<int, std::string> _map;
    };

    typedef boost::variant<int, float, bool, std::string, Enum> SupportedValue;

    class ParameterConstraint;
    class ParameterCategory;

    /**
     * @brief Abstracts module parameters. Each argument is grouped under a {@link ParameterCategory}
     * and has a name, value, and an optional description.
     *
     * From the commandline, arguments can be set using --[category]_[name] [value]
     */
    class Parameter
    {
        friend class ParameterCategory;
        friend class EnumParameter;
    public:
        Parameter(const std::string &category,
                  const std::string &name,
                  const SupportedValue &value,
                  const std::string &description = "",
                  std::initializer_list<std::shared_ptr<ParameterConstraint> > constraints = {});

        /**
         * @brief The identifying name of the console argument.
         */        
        std::string& name();

        /**
         * @brief Description of the argument to be displayed in help texts.
         */        
        std::string& description();

        /**
         * @brief The {@link ParameterCategory}.
         */
        ParameterCategory category();

        /**
         * @brief Returns the globally identifying name of the parameter.
         * @return Globally identifying name, i.e. "<category>_<name>".
         */
        std::string parseName() const;

        /**
         * @brief Retrieves value of the console argument.
         * @return The value.
         */
        template <typename T>
        inline T value() const
        {
            return boost::get<T>(_value);
        }

        /**
         * @brief Defines the value of the console argument.
         * @param value The value.
         * @return True, if new value fulfilled all constraints,
         * otherwise the operation is revoked.
         */
        virtual inline bool setValue(const SupportedValue &value)
        {
            const SupportedValue old = _value;
            _value = value;
            if (!isValid())
                _value = old;
        }

        /**
         * @brief Retrieves the unchanged value of type SupportedValue.
         * @return The unchanged value of type SupportedValue.
         */
        inline SupportedValue unconvertedValue() const
        {
            return _value;
        }

        /**
         * @brief Determines whether the parameter is a number.
         * @return True, if the parameter is a number.
         */
        bool isNumber() const;

        /**
         * @brief Computes the numerical representation of the parameter.
         * @return Parameter value as a double number.
         */
        double numericalValue() const;

        /**
         * @brief Tries to assign a numerical value to this parameter.
         * Rounds to integer values if necessary.
         * @param value A numerical value.
         * @return True, if the parameter is a number.
         */
        bool setNumericalValue(double value);

        /**
         * @brief Returns a reference to the constraints imposed on the parameter.
         * @return Reference to the constraints imposed on the parameter.
         */
        std::vector<std::shared_ptr<ParameterConstraint> > &constraints();

        /**
         * @brief Determines wether the parameter satisfies all imposed constraints.
         * @return True, if the parameter satisfies all imposed constraints.
         */
        bool isValid();

        /**
         * @brief Returns the lowest valid value in accordance with the constraints.
         * If no constraints apply towards the lower bound of this numerical parameter,
         * the provided default value is returned.
         * @param defaultValue The default value to be returned if no constraints apply.
         * @return The minimum valid value or the default value.
         */
        double lowerBound(double defaultValue = 1.0) const;

        /**
         * @brief Returns the highest valid value in accordance with the constraints.
         * If no constraints apply towards the upper bound of this numerical parameter,
         * the provided default value is returned.
         * @param defaultValue The default value to be returned if no constraints apply.
         * @return The maximum valid value or the default value.
         */
        double upperBound(double defaultValue = 100.0) const;


        /*********************************************************************************
         *                               Static methods                                  *
         * *******************************************************************************/

        /**
         * @brief Prints all registered arguments with their descriptions and default values
         * to stdout.
         */
        static void displayAll();

        /**
         * @brief Parses all registered arguments from the command-line.
         * @param argc Number of command-line strings.
         * @param argv Array of command-line strings.
         */
        static void parseAll(int argc, char *argv[]);

        /**
         * @brief Retrieves the converted parameter value, or returns default value.
         * @param parseName Globally identifying name, i.e. "<category>_<name>".
         * @param def Default value to be returned if parameter was not found.
         * @return Parameter value, or def if parameter has not been found.
         */
        template <typename T>
        static inline T getOrDefault(const std::string &parseName, const T &def)
        {
            if (_allArgs.find(parseName) == _allArgs.end())
                return def;
            return _allArgs[parseName]->value<T>();
        }

        /**
         * @brief Retrieves parameter based on its name.
         * @param parseName Globally identifying name, i.e. "<category>_<name>".
         * @return Parameter, or NULL if not found.
         */
        static Parameter *get(std::string parseName);

        /**
         * @brief Retrieves all parameters under the given category name.
         * @param category The name of the category.
         * @return Parameters that are grouped under the given category.
         */
        static std::vector<Parameter*> getAll(const std::string &category);

        /**
         * @brief Retrieves all parameters under the given pipeline module type.
         * @param moduleType The pipeline module type.
         * @return Parameters that are grouped under the given pipeline module type.
         */
        static std::vector<Parameter*> getAll(PipelineModuleType::Type moduleType);

        /**
         * @brief Saves all parameters to a JSON file.
         * @param filename The file name of the JSON file.
         * @return Whether the parameters were saved successfully.
         */
        static bool saveAll(const std::string &filename = "configuration.json");

        /**
         * @brief Loads parameters from a JSON file.
         * @param filename The file name of the JSON file.
         * @return Whether the parameters were loaded successfully.
         */
        static bool loadAll(const std::string &filename = "configuration.json");

    private:
        std::string _name;
        std::string _description;
        std::string _category;
        SupportedValue _value;

        std::vector<std::shared_ptr<ParameterConstraint> > _constraints;

        virtual void _display(int indent = 0);
        static std::string _type_name(const SupportedValue &v);

        static std::map<std::string, Parameter*> _allArgs;
        static std::map<std::string, std::vector<Parameter*> > _categorized;
        static std::map<std::string, std::string> _categories;
        static std::map<std::string, std::string> _unparsed;
        static std::map<PipelineModuleType::Type, std::vector<std::string> > _modules;

        template <typename T>
        inline int _parse_helper(int argc, char *argv[])
        {
            T arg;
            int r = pcl::console::parse_argument(
                        argc,
                        argv,
                        ("--"+parseName()).c_str(),
                        arg);
            if (r >= 0)
                _value = arg;
            return r;
        }

        int _parse(int argc, char *argv[]);
        static void _defineCategory(const std::string &name, const std::string &description,
                                    PipelineModuleType::Type moduleType);
    };

    /**
     * @brief Abstract an enumeration parameter that takes a value from a discrete set of names.
     */
    class EnumParameter : public Parameter
    {
    public:
        EnumParameter(const std::string &category,
                      const std::string &name,
                      Enum &value,
                      const std::string &description = "",
                      std::initializer_list<std::shared_ptr<ParameterConstraint> > constraints = {});

        EnumParameter(const std::string &category,
                      const std::string &name,
                      std::initializer_list<std::string> value,
                      const std::string &description = "",
                      std::initializer_list<std::shared_ptr<ParameterConstraint> > constraints = {});

        /**
         * @brief Defines the value of the console argument.
         * @param value The value.
         */
        virtual bool setValue(const Enum &value);

        /**
         * @brief Defines the value of the console argument.
         * @param value The value.
         */
        virtual bool setValue(const std::initializer_list<std::string> &value);

        /**
         * @brief Defines the value of the console argument.
         * @param value The value.
         */
        virtual bool setValue(const std::string &value);

    private:
        virtual void _display(int indent = 0);
    };


    /**
     * @brief Category to group {@link ConsoleArgument} of the same module together and provide
     * a general description of that module in help texts.
     */
    class ParameterCategory
    {
    public:
        ParameterCategory(const std::string &name, const std::string &description = "",
                          PipelineModuleType::Type moduleType = PipelineModuleType::Miscellaneous);

        ParameterCategory(const ParameterCategory &category);

        std::vector<Parameter*> parameters() const;

        std::string name() const;
        std::string description() const;

        static ParameterCategory& EmptyCategory();

    private:
        std::string _name;
    };


    namespace ParameterConstraintType
    {
        enum Type
        {
            GreaterThan,
            LessThan,
            GreaterThanOrEqual,
            LessThanOrEqual,
            Equal,
            NotEqual
        };

        static std::string str(Type t);
    }

    /**
     * @brief Represents a numerical constraint on the parameter.
     */
    class ParameterConstraint
    {
    public:
        ParameterConstraint(ParameterConstraintType::Type t);

        virtual bool isFulfilled(Parameter*) const = 0;
        virtual std::string str() const = 0;
        virtual double resolveNumericalValue() const = 0;

        ParameterConstraintType::Type type() const;

    protected:
        const ParameterConstraintType::Type _type;
        bool _basicFulfillmentTest(double value, Parameter *parameter) const;
    };

    /**
     * @brief Represents a numerical constraint by a constant on the parameter.
     */
    class ConstantConstraint : public ParameterConstraint
    {
    public:
         ConstantConstraint(ParameterConstraintType::Type t, double constant);

         bool isFulfilled(Parameter *parameter) const;
         std::string str() const;
         double resolveNumericalValue() const;

    private:
         const double _constant;
    };

    /**
     * @brief Represents a numerical constraint by another parameter on the parameter.
     */
    class VariableConstraint : public ParameterConstraint
    {
    public:
         VariableConstraint(ParameterConstraintType::Type t, const std::string &parameterName);

         bool isFulfilled(Parameter *parameter) const;
         std::string str() const;
         double resolveNumericalValue() const;

    private:
         const std::string _parameterName;
    };

#define NUMERICAL_PARAMETER_RANGE(min, max) \
    { \
        std::make_shared<ConstantConstraint>(ParameterConstraintType::GreaterThanOrEqual, min), \
        std::make_shared<ConstantConstraint>(ParameterConstraintType::LessThanOrEqual, max) \
    }

}
