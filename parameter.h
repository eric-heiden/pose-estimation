#pragma once

#include <map>
#include <vector>
#include <ostream>
#include <initializer_list>

#include <boost/variant.hpp>
#include <boost/bimap.hpp>

#include <pcl/console/parse.h>

#include "pipelinemodule.hpp"

namespace PoseEstimation
{
    /**
     * @brief Abstracts an element in a set of discrete values.
     */
    class Enum //TODO: Make generic? Now only string values ("names") are supported.
    {
        friend class EnumParameter;
    public:
        Enum();

        int value;
        std::string valueName() const;

        bool get(std::string name, int &id) const;
        bool get(int id, std::string &name) const;

        size_t size() const;
        std::vector<std::string> names() const;

        static Enum define(std::initializer_list<std::string> _names);

        friend std::ostream& operator<<(std::ostream &os, Enum const &e)
        {
            return os << e.valueName();
        }

    private:
        boost::bimap<int, std::string> _map;
    };

    typedef boost::variant<int, float, char, bool, std::string, Enum> SupportedValue;

    /**
     * @brief Abstracts module parameters. Each argument is grouped under a {@see ParameterCategory}
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
                  const std::string &description = "");

        /**
         * @brief The identifying name of the console argument.
         */
        std::string& name();
        /**
         * @brief Description of the argument to be displayed in help texts.
         */
        std::string& description();
        /**
         * @brief The name of the {@see ConsoleArgumentCategory}.
         */
        std::string& category();

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
         */
        virtual inline void setValue(const SupportedValue &value)
        {
            _value = value;
        }

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

        template <typename T>
        static inline T getOrDefault(const std::string &arg_name, const T &def)
        {
            if (_allArgs.find(arg_name) == _allArgs.end())
                return def;
            return _allArgs[arg_name]->value<T>();
        }

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

        std::string _parseName() const;
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
                        _parseName().c_str(),
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
                      const std::string &description = "");

        EnumParameter(const std::string &category,
                      const std::string &name,
                      std::initializer_list<std::string> value,
                      const std::string &description = "");

        /**
         * @brief Defines the value of the console argument.
         * @param value The value.
         */
        virtual inline void setValue(const Enum &value);

        /**
         * @brief Defines the value of the console argument.
         * @param value The value.
         */
        virtual inline void setValue(const std::initializer_list<std::string> &value);

        /**
         * @brief Defines the value of the console argument.
         * @param value The value.
         */
        virtual inline void setValue(const std::string &value);

    private:
        virtual void _display();
    };

    /**
     * @brief Category to group {@see ConsoleArgument} of the same module together and provide
     * a general description of that module in help texts.
     */
    class ParameterCategory
    {
    public:
        ParameterCategory(const std::string &name, const std::string &description,
                          PipelineModuleType::Type moduleType = PipelineModuleType::Miscellaneous);

        /**
         * @brief Helper method to enforce static initialization and prevent the linker from
         * removing parameter category definitions.
         */
        void define();

    private:
        std::string _name;
        std::string _description;
        PipelineModuleType::Type _moduleType;
    };
}
