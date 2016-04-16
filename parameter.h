#ifndef CONSOLEARGUMENT_H
#define CONSOLEARGUMENT_H

#include <map>
#include <vector>
#include <boost/variant.hpp>
#include <pcl/console/parse.h>

namespace PoseEstimation
{
    typedef boost::variant<int, float, char, bool, std::string> SupportedValue;

    class ParameterCategory;

    /**
     * @brief Abstracts command-line interface arguments. Each argument is grouped under a {@see ConsoleArgumentCategory}
     * and has a name, value, and an optional description.
     *
     * From the commandline, arguments can be set using --[category]_[name] [value]
     */
    class Parameter
    {
        friend class ParameterCategory;
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
        void setValue(const SupportedValue &value);

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
        std::string _parseName() const;
        SupportedValue _value;
        static std::map<std::string, Parameter*> _allArgs;
        static std::map<std::string, std::vector<Parameter*> > _categorized;
        static std::map<std::string, std::string> _categories;
        static std::map<std::string, std::string> _unparsed;

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
        static void _defineCategory(const std::string &name, const std::string &description = "");
    };

    /**
     * @brief Category to group {@see ConsoleArgument} of the same module together and provide
     * a general description of that module in help texts.
     */
    class ParameterCategory
    {
    public:
        ParameterCategory(const std::string &name, const std::string &description = "");
    };
}

#endif // CONSOLEARGUMENT_H
