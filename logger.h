#pragma once

#include <boost/format.hpp>

#include "types.h"

namespace PoseEstimation
{
    /**
     * @brief Simple logger to send information to stdout/stderr.
     */
    class Logger
    {
    public:
        static void debug(const std::string &text);
        static void debug(const boost::format &fmt);

        static void log(const std::string &text);
        static void log(const boost::format &fmt);

        static void error(const std::string &text);
        static void error(const boost::format &fmt);

        static void warning(const std::string &text);
        static void warning(const boost::format &fmt);

        static void tic(const std::string &title);
        static void toc(const std::string &title);

    private:
        Logger();
        static Logger _instance;

        static inline std::string _timestr();

        static boost::format _log_format;
    };


    /**
     * @brief Assert function that displays a message in case of condition failure.
     */
    #ifdef NDEBUG
    #   define ASSERT(condition, message) \
        do { \
            if (!(condition)) { \
                Logger::error(boost::format("Assertion \"" #condition "\" failed in %s line %d: %s") % __FILE__ % __LINE__ % message); \
                std::exit(EXIT_FAILURE); \
            } \
        } while (false)
    #else
    #   define ASSERT(condition, message) do { } while (false)
    #endif
}
