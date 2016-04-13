#include <ctime>
#include <map>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "logger.h"


//TODO remove this
#define NDEBUG


using namespace PoseEstimation;

std::map<std::string, boost::posix_time::ptime> _tics;

Logger Logger::_instance = Logger();
boost::format Logger::_log_format = boost::format("[%s %s] %s");

void Logger::debug(const std::string &text)
{
#ifdef NDEBUG
    std::cout << (_log_format % "DEBUG" % _timestr() % text) << std::endl;
#endif
}

void Logger::debug(const boost::format &fmt)
{
#ifdef NDEBUG
    std::cout << (_log_format % "DEBUG" % _timestr() % fmt.str()) << std::endl;
#endif
}

void Logger::log(const std::string &text)
{
    std::cout << (_log_format % "LOG" % _timestr() % text) << std::endl;
}

void Logger::log(const boost::format &fmt)
{
    std::cout << (_log_format % "LOG" % _timestr() % fmt.str()) << std::endl;
}

void Logger::error(const std::string &text)
{
    std::cerr << (_log_format % "ERROR" % _timestr() % text) << std::endl;
}

void Logger::error(const boost::format &fmt)
{
    std::cerr << (_log_format % "ERROR" % _timestr() % fmt.str()) << std::endl;
}

void Logger::tic(const std::string &title)
{
    _tics[title] = boost::posix_time::microsec_clock::local_time();
}

void Logger::toc(const std::string &title)
{
    if (_tics.find(title) != _tics.end())
    {
        boost::posix_time::ptime toc = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration diff = toc - _tics[title];
        log(boost::format("%1% finished in %2%") % title % diff);
    }
}

std::string Logger::_timestr()
{
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
    return std::string(buffer);
}

Logger::Logger() {}
