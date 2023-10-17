#include "log.h"

#define LOG4CPLUS_CONF_FILE "./log4cplus.properties"

Log& log1 = Log::getInstance();

using namespace std;

Log::Log()
{
    std::cout<<"constructor called!"<<std::endl;
    log4cplus::PropertyConfigurator::doConfigure(LOG4CPLUS_TEXT(LOG4CPLUS_CONF_FILE));
}
 
Log::~Log()
{
    log4cplus::Logger::shutdown();
    std::cout<<"destructor called!"<<std::endl;
}

Log& Log::getInstance()
{
    static Log instance;
    return instance;
}
