#ifndef __MD5_H__
#define __MD5_H__

#include <openssl/md5.h>
#include <string>
#include "log.h"
std::string calculateMD5(const std::string &filename);
std::string calculateMD5(char * buf, int len);
std::string calculateMD5byOpenSSL(const std::string &filename);
#endif
