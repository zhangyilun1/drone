#ifndef __MD5_H__
#define __MD5_H__

#include <string>

std::string calculateMD5(const std::string &filename);
std::string calculateMD5(char * buf, int len);

#endif
