#include <iostream>
#include <sstream>
#include <openssl/md5.h>
#include <iomanip>


std::string calculateMD5byOpenSSL(const std::string &filename) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return "";
    }

    MD5_CTX md5Context;
    MD5_Init(&md5Context);

    unsigned char buffer[1024];
    size_t bytesRead;
    std::cout << sizeof(buffer) << std::endl;
    int i = 1;
    while ((bytesRead = fread(buffer, 1, sizeof(buffer), file)) != 0) {
        
        std::cout << i << "  bytesRead  : " << bytesRead << std::endl;
        MD5_Update(&md5Context, buffer, bytesRead);
        i++;
    }

    fclose(file);

    unsigned char md5Digest[MD5_DIGEST_LENGTH];
    MD5_Final(md5Digest, &md5Context);
    
    std::stringstream md5Stream;
    for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
        md5Stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(md5Digest[i]);
    }

    return md5Stream.str();
}

int main() {
    std::string filename = "/Users/zhangyilun/Desktop/socket_connect/client/clinet.cpp";
    for(int i = 0; i < 10; i++){
        std::string md5Value = calculateMD5(filename);
        std::cout << "MD5 value for " << filename << ": " << md5Value << std::endl;
        std::cout << md5Value.size() << std::endl;
    }
   
    return 0;
}
