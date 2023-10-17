#include <sys/types.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <fcntl.h>

#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT     32321
#define SEND_DATA       "0x00, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x44, 0x30, 0x30, 0x46"

using namespace std;

int main(int argc, char* argv[])
{
    unsigned char data[] = {0x0a, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x44, 0x30, 0x30, 0x46};
    unsigned char data2[] = {0x06, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x44, 0x30, 0x30, 0x46};
    unsigned char data3[] = {0x0a, 0x00, 0x39, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x41, 0x44, 0x30, 0x30, 0x46, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x4d, 0x33, 0x30, 0x54, 0x07, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55,
    0x4e, 0x4b, 0x4e, 0x4f, 0x57, 0x4e, 0x00, 0x00, 0x20, 0x41, 0x05, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x31, 0x2e, 0x30, 0x2e, 0x30};
    unsigned char data4[] = {0x08, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x62, 0xec, 0x64, 0x46, 0x11, 0x11, 0xff};
    unsigned char data5[] = {0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x40,0x64,0x40,0xdd,0xda,0x78,0x66,0x38,0x88,0x36,0x40,0x48,0x83
,0x80,0x21,0x50,0x7c,0x5c,0x40,0x08,0x05,0xd2,0x4e,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x5d,0x3e,0xf5
,0x64,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xe4,0x6d,0x11,0xa6,0x3c,0x79,0x5a
,0x40,0xd3,0xe1,0x2b,0x02,0xad,0x43,0x3d,0x40,0x9e,0xef,0xa9,0x43,0x01,0x00,0x00
,0x00,0xee,0xc2,0x00,0x00,0xb4,0xc2,0x00,0x00,0x30,0x2a,0x3a,0x00,0x00,0x00,0x00
,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x15,0xff,0xff,0xff,0xff,0x01,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x30,0x4e,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x73,0x68,0x91,0xed,0x7c,0x6f,0x71,0x40,0x1f,0x9e
,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc,0xa2,0xa5,0x3c,0x79,0x5a,0x40,0x73,0x68
,0x91,0xed,0x7c,0x6f,0x71,0x40,0x1f,0x9e,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc
,0xa2,0xa5,0x3c,0x79,0x5a,0x40,0x73,0x68,0x91,0xed,0x7c,0x6f,0x71,0x40,0x1f,0x9e
,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc,0xa2,0xa5,0x3c,0x79,0x5a,0x40,0x00,0x00
,0x40,0x40,0x00,0x8c,0x2d,0x3a,0x81,0xff,0xff,0x00,0x01,0x02,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0xad,0xa3,0x6d,0xa4
,0x39,0xba,0xab,0x30,0x6f,0x60,0x32,0x42,0x6f,0x60,0x32,0x01,0x80,0x2d,0xbd,0x01
,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xa0,0x40,0x90,0x29,0x3a,0x81,0x00
,0x00,0x00,0x00,0x6c,0xce,0xef,0xf7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x20,0xc1,0x1e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0xe4
,0x6d,0x11,0xa6,0x3c,0x79,0x5a,0x40,0xd3,0xe1,0x2b,0x02,0xad,0x43,0x3d,0x40,0x2f
,0x7d,0x9a,0x43,0x01,0x00,0x00,0x00,0xee,0xc2,0x00,0x00,0xb4,0xc2,0x01,0x00,0x30
,0x2a,0x3a,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x15,0xff
,0xff,0xff,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x4e,0x00,0x00,0x00
,0x8b,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x73,0x68,0x91,0xed
,0x7c,0x6f,0x71,0x40,0x1f,0x9e,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc,0xa2,0xa5
,0x3c,0x79,0x5a,0x40,0x73,0x68,0x91,0xed,0x7c,0x6f,0x71,0x40,0x1f,0x9e,0xa9,0x00
,0xad,0x43,0x3d,0x40,0x58,0xbc,0xa2,0xa5,0x3c,0x79,0x5a,0x40,0x73,0x68,0x91,0xed
,0x7c,0x6f,0x71,0x40,0x1f,0x9e,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc,0xa2,0xa5
,0x3c,0x79,0x5a,0x40,0x00,0x00,0x40,0x40,0x00,0x8c,0x2d,0x3a,0x81,0xff,0xff,0x00
,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x60,0xad,0xa3,0x6d,0xa4,0x39,0xba,0xab,0x30,0x6f,0x60,0x32,0x42,0x6f,0x60
,0x32,0x01,0x80,0x2d,0xbd,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xa0
,0x40,0x90,0x29,0x3a,0x81,0x00,0x00,0x00,0x00,0x6c,0xce,0xef,0xf7,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0xc1,0x1e,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x01,0x00,0x00,0x00,0x7c,0x4c,0x0a,0x26,0x37,0x79,0x5a,0x40,0x02,0xdb,0xaa
,0xf0,0xa1,0x43,0x3d,0x40,0xf2,0x72,0x9f,0x43,0x01,0x00,0x00,0x00,0xea,0xc2,0x00
,0x00,0xb4,0xc2,0x01,0x00,0x30,0x2a,0x3a,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x15,0xff,0xff,0xff,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x30,0x4e,0x00,0x00,0x00,0x8c,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x91,0xed,0x7c,0x3f,0x35,0x0e,0x72,0x40,0x4e,0x97,0x28,0xef,0xa1,0x43
,0x3d,0x40,0xfc,0x9a,0x9b,0x25,0x37,0x79,0x5a,0x40,0x91,0xed,0x7c,0x3f,0x35,0x0e
,0x72,0x40,0x4e,0x97,0x28,0xef,0xa1,0x43,0x3d,0x40,0xfc,0x9a,0x9b,0x25,0x37,0x79
,0x5a,0x40,0x91,0xed,0x7c,0x3f,0x35,0x0e,0x72,0x40,0x4e,0x97,0x28,0xef,0xa1,0x43
,0x3d,0x40,0xfc,0x9a,0x9b,0x25,0x37,0x79,0x5a,0x40,0x00,0x00,0x40,0x40,0x00,0x8c
,0x2d,0x3a,0x81,0xff,0xff,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0xad,0xa3,0x6d,0xa4,0x39,0xba,0xab,0x30
,0x6f,0x60,0x32,0x42,0x6f,0x60,0x32,0x01,0x80,0x2d,0xbd,0x01,0x00,0x00,0x00,0x01
,0x00,0x00,0x00,0x00,0x00,0xa0,0x40,0x90,0x29,0x3a,0x81,0x00,0x00,0x00,0x00,0x6c
,0xce,0xef,0xf7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0xc1,0x1e
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x82,0x09,0xd0,0xe9,0x32
,0x79,0x5a,0x40,0xe2,0x2a,0x72,0x33,0x9a,0x43,0x3d,0x40,0x5c,0x3f,0x9a,0x43,0x01
,0x00,0x00,0x00,0xea,0xc2,0x00,0x00,0xb4,0xc2,0x01,0x00,0x30,0x2a,0x3a,0x00,0x00
,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x15,0xff,0xff,0xff,0xff,0x01
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x4e,0x00,0x00,0x00,0x8d,0x00,0x00,0x00
,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf6,0x28,0x5c,0x8f,0xc2,0x67,0x71,0x40
,0x2e,0xe7,0xef,0x31,0x9a,0x43,0x3d,0x40,0x0a,0x58,0x61,0xe9,0x32,0x79,0x5a,0x40
,0xf6,0x28,0x5c,0x8f,0xc2,0x67,0x71,0x40,0x2e,0xe7,0xef,0x31,0x9a,0x43,0x3d,0x40
,0x0a,0x58,0x61,0xe9,0x32,0x79,0x5a,0x40,0xf6,0x28,0x5c,0x8f,0xc2,0x67,0x71,0x40
,0x2e,0xe7,0xef,0x31,0x9a,0x43,0x3d,0x40,0x0a,0x58,0x61,0xe9,0x32,0x79,0x5a,0x40
,0x00,0x00,0x40,0x40,0x00,0x8c,0x2d,0x3a,0x81,0xff,0xff,0x00,0x01,0x02,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0xad,0xa3
,0x6d,0xa4,0x39,0xba,0xab,0x30,0x6f,0x60,0x32,0x42,0x6f,0x60,0x32,0x01,0x80,0x2d
,0xbd,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x20,0x41,0x90,0x29,0x3a
,0x81,0x00,0x00,0x00,0x00,0x6c,0xce,0xef,0xf7,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x20,0xc1,0x1e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00
,0x00,0x69,0x1b,0x7f,0xa2,0x32,0x79,0x5a,0x40,0xb5,0xa5,0x0e,0xf2,0x7a,0x44,0x3d
,0x40,0x9e,0xef,0x90,0x43,0x03,0x00,0x00,0x00,0xee,0xc2,0x00,0x00,0xb4,0xc2,0x01
,0x00,0x30,0x2a,0x3a,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x07,0xff,0xff,0xff,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x4e,0x00
,0x00,0x00,0x8b,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x73,0x68
,0x91,0xed,0x7c,0x6f,0x71,0x40,0x1f,0x9e,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc
,0xa2,0xa5,0x3c,0x79,0x5a,0x40,0x73,0x68,0x91,0xed,0x7c,0x6f,0x71,0x40,0x1f,0x9e
,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc,0xa2,0xa5,0x3c,0x79,0x5a,0x40,0x73,0x68
,0x91,0xed,0x7c,0x6f,0x71,0x40,0x1f,0x9e,0xa9,0x00,0xad,0x43,0x3d,0x40,0x58,0xbc
,0xa2,0xa5,0x3c,0x79,0x5a,0x40,0x00,0x00,0x40,0x40,0x00,0x8c,0x2d,0x3a,0x81,0xff
,0xff,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x60,0xad,0xa3,0x6d,0xa4,0x39,0xba,0xab,0x30,0x6f,0x60,0x32,0x42
,0x6f,0x60,0x32,0x01,0x80,0x2d,0xbd,0x01,0x00,0x00,0x10,0xb5,0x88,0x5b,0xc7,0x3d
,0x00,0x0e,0xc6,0x03,0x45,0x04,0x08,0x00,0x45,0x00
,0x00,0x58,0xf6,0xba,0x40,0x00,0x40,0x06,0xbb,0xd8,0xc0,0xa8,0x03,0xaf,0xc0,0xa8
,0x03,0x0d,0x88,0x34,0x7e,0x41,0x0b,0x49,0x42,0x02,0x7e,0xb8,0xb2,0x8f,0x80,0x18
,0x00,0x3f,0x22,0xfd,0x00,0x00,0x01,0x01,0x08,0x0a,0x46,0x48,0xc4,0x7d,0x9b,0x34
,0xfd,0x5c,0x00,0x00,0x21,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0xc0,0x67,0x40,0x4f,0x1c,0xe5,0x05,0x7d,0x88,0x36,0x40,0x68,0x66
,0x1c,0xd9,0x4b,0x7c,0x5c,0x40

};
    //1.创建一个socket
    int clientfd = socket(AF_INET, SOCK_STREAM, 0);
    //cout << "begin connect" << endl;
    if (clientfd == -1)
    {
        std::cout << "create client socket error." << std::endl;
        close(clientfd);
        return -1;
    }
    //cout << "great" << endl;
    //2.连接服务器
    struct sockaddr_in serveraddr;
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    serveraddr.sin_port = htons(SERVER_PORT);
   // cout << "great" << endl;
    if (connect(clientfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) == -1)
    {
        std::cout << "connect socket error." << std::endl;
        return -1;
    }
    int oldSocketFlag = fcntl(clientfd, F_GETFL, 0);
    int newSocketFlag = oldSocketFlag | O_NONBLOCK;
    //cout << "great" << endl;
    if (fcntl(clientfd, F_SETFL,  newSocketFlag) == -1)
    {
        close(clientfd);
       // std::TRACE( "set socket to nonblock error." );
        return -1;
    }

    //3. 不断向服务器发送数据，或者出错退出
    int count = 0;
    cout << "begin send" << endl;
    int ret = send(clientfd, data3, sizeof(data3), 0);
    cout << ret << endl;
    int ret2 = send(clientfd, SEND_DATA, strlen(SEND_DATA), 0);
    cout << ret2 << endl;
    //int ret3 = send(clientfd, data2, sizeof(data2), 0);
    // if(int len = recv(clientfd, buff, 1024, 0)){
    //     cout << buff << endl; 
    //     int ret1 = send(clientfd, "hello world", strlen("hello world"), 0);
    // }

    // cout << "send to server : " <<  "gogogo" << endl; 
    // int a = send(clientfd, "gogogo", strlen("gogogo"), 0);

    //int i = 1;
    while(true){
        //int ret = send(clientfd, SEND_DATA, strlen(SEND_DATA), 0);
        //int ret1 = send(clientfd, "helloworld", strlen("helloworld"), 0);
        // cout << "i : "  << i << endl; 
        uint8_t buff[100000];
        int len = recv(clientfd, buff, sizeof(buff), MSG_DONTWAIT);
        cout << "len : " << len << endl; 
        cout << "buff[0] : " << buff[0] << endl; 
        if(len > 0 && len < 1000){
            // cout << "len : " << len << endl;
            // string receivedData(buff, 1024);
            // cout << "buff : " << receivedData << endl; 
            int ret1 = send(clientfd, data2, sizeof(data2), 0);
            cout << "ret1 : " << ret1 << endl; 
            // if(ret1 > 0){
            //     int ret2 = send(clientfd, data4, sizeof(data4), 0);
            //     cout << "ret2 : " << ret2 << endl; 
            // }
           
            // cout << "ret : "  << ret1 << endl; 
        }
        else if(len > 1000)
        {
            // if(data5)
            int ret2 = send(clientfd, data5, sizeof(data5), 0); 
            cout << "ret2 : " << ret2 << endl; 

        }
        memset(buff, 0, sizeof(buff));
       // i++;
        sleep(10);
    }







    // while (true)
    // {
    //    int ret = send(clientfd, "gogogo", strlen("gogogo"), 0);
    //    if(ret > 0)
    //    {
    //         cout << "send to server : " <<  "gogogo" << endl; 
    //    }

    //    char buff[1024];
    //    if(int len = recv(clientfd, buff, 1024, 0)){
    //         cout << buff << endl; 
    //         //int ret1 = send(clientfd, "hello world", strlen("hello world"), 0);
    //     }
    //     sleep(10);
    // }
    
    // while (true)
    // {
    //     int ret = send(clientfd, SEND_DATA, strlen(SEND_DATA), 0);
    //     if (ret != strlen(SEND_DATA))
    //     {
    //         std::cout << "send data error." << std::endl;
    //         break;
    //     } 
    //     else
    //     {
    //         count ++;
    //         std::cout << "send data successfully, count = " << count << std::endl;
    //     }
    // }

    //5. 关闭socket
    close(clientfd);

    return 0;
}