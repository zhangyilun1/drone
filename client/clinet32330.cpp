#include <sys/types.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <fcntl.h>
#include <iomanip>
#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT     32330
#define SEND_DATA       "0x0a, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x44, 0x30, 0x30, 0x46"

using namespace std;

int main(int argc, char* argv[])
{
    // unsigned char data[] = {0x0a, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x44, 0x30, 0x30, 0x46};
    // unsigned char data2[] = {0x06, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x44, 0x30, 0x30, 0x46};
    // unsigned char data3[] = {0x0a, 0x00, 0x39, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 0x00, 0x41, 0x44, 0x30, 0x30, 0x46, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    // 0x00, 0x4d, 0x33, 0x30, 0x54, 0x07, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55,
    // 0x4e, 0x4b, 0x4e, 0x4f, 0x57, 0x4e, 0x00, 0x00, 0x20, 0x41, 0x05, 0x00, 0x00, 0x00, 
    // 0x00, 0x00, 0x00, 0x00, 0x31, 0x2e, 0x30, 0x2e, 0x30};
    // unsigned char data4[] = {0x08, 0x00, 0x0d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x62, 0xec, 0x64, 0x46, 0x11, 0x11, 0xff};
   
    const char snCode[24] = "MYdrone0102000000000000";
    const char versionNumber[12] = "10.11.12000";
    char combinedData[36];
    
    // buff.inser = "MYdrone0102000000000000010.11.120000";
    std::memcpy(combinedData, snCode, sizeof(snCode));
    std::memcpy(combinedData + sizeof(snCode), versionNumber, sizeof(versionNumber));

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
    int ret = send(clientfd, combinedData, sizeof(combinedData), 0);
    char buff[10240];
    while(true)
    {
        int ret = recv(clientfd, buff, 10240, 0);
        cout << ret << endl;
        if(ret){
            for (int i = 0; i < 50; i++) {
                uint8_t byte = static_cast<uint8_t>(buff[i]);
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
        }
        sleep(1);
    }
    //int ret3 = send(clientfd, data2, sizeof(data2), 0);
    // if(int len = recv(clientfd, buff, 1024, 0)){
    //     cout << buff << endl; 
    //     int ret1 = send(clientfd, "hello world", strlen("hello world"), 0);
    // }

    // cout << "send to server : " <<  "gogogo" << endl; 
    // int a = send(clientfd, "gogogo", strlen("gogogo"), 0);

    // //int i = 1;
    // while(true){
    //     //int ret1 = send(clientfd, "helloworld", strlen("helloworld"), 0);
    //     // cout << "i : "  << i << endl; 
    //     uint8_t buff[100000];
    //     int len = recv(clientfd, buff, sizeof(buff), MSG_DONTWAIT);
    //     cout << "len : " << len << endl; 
    //     cout << "buff[0] : " << buff[0] << endl; 
        // if(len > 0 && len < 1000){
        //     // cout << "len : " << len << endl;
        //     // string receivedData(buff, 1024);
        //     // cout << "buff : " << receivedData << endl; 
        //     int ret1 = send(clientfd, data2, sizeof(data2), 0);
        //     cout << "ret1 : " << ret1 << endl; 
        //     // if(ret1 > 0){
        //     //     int ret2 = send(clientfd, data4, sizeof(data4), 0);
        //     //     cout << "ret2 : " << ret2 << endl; 
        //     // }
           
        //     // cout << "ret : "  << ret1 << endl; 
        // }
        // else if(len > 1000)
        // {
        //     int ret2 = send(clientfd, data5, sizeof(data5), 0); 
        //     cout << "ret2 : " << ret2 << endl; 
        // }
    //     memset(buff, 0, sizeof(buff));
    //    // i++;
    //     sleep(10);
    // }


    //5. 关闭socket
    close(clientfd);

    return 0;
}