// Link layer protocol implementation

#include "link_layer.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>



// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FLAG 0x7E
#define ADDR_Tx 0x03
#define ADDR_Rx 0x03

typedef enum states{
    Start, FLAG_RCV, A_RCV, BCC_RCV, C_RCV, STATE_STOP
};
int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters)
{
    enum states state = Start;  
    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        return -1;
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
   
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

   
    unsigned char buffer[5] = {0};
    switch (connectionParameters.role)
    {
    case LlTx:
        (void)signal(SIGALRM, alarmHandler);
       
        while((alarmCount < connectionParameters.nRetransmissions) && (state != STATE_STOP)){
        int bytes = write(fd, buf, BUF_SIZE);
        alarm(connectionParameters.timeout);
        alarmEnabled = TRUE;
        printf("%d bytes written\n", bytes);

    //Read the UA
        while((state != STATE_STOP) && (alarmEnabled == TRUE)){
            int b1 = read(fd,buffer,1);

            if(b1 > 0){
                printf("var = 0x%02X\n", buf[0]);
            }
            else{
                continue;
            }
            printf("%d\n",state);
            switch(state){
                case Start:  
                    if(buf[0] == 0x7E){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(buf[0] == 0x7E){
                        state = FLAG_RCV;
                    }
                    else if (buf[0] == 0x03){
                        state = A_RCV;
                    }
                    else{
                        state = Start;
                    }
                    break;
                case A_RCV:
                    if(buf[0] == 0x7E){
                        state = FLAG_RCV;
                    }
                    else if(buf[0] == 0x07){
                        state = C_RCV;
                    }
                    else{
                        state = Start;
                    }
                    break;
                case C_RCV:
                    if(buf[0] == 0x7E){
                        state = FLAG_RCV;
                    }
                    else if (buf[0] == 0x03 ^ 0x07){
                        state =  BCC_RCV;
                    }  
                    else{
                        state = Start;
                    }
                    break;
                case BCC_RCV:
                    if(buf[0] == 0x7E){
                        state = STATE_STOP;
                        alarm(0);
                        alarmEnabled = 0;
                    }
                    else{
                        state = Start;
                    }
                default:
                    printf("%d",state);
            }

        }
    }
        break;
   
    case LlRx:
       
        break;
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
