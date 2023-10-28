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


int alarmEnabled = FALSE;
int alarmCount = 0;
int maxNRetransmissions = 0;

// Alarm handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int sendTrama(int fd, unsigned char Address, unsigned char Control){
    unsigned char BUFFER[5] = {FLAG, Address, Control, Address ^ Control, FLAG};
    return write(fd, BUFFER, 5);
}

// llopen function
int llopen(LinkLayer connectionParameters)
{
    unsigned char read_buffer[5] = {0};
    maxNRetransmissions = connectionParameters.nRetransmissions;
    states state = START;  
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
        return -1;
    }

    //State Machine
    switch (connectionParameters.role)
    {
    case LlTx: // Transmitter
        (void)signal(SIGALRM, alarmHandler);
    
        while((alarmCount < maxNRetransmissions) && (state != STATE_STOP)){
        sendTrama(fd, ADDR_Tx, CTRL_SET);
        alarm(connectionParameters.timeout); // Activates alarm during timeout seconds
        alarmEnabled = TRUE;

    //Read the UA
        while((state != STATE_STOP) && (alarmEnabled == TRUE)){
            int b1 = read(fd,read_buffer,1); //Reads one byte
            if(b1 > 0){ //If read was successful
                printf("var = 0x%02X\n", read_buffer[0]);
            }
            else{
                continue;
            }
            switch(state){
                case START:  
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    else if (read_buffer[0] == ADDR_Tx){
                        state = A_RCV;
                    }
                    else{
                        state = START;
                    }
                    break;
                case A_RCV:
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    else if(read_buffer[0] == CTRL_UA){
                        state = C_RCV;
                    }
                    else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    else if (read_buffer[0] == ADDR_Tx ^ CTRL_UA){
                        state =  BCC_RCV;
                    }  
                    else{
                        state = START;
                    }
                    break;
                case BCC_RCV:
                    if(read_buffer[0] == FLAG){
                        state = STATE_STOP;
                        alarm(0);
                        alarmEnabled = 0;
                    }
                    else{
                        state = START;
                    }
                default:
                    printf("%d",state);
            }

        }
    }
    break;
   
    case LlRx: // Receiver    
        //Read the SET
        while(state != STATE_STOP){
            int b1 = read(fd,read_buffer,1);
            if(b1 > 0){
                printf("var = 0x%02X\n", read_buffer[0]);
            }
            else{
                continue;
            }
            switch(state){
                case START:  
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    else if (read_buffer[0] == ADDR_Tx){
                        state = A_RCV;
                    }
                    else{
                        state = START;
                    }
                    break;
                case A_RCV:
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    else if(read_buffer[0] == CTRL_SET){
                        state = C_RCV;
                    }
                    else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(read_buffer[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    else if (read_buffer[0] == ADDR_Tx ^ CTRL_SET){
                        state =  BCC_RCV;
                    }  
                    else{
                        state = START;
                    }
                    break;
                case BCC_RCV:
                    if(read_buffer[0] == FLAG){
                        state = STATE_STOP;
                    }
                    else{
                        state = START;
                    }
                default:
                    printf("%d",state);
            }

        }
        sendTrama(fd, ADDR_Tx, CTRL_UA);
        break;

        default: // Error
            return -1;
    }
    return fd;
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

