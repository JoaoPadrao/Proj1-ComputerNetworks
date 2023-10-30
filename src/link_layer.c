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
int TxInfNumber = 0;
int RxInfNumber = 1;
int fd;

// Alarm handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

// Send a supervision frame
int sendTrama(int fd, unsigned char Address, unsigned char Control){
    unsigned char BUFFER[5] = {FLAG, Address, Control, Address ^ Control, FLAG};
    return write(fd, BUFFER, 5);
}

int getResponse(){
    unsigned char response_buffer;
    unsigned char ctrl_field;
    enum states state = START;
     while(state != STATE_STOP){
            int b1 = read(fd,&response_buffer,1);
            if(b1 > 0){ //If read was successful
                switch(state){
                    case START:  
                        if(response_buffer == FLAG){
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if(response_buffer[0] == FLAG){
                            state = FLAG_RCV;
                        }
                        else if (response_buffer == ADDR_Rx){
                            state = A_RCV;
                        }
                        else{
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if(response_buffer == FLAG){
                            state = FLAG_RCV;
                        }
                         if(response_buffer == CTRL_REJ0 || response_buffer == CTRL_REJ1 || response_buffer == CTRL_RR0 || response_buffer == CTRL_RR1 || CTRL_DISC){
                            state = C_RCV;
                            ctrl_field = response_buffer;
                        }
                        else{
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if(response_buffer == FLAG){
                            state = FLAG_RCV;
                        }
                        else if (response_buffer == ADDR_Rx ^ ctrl_field){
                            state =  BCC_RCV;
                        }  
                        else{
                            state = START;
                        }
                        break;
                    case BCC_RCV:
                        if(response_buffer == FLAG){
                            state = STATE_STOP;
                        }
                        else{
                            state = START;
                        }
                        break;
                    default:
                        printf("%d",state);
                        break;
                }
            }
        }
    return ctrl_field;
}

// llopen function
int llopen(LinkLayer connectionParameters)
{
    unsigned char read_buffer[5] = {0};
    maxNRetransmissions = connectionParameters.nRetransmissions;
    states state = START;  
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

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
                    else if (read_buffer[0] == ADDR_Rx){
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
                    else if (read_buffer[0] == ADDR_Rx ^ CTRL_UA){
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
                    break;
                default:
                    printf("%d",state); 
                    break;
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
                    break;
                default:
                    printf("%d",state);
                    break;
            }

        }
        sendTrama(fd, ADDR_Rx, CTRL_UA);
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
    int frameSize = 6 + bufSize; // 6 bytes are the fixed size of the frame

    // Create frame
    unsigned char* frame = malloc(frameSize);
    frame[0] = FLAG;
    frame[1] = ADDR_Tx; 
    frame[2] = N(TxInfNumber); // To distinguish between frames (Control field) 
    frame[3] = frame[1] ^ frame[2]; // BCC1 = A ^ C

    unsigned char BCC2 = 0;
    // Calculate BCC2
    for (int i = 0; i < bufSize; i++)
    {
        BCC2 ^= buf[i];
    }

    // Stuffing 
    int frameIndex = 4; // Start at 4 because we already have 4 bytes
    for (int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG || buf[i] == ESC)
        {
            frameSize += 2; // Increase frame size by 2
            frame = realloc(frame, frameSize); 
            frame[frameIndex++] = ESC;
            frame[frameIndex++] = buf[i] ^ XOR_STUFFING; // We add the ESC and after XOR between the byte and 0x20
        }
        else
        {
            frame[frameIndex++] = buf[i];
        }
    }

    // Continue creating the frame
    frame[frameIndex++] = BCC2; 
    frame[frameIndex] = FLAG; // End of frame

    int transmissionsCounter = 0;
    int accepted = FALSE;
    int rejected = FALSE;

    //Loop to send the frame
    while(transmissionsCounter < maxNRetransmissions){
        alarmEnabled = FALSE;
        alarm(connectionParameters.timeout); // Activates alarm during timeout seconds
        accepted = FALSE;
        rejected = FALSE;
        while((alarmEnabled == FALSE) && (accepted == FALSE) && (rejected == FALSE)){
            // Send frame
            write(fd,frame, frameSize);
            // Get response
            unsigned char response = getResponse();
            if((response == CTRL_RR0) || (response == CTRL_RR1)){
                // Response is RR0 or RR1, frame accepted 
                accepted = TRUE;
                TxInfNumber = (TxInfNumber+1) % 2;   // Change the information number of the frame
            }
            else if(response == CTRL_REJ0 || response == CTRL_REJ1){ 
                // Response is REJ0 or REJ1, frame rejected
                rejected = TRUE;
            }
            else continue;  
        }
        if(accepted) break;
        transmissionsCounter++;
    }
    free(frame);
    if(accepted){
        return frameSize; // Return the number of bytes written
    }
    else{
        llclose(fd);
        return -1;
    }
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

