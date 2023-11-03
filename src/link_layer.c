// Link layer protocol implementation

#include "link_layer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>


// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

struct termios oldtio;
struct termios newtio;
int alarmEnabled = FALSE;
int alarmCount = 0;

int maxNRetransmissions = 0;
int timeout = 0;
int fd;
int role;

unsigned char frame_tx = 0;
unsigned char frame_rx = 0;

// Alarm function handler
void alarmHandler(int signal){
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
    states state = START;
    int b1;

    printf("in getResponse\n");
    while(state != STATE_STOP){
        b1 = read(fd,&response_buffer,1);
        if(b1 > 0){ //If read was successful
            switch (state){
                case START:
                    if(response_buffer == FLAG) 
                        state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    printf("FLAG_RCV\n");
                    if(response_buffer == FLAG){
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
                    printf("A_RCV\n");
                    printf("response_buffer = 0x%02X\n", response_buffer);
                    if(response_buffer == FLAG){
                        state = FLAG_RCV;
                    }
                        if(response_buffer == CTRL_REJ0 || response_buffer == CTRL_REJ1 || response_buffer == CTRL_RR0 || response_buffer == CTRL_RR1 || response_buffer == CTRL_DISC){
                        state = C_RCV;
                        ctrl_field = response_buffer;
                    }
                    else{
                        state = START;
                    }
                    break;
                case C_RCV:
                    printf("C_RCV\n");
                    if(response_buffer == FLAG){
                        state = FLAG_RCV;
                    }
                    else if (response_buffer == (ADDR_Rx ^ ctrl_field)){
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
                    break;
            }
        }
        else if(b1 == 0){
            printf("Timeout -> NOTHING TO READ \n");
            return 0;
        }
        else{
            printf("Error reading from serial port\n");
            return -1;
        }
    }
    return ctrl_field;
}


// llopen function
int llopen(LinkLayer connectionParameters){

    unsigned char read_buffer[5] = {0};
    maxNRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    states state = START;
    role = connectionParameters.role;

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        return -1;
    }

    

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
    switch (connectionParameters.role){
        case LlTx:{ //Transmitter
            (void) signal(SIGALRM, alarmHandler);
            alarmEnabled = FALSE;
            while(alarmCount < maxNRetransmissions && state != STATE_STOP){
                if(alarmEnabled == FALSE){
                    //Send SET
                    printf("Sending SET\n");
                    sendTrama(fd, ADDR_Tx, CTRL_SET);
                    alarm(timeout); // Activates alarm during timeout seconds
                    alarmEnabled = TRUE;
                }
                //Read the UA response
                while(state != STATE_STOP && alarmCount < 3){
                    int b1 = read(fd,read_buffer,1); //Reads one byte
                    if(b1 > 0){
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
                                else if (read_buffer[0] == (ADDR_Rx ^ CTRL_UA)){
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
            }
            if (state != STATE_STOP) return -1;
            break;
        }
        case LlRx:{
            //Read SET
            while(state != STATE_STOP){
            int b1 = read(fd,read_buffer,1);
                if(b1 > 0){
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
                        else if (read_buffer[0] == (ADDR_Tx ^ CTRL_SET)){
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
            }
            //Send UA
            printf("Sending UA\n");
            sendTrama(fd, ADDR_Rx, CTRL_UA);
            break;
        }
        default: //Error
            printf("Error: Invalid role\n");
            return -1;
        }
    return fd;
}


// llwrite function
int llwrite(const unsigned char *buf, int bufSize){

    printf("Entered llwrite\n");
    int frameSize = 6 + bufSize; // 6 bytes are the fixed size of the Information frame

    (void) signal(SIGALRM, alarmHandler);

    // Create frame
    unsigned char* frame = malloc(frameSize);
    frame[0] = FLAG;
    frame[1] = ADDR_Tx; 
    // To distinguish between frames (Control field) we use the frame_tx variable
    if(frame_tx % 2 == 0) frame[2] = C_I0;
    else frame[2] = C_I1;
    frame[3] = frame[1] ^ frame[2]; // BCC1 = A ^ C


    unsigned char BCC2 = 0;
    
    // Calculate BCC2
    for (int i = 0; i < bufSize; i++) BCC2 ^= buf[i];

    // Stuffing
    int frameIndex = 4; // Start at 4 because we already have 4 bytes
    for(int i = 0; i < bufSize; i++){
        if(buf[i] == FLAG || buf[i] == ESC){ // If we find a FLAG or ESC byte we need to stuff
            frame = realloc(frame, ++frameSize); // Increase frame size by 1 because we need to add the ESC byte
            frame[frameIndex++] = ESC;
            frame[frameIndex++] = buf[i] ^ XOR_STUFFING; // We add the ESC and after XOR between the byte and 0x20
        }
        else { // If we don't find a FLAG or ESC byte we just add the byte to the frame
            frame[frameIndex++] = buf[i];
        }
    }

    // Continue creating the frame
    frame[frameIndex++] = BCC2; 
    frame[frameIndex] = FLAG; // End of frame


    int transmissionsCounter = 0;
    int accepted = FALSE;
    alarmEnabled = FALSE;

    while((transmissionsCounter <= maxNRetransmissions)){
        printf("Transmissions counter: %d\n", transmissionsCounter);
        if(alarmEnabled == FALSE){
            // Send frame
            printf("Sending frame\n");
            write(fd,frame, frameSize);
            alarm(timeout);
            sleep(1); // Sleep for 1 second to avoid sending the next frame before the receiver sends the RR 
            alarmEnabled = TRUE;
            transmissionsCounter++;
        }
        // Get response
        unsigned char response = getResponse();
        if(response == CTRL_RR0 || response == CTRL_RR1){
            printf("Receive RR\n");
            accepted = TRUE;
            alarm(0);
            alarmEnabled = FALSE; 
            frame_tx = (frame_tx+1) % 2; // Increment frame_tx to change the control field of the next frame
        }
        
        if(accepted) break;
    }
    if(accepted){
        printf("It was accepted (Received RR)\n");
        return frameSize; // Return the number of bytes written
    } 
    else{
        //llclose(0);
        return -1;

    }
}


// llread function
int llread(unsigned char *packet){
    int index = 0;
    unsigned char b_read;
    unsigned char control_field;
    states state = START;
    printf("Initiating reading process\n");

    while(state != STATE_STOP){

        if(read(fd, &b_read, 1) > 0){

            switch (state){

            case START:
                if(b_read == FLAG)
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if(b_read == ADDR_Tx)
                    state = A_RCV;
                else if(b_read != FLAG)
                    state = START;
                break;
            
            case A_RCV:
                //Confirm that frames are NOT repeated
                if((b_read == C_I0 && (frame_rx%2==0)) || (b_read == C_I1 && (frame_rx%2==1))){
                    state = C_RCV;
                    control_field = b_read;
                }
                else if ((b_read == C_I0 && (frame_rx%2!=0)) || (b_read == C_I1 && (frame_rx%2!=1))){
                    //If it's a duplicate, the data field is discarded, but we must CONFIRM the frame with RR
                    sendTrama(fd, ADDR_Tx, (R((frame_rx + 1)%2) | 0x05));  //Send RR0 or RR1
                    return 0;
                }
                else if (b_read == CTRL_DISC) {
                    sendTrama(fd, ADDR_Tx, CTRL_DISC);
                    return 0;
                }  
                else if (b_read == FLAG){
                    state = FLAG_RCV; // We can have more than one flag
                }
                else
                    state = START;
                break;
        
            case C_RCV:
                if(b_read == (control_field ^ ADDR_Tx))
                    state = DATA_FIELD;
                else if(b_read == FLAG)
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case DATA_FIELD:
                    if (b_read == ESC)
                        state = DESTUFFING;

                    else if (b_read == FLAG){
                        index--;
                        unsigned char BBC2 = packet[index];
                        packet[index] = '\0';
                        unsigned char data_check = packet[0];

                        for (unsigned int k = 1; k < index; k++){
                            data_check ^= packet[k];
                        }

                        if (BBC2 == data_check){
                            state = STATE_STOP;
                            
                            if(frame_rx % 2 == 0){
                            printf("BCC2 CORRECT: Sending confirmation of frame 0\n");
                            sendTrama(fd, ADDR_Rx, CTRL_RR0);
                            }
                            else if(frame_rx % 2 == 1){
                            printf("BCC2 CORRECT: Sending confirmation of frame 1\n");
                            sendTrama(fd, ADDR_Rx, CTRL_RR1);
                            }

                            frame_rx = (frame_rx + 1)%2;
                            return index; 
                        }
                        else if(BBC2 != data_check){
                    
                            if(frame_rx % 2 == 0){
                            printf("ERROR! BCC2 INCORRECT: Frame 0 Rejected\n");
                            sendTrama(fd, ADDR_Rx, CTRL_REJ0);
                            }
                            else if(frame_rx % 2 == 1){
                            printf("ERROR! BCC2 INCORRECT: Frame 1 Rejected\n");
                            sendTrama(fd, ADDR_Rx, CTRL_REJ1);
                            }

                            return -1;
                        };

                    }
                    else{
                        packet[index++] = b_read;
                    }
                    break;

            case DESTUFFING:
                state = DATA_FIELD;
                packet[index++] = b_read^ XOR_STUFFING;
                break;

            default:
                break;
            }
        }
    }
    return 1;
}

//llclose function
int llclose(int showStatistics){
    states state = START;
    unsigned char read_buffer[5] = {0};
    (void) signal(SIGALRM, alarmHandler);

    switch(role){
        case LlTx:{
            while(state != STATE_STOP){
                //Send first DISC to receiver
                printf("Sending DISC to Receiver\n");
                sendTrama(fd, ADDR_Tx, CTRL_DISC);
                alarm(timeout); // Activates alarm during timeout seconds
                alarmEnabled = TRUE;

                //Read the DISC that was sent back
                while(alarmEnabled == TRUE && state != STATE_STOP){
                    int b1 = read(fd,read_buffer,1); //Reads one byte
                    if(b1 > 0){
                        switch(state){
                        case START:  
                            printf("START CLOSE\n");
                            if(read_buffer[0] == FLAG){
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            printf("FLAG_RCV CLOSE\n");
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
                            printf("A_RCV CLOSE\n");
                            if(read_buffer[0] == FLAG){
                                state = FLAG_RCV;
                            }
                            else if(read_buffer[0] == CTRL_DISC){
                                state = C_RCV;
                            }
                            else{
                                state = START;
                            }
                            break;
                        case C_RCV:
                            printf("C_RCV CLOSE\n");
                            if(read_buffer[0] == FLAG){
                                state = FLAG_RCV;
                            }
                            else if (read_buffer[0] == (ADDR_Rx ^ CTRL_DISC)){
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
                                alarmEnabled = FALSE;
                            }
                            else{
                                state = START;       
                            }
                            break;
                        default:
                            break;
                        }
                    }
                }
                if(state != STATE_STOP) return -1;
            }
        }
        case LlRx:{
            //Read DISC from transmitter
            while(state != STATE_STOP){
                    int b1 = read(fd,read_buffer,1);
                    if(b1 > 0){
                        switch(state){
                        case START:  
                            printf("START CLOSE\n");
                            if(read_buffer[0] == FLAG){
                                state = FLAG_RCV;
                            }
                            break;
                        case FLAG_RCV:
                            printf("FLAG_RCV CLOSE\n");
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
                            printf("A_RCV CLOSE\n");
                            if(read_buffer[0] == FLAG){
                                state = FLAG_RCV;
                            }
                            else if(read_buffer[0] == CTRL_DISC){
                                state = C_RCV;
                            }
                            else{
                                state = START;
                            }
                            break;
                        case C_RCV:
                            printf("C_RCV CLOSE\n");
                            if(read_buffer[0] == FLAG){
                                state = FLAG_RCV;
                            }
                            else if (read_buffer[0] == (ADDR_Tx ^ CTRL_DISC)){
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
                            else
                                state = START;
                            break;
                        default:
                            break;
                        }
                    }
            }
        // Send DISC to transmitter
        printf("Sending DISC to Transmitter\n");    
        sendTrama(fd, ADDR_Rx, CTRL_DISC);
    }
    default:
        break;
    }

    //Send UA to close connection
    if(role == LlTx) {
        printf("Sending UA to close connection\n");
        sendTrama(fd, ADDR_Tx, CTRL_UA);
    }
        

    if(close(fd) < 0)
        return -1;
    
    return 1;
}
