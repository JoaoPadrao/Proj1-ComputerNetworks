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

struct termios oldtio;
struct termios newtio;
int alarmEnabled = FALSE;
int alarmCount = 0;
int maxNRetransmissions = 0;
int TxInfNumber = 0;
int RxInfNumber = 1;
int fd;
int timeout = 0;
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
    states state = START;
    printf("in getResponse\n");
     while(state != STATE_STOP){
            int b1 = read(fd,&response_buffer,1);
            if(b1 > 0){ //If read was successful
                switch(state){
                    case START:  
                        printf("START\n");
                        if(response_buffer == FLAG){
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        printf("FLAG_RCV\n");
                        printf("response_buffer = 0x%02X\n", response_buffer);
                        if(response_buffer == FLAG){
                            state = FLAG_RCV;
                        }
                        else if (response_buffer == ADDR_Tx){
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
                        else if (response_buffer == (ADDR_Tx ^ ctrl_field)){
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
    timeout = connectionParameters.timeout;
    states state = START;  
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
    switch (connectionParameters.role)
    {
    case LlTx: // Transmitter
        printf("Vou enviar o SET\n");
        (void)signal(SIGALRM, alarmHandler);
    
        while((alarmCount < maxNRetransmissions) && (state != STATE_STOP)){
        sendTrama(fd, ADDR_Tx, CTRL_SET);
        alarm(timeout); // Activates alarm during timeout seconds
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
    break;
   
    case LlRx: // Receiver    
        //Read the SET
        printf("Vou receber o SET\n");
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
        printf("Vou enviar o UA\n");
        sendTrama(fd, ADDR_Rx, CTRL_UA);
        break;

        default: // Error
            return -1;
    }
    printf("Vou retornar o fd\n");
    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    printf("in llwrite\n");
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
            frame = realloc(frame, ++frameSize); // Increase frame size by 1 because we need to add the ESC byte
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
        printf("Transmission #%d\n", transmissionsCounter+1);
        alarmEnabled = FALSE;
        alarm(timeout); // Activates alarm during timeout seconds
        accepted = FALSE;
        rejected = FALSE;
        while((alarmEnabled == FALSE) && (accepted == FALSE) && (rejected == FALSE)){
            // Send frame
            printf("Sending frame... DENTRO DO 2º WHILE\n");
            write(fd,frame, frameSize);
            // Get response
            unsigned char response = getResponse();
            printf("Response: 0x%02X\n", response);
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
    alarm(0);
    free(frame);
    if(accepted){
        return frameSize; // Return the number of bytes written
    }
    else{
        llclose(0);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    printf("in llread\n");
    states state = START; //inicializa maq de estados no START
    unsigned char control_field, byte_read;  //byte lido pela função read e byte de controlo
    int index = 0;

    while(state != STATE_STOP){

        if(read(fd, &byte_read, 1) > 0){
            
            // maq de estados
            switch (state)
            {
            case START:
                if (byte_read == FLAG) 
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte_read == ADDR_Tx) //(0x03) = Comandos enviados pelo Emissor e Respostas enviadas pelo Receptor
                    state = A_RCV;
                else if (byte_read == FLAG) //caso em q trama é iniciada por + q 1a flag
                    state = FLAG_RCV;
                else
                    state = START;
                break;

            case A_RCV:
            //confirmar que tramas NÃO são repetidas
                if((byte_read == C_I0 && TxInfNumber==0) || (byte_read == C_I1 && TxInfNumber==1)){
                    state = C_RCV;
                    control_field = byte_read;
                }
                else if ((byte_read == C_I0 && TxInfNumber!=0) || (byte_read == C_I1 && TxInfNumber!=1)){
                    // Se se tratar dum duplicado, o campo de dados é descartado, mas deve fazer-se CONFIRMAÇÃO da trama com RR
                    sendTrama(fd, ADDR_Tx, (R((TxInfNumber + 1)%2) | 0x05)); //envio de RR0 ou RR1
                    return 0;
                }
                else if (byte_read == CTRL_DISC) {
                    sendTrama(fd, ADDR_Tx, CTRL_DISC);
                    return 0;
                }  
                else if (byte_read == FLAG){
                    state = FLAG_RCV; //podemos ter + que uma flag
                }
                else
                    state = START;
                break;
                
            case C_RCV:
                //A XOR C -> Field to detect the occurrence of errors in the header
                if (byte_read == (ADDR_Tx ^ control_field))
                    state = DATA_FIELD;
                else if (byte_read == FLAG)
                    state = FLAG_RCV;
                else 
                    state = START;
                break;

            // data
            case DATA_FIELD:
                if (byte_read == ESC) 
                    state = DESTUFFING;
                else if (byte_read == FLAG){  //fim da trama
                                    
                    index--;
                    unsigned char bcc2 = packet[index]; //bcc2 é o ultimo byte do packet
                    packet[index] = '\0';
                    unsigned char check = packet[0];

                    for(int k = 1; k < index; k++){
                        //The BCC is calculated by performing an XOR operation on all the bytes in the packet except the last one
                        check ^= packet[k];  //D1 XOR D2 XOR D3 … XOR DN
                    }

                    if (bcc2 == check){ //Reach the end
                                    
                        state = STATE_STOP;
                        //send confirmation
                        if(TxInfNumber==0){
                            sendTrama(fd, ADDR_Tx, CTRL_RR0);
                        }
                        else if(TxInfNumber==1){
                            sendTrama(fd, ADDR_Tx, CTRL_RR1);
                        }
                        TxInfNumber = (TxInfNumber+1)%2; //actualiza o frame number
                        
                        return index; // bytes read

                    }
                    else {
                        // houve erro nos dados, envio de supervisao  
                        if(TxInfNumber==0){
                            sendTrama(fd, ADDR_Tx, CTRL_REJ0);
                        }
                        else if(TxInfNumber==1){
                            sendTrama(fd, ADDR_Tx, CTRL_REJ1);
                        }
                        return -1;
                    }
                }
                else{
                    packet[index] = byte_read;
                    index++;
                }              
                break;

                
            case DESTUFFING: //DESTUFFING
                packet[index] = byte_read ^ XOR_STUFFING; 
                index++;
                state = DATA_FIELD;
                break;
            default:
                break;
            }
        }
    }

    return -1;    
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    states state = START;
    unsigned char read_buffer[5] = {0};
    (void) signal(SIGALRM, alarmHandler);

    
    while((alarmCount < maxNRetransmissions) && (state != STATE_STOP)){
        //Send first DISC to receiver
        sendTrama(fd, ADDR_Tx, CTRL_DISC);
        alarm(timeout); // Activates alarm during timeout seconds
        alarmEnabled = TRUE;
        
       //Read the DISC that was sent back
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
                    printf("BCC_RCV CLOSE\n");
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

    if(state!=STATE_STOP){
        printf("Error: DISC not received\n");
        return -1;
    }
    //Send UA to close connection
    sendTrama(fd, ADDR_Tx, CTRL_UA);

    return 1;
}

