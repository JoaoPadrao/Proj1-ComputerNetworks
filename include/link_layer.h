// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

typedef enum 
{
    START, 
    FLAG_RCV, 
    A_RCV, 
    C_RCV, 
    BCC_RCV, 
    STATE_STOP
} states;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE     0
#define TRUE      1
#define FLAG      0x7E
#define ADDR_Tx   0x03
#define ADDR_Rx   0x01
#define CTRL_SET  0x03
#define CTRL_UA   0x07
#define CTRL_DISC 0x0B
#define CTRL_RR0  0x05
#define CTRL_RR1  0x85
#define CTRL_REJ0 0x01
#define CTRL_REJ1 0x81
#define ESC       0x7D
#define ESC_FLAG  0x5E


// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(unsigned char *packet);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console on close.
// Return "1" on success or "-1" on error.
int llclose(int showStatistics);

#endif // _LINK_LAYER_H_
