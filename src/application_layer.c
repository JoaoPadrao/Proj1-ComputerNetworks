// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>




int sendControlPacket(const char* filename,const int control_packet,long int fileLength){
    
    // Get file length in bytes
    size_t fileLengthL1 = 0;
    long int tempLength = fileLength;
    while (tempLength > 0) {
      fileLengthL1++;
      tempLength >>= 8;
    }
    long int fileNameLengthL2 = strlen(filename)+1;

    // Create control packet
    long int packet_size = 5 + fileLengthL1 + fileNameLengthL2; 
    unsigned char* controlPacket = (unsigned char*)malloc(packet_size);

    // Fill control packet
    int idx = 0;
    controlPacket[idx++] = control_packet;

    //File size 
    controlPacket[idx++] = 0x00; // T1 (file size)
    memcpy(controlPacket+idx, &fileLengthL1, sizeof(size_t));
    idx += sizeof(size_t);

    //File name
    controlPacket[idx++] = 0x01; // T2 (file name)
    memcpy(controlPacket + idx, filename, fileNameLengthL2);

    // Send control packet
    if(llwrite(controlPacket,packet_size) < 0){
        printf("Error sending control packet in llwrite().\n");
        free(controlPacket);
        return -1;
    }
    free(controlPacket);

    return 1;
}

int readControlPacket(const int control_packet,size_t * fileLength,unsigned char* packet){

    // Check control packet
    int idx = 0;
    if (packet[idx++] != control_packet) {
        printf("Invalid control packet.\n");
        return -1; 
    }

    // Check file size
    if (packet[idx++] != 0x00) { // T1 (file size)
        printf("Invalid type file size packet.\n");
        return -1; 
    }

    // Get file length 
    unsigned char fileSizeBytes = packet[idx]; // L1 (file size) 
    unsigned char aux[fileSizeBytes];
    memcpy(aux, packet + idx + 1, fileSizeBytes); // aux = V1 (file size)
    *fileLength = 0;
    for (int i = fileSizeBytes - 1; i >= 0; i--) {
        *fileLength |= aux[i] << (8 * i); // V1 (file size)
    }
 
    //Get file name
    unsigned char fileNameLength = packet[fileSizeBytes+4]; // L2 (file name)
    unsigned char* fileName = (unsigned char*)malloc(fileNameLength);
    memcpy(fileName, packet + fileSizeBytes + 5, fileNameLength); // V2 (file name)

    return 1;
}


int sendDataPacket(int dataSize,unsigned char* data){

    // Create data packet
    long int packet_size = 1 + 1 + 1 + dataSize; 
    unsigned char* dataPacket = (unsigned char*)malloc(packet_size);

    // Fill data packet
    int idx = 0;
    dataPacket[idx++] = DATA_PACKET; // T1 (data)
    // K = 256 * L2 + L1   
    dataPacket[idx++] = dataSize / 256; // L2 (data)  
    dataPacket[idx++] = dataSize % 256; // L1 (data) 
    for (int i = 0; i < dataSize; i++) {
        dataPacket[idx++] = data[i]; // P (data)
    }
    
    // Send data packet
    if(llwrite(dataPacket, packet_size) < 0){
        printf("Error sending data packet in llwrite().\n");
        return -1;
    }
    return 0;
}

int sendFile(const char* filename){

     // Open file
    FILE* file = fopen(filename, "rb"); 
    if (file == NULL) {
        printf("Error opening file.\n");
        exit(-1);
    }
 
    // Get file size (Mudar)
    int previousFile = ftell(file); // save previous position
    fseek(file, 0L, SEEK_END); // file pointer at end of file
    long int fileSize = ftell(file) - previousFile; // get file length
    fseek(file, previousFile, SEEK_SET); // go back to previous position

     // Send start control packet
    if(sendControlPacket(filename, START_PACKET, fileSize) < 0){
        printf("Error sending start control packet.\n");
        return -1;
    }

    // Send file
    unsigned char* data = (unsigned char*)malloc(MAX_PAYLOAD_SIZE-3); // -3 because of the header (C, L2, L1)
    int chunkDataSize;
    while((chunkDataSize = fread(data, 1, MAX_PAYLOAD_SIZE-3, file)) > 0){
        if(sendDataPacket(chunkDataSize,data) < 0){
            printf("Error sending data packet.\n");
            return -1;
        }
    }   
    free(data);
    fclose(file);

    // Send end control packet
    if(sendControlPacket(filename, END_PACKET, fileSize) < 0){
        printf("Error sending end control packet.\n");
        return -1;
    }
    printf("Sent end packet.\n");

    // Close connection
    if (llclose(0) < 0) {
        printf("Error send closing connection.\n");
        return -1;
    }

    return 1;
}


int receiveFile(const char* filename){
    size_t packetSize;

    unsigned char* packet = (unsigned char*)malloc(MAX_PAYLOAD_SIZE);
    if(llread(packet) < 0){
        printf("Error receiving start control packet.\n");
        return -1;
    }

    // Read start control packet
    if(readControlPacket(START_PACKET, &packetSize, packet) < 0){
        printf("Error reading start control packet.\n");
        return -1;
    }


    FILE* receiveFile = fopen((char *) filename, "wb+");

    if(receiveFile == NULL){
        printf("Error opening received file.\n");
        exit(-1);
    }

    // Receive file
    free(packet);
    int dataSize = 0;
     while((dataSize = llread(packet)) >= 0){
        if(packet[0] == DATA_PACKET){
            printf("Data packet received.\n");
            fwrite(packet + 3, 1, dataSize - 3, receiveFile);
            }
        else if(packet[0] == END_PACKET){
                printf("End packet received.\n");
                break;  
            }
        }
    free(packet);
    fclose(receiveFile);
    printf("Before closing connection.\n");
    if(llclose(0) < 0){
        printf("Error receive closing connection.\n");
        return -1;
    }   
    return 1;
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename){

    // Create connection parameters
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    strcpy(connectionParameters.serialPort, serialPort);
    if (strcmp(role, "tx") == 0) connectionParameters.role = LlTx;
    else if (strcmp(role, "rx") == 0) connectionParameters.role = LlRx;
    else printf("Invalid role\n");

    // Establish connection 
    int fd = llopen(connectionParameters);
    if (fd < 0) {
        printf("Error opening serial port.\n");
        exit(-1);
    }

    // Send / receive file 
    switch (connectionParameters.role){
        case LlTx:
            if(sendFile(filename) < 0){
                printf("Error sending file.\n");
                exit(-1);
            }
            break;
        case LlRx:
             if(receiveFile(filename) < 0){
                printf("Error sending file.\n");
                exit(-1);
            }
            break;
        default:
            break;
    }
}