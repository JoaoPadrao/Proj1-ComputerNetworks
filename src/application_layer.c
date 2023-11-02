// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#define FILE_SIZE 0
#define FILE_NAME 1
#define DATA_PCKT 1
#define START_PCKT 2
#define END_PCKT 3

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


int readCtrlPacket(unsigned char control, unsigned char* buf, size_t* file_size, char* filename){
    
    int bufSize;
    if((bufSize = llread(buf)) < 0){
        printf("Error reading control packet\n");
        return -1;
    }

    if(buf[0] != control){
        printf("Invalid control packet\n");
        return -1;
    }

    int i = 1;
    unsigned char type;
    while(i < bufSize){
        type = buf[i++];
        if(type == 0){
            printf("viu o size\n");
            *file_size = buf[i];
            i += sizeof(size_t);
        }
        else if(type == 1){
            printf("viu o name\n");
            *filename = buf[i];
            i += *filename;
        }
        else{
            printf("Invalid control packet type\n");
            return -1;
        }
    }
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
    return 1;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename){

    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort,serialPort);
    if(strcmp(role,"tx") == 0) linkLayer.role = LlTx;
    else if(strcmp(role,"rx") == 0) linkLayer.role = LlRx;
    else printf("Invalid role\n");
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;
    if(llopen(linkLayer) < 0){
        printf("Connection error\n");
    }

    switch (linkLayer.role){
        case LlRx:{
            size_t packetSize;
            char receivedFilename[0xff];
            unsigned char* buf = (unsigned char*)malloc(MAX_PAYLOAD_SIZE);
            printf("antes de ler start packet\n");

            if(readCtrlPacket(START_PCKT, buf, &packetSize, receivedFilename) < 0){
                printf("Error readind control file");
                exit(-1);
            }

            printf("depois do read control packet\n");

            FILE* fileOut = fopen((char *) filename, "wb+");

            if(fileOut == NULL){
                printf("Error opening file.\n");
                exit(-1);
            }

            printf("abre o file\n");
            free(buf);
            int dataSize;
            while((dataSize = llread(buf)) >= 0){

                if(buf[0] == END_PCKT){
                    printf("recebeu packet final\n");
                    break;
                }
                else{
                    printf("esta a escrever no file\n");
                    fwrite(buf+3, 1, buf[1] * 256 + buf[2], fileOut);
                }
            }
            free(buf);
            fclose(fileOut);
            printf("fecha o file\n");
            printf("antes do llclose\n");
            if(llclose(FALSE) < 0) exit(-1);
            printf("depois do llclose\n");
            break;
            }
        case LlTx:{
            FILE* file = fopen(filename, "rb");
            if(file == NULL){
                printf("Couldn't read file");
                exit(-1);
            }

            int prev = ftell(file);
            fseek(file, 0L, SEEK_END);
            long int fileSize = ftell(file) - prev;
            fseek(file, prev, SEEK_SET);
            printf("antes enviar start packet\n");

            if(sendControlPacket(filename,START_PCKT,fileSize) == -1){
                printf("Error sending control packet\n");
                exit(-1);
            }
            /*
            if(sendCtrlPacket(START_PCKT, filename, fileSize) == -1){
                printf("Error sending control packet\n");
                exit(-1);
            };
            */
            printf("depois de enviar start packet\n");

            unsigned char* buf = (unsigned char*)malloc(MAX_PAYLOAD_SIZE-3);

            int dataSize;
            while((dataSize = fread(buf, 1, MAX_PAYLOAD_SIZE-3, file)) > 0){
                
                if(sendDataPacket(dataSize,buf)){
                    exit(-1);
                };
            }
            fclose(file);

            printf("depois de enviar todos os data packet\n");
            /*
            if(sendCtrlPacket(END_PCKT,filename,fileSize) == -1){
                printf("Error sending control packet\n");
                exit(-1);
            }
            */      
           if(sendControlPacket(filename,END_PCKT,fileSize) == -1){
                printf("Error sending control packet\n");
                exit(-1);
            }
            printf("depois de enviar end packet\n");

            if(llclose(0) < 0) exit(-1);
            printf("depois do llclose\n");
            break;
            }
        default:
            break;
    }
    

}