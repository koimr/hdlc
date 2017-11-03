/*!
 * @file
 * @author Mark Koi
 *
 * This file for parsing multiple hdlc frames from multiple
 * interfaces either using a byte by byte read, or block read
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "hdlc.h"

//#define DEBUG
#define FLAG_SEQUENCE   0x7e    // Async HDLC flag
#define CONTROL_ESCAPE  0x7d    // Control Sequence flag
#define PPPINITFCS16    0xffff  // Initial FCS value
#define PPPGOODFCS16    0xf0b8  // Good final FCS value
#define ERROR           -1      // Return error code

static unsigned short fcstab[256] = {
      0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
      0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
      0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
      0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
      0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
      0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
      0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
      0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
      0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
      0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
      0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
      0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
      0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
      0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
      0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
      0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
      0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
      0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
      0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
      0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
      0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
      0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
      0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
      0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
      0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
      0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
      0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
      0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
      0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
      0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
      0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
      0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

enum HDLC_StateType
{
    STARTING,
    STARTED,
    ESCAPED,
};

enum HDLC_ErrorType
{
    ERR_NO_DATA,
    ERR_CRC,
    ERR_MAX_LEN
};

// The special //!< comment creates doxygen information related to defines
#define MAX_BLOCKS 5     // A simplistic way to become multi-thread safe for multiple simultaneous threads
// Misc


// Globally defined variables
struct hdlc_buffer
{
    int   block;                // A memory block for multiple instances
    int   size;                 // Maximum incoming message size
    int   pfd[2];               // Two file descriptors for FIFO (byte parser)

    unsigned char *bufferDecoded;   // Block of memory for single decoded message
    int            bufferDecodedLen;// Size of single decoded message

    int   dataLenCRC;           // The data length for CRC calculations
    unsigned char crc1;         // crc1 byte
    unsigned char crc2;         // crc2 byte
    unsigned short fcs;         // Frame Check Sequence
    enum HDLC_StateType state;  // State of the partial/complete buffer block

    unsigned char *bufferEncoded; // An allocated memory segment for encoding outbound messages
};

// Locally defined variables
static struct hdlc_buffer *hdlc[MAX_BLOCKS+1] = {NULL};  // Block of hdlc comm channels
static struct hdlc_buffer *ptr;                          // Pointer to one comm channel

// Locally defined functions (see below for function header information)
static int hdlc_delete_it(int block);
static int hdlc_check_bounds(int block);

/**
 * @brief HDLC init buffer
 *
 * Initialize a buffer for incoming/outgoing messages and associate the buffer
 * block by returning its value.
 *
 * @param[in] size - the incoming buffer size maximum
 *
 * @return -1 error,
 *          1-MAX_BLOCKS allocated message block for storing partial messages and used in decode
 *
 * @note None
 * @warning None
 */
int hdlc_init(int size)
{
    int val;

    for (val = 1; val <= MAX_BLOCKS; val++) {
        if (hdlc[val] == NULL) {
            break;
        }
    }

    if (val >= MAX_BLOCKS) { // Failure in finding a free block
        printf("hdlc: Failure allocating a free block. Maximum(%d) exceeded.",MAX_BLOCKS);
        return -1; // Failure
    }
    printf("hdlc: Allocating block(%d)\n",val);

    if (hdlc[val] == NULL)
    {
        int opt;
        hdlc[val] = malloc(sizeof(struct hdlc_buffer));
        ptr = hdlc[val]; // Make it easy to reference

        ptr->block = val;
        ptr->size  = size;
        ptr->bufferDecoded = malloc(ptr->size*2);
        ptr->bufferDecodedLen = 0;
        ptr->dataLenCRC = 0;
        ptr->crc1 = ptr->crc2 = 0;
        ptr->fcs = PPPINITFCS16;
        ptr->state = STARTING;
        ptr->bufferEncoded = malloc(ptr->size*2 + 6);

        if (pipe(ptr->pfd) == -1) { perror("pipe init"); return -1; } // Failed

        // Set to NON BLOCKING
        if ((opt = fcntl(ptr->pfd[0],F_GETFL,0)) < 0)
            perror("hdlc fcntl get failure:");
        opt |= O_NONBLOCK;
        if (fcntl(ptr->pfd[0],F_SETFL,opt) < 0)
            perror("hdlc fnctl set failure:");
    }
    else
    { // Failed right now
        printf("hdlc: Buffer already allocated\n");fflush(stdout);
        return -1;
    }
    return ptr->block; //  Right now only using one static block
}


/**
 * @brief HDLC delete a single buffer
 *
 * Delete the single allocated buffer
 *
 * @return -1 error
 *          0 successful for buffer(s)
 *
 * @note None
 * @warning None
 */
int hdlc_delete(void)
{
    return hdlc_delete_num(1);
}

/**
 * @brief HDLC delete buffer
 *
 * Delete a specified buffer or delete all buffers
 *
 * @param[in] number - number representing buffer to deallocate memory (0 = all buffers)
 *
 * @return -1 error
 *          0 successful for buffer(s)
 *
 * @note None
 * @warning None
 */
int hdlc_delete_num(int number)
{
    int i,failed = 0;

    if (number == 0)
    { // Go through all comm blocks
        printf("hdlc: Delete all blocks(1-%d)\n",MAX_BLOCKS);
        for (i=1;i<=MAX_BLOCKS;i++)
            if (hdlc_delete_it(i) < 0)
                failed = 1;
    }
    else
    { // Delete on comm channel buffer
        if (hdlc_delete_it(number) < 0)
            failed = 1;
    }
    
    if (failed) return -1; // Failed
    else return 0; // Successful
}

/**
 * @brief HDLC delete buffer block
 *
 * Delete a buffer block that has been allocated
 *
 * @param[in] block - number representing buffer block
 *
 * @return 0 pass
 *        -1 failure
 */
int hdlc_delete_it(int block)
{
    if (hdlc_check_bounds(block) < 0) return -1;

    printf("hdlc: Deleting block(%d)\n", block);
    ptr = hdlc[block]; // Point to hdlc channel of interest
    free(ptr->bufferDecoded);
    close(ptr->pfd[0]);
    close(ptr->pfd[1]);
    free(ptr);
    hdlc[block] = NULL;
    return 0; // Success
}

/**
 * @brief HDLC check bounds
 *
 * Check the message block for proper memory allocation and range
 *
 * @param[in] block - number representing buffer block
 *
 * @return 0 pass
 *        -1 failure
 */
int hdlc_check_bounds(int block)
{
    if (block > MAX_BLOCKS || block < 0 || hdlc[block] == NULL)
    {
        printf("hdlc: Not allocated or out of bounds for block (%d)\n",block);
        return -1; // Failure
    }
    else
        return 0;  // Pass
}

/**
 * @brief HDLC add incoming message content a single buffer
 *
 * Add incoming message content that could contain multiple messages or just a
 * partial message.  This is a storage container for the incoming data.
 *
 * @param[in] *in   - input buffer address
 * @param[in] size   - size of input buffer
 *
 * @note None
 * @warning None
 */
int hdlc_msg_add(unsigned char *in, int size)
{
    return hdlc_msg_add_num(1,in,size);
}

/**
 * @brief HDLC add incoming message content
 *
 * Add incoming message content that could contain multiple messages or just a
 * partial message.  This is a storage container for the incoming data.
 *
 * @param[in] number - number representing buffer to add data to
 * @param[in] *in   - input buffer address
 * @param[in] size   - size of input buffer
 *
 * @return 0 indicates no data was written to requested buffer (FIFO)
 *         1-size size of decoded data in out buffer
 *        -1 failure
 *
 * @note None
 * @warning None
 */
int hdlc_msg_add_num(int number, unsigned char *in, int size)
{
    // This is where the lookup would be for the block number
    if (hdlc_check_bounds(number) < 0) return -1;

    if (in == NULL || size <= 0)
    { // incoming size is invalid or no data
        return -1;
    }

    // Rewrite the data to an unnamed pipe to abstract from original fd
    if (write(hdlc[number]->pfd[1],in,size) != size) { perror("pipe write:"); return 0;}

    return size; // Identifies the amount of data copied correctly
}

#ifdef DEBUG
/**
 * @brief HDLC buffer dump
 *
 * Dump incoming and outgoing messages for debug of interface
 *
 * @param[in] *buffer - input buffer address
 * @param[in] size    - size of input buffer
 * @param[in] *dir    - direction of interface
 *
 * @note None
 * @warning None
 */
void dump_buffer(unsigned char *buffer, int size,char *dir)
{
    int i;
    printf("hdlc buffer size(%d) %3s:",size,dir);
    for (i=0; i < size; i++)
    {
        printf("%02X ",0xff & buffer[i]);
    }
    printf("\n");
}
#endif

/**
 * @brief HDLC search through buffer and decode a message(s) if one available
 *
 * Search through buffer for any message(s), if available.  The current state for a
 * partial message read will be saved and continued upon next call.
 *
 * @param[out] **out  - address of message pointer set to valid message, if found otherwise NULL
 *
 * @return 0 No complete message in buffer currently
 *         1-size size of decoded data in out buffer
 *         -1 failure
 *
 * @note None
 * @warning None
 */
int hdlc_msg_decode(unsigned char **out)
{
    return hdlc_msg_decode_num(1,out);
}

/**
 * @brief HDLC search through current buffer and decode a message(s) if one available
 *
 * Search through buffer for any message(s), if available.  The current state for a
 * partial message read will be saved and continued upon next call.
 *
 * @param[in] number - number representing buffer to look for additional messages
 * @param[out] **out  - address of message pointer set to valid message, if found otherwise NULL
 *
 * @return 0 No complete message in buffer currently
 *         1-size size of decoded data in out buffer
 *         -1 failure
 *
 * @note None
 * @warning None
 */
int hdlc_msg_decode_num(int number, unsigned char **out)
{
    unsigned char c;
    int num;
    static int cnt=0;

    if (hdlc_check_bounds(number) < 0) return -1;
    ptr = hdlc[number]; // Make it easy to reference
   
    while ((num=read(ptr->pfd[0],&c,1)) > 0)
    { // Got one character from the unnamed pipe
        // printf("byte: %03d num(%d) %02X \n", cnt, num,c);fflush(stdout);
        cnt++;
        switch (ptr->state)
        {
            case STARTING:
                if (c == FLAG_SEQUENCE)
                { // Started and got flag
                    ptr->crc1 = ptr->crc2 = ptr->bufferDecodedLen = ptr->dataLenCRC = 0; // Reset
                    ptr->fcs = PPPINITFCS16;
                    ptr->state = STARTED;
                    cnt=1;
                }
                break;
            case STARTED:
                if (c == FLAG_SEQUENCE)
                {
                    if (ptr->bufferDecodedLen == 0)
                    { // Got two FLAG SEQUENCES in a row
                        ptr->dataLenCRC = 0;
                        ptr->fcs = PPPINITFCS16;
                    }
                    else if (ptr->fcs != PPPGOODFCS16)
                    { // Failed to achieve fast frame check sequence (FCS)
                        printf("Failed FCS for %d bytes with FCS(%04X) instead of %04X\n", ptr->bufferDecodedLen, ptr->fcs, PPPGOODFCS16);
                        //printf("Original Buffer: "); dump_buffer(ptr->bufferDecoded, ptr->bufferDecodedLe, "FCS failed");
                        ptr->state = STARTING;
                    }
                    else // Good message with FCS
                    { // FCS == PPPGOODFCS16
#ifdef DEBUG
                        dump_buffer(ptr->bufferDecoded, ptr->bufferDecodedLen,"IN");
#endif
                        ptr->state = STARTING;
                        *out = ptr->bufferDecoded;
                        return ptr->bufferDecodedLen;
                    }
                }
                else if (c == CONTROL_ESCAPE)
                    ptr->state = ESCAPED;
                else // Everything else
                {
                    if (ptr->dataLenCRC >= 2)
                    {
                        if (ptr->bufferDecodedLen >= ptr->size * 2)
                        { // No end
                            printf("Failed finding end.  Resync.\n");
                            ptr->state = STARTING;
                        }
                        ptr->bufferDecoded[ptr->bufferDecodedLen++] = ptr->crc1;
                    }
                    ptr->crc1 = ptr->crc2;
                    ptr->crc2 = c;
                    ptr->dataLenCRC++;
                    ptr->fcs = (ptr->fcs >> 8) ^ fcstab[(ptr->fcs ^ c) & 0xff];
                }
                break;
            case ESCAPED:
                if (c == FLAG_SEQUENCE)
                {
                    ptr->bufferDecodedLen = ptr->dataLenCRC = 0;
                    ptr->fcs = PPPINITFCS16;
                    ptr->state = STARTED;
                }
                else
                {
                    c ^= 0x20;
                    if (ptr->dataLenCRC >= 2)
                    {
                        if (ptr->bufferDecodedLen >= ptr->size * 2)
                        {
                            printf("Overran buffer on ESCAPED.  Resync.\n");
                            ptr->state = STARTING;
                        }
                        ptr->bufferDecoded[ptr->bufferDecodedLen++] = ptr->crc1;
                    }
                    ptr->crc1 = ptr->crc2;
                    ptr->crc2 = c;
                    ptr->dataLenCRC++;
                    ptr->fcs = (ptr->fcs >> 8) ^ fcstab[(ptr->fcs ^ c) & 0xff];
                    ptr->state = STARTED;
                }
                break;
        }
    }
    *out = NULL;
    return 0;
}

/**
 * @brief HDLC encode buffer used for output
 *
 * Encode an HDLC like framing, See RFC 1662 for any additional information.
 *
 * @param[in] *in  - input message buffer address
 * @param[in] len  - size of incoming buffer
 * @param[out] **out - address of message pointer set to valid HDLC encoded message buffer
 *
 * @return -1 Failed to create a buffer
 *         x  size of data in out buffer
 *
 * @note None
 * @warning None
 */
int hdlc_msg_encode(unsigned char *in, int len, unsigned char **out)
{
    return hdlc_msg_encode_num(1,in,len,out);
}

/**
 * @brief HDLC encode buffer used for output
 *
 * Encode an HDLC like framing, See RFC 1662 for any additional information.
 *
 * @param[in] number - block number representing buffer
 * @param[in] *in  - input message buffer address
 * @param[in] len  - size of incoming buffer
 * @param[out] **out - address of message pointer set to valid HDLC encoded message buffer
 *
 * @return -1 Failed to create a buffer
 *         x  size of data in out buffer
 *
 * @note None
 * @warning None
 */
int hdlc_msg_encode_num(int number, unsigned char *in, int len, unsigned char **out)
{
    unsigned short fcs = PPPINITFCS16;
    unsigned char c;
    int i, txCnt = 0;

    if (hdlc_check_bounds(number) < 0 || in == NULL)
    {
        *out = NULL;
        return -1;
    }

    ptr = hdlc[number]; // Make it easy to reference
    ptr->bufferEncoded[txCnt++] = FLAG_SEQUENCE;
    for (i = 0; i < len; i++)
    { // Iterate through full length of buffer and encode any FLAG SEQUENCE or CONTROL ESCAPE
        c = *(in+i);
        fcs = (fcs >> 8) ^ fcstab[(fcs ^ c) & 0xff];
        if (c == FLAG_SEQUENCE || c == CONTROL_ESCAPE)
        {
            ptr->bufferEncoded[txCnt++] = CONTROL_ESCAPE;
            c ^= 0x20;
        }
        ptr->bufferEncoded[txCnt++] = c;
        // Buffer to big even considering worst case checksum
        if (txCnt > ptr->size*2 + 1) { *out = NULL; return 0; }
    }
    fcs ^= 0xffff;
    for (len = 0, c = fcs & 0xff; len < 2; len++, c = fcs >> 8)
    { // Go through the fcs and encode any FLAG SEQUENCE or CONTROL SEQUENCE
        if (c == FLAG_SEQUENCE || c == CONTROL_ESCAPE)
        {
            ptr->bufferEncoded[txCnt++] = CONTROL_ESCAPE;
            ptr->bufferEncoded[txCnt++] = c ^ 0x20;
        }
        else
            ptr->bufferEncoded[txCnt++] = c;
    }
    ptr->bufferEncoded[txCnt++] = FLAG_SEQUENCE;
    *out = ptr->bufferEncoded;

#ifdef DEBUG
    dump_buffer(*out,txCnt,"OUT");
#endif
    return txCnt;
}


