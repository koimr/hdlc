/*!
 * @file
 * @author Mark Koi - Scientia LLC
 * @date December 14, 2010
 *
 * This file is the main entry point for the hdlc framing test program.
 * It will run starting in a PASS condition, and exit on FAILURE.  It will
 * complete in a PASS condition if all iterations are completed without failure.
 *
 * @note Unclassified
 * @warning None
 */
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>
#include "hdlc.h"

#define DEFAULT_BUFF_SIZE 2048

// Local functions
void print_usage(char *argv[]);


/**
 * @brief Main routine
 *
 * This is the main portion of the file where most everything takes
 * place.  Start the hdlc framing test and run continuously until
 * a failure is found.
 *
 * @param[in] argc - the number(count) of arguments coming into the function
 * @param[in] argv - the arguments themselves; easy to refer to each argv
 *                     in array notation, e.g. argv[1], argv[2], etc...
 * @return -1 error
 *          0 successful
 *         >0 error code from assert
 *
 * @note None
 * @warning None
 */
int main (int argc, char *argv[])
{
    int count=0, debug=0, *block,opt,buff_size=DEFAULT_BUFF_SIZE,iterate=10000, i, num=1, out_size;
    unsigned char *buf, *out;
    char *data_decode=NULL, *data_encode=NULL;

    // Parse some arguments (if necessary)
    while( (opt = getopt(argc, argv, "hb:D:i:n:s:S:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                buff_size = strtol(optarg,NULL,10);
                break;
            case 'h':
                print_usage(argv);
                exit(0);
                break;
            case 'D':
                debug = strtol(optarg, NULL, 10);
                break;
            case 'i':
                iterate = strtol(optarg,NULL,10);
                break;
            case 'n':
                num = strtol(optarg,NULL,10);
                break;
            case 's':
                data_encode = optarg; // Access this later for generating an encoded buffer
                break;
            case 'S':
                data_decode = optarg; // This is the string to decode
                break;
            default:
                print_usage(argv);
                exit(1);
                break;
        }
    }
    // Output some information about runtime parameters
    printf("buff_size    = %d\n",buff_size);
    printf("debug        = %d\n",debug);
    printf("iterate      = %d\n",iterate);
    printf("num buffers  = %d\n",num);
    if (data_encode != NULL) printf("encode data  = \"%s\"\n",data_encode);
    if (data_decode != NULL) printf("decode data  = \"%s\"\n",data_decode);

    // THIS WILL CONVERT A RAW HEX STRING into an encoded HDLC frame
    if (data_encode != NULL)
    {
        char *y;
        unsigned char *buff=NULL;
        int sz=0;

        assert((hdlc_init(buff_size)) >= 0); // Must be successfull to continue, Create multiple hdlc buffers

        printf("Unencoded HDLC frame: ");
        while ((y = strsep(&data_encode," ")))
        {
            sz++;                           // Increase the size required
            buff = realloc(buff,sz);        // Allocate one byte at a time
            buff[sz-1] = strtol(y,NULL,16); // Copy the byte into unencoded buffer
            printf("%02X ",buff[sz-1]);
        }
        // Get an encoded message
        assert((out_size = hdlc_msg_encode(buff,sz,&out)) > 0);

        printf ("\nEncoded   HDLC frame: ");
        for (i=0; i < out_size; i++)
            printf("%02X ",*(out+i));
        printf("\n");

        exit(0); // Dont continue since this is a one time conversion
    }

    // THIS WILL CONVERT A ENCODED HDLC FRAME to a decoded frame
    if (data_decode != NULL)
    {
        char *y,*endptr;
        long tmp;

        assert((hdlc_init(buff_size)) >= 0); // Must be successfull to continue, Create multiple hdlc buffers

        printf("Encoded HDLC frame: %s\n",data_decode);
        while ((y = strsep(&data_decode," ")))
        {
            errno = 0;
            tmp = strtol(y,&endptr,16); // Copy the byte into encoded buffer
            if (errno == ERANGE || tmp < 0 || tmp > 255 || (errno != 0 && tmp == 0))
            {
              perror("strtol");
              exit(EXIT_FAILURE);
            }
            unsigned char byte = tmp;
            assert(hdlc_msg_add(&byte,1) > 0);
        }
        // Get an encoded message
        assert((out_size = hdlc_msg_decode(&out)) > 0);

        printf ("\nDecoded HDLC frame: ");
        for (i=0; i < out_size; i++)
            printf("%02X ",*(out+i));
        printf("\n");

        exit(0); // Dont continue since this is a one time conversion
    }


    // Allocate number of memory blocks
    assert((block = malloc(num * sizeof(int))) != NULL);

    // Allocate message buffer size
    assert((buf = malloc(buff_size)) != NULL); // Failed to allocate

    for (i=0; i < num; i++)
    {
        assert((block[i]=hdlc_init(buff_size)) >= 0); // Must be successfull to continue, Create multiple hdlc buffers
        assert(hdlc_delete_num(block[i]) == 0); // Try to delete buffer for each block number, as test
    }
    
    assert(hdlc_delete_num(0)== -1);     // Try to delete all buffers

    for (i=0; i < num; i++)
        assert((block[i]=hdlc_init(buff_size)) >= 0); // Re-create buffer

    for (i=0; i < num; i++)
        printf("Allocated an HDLC buffer called block(%d)\n",block[i]);
    printf("  Creating random HDLC buffers from size 1-%d\n",buff_size);
    printf("  Encoding....  Decoding....\n");
    printf("  CURRENT STATUS:  PASSED\n");

    while (count < iterate || iterate == 0)
    {
        unsigned char *o;          // Preserve address of original encoded buffer


        if (debug > 2) printf("--\nRandom buffer:      ");
        for (i=0; i < buff_size; i++) {
            buf[i] = (unsigned char) (256.0 * (rand() / (RAND_MAX + 1.0)));
            if (debug > 2) printf("%02X ",buf[i]);
        }
        if (debug > 2) printf("\n");

        // Get an encoded message
        assert((out_size = hdlc_msg_encode(buf,buff_size,&out)) > 0);
        o = out; // Keep original preserved

        if (debug > 2) {
            printf ("Encoded HDLC frame: ");
            for (i=0; i < out_size; i++)
                printf("%02X ",*(out+i));
            printf("\n");
        }

        // Divide message into two chunks and send in two packets
        if (out_size >= 2)
        {
            int part1,part2;
            part1 = out_size/2;       // Half of message
            part2 = out_size - part1; // Other half

            // Continue if size added to FIFO is same as incoming message
            for (i=0; i < num; i++)
                assert(hdlc_msg_add_num(block[i],o,part1) == part1);

            // Try to find complete message.. Only a partial... original address of out is lost now...
            for (i=0; i < num; i++)
                assert(hdlc_msg_decode_num(block[i],&out) == 0);

             // Continue if size added to FIFO is same as incoming message
             for (i=0; i < num; i++)
                assert(hdlc_msg_add_num(block[i],(o+part1),part2) == part2);

             // Continue if message matches original put in
            for (i=0; i < num; i++)
                assert(hdlc_msg_decode_num(block[i],&out) == buff_size);
        }

        // Continue if size added to FIFO is same as incoming message
        for (i=0; i < num; i++)
            assert(hdlc_msg_add_num(block[i],o,out_size) == out_size);

        // Continue if message matches original put in
        for (i=0; i < num; i++)
            assert(hdlc_msg_decode_num(block[i],&out) == buff_size);

       if (debug > 2) {
            printf ("Decoded HDLC frame: ");
            for (i=0; i < buff_size; i++)
                printf("%02X ",*(out+i));
            printf("\n");
        }

        // Compare memory and keep going if matches
        assert(memcmp(buf,out,buff_size) == 0);

        ++count; // Increase until done
    }

    return 0; // Successfully terminated
}

/**
 * @brief print_usage
 *
 * Dump the argv usage for this hdlc test program
 *
 * @param[in] argv - the arguments themselves; easy to refer to each argv
 *                     in array notation, e.g. argv[1], argv[2], etc...
 *
 * @note None
 * @warning None
 */
void print_usage(char *argv[])
{
        printf("Usage:\n");
        printf("%s [-bhDin]\n", *argv);
        printf("%s -h\n", *argv);
        printf("   -b <buff_size>      max buffer size to test\n");
        printf("   -D <debug>          set <debug> value\n");
        printf("   -h                  help menu for options\n");
        printf("   -i <iterate>        number of iterations then exit with result.  0=Infinite, Default=10,000\n");
        printf("   -n <num>            number of hdlc io handlers to test.  Default = 1\n");
        printf("   -s <string: 01 fe>  string in quotes, and generate an ENCODED single HDLC frame with checksum in hex\n");
        printf("   -S <string: 7e ..>  string in quotes, and generate a  DECODED single packet if HDLC frame valid\n");
}
