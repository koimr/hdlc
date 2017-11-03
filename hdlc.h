/*!
 * @file
 * @author Mark Koi
 *
 * This is the header file for parsing multiple hdlc frames from multiple
 * interfaces either using a byte by byte read, or block read
 */
#ifndef HDLC_H
#define HDLC_H

#define HDLC_MAX        1027

// Globally defined functions
int hdlc_init(int size);

// API calls that only interact with a single comm channel/buffer (number = 1)
int hdlc_delete(void);
int hdlc_msg_add(unsigned char *in, int size);
int hdlc_msg_decode(unsigned char **out);
int hdlc_msg_encode(unsigned char *in, int len, unsigned char **out);

// API calls that allow multiple concurrent comm channels/buffers
int hdlc_delete_num(int number);
int hdlc_msg_add_num(int number, unsigned char *in, int size);
int hdlc_msg_decode_num(int number, unsigned char **out);
int hdlc_msg_encode_num(int number, unsigned char *in, int len, unsigned char **out);

#endif
