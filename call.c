// Author: Alexandros Sacranie
// Module: Safety Critical Monitor (call.c)
// Project: Distributed Elevator Control System

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include "shared.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//                  Network Information                 //
// The TCP-IP Server for the car runs on Port 3000
// or 127.0.0.1
#define CTRL_PORT 3000
#define LOCALHOST "127.0.0.1"

//                  TCP Helpers                 //
static ssize_t write_all(int fd, const void* buf, size_t n)
{
    // Point to the current position in the buffer
    // and track bytes left to write
    const unsigned char* p = (const unsigned char*)buf;
    size_t left = n;
    // Write until no bytes left to write
    while (left > 0)
    {
        // Track bytes written
        // and catch errors
        ssize_t Written = write(fd, p, left);
        if (Written < 0)
        {
            if (errno == EINTR) continue;
            return -1;
        }
        else if (Written == 0)
        {
            return -1;
        }
        // Shift the buffer pointer and decrease count
        p += Written;
        left -= (size_t)Written;
    }
    // Return total bytes written
    return (ssize_t)n;
}

static ssize_t read_all(int fd, void* buf, size_t n)
{
    // Point to the current position in the buffer
    // and track bytes left to read
    unsigned char* p = (unsigned char*)buf;
    size_t left = n;
    // Read until no bytes left to read
    while (left > 0)
    {
        // Track bytes read and catch errors
        ssize_t Read = read(fd, p, left);
        if (Read < 0)
        {
            if (errno == EINTR) continue;
            return -1;
        }
        else if (Read == 0)
        {
            return -1;
        }
        // Shift the buffer pointer and decrease count
        p += Read;
        left -= (size_t)Read;
    }
    // Return total bytes read
    return (ssize_t)n;
}

static int send_frame(int fd, const char* s)
{
    // Store length of message sent
    size_t len = strlen(s);
    // Clamp the message to 16 bits
    if (len > 0xFFFF)
    {
        len = 0xFFFF;
    }
    // Convert to network order
    uint16_t nlen = htons((uint16_t)len);
    // Send the length of the message first
    if (write_all(fd, &nlen, sizeof nlen) < 0) 
    {
        return -1;
    }
    // Send the message second
    if (write_all(fd, s, len) < 0)
    {
        return -1;
    }
    // Message sent successfully
    return 0;
}

static int receive_frame(int fd, char* buf, size_t capacity)
{
    // Create a variable for incoming message length
    uint16_t hlen;

    // Attempt read of message length
    if (read_all(fd, &hlen, sizeof hlen) < 0)
    {
        return -1;
    }

    // Revert length from network order
    size_t len = ntohs(hlen);

    // Check to make sure the incoming message is
    // smaller than the buffer
    if (len >= capacity)
    {
        // Read what can fit and make room for null terminator
        size_t keep = capacity - 1;
        if (read_all(fd, buf, keep) < 0)
        {
            return -1;
        }

        // Store the remainder in a temporary buffer
        // to read and discard
        size_t remainder = len - keep;
        char dump[512];

        while (remainder > 0)
        {
            // Attempt read of the remainder
            size_t chunk = remainder > sizeof dump ? sizeof dump : remainder;
            if (read_all(fd, dump, chunk) < 0) 
            {
                return -1;
            }
            remainder -= chunk;
        }

        buf[capacity - 1] = '\0';
    }
    // Othrwise read normally
    else
    {
        if (read_all(fd, buf, len) < 0)
        {
            return -1;
        }
        buf[len] = '\0';
    }
    // Message received successfully
    return 0;
}


//                  Floor Handlers                  //
static int floor_num_handler(const char *f, int *out)
{
    // Check to ensure that f is valid
    if (!f || !*f)
    {
        return 0;
    }

    // Handle negative conversions for basement floors (b or B)
    if (f[0] == 'b' || f[0] == 'B')
    {
        // Convert the string to long integer ingoring leading B/b
        char *end = NULL;
        long i = strtol(f + 1, &end, 10);
        // Check to make sure floor has been parsed
        // and is within 1 to 99
        if (end == f + 1 || i < 1 || i > 99)
        {
            return 0;
        }
        // Valid floor is passed to pointer and returns successfully
        *out = -(int)i;
        return 1;
    }
    // Otherwise handle positive floor numbers
    else
    {
        // Convert the string to long integer
        char *end = NULL;
        long i = strtol(f, &end, 10);
        // Check to make sure floor has been parsed
        // and is within 1 to 999
        if (end == f || i < 1 || i > 999)
        {
            return 0;
        }
        // Valid floor is passed to pointer and returns successfully
        *out = (int)i;
        return 1;
    }
}

int main(int argc, char* argv[]) 
{
    if (argc != 3) 
    {
        fprintf(stderr, "Usage: %s {source floor} {destination floor}\n", argv[0]);
        return 1;
    }

    // Map inputs
    const char* src_floor = argv[1];
    const char* dst_floor = argv[2];

    int src_floor_int = 0, dst_floor_int = 0;

    // Check to make sure the floor inputs are valid
    if (!floor_num_handler(src_floor, &src_floor_int)|| !floor_num_handler(dst_floor, &dst_floor_int))
    {
        printf("Invalid floor(s) specified.\n");
        return 0;
    }
    // Check if car is already at the destination floor
    if (src_floor_int == dst_floor_int)
    {
        printf("You are already on that floor!\n");
        return 0;
    }

    // Initialise the socket to IPv4
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == -1)
    {
        printf("Unable to connect to elevator system.\n");
        return 0;
    }

    // Initialise network address
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(CTRL_PORT);
    if (inet_pton(AF_INET, LOCALHOST, &addr.sin_addr) != 1)
    {
        close(s);
        printf("Unable to connect to elevator system.\n");
        return 0;
    }
    // Attempt to connect to server
    if (connect(s, (struct sockaddr*)&addr, sizeof addr) == -1)
    {
        close(s);
        printf("Unable to connect to elevator system.\n");
        return 0;
    }

    // Prepare the call message frame and attempt send
    char tx_buf[64];
    snprintf(tx_buf, sizeof tx_buf, "CALL %s %s", src_floor, dst_floor);
    if (send_frame(s, tx_buf) < 0)
    {
        close(s);
        printf("Unable to connect to elevator system.\n");
        return 0;
    }
    // Prepare to recieve response frame
    char rx_buf[64];
    if (receive_frame(s, rx_buf, sizeof rx_buf) < 0)
    {
        close(s);
        printf("Unable to connect to elevator system.\n");
        return 0;
    }
    // Check if response is a  CAR frame
    if (strncmp(rx_buf, "CAR ", 4) == 0)
    {
        char name[32] = {0};
        (void)sscanf(rx_buf + 4, "%31s", name);
        printf("Car %s is arriving.\n", name);
    }
    else
    {
        printf("Sorry, no car is available to take this request.\n");
    }

    // Gracefully shutdown sockets and close file descriptor
    shutdown(s, SHUT_RDWR);
    close(s);
    //success
    return 0;
}
