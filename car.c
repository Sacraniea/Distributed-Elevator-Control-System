// Alexandros Sacranie

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include "shared.h"

#include <sys/mman.h>
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>

//                  Network Information                 //
// The TCP-IP Server for the car runs on Port 3000
// or 127.0.0.1
#define CTRL_PORT 3000
#define LOCALHOST "127.0.0.1"

// Prototypes for TCP threads
static void *tcp_receive_thread(void *arg);
static void *tcp_transmit_thread(void *arg);
static void *tcp_thread(void *arg);


//                  Macros                  //
#define CAR_LOCK(shm)   pthread_mutex_lock(&(shm)->mutex)
#define CAR_UNLOCK(shm) pthread_mutex_unlock(&(shm)->mutex)
#define CAR_NOTIFY(shm) pthread_cond_broadcast(&(shm)->cond)

//                  Global Variables                    //

// Shared Memory
char g_shm_name[32];
static car_shared_mem *g_shm_ptr = NULL;

// Inputs
static char g_car_name[32];
static unsigned g_delay_ms = 1000;
static char g_highest_floor[4];
static char g_lowest_floor[4];

// Conversions
static int  g_lowest_floor_int = 0;  
static int  g_highest_floor_int = 0;

// Initialise beteen and pending states to 0
static char pending_floor[4] = {0};
static int  has_pending = 0;

//                  Status Flags                    //
static pthread_mutex_t g_tx_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_tx_cv = PTHREAD_COND_INITIALIZER;

static int g_tx_flag = 0;
static volatile sig_atomic_t g_shutdown = 0;


static void flag_status(void)
{
    // Lock the mutex for the notification system
    pthread_mutex_lock(&g_tx_mx);
    // Set flag to indicate status change
    g_tx_flag = 1;
    // Wake cond to check the flag
    pthread_cond_signal(&g_tx_cv);
    // Unlock the mutex
    pthread_mutex_unlock(&g_tx_mx);
}

static void on_SIGINT(int sig)
{
    (void)sig;
    // Set shutdown flag
    g_shutdown = 1;
    // Lock the mutex for notifaction system
    pthread_mutex_lock(&g_tx_mx);
    // Wake up any thread wating on cond
    pthread_cond_broadcast(&g_tx_cv);
    // Unlock the mutex
    pthread_mutex_unlock(&g_tx_mx);

    // Check for shared memory
    if (g_shm_ptr)
    {
        // If shared memory is mapped lock the mutex
        CAR_LOCK(g_shm_ptr);
        // Notify all waiting threads
        pthread_cond_broadcast(&g_shm_ptr->cond);
        // Unlock the mutex
        CAR_UNLOCK(g_shm_ptr);
    }
}

//                  Delay Handlers                  //

static void sleep_ms(unsigned ms)
{
    // Pauses the thread for a specific delay (ms)
    struct timespec ts;
    // Convert whole ms to sec
    ts.tv_sec = ms / 1000u;
    // Convert remainder to ns
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    // Pause for the specified delay 
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR)
    {}
}

static struct timespec abs_timeout_ms(unsigned ms)
{
    struct timespec ts;
    // Get current time
    clock_gettime(CLOCK_REALTIME, &ts);
    // Perform addition of delay to realtime to let system
    // know absolute time to wake
    ts.tv_sec += ms / 1000u;
    // Handle conditions where nanosec exceeds a second
    ts.tv_nsec += (long)(ms % 1000u) * 1000000L;
    if (ts.tv_nsec >= 1000000000L)
    {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    // return absolute time
    return ts;
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

static void index_handler(int index, char out[4])
{
    // Checks if a floor is negative based on the index
    if (index < 0)
    {
        // If it is negative then the string should
        // be formatted with a leading B
        snprintf(out, 4, "B%d", -index);
    }
    else 
    {
        // Otherwise format as a standard number
        snprintf(out, 4, "%d", index);
    }
}

static int floor_validator(int f)
{
    // If the input floor is lower than the min floor
    // set to the min floor
    if (f < g_lowest_floor_int)
    {
        return g_lowest_floor_int;
    }
    // Otherwise if the input floor is higher than the max floor
    // set to the max floor
    else if (f > g_highest_floor_int) 
    {
        return g_highest_floor_int;
    }
    return f;
}

static int next_floor(int current_floor, int destination_floor)
{
    // If the destination floor is higher than the current floor
    // the car needs to move up
    if (current_floor < destination_floor)
    {
        int nf = current_floor + 1;
        if (nf == 0) nf = 1;
        return nf;
    }
    // If the destination floor is lower than the current floor
    // the car needs to move down
    if (current_floor > destination_floor)
    {
        int nf = current_floor - 1;
        if (nf == 0) nf = -1;
        return nf;
    }
    // Otherwise car is at the destination
    return current_floor;
}

static int at_destination(void)
{
    int s;
    // Lock the mutex
    CAR_LOCK(g_shm_ptr);
    // Compare current floor and destination floor and store status
    s = (strcmp(g_shm_ptr->current_floor, g_shm_ptr->destination_floor) == 0);
    // Unlock mutex
    CAR_UNLOCK(g_shm_ptr);
    // Return status
    return s;
}

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

//                  Status Handlers                  //

static int fetch_status(const char* status)
{
    int s;
    // Lock the mutex
    CAR_LOCK(g_shm_ptr);
    //Compare input and current status and store result
    s = (strcmp(g_shm_ptr->status, status) == 0);
    // Unlock the mutex
    CAR_UNLOCK(g_shm_ptr);
    // Return status
    return s;
}

static int is_service_mode(void)
{
    int s;
    // Lock the mutex
    CAR_LOCK(g_shm_ptr);
    // Store comparison value of service mode
    s = (g_shm_ptr->individual_service_mode != 0);
    // Unlock mutex
    CAR_UNLOCK(g_shm_ptr);
    // Return the status of service mode
    return s;
}

static int is_emergency_mode(void)
{
    int s;
    // Lock the mutex
    CAR_LOCK(g_shm_ptr);
    // Store comparison value of emergency mode
    s = (g_shm_ptr->emergency_mode != 0);
    // Unlock mutex
    CAR_UNLOCK(g_shm_ptr);
    // Return the status of emergency mode
    return s;
}

static void status_handler(const char* status_update, unsigned delay_ms, char output[8])
{
    // Lock the mutex to update shared status
    CAR_LOCK(g_shm_ptr);
    // Copy and update the new status
    strcpy(g_shm_ptr->status, status_update);
    // Notify the system of status change
    CAR_NOTIFY(g_shm_ptr);
    // Unlock the mutex before delay to ensure mutex is not held
    CAR_UNLOCK(g_shm_ptr);
    // Flag status change
    flag_status();

    // Delay for specified time
    sleep_ms(delay_ms);

    // Lock mutex to capture current status
    CAR_LOCK(g_shm_ptr);
    // Copy current status into output and add null terminator
    strncpy(output, g_shm_ptr->status, sizeof g_shm_ptr->status - 1);
    output[sizeof g_shm_ptr->status - 1] = '\0';
    // Unlock mutex
    CAR_UNLOCK(g_shm_ptr);
    // Flag status change
    flag_status();
}

static void open_status_handler(const char* status_update, unsigned delay_ms, char output[8])
{
    // Open status handler does the same as the status handler however aims to handle the logic
    // of the open button and close button during the open state

    // Lock mutex to set status modify shared state
    CAR_LOCK(g_shm_ptr);

    // Copy and update the new status
    strcpy(g_shm_ptr->status, status_update); 
    // Notify the system of status change 
    CAR_NOTIFY(g_shm_ptr);
    // Unlock the mutex to flag status change
    CAR_UNLOCK(g_shm_ptr);
    // Flag status change
    flag_status();
    // Lock the mutex before calculating absolute timeout
    CAR_LOCK(g_shm_ptr);

    // Create absolute timeout for open window
    struct timespec open_window = abs_timeout_ms(delay_ms);

    // Wait until either close button is pressed or timeout occurs
    while (g_shm_ptr->close_button == 0)
    {
        // If open button is pressed during open window
        if (g_shm_ptr->open_button == 1)
        {
            // Reset open button and extend open window by car delay
            g_shm_ptr->open_button = 0;
            open_window = abs_timeout_ms(delay_ms);
            continue;
        }
        // Check for timeout
        int err = pthread_cond_timedwait(&g_shm_ptr->cond, &g_shm_ptr->mutex, &open_window);
        if (err == ETIMEDOUT)
        {
            break;
        }
    }
    // Check for close button press
    if (g_shm_ptr->close_button == 1)
    {
        // If pressed reset the close button
        g_shm_ptr->close_button = 0;
    }

    // Transition to Closing state
    strcpy(g_shm_ptr->status, "Closing");

    // Notify the system of status change
    CAR_NOTIFY(g_shm_ptr);
    // Unlock the mutex to raise status flag and not hold mutex during delay
    CAR_UNLOCK(g_shm_ptr);
    flag_status();

    // Delay for Closing
    sleep_ms(delay_ms);

    // Lock mutex to check and update status
    CAR_LOCK(g_shm_ptr);
    // If still in Closing state after delay transition to Closed
    if (strcmp(g_shm_ptr->status, "Closing") == 0)
    {
        strcpy(g_shm_ptr->status, "Closed");
        CAR_NOTIFY(g_shm_ptr);
    }
    // Copy current status into output and add null terminator
    strncpy(output, g_shm_ptr->status, sizeof g_shm_ptr->status - 1);
    output[sizeof g_shm_ptr->status - 1] = '\0';
    // Unlock mutex and flag status change
    CAR_UNLOCK(g_shm_ptr);
    flag_status();
}

static void to_close(void)
{
    // Lock mutex to set status to closed
    CAR_LOCK(g_shm_ptr);
    // Copy and update status to Closed
    strcpy(g_shm_ptr->status, "Closed");
    // Notify the system of status change
    CAR_NOTIFY(g_shm_ptr);
    // Unlock mutex and flag status change
    CAR_UNLOCK(g_shm_ptr);
    flag_status();
}

static void to_open(unsigned delay_ms)
{
    
    char out[8];
    // Begin opening using status handler
    status_handler("Opening", delay_ms, out);

    // Check if still in opening state
    if (strcmp(out, "Opening") != 0)
    {
        return; 
    }

    // If the car status is opening progress logically to
    // open state using open status handler
    open_status_handler("Open", delay_ms, out);
}


static void exists_pending(void)
{
    // Lock mutex to check for pending floors
    CAR_LOCK(g_shm_ptr);
    // Check for pending floors
    if (has_pending)
    {
        // Copy pending floor to destination floor and add null terminator
        strncpy(g_shm_ptr->destination_floor, pending_floor, sizeof g_shm_ptr->destination_floor - 1);
        g_shm_ptr->destination_floor[sizeof g_shm_ptr->destination_floor - 1] = '\0';
        // Reset pending floor status
        has_pending = 0;
        pending_floor[0] = '\0';
        // Notify system of status change
        CAR_NOTIFY(g_shm_ptr);
    }
    // Unlock mutex and flag status change
    CAR_UNLOCK(g_shm_ptr);
    // Flag status change
    flag_status();
}

static void move_one_floor(unsigned delay_ms)
{
    char out[8];
    // Use status handler to transition to Between state
    status_handler("Between", delay_ms, out);

    // Lock mutex to update current floor
    CAR_LOCK(g_shm_ptr);
    // Check if still in Between state
    if (strcmp(g_shm_ptr->status, "Between") == 0)
    {
        // Convert current and destination floors to integers
        int current = 0, destination = 0;
        (void)floor_num_handler(g_shm_ptr->current_floor, &current);
        (void)floor_num_handler(g_shm_ptr->destination_floor, &destination);
        // Calculate next floor to move to
        int next = next_floor(current, destination);
        // Validate next floor to ensure within valid range
        next = floor_validator(next);
        // Convert next floor back to string and update current floor
        index_handler(next, g_shm_ptr->current_floor);
        // Update status to Closed and notify system of
        // changed floor
        strcpy(g_shm_ptr->status, "Closed");
        CAR_NOTIFY(g_shm_ptr);
    }
    // Unlock mutex and flag status change
    CAR_UNLOCK(g_shm_ptr);
    flag_status();
}

static void service_between(unsigned delay_ms)
{
    // Check if in service mode and currently closed
    if (!is_service_mode() || !fetch_status("Closed"))
    {
        // If the car is not in service mode it should return 
        // if the car is in service mode but not closed it should
        // finish its next logical operation before returning
        return;
    }
    // Lock mutex to read current and destination floors
    CAR_LOCK(g_shm_ptr);
    int current = 0, destination = 0;
    (void)floor_num_handler(g_shm_ptr->current_floor, &current);
    (void)floor_num_handler(g_shm_ptr->destination_floor, &destination);
    // Unlock mutex
    CAR_UNLOCK(g_shm_ptr);

    // Technichian is only able to move elevaotr manually up or down one floor
    if (!(destination == current + 1 || destination == current - 1))
    {
        // If the destination floor exceeds one floor limit in safety mode
        // set the destination floor to the current floor and await another input
        CAR_LOCK(g_shm_ptr);
        strncpy(g_shm_ptr->destination_floor, g_shm_ptr->current_floor, sizeof g_shm_ptr->destination_floor - 1);
        g_shm_ptr->destination_floor[sizeof g_shm_ptr->destination_floor - 1] = '\0';
        // Notify system of status change
        CAR_NOTIFY(g_shm_ptr);
        CAR_UNLOCK(g_shm_ptr);
        return;
    }
    // Move the elevator one floor in the input direction
    move_one_floor(delay_ms);
}

static int post_status(int fd)
{
    // Prepare buffers for status information
    char status[8], currentfloor[4], destinationfloor[4];
    // Lock the mutex to allow capture of system status
    CAR_LOCK(g_shm_ptr);
    // Capture status
    strncpy(status, g_shm_ptr->status, 7);
    status[7] = '\0';
    // Capture current floor
    strncpy(currentfloor, g_shm_ptr->current_floor, 3);
    currentfloor[3] = '\0';
    // Capture destination floor
    strncpy(destinationfloor, g_shm_ptr->destination_floor, 3);
    destinationfloor[3] = '\0';
    // Unlock mutex
    CAR_UNLOCK(g_shm_ptr);
    // Prepare message frame to send
    char tx_buf[64];
    // Format the status message
    snprintf(tx_buf, sizeof tx_buf, "STATUS %s %s %s", status, currentfloor, destinationfloor);
    // Send the status message
    return send_frame(fd, tx_buf);
}

//                  TCP and Thread Handlers                 //

// Structure to hold TCP thread arguments
typedef struct
{
    int socket_fd;
} tcp_args_t;

static void *tcp_receive_thread(void *arg)
{
    // Extract socket file descriptor from arguments
    tcp_args_t * args = (tcp_args_t*) arg;
    int s = args->socket_fd;
    // Deallocate argument structure
    free(args);
    // Begin reciever loop
    for (;;)
    {
        // Check for shutdown signal
        if (g_shutdown) 
        {
            return NULL;
        }
        // Prepare buffer for incoming message
        char rx_buf[64];
        
        // Attempt to receive message
        if (receive_frame(s, rx_buf, sizeof rx_buf) < 0) 
        {
            break;
        }
        // Check the message for a floor command
        if (!strncmp(rx_buf, "FLOOR ", 6))
        {
            // Extract the floor from the message
            char floor[4] = {0};
            (void)sscanf(rx_buf, "FLOOR %3s", floor);
            // Lock mutex to check status
            CAR_LOCK(g_shm_ptr);
            // Check if car is currently between floors
            int between = (strcmp(g_shm_ptr->status, "Between") == 0);
            // Unlock mutex
            CAR_UNLOCK(g_shm_ptr);

            // If the car is between floors store the floor as pending so the car doesnt
            // overwrite destination mid movement
            if (between)
            {
                CAR_LOCK(g_shm_ptr);
                strncpy(pending_floor, floor, sizeof pending_floor - 1);
                pending_floor[sizeof pending_floor - 1] = '\0';
                has_pending = 1;
                CAR_NOTIFY(g_shm_ptr);
                CAR_UNLOCK(g_shm_ptr);
            }
            // Otherwise set the destination floor directly
            else
            {
                CAR_LOCK(g_shm_ptr);
                strncpy(g_shm_ptr->destination_floor, floor, sizeof g_shm_ptr->destination_floor - 1);
                g_shm_ptr->destination_floor[sizeof g_shm_ptr->destination_floor - 1] = '\0';
                CAR_NOTIFY(g_shm_ptr);
                CAR_UNLOCK(g_shm_ptr);
                flag_status();
            }
        }
    }
    // Error or shutdown
    return NULL;
}

static void *tcp_transmit_thread(void *arg)
{
    // Extract socket file descriptor from arguments
    tcp_args_t *args = (tcp_args_t*) arg;
    int s = args->socket_fd;
    // Deallocate argument structure
    free(args);
    // Create initial transmit timeout
    struct timespec transmit_timeout = abs_timeout_ms(g_delay_ms);
    // Begin transmit loop
    for (;;)
    {
        // Check for shutdown signal
        if (g_shutdown)
        {
            break;
        }
        // Lock the mutex for condition and status flag
        pthread_mutex_lock(&g_tx_mx);
        // Wait for either status flag or timeout
        while (!g_tx_flag)
        {
            
            pthread_cond_timedwait(&g_tx_cv, &g_tx_mx, &transmit_timeout);
            if (g_shutdown)
            {
                break;
            }
        }
        // Capture and reset status flag
        int raised = g_tx_flag;
        g_tx_flag = 0;
        // Unlock the mutex
        pthread_mutex_unlock(&g_tx_mx);
        // Check for shutdown signal
        if (g_shutdown) 
        {
            break;
        }
        // If status flag was raised send status update
        if (raised)
        {
            // Send status update
            if (post_status(s) < 0)
            {
                break;
            }
            // Reset transmit timeout
            transmit_timeout = abs_timeout_ms(g_delay_ms);
        }
        // Check for safety system transmit timeout
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        // If timeout has occurred increment safety system counter
        if ((now.tv_sec > transmit_timeout.tv_sec) || (now.tv_sec == transmit_timeout.tv_sec && now.tv_nsec >= transmit_timeout.tv_nsec))
        {
            
            int val;
            CAR_LOCK(g_shm_ptr);
            val = (int)g_shm_ptr->safety_system + 1;
            g_shm_ptr->safety_system = (uint8_t)val;
            CAR_NOTIFY(g_shm_ptr);
            CAR_UNLOCK(g_shm_ptr);

            // If safety system has timed out 3 times enter emergency mode
            if (val >= 3)
            {
                // Notify system of emergency mode
                fprintf(stdout, "Safety system disconnected! Entering emergency mode.\n");
                CAR_LOCK(g_shm_ptr);
                g_shm_ptr->emergency_mode = 1;
                CAR_NOTIFY(g_shm_ptr);
                CAR_UNLOCK(g_shm_ptr);
                (void)send_frame(s, "EMERGENCY");
                break;
            }
            // Reset  the transmit timeout
            transmit_timeout = abs_timeout_ms(g_delay_ms);
        }
        // Lock the mutex to check for service or emergency mode
        CAR_LOCK(g_shm_ptr);
        int service_mode  = (g_shm_ptr->individual_service_mode != 0);
        int emergency_mode = (g_shm_ptr->emergency_mode != 0);
        CAR_UNLOCK(g_shm_ptr);
        // If either service or emergency mode is active send notification
        if (service_mode)
        {
            (void)send_frame(s, "INDIVIDUAL SERVICE");
            break;
        }
        if (emergency_mode)
        {
            (void)send_frame(s, "EMERGENCY");
            break;
        }

    }
    // Error or shutdown
    return NULL;
}

static void *tcp_thread(void *arg)
{
    (void)arg;

    // TCP connection loop
    for (;;)
    {
        // Check for shutdown signal
        if (g_shutdown) 
        {
            return NULL;
        }  
        int can_connect = 0;
        // Wait until safety system is active and not in service or emergency mode
        while (!g_shutdown)
        {
            // Lock mutex to check system status
            CAR_LOCK(g_shm_ptr);
            int service = (g_shm_ptr->individual_service_mode != 0);
            int emergency = (g_shm_ptr->emergency_mode != 0);
            // Unlock mutex
            CAR_UNLOCK(g_shm_ptr);

            // If the saftey system is active and not in service or emergency mode
            if (!service && !emergency)
            {
                // Can conncet
                can_connect = 1;
                break;
            }

            // Wait before rechecking
            sleep_ms(g_delay_ms);
        }

        // If cannor connect or shutdown signal try again
        if (!can_connect || g_shutdown)
        { 
            continue;
        }

        // Initialise the socket to IPv4
        int s = socket(AF_INET, SOCK_STREAM, 0);
        if (s == -1)
        {
            sleep_ms(g_delay_ms);
            continue;
        }
        // Initialise network address
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof addr);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(CTRL_PORT);
        inet_pton(AF_INET, LOCALHOST, &addr.sin_addr);

        // Attempt to connect to server
        if (connect(s, (struct sockaddr*)&addr, sizeof addr) == -1)
        {
            close(s);
            sleep_ms(g_delay_ms);
            continue;
        }
        // Send car id frame
        char car_id[64];
        snprintf(car_id, sizeof car_id, "CAR %s %s %s", g_car_name, g_lowest_floor, g_highest_floor);
        if (send_frame(s, car_id) < 0)
        {
            close(s);
            sleep_ms(g_delay_ms);
            continue;
        }
        // Post initial status
        if (post_status(s) < 0)
        {
            close(s);
            sleep_ms(g_delay_ms);
            continue;
        }
        // Create TCP receive and transmit threads
        pthread_t rx_tid, tx_tid;

        // Initialise receive thread
        tcp_args_t *rx_args = (tcp_args_t*)malloc(sizeof(*rx_args));
        rx_args->socket_fd = s;
        pthread_create(&rx_tid, NULL, tcp_receive_thread, rx_args);

        // Initialise transmit thread
        tcp_args_t *tx_args = (tcp_args_t*)malloc(sizeof(*tx_args));
        tx_args->socket_fd = s;
        pthread_create(&tx_tid, NULL, tcp_transmit_thread, tx_args);

        // Wait for disconnect
        pthread_join(rx_tid, NULL);
        pthread_join(tx_tid, NULL);
        shutdown(s, SHUT_RDWR);
        close(s);
    }
    return NULL;
}

//                  MAIN                    //

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        fprintf(stderr, "Usage: %s {name} {lowest_floor} {highest_floor} {delay}\n", argv[0]);
        return 1;
    }

    // Map inputs
    strncpy(g_car_name, argv[1], sizeof g_car_name - 1);
    g_car_name[sizeof g_car_name - 1] = '\0';

    strncpy(g_lowest_floor, argv[2], sizeof g_lowest_floor - 1);
    g_lowest_floor[sizeof g_lowest_floor - 1] = '\0';

    strncpy(g_highest_floor, argv[3], sizeof g_highest_floor - 1);
    g_highest_floor[sizeof g_highest_floor - 1] = '\0';

    g_delay_ms = (unsigned)strtoul(argv[4], NULL, 10);

    // Validate floor inputs
    if (!floor_num_handler(g_highest_floor, &g_highest_floor_int) ||
        !floor_num_handler(g_lowest_floor, &g_lowest_floor_int) ||
        g_highest_floor_int < g_lowest_floor_int)
    {
        fprintf(stderr, "Invalid floor range.\n");
        return 1;
    }
    // Signal handling 
    signal(SIGPIPE, SIG_IGN);
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sigemptyset(&sa.sa_mask);
    sa.sa_handler = on_SIGINT;
    sigaction(SIGINT, &sa, NULL);

    // Shared memory setup
    snprintf(g_shm_name, sizeof(g_shm_name), "/car%s", g_car_name);
    int fd = shm_open(g_shm_name, O_CREAT | O_RDWR, 0666);
    if (fd == -1)
    {
        perror("shm_open failed");
        return 1;
    }

    if (ftruncate(fd, sizeof(car_shared_mem)) == -1)
    {
        perror("ftruncate failed");
        close(fd);
        shm_unlink(g_shm_name);
        return 1;
    }
    // Map shared memory
    g_shm_ptr = mmap(NULL, sizeof(car_shared_mem), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (g_shm_ptr == MAP_FAILED)
    {
        perror("mmap failed");
        close(fd);
        shm_unlink(g_shm_name);
        return 1;
    }
    
    close(fd);

    // Initialise mutex and condition variable attributes
    pthread_mutexattr_t mutex_var;
    pthread_condattr_t  cond_var;

    pthread_mutexattr_init(&mutex_var);
    pthread_mutexattr_setpshared(&mutex_var, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&g_shm_ptr->mutex, &mutex_var);

    pthread_condattr_init(&cond_var);
    pthread_condattr_setpshared(&cond_var, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(&g_shm_ptr->cond, &cond_var);

    strcpy(g_shm_ptr->current_floor, g_lowest_floor);
    strcpy(g_shm_ptr->destination_floor, g_lowest_floor);
    strcpy(g_shm_ptr->status, "Closed");

    // Initialise car state variables
    g_shm_ptr->open_button = 0;
    g_shm_ptr->close_button = 0;
    g_shm_ptr->safety_system = 0;
    g_shm_ptr->door_obstruction = 0;
    g_shm_ptr->overload = 0;
    g_shm_ptr->emergency_stop = 0;
    g_shm_ptr->individual_service_mode = 0;
    g_shm_ptr->emergency_mode = 0;

    // Start TCP thread and detatch
    pthread_t tcp_tid;
    pthread_create(&tcp_tid, NULL, tcp_thread, NULL);
    pthread_detach(tcp_tid);

    // Main operation loop
    while (!g_shutdown)
    {
        // Lock mutex to check for button presses or mode changes
        CAR_LOCK(g_shm_ptr);

        // Wait until button press, mode change, or floor change
        while (!g_shutdown
            && g_shm_ptr->open_button == 0
            && g_shm_ptr->close_button == 0
            && g_shm_ptr->individual_service_mode == 0
            && g_shm_ptr->emergency_mode == 0
            && strcmp(g_shm_ptr->current_floor, g_shm_ptr->destination_floor) == 0)
        {
            struct timespec timeout = abs_timeout_ms(200);
            pthread_cond_timedwait(&g_shm_ptr->cond, &g_shm_ptr->mutex, &timeout);
        }

        // Unlock mutex to check what changed
        CAR_UNLOCK(g_shm_ptr);

        // Check for shutdown signal
        if (g_shutdown)
        {
            break;
        }

        // Service Mode
        if (is_service_mode())
        {
            // Conduct sercive operation
            service_between(g_delay_ms);

            // Handle button presses during service mode
            CAR_LOCK(g_shm_ptr);
            int open = g_shm_ptr->open_button;
            int close = g_shm_ptr->close_button;
            g_shm_ptr->open_button = 0;
            g_shm_ptr->close_button = 0;
            CAR_UNLOCK(g_shm_ptr);

            // If open button was pressed door is to conduct open operation and remain
            // open until manually closed
            if (open)
            {
                if (fetch_status("Closed") || fetch_status("Closing"))
                {
                    char out[8];
                    status_handler("Opening", g_delay_ms, out);
                    if (strcmp(out, "Opening") == 0)
                    {
                        CAR_LOCK(g_shm_ptr);
                        strcpy(g_shm_ptr->status, "Open");
                        CAR_NOTIFY(g_shm_ptr);
                        CAR_UNLOCK(g_shm_ptr);
                        flag_status();
                    }
                }
            }
            // If the close button was pressed door is to conduct close operation
            // and is to stay closed
            if (close)
            {
                if (fetch_status("Open"))
                {
                    char out[8];
                    status_handler("Closing", g_delay_ms, out);
                    to_close();
                }
            }
            // Lock mutex to wait before next check
            CAR_LOCK(g_shm_ptr);
            struct timespec ts = abs_timeout_ms(100);
            pthread_cond_timedwait(&g_shm_ptr->cond, &g_shm_ptr->mutex, &ts);
            CAR_UNLOCK(g_shm_ptr);
            continue;
        }

        // Emergency Mode
        if (is_emergency_mode())
        {
            // Lock mutex to check for button presses
            CAR_LOCK(g_shm_ptr);
            int open = g_shm_ptr->open_button;
            int close = g_shm_ptr->close_button;
            g_shm_ptr->open_button = 0;
            g_shm_ptr->close_button = 0;
            CAR_UNLOCK(g_shm_ptr);

            // If the door is open the door is to remain open until manually closed
            if (open)
            {
                if (fetch_status("Closed") || fetch_status("Closing"))
                {
                    char out[8];
                    status_handler("Opening", g_delay_ms, out);
                    if (strcmp(out, "Opening") == 0)
                    {
                        CAR_LOCK(g_shm_ptr);
                        strcpy(g_shm_ptr->status, "Open");
                        CAR_NOTIFY(g_shm_ptr);
                        CAR_UNLOCK(g_shm_ptr);
                        flag_status();
                    }
                }
            }
            // If the close button is pressed the door is to close and remain closed
            // until manually opened. car must not respond to open buttons while door is closing
            if (close)
            {
                // If the door was caught open it should proceed with logical progressoin to closed
                if (fetch_status("Open"))
                {
                    char out[8];
                    status_handler("Closing", g_delay_ms, out);
                    if (strcmp(out, "Closing") == 0)
                    {
                        CAR_LOCK(g_shm_ptr);
                        strcpy(g_shm_ptr->status, "Closed");
                        CAR_NOTIFY(g_shm_ptr);
                        CAR_UNLOCK(g_shm_ptr);
                        flag_status();
                    }
                }
                // If the door was caught closing it should proceed with logical progression to closed
                else if (fetch_status("Closing"))
                {
                    CAR_LOCK(g_shm_ptr);
                    strcpy(g_shm_ptr->status, "Closed");
                    CAR_NOTIFY(g_shm_ptr);
                    CAR_UNLOCK(g_shm_ptr);
                    flag_status();
                }
            }
            // Lock the mutex
            CAR_LOCK(g_shm_ptr);
            // Wait before next check
            struct timespec ts = abs_timeout_ms(100);
            pthread_cond_timedwait(&g_shm_ptr->cond, &g_shm_ptr->mutex, &ts);
            CAR_UNLOCK(g_shm_ptr);
            continue;
        }

        // Normal Operation
        if (at_destination())
        {
            // If the car is at the destination floor start open sequence
            to_open(g_delay_ms);
            // Check for pending and proceed
            exists_pending();
        }
        else
        {
            // If the car is not at the destination floor make sure the door is closed
            // before moving between
            if (fetch_status("Closed"))
            {
                // If the doors are closed proceed with between sequence
                move_one_floor(g_delay_ms);

                // If after moving the car is at the destination floor
                if (at_destination())
                {
                    // begin open sequence
                    to_open(g_delay_ms);
                }
                // Check for pending and proceed
                exists_pending();
            }
            // If caught mid closing sequence
            else if (fetch_status("Closing"))
            {
                // Car is to complete closing before moving
                char out[8];
                status_handler("Closing", g_delay_ms, out);
                to_close();
            }
            // If caught mid opening sequence
            else if (fetch_status("Opening"))
            {
                // Car is to logically proceed to closed status
                // before moving to between
                to_open(g_delay_ms);
            }
        }

        // Lock the mutex to check for button presses during operation
        CAR_LOCK(g_shm_ptr);
        int open = g_shm_ptr->open_button;
        int close = g_shm_ptr->close_button;
        g_shm_ptr->open_button = 0;
        g_shm_ptr->close_button = 0;
        CAR_UNLOCK(g_shm_ptr);

        // If open button was pressed car is to conduct opening sequence
        if (open)
        {
            // Car can only proceed to opening if it is closing or closed
            if (fetch_status("Closed") || fetch_status("Closing"))
            {
                to_open(g_delay_ms);
            }
        }
        // If the closed button was pressed car is to conduct closing sequence
        if (close)
        {
            // Car can only proceed to closed if it is open
            if (fetch_status("Open"))
            {
                char out2[8];
                status_handler("Closing", g_delay_ms, out2);
                to_close();
            }
        }
        // Lock the mutex
        CAR_LOCK(g_shm_ptr);
        // Wait before next check
        struct timespec ts = abs_timeout_ms(50);
        pthread_cond_timedwait(&g_shm_ptr->cond, &g_shm_ptr->mutex, &ts);
        CAR_UNLOCK(g_shm_ptr);
    }
    // Handle clean up and shutdown
    if (g_shm_ptr != NULL)
    {
        // Destroy mutex and condition variable
        pthread_mutex_destroy(&g_shm_ptr->mutex);
        pthread_cond_destroy(&g_shm_ptr->cond);
        // Unmap shared memory
        munmap(g_shm_ptr, sizeof(car_shared_mem));
    }
    // Unlink shared memory
    shm_unlink(g_shm_name);
    
    //Success
    return 0;
}
