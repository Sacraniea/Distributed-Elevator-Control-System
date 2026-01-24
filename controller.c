// Author: Alexandros Sacranie
// Module: Safety Critical Monitor (controller.c)
// Project: Distributed Elevator Control System

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
#include <stdbool.h>



//                  Network Information                 //
// The TCP-IP Server for the controller runs on Port 3000
// or 127.0.0.1
#define CTRL_PORT 3000
#define LOCALHOST "127.0.0.1"


//                  Macros                  //
#define REGISTRY_LOCK()   pthread_mutex_lock(&g_cars_mtx)
#define REGISTRY_UNLOCK() pthread_mutex_unlock(&g_cars_mtx)
#define MAX_CARS 16
#define MAX_QUEUE 32

//                  Global Variables and Structures                //
typedef struct
{
    int in_use;
    int shm_fd;
    int socket_fd;
    char name[32];
    int lowest_floor, highest_floor;
    int q[MAX_QUEUE];
    int queue_len;
    char status[16];
    char cur_floor[4];
    char dst_floor[4];
    car_shared_mem*  shm_ptr;
} CarID;

static CarID g_cars[MAX_CARS];
static pthread_mutex_t g_cars_mtx = PTHREAD_MUTEX_INITIALIZER;

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


//                  Queue Operations                    //

static bool in_queue(const CarID* car, int fnum)
{
    // Loop through queue length to find floor
    for (int i = 0; i < car->queue_len; ++i) 
    {
        if (car->q[i] == fnum) 
        {
            // if floor found return true
            return true;
        }
    }

    // Otherwise return false
    return false;
}

static void queue_floor(CarID* car, int fnum)
{
    // Check to ensure there is space in the queue
    if (car->queue_len < MAX_QUEUE)
    {
        // If there is space add floor to queue
        car->q[car->queue_len++] = fnum;
    }
}

static void dequeue_floor(CarID* car)
{
    // Check to ensure there is something to dequeue
    if (car->queue_len <= 0) 
    {
        // if not return early
        return;
    }
    // Shift all floors forward in the queue
    for (int i = 1; i < car->queue_len; ++i) 
    {
        car->q[i-1] = car->q[i];
    }
    // Decrease queue length
    car->queue_len--;
}

static void enqueue(CarID* car, int src_floor, int dst_floor)
{
    // Check to ensure car is valid and is able to add to queue
    if (!car || src_floor == dst_floor)
    {
        return;
    }

    // Check to make sure source floor is not already in queue
    if (!in_queue(car, src_floor)) 
    {
        // Add source floor to queue
        queue_floor(car, src_floor);
    }

    // Ensure destination floor is after source floor in queue
    int src_index = -1, dst_index = -1;
    for (int i = 0; i < car->queue_len; ++i)
    {
        // Find the indexes of source and destination floors
        if (car->q[i] == src_floor && src_index < 0) 
        {
            // Store source index
            src_index = i;
        }
        if (car->q[i] == dst_floor && dst_index < 0) 
        {
            // Store destination index
            dst_index = i;
        }
    }
    // Check if destination is before source
    if (dst_index >= 0 && dst_index < src_index)
    {
        // If destination is before source remove destination from queue
        for (int i = dst_index + 1; i < car->queue_len; ++i)
        {
            car->q[i-1] = car->q[i];
        }
        // Decrease queue length and reset destination index
        car->queue_len--;
        dst_index = -1;
    }
    // If destination is not in queue add to end of queue
    if (dst_index < 0)
    {
        queue_floor(car, dst_floor);
    }
}


//                  SHM Helper Functions                 //

static void shm_attach_car(CarID* car)
{
    // Check if the car is valid
    if (!car)
    {
        return;
    }

    // Create shared memory frame with the cars properties
    char shm_name[64];
    snprintf(shm_name, 64, "/car%s", car->name);
    int fd = shm_open(shm_name, O_RDWR, 0666);
    if (fd == -1) 
    {
        // Failed to open shared memory
        car->shm_fd = -1;
        car->shm_ptr = NULL;
        return;
    }

    // Map shared memory
    void* p = mmap(NULL, sizeof(car_shared_mem), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED)
    {
        // Failed to map shared memory
        close(fd);
        car->shm_fd = -1;
        car->shm_ptr = NULL;
        return;
    }

    // Store shared memory details in car structure
    car->shm_fd  = fd;
    car->shm_ptr = (car_shared_mem*)p;

    // Initialise car properties in shared memory
    pthread_mutex_lock(&car->shm_ptr->mutex);
    // Initialise car to closed
    strncpy(car->shm_ptr->status, "Closed", sizeof car->shm_ptr->status - 1);
    // Initialise current and destination floors
    strncpy(car->shm_ptr->current_floor, car->cur_floor, sizeof car->shm_ptr->current_floor - 1);
    strncpy(car->shm_ptr->destination_floor, car->dst_floor, sizeof car->shm_ptr->destination_floor - 1);
    // Notify any waiting processes of changes
    pthread_cond_broadcast(&car->shm_ptr->cond);
    // Unlock the mutex
    pthread_mutex_unlock(&car->shm_ptr->mutex);
}

static void shm_detach_car(CarID* car)
{
    // Check if the car is valid
    if (!car)
    {
        return;
    }
    // Check if shared memory is mapped
    if (car->shm_ptr && car->shm_ptr != MAP_FAILED) 
    {
        // Unmap shared memory
        munmap(car->shm_ptr, sizeof(car_shared_mem));
        // Reset pointer
        car->shm_ptr = NULL;
    }
    // Check if shared memory file descriptor is valid
    if (car->shm_fd >= 0)
    {
        // Close the file descriptor
        close(car->shm_fd);
        // Reset file descriptor
        car->shm_fd = -1;
    }
}

static void fetch_shm_status(CarID* car, const char* status, const char* cur, const char* dst) 
{
    // Check if the car and its shared memory are valid
    if (!car || !car->shm_ptr) 
    {
        return;
    }
    // Lock the mutex and copy status information
    pthread_mutex_lock(&car->shm_ptr->mutex);
    strncpy(car->shm_ptr->status, status, sizeof car->shm_ptr->status - 1);
    strncpy(car->shm_ptr->current_floor, cur, sizeof car->shm_ptr->current_floor - 1);
    strncpy(car->shm_ptr->destination_floor, dst, sizeof car->shm_ptr->destination_floor - 1);
    // Notify any waiting processes of changes
    pthread_cond_broadcast(&car->shm_ptr->cond);
    // Unlock the mutex
    pthread_mutex_unlock(&car->shm_ptr->mutex);
}


//                  Handlers                  //

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


static void update_status(int socket_fd, const char* status, const char* cur, const char* dst)
{
    REGISTRY_LOCK();
    // loop over cars to find matching socket fd
    for (int i = 0; i < MAX_CARS; ++i)
    {
        // if car found update its status
        if (g_cars[i].in_use && g_cars[i].socket_fd == socket_fd)
        {
            // Update car status 
            strncpy(g_cars[i].status, status, sizeof g_cars[i].status - 1);
            // Update current and destination floors
            strncpy(g_cars[i].cur_floor, cur, sizeof g_cars[i].cur_floor - 1);
            strncpy(g_cars[i].dst_floor, dst, sizeof g_cars[i].dst_floor - 1);
            // Update shared memory status
            fetch_shm_status(&g_cars[i], status, cur, dst);
            break;
        }
    }
    REGISTRY_UNLOCK();
}

//                  Registry Functions                  //

static CarID* find_registry(const char* name)
{
    // Loop through cars to find matching name
    for (int i = 0; i < MAX_CARS; ++i)
    {   
        // If car found return pointer to it
        if (g_cars[i].in_use && strcmp(g_cars[i].name, name) == 0)
        {
            return &g_cars[i];
        }
    }
    // Otherwise nothing found under that name
    return NULL;
}

//                  Car Managers                  //

static bool can_service(const CarID* car, int src_floor, int dst_floor)
{
    // Check to make sure car is valid and in use
    if (!car || !car->in_use) 
    {
        return false;
    }
    // Check to make sure source floor is within the cars service range
    if (src_floor < car->lowest_floor || src_floor > car->highest_floor) 
    {
        return false;
    }
    // Check to make sure destination floor is within the cars service range
    if (dst_floor < car->lowest_floor || dst_floor > car->highest_floor) 
    {
        return false;
    }

    // Car can service the requested trip
    return true;
}

static void remove_car(int socket_fd)
{
    REGISTRY_LOCK();
    // Loop through cars to find matching socket fd
    for (int i = 0; i < MAX_CARS; ++i)
    {
        // if found detach shm and remove car from registry
        if (g_cars[i].in_use && g_cars[i].socket_fd == socket_fd)
        {
            shm_detach_car(&g_cars[i]);
            g_cars[i].in_use = 0;
            g_cars[i].socket_fd = -1;
            g_cars[i].name[0] = '\0';
            g_cars[i].queue_len = 0;
            break;
        }
    }
    REGISTRY_UNLOCK();
}

static void send_car(CarID* car)
{
    // Check to ensure car is valid and has something in the queue
    if (!car || car->queue_len <= 0)
    {
        return;
    }
    // Format and send the next floor in the queue to the car
    char front_str[16];
    index_handler(car->q[0], front_str);
    // Create the FLOOR frame
    char tx_buf[32];
    snprintf(tx_buf, sizeof tx_buf, "FLOOR %s", front_str);
    // Send the frame to the car
    (void)send_frame(car->socket_fd, tx_buf);
}


static bool car_selector(int src_floor, int dst_floor, char out_name[32])
{
    bool found = false;
    REGISTRY_LOCK();
    // Loop through cars to find one that can service the trip
    for (int i = 0; i < MAX_CARS; ++i)
    {
        // If a car is in use
        if (!g_cars[i].in_use)
        {
            // skip to next car
            continue;
        }
        // If car can service source and desitnation floors
        if (can_service(&g_cars[i], src_floor, dst_floor))
        {
            // Copy car name to output and mark as found
            strncpy(out_name, g_cars[i].name, 31);
            out_name[31] = '\0';
            found = true;
            break;
        }
    }
    REGISTRY_UNLOCK();
    // Return result
    return found;
}

static int car_connection_manager(int socket_fd, const char* name, const char* lowest, const char* highest)
{
    int lowest_floor, highest_floor;
    // Check to ensure lowest and highest floors are valid
    if (!floor_num_handler(lowest, &lowest_floor) || !floor_num_handler(highest, &highest_floor)) 
    {
        return -1;
    }
    // Check to make sure the floor inputs were formatted correctly
    if (lowest_floor > highest_floor)
    { 
        // Swap values if they are in the wrong order
        int swap = lowest_floor;
        lowest_floor = highest_floor;
        highest_floor = swap;
    }

    REGISTRY_LOCK();
    int free_idx = -1, index = -1;
    // Loop through cars to find existing car or free slot
    for (int i = 0; i < MAX_CARS; ++i)
    {
        // Check for existing car with same name
        if(g_cars[i].in_use && strcmp(g_cars[i].name, name) == 0)
        { 
            // If found return error
            index = i;
            break;
        }
        if(!g_cars[i].in_use && free_idx < 0) 
        {
            // Store first free index found
            free_idx = i;
        }
    }
    // If a valid index was found
    if (index < 0) 
    {
        // Use the free index
        index = free_idx;
    }
    // If no index valid index found
    if (index < 0) 
    { 
        // No free slots remaining
        REGISTRY_UNLOCK(); 
        return -2; 
    }

    // Initialise car properties in registry
    g_cars[index].in_use = 1;
    g_cars[index].socket_fd = socket_fd;
    // Copy car name
    strncpy(g_cars[index].name, name, sizeof g_cars[index].name - 1);
    g_cars[index].name[sizeof g_cars[index].name - 1] = '\0';
    // Store lowest and highest floors
    g_cars[index].lowest_floor = lowest_floor;
    g_cars[index].highest_floor = highest_floor;

    // Initialise car status and floors
    strncpy(g_cars[index].status, "Closed", sizeof g_cars[index].status - 1);
    strncpy(g_cars[index].cur_floor, lowest, sizeof g_cars[index].cur_floor - 1);
    strncpy(g_cars[index].dst_floor, lowest, sizeof g_cars[index].dst_floor - 1);
    g_cars[index].queue_len = 0;

    // Initialise shared memory details
    g_cars[index].shm_fd  = -1;
    g_cars[index].shm_ptr = NULL;

    REGISTRY_UNLOCK();

    // Attach shared memory to the car
    shm_attach_car(&g_cars[index]);
    return index;
}


static void car_scheduler_handler(CarID* car) 
{
    // Check to make sure car is valid
    if (!car)
    {
        return;
    }

    // Make sure the queue length is greater than 0
    if (car->queue_len > 0) 
    {
        // Capture the head of the queue
        char head_str[16];
        index_handler(car->q[0], head_str);
        // If the car is the desitnation floor and has a status of Opening dequeue it
        if (strcmp(car->status, "Opening") == 0 && strcmp(car->cur_floor, head_str) == 0)
        {
            // Floor has been serviced
            dequeue_floor(car);
        }
    }
    // If there are still floors in the queue
    if (car->queue_len > 0)
    {
        // Send car to the next floor in the queue
        send_car(car);
    }
}

//                  TCP and Thread Handlers                 //

// Structure to hold TCP thread arguments
typedef struct 
{
    int socket_fd; 
} tcp_args_t;


static void tcp_car_thread(int socket_fd, const char* name)
{

    char frame[256];
    // Receiver loop
    for (;;)
    {
        // Attempt to receive frame and handle errors
        if (receive_frame(socket_fd, frame, sizeof frame) < 0)
        {
            remove_car(socket_fd);
            close(socket_fd);
            break;
        }

        // Check the message for a STATUS command
        if (strncmp(frame, "STATUS ", 7) == 0)
        {
            // Extract status information from the message
            char status[16]={0}, curr_floor[4]={0}, dst_floor[4]={0};
            (void)sscanf(frame + 7, "%15s %3s %3s", status, curr_floor, dst_floor);

            // Update the car's status in the registry
            update_status(socket_fd, status, curr_floor, dst_floor);

            REGISTRY_LOCK();
            // Find the car in the registry
            CarID* car = find_registry(name);
            if (car)
            {
                // Handle the car scheduler
                car_scheduler_handler(car);
            }
            REGISTRY_UNLOCK();
        }
        // If service or emergency frames are detected
        else if (strcmp(frame, "INDIVIDUAL SERVICE") == 0 || strcmp(frame, "EMERGENCY") == 0)
        {
            // innore the frame
            continue;
        }
        // Otherwise unknown frame received
        else 
        {
            // Remove the car from the registry and close the socket
            remove_car(socket_fd);
            close(socket_fd);
            break;
        }
    }
}

static void tcp_call_thread(int socket_fd, const char* frame)
{
    // Extract source and destination floors from the CALL frame
    char src_floor[4]={0}, dst_floor[4]={0};
    (void)sscanf(frame, "CALL %3s %3s", src_floor, dst_floor);
    int src_floor_int, dst_floor_int;
    // Check the make sure the floor inputs are valid
    if (!floor_num_handler(src_floor, &src_floor_int)|| !floor_num_handler(dst_floor, &dst_floor_int)|| src_floor_int == dst_floor_int)
    {
        // If the floor inputs are invalid request cannot be serviced
        // shut down and close the socket
        (void)send_frame(socket_fd, "UNAVAILABLE");
        shutdown(socket_fd, SHUT_WR);
        close(socket_fd);
        return;
    }

    char car_name[32];
    // Check to see if a car can service the trip
    if (car_selector(src_floor_int, dst_floor_int, car_name))
    {
        
        char tx_buf[64];
        // If a car is found send the CAR frame
        snprintf(tx_buf, sizeof tx_buf, "CAR %s", car_name);
        (void)send_frame(socket_fd, tx_buf);

        REGISTRY_LOCK();
        // Find the car in the registry
        CarID* car = find_registry(car_name);
        // Check to make sure the car is valid
        if (car)
        {
            // If it is valid send the car to service the request
            enqueue(car, src_floor_int, dst_floor_int);
            send_car(car);
        }
        REGISTRY_UNLOCK();
    } 
    // Otherwise no car available to service the trip
    else
    {
        // Notify caller that no car is available
        (void)send_frame(socket_fd, "UNAVAILABLE");
    }
    // Shut down and close the socket
    shutdown(socket_fd, SHUT_WR);
    close(socket_fd);
}

static void *tcp_thread(void *arg)
{
    // Extract socket file descriptor from arguments
    tcp_args_t *args = (tcp_args_t*) arg;
    int socket_fd = args->socket_fd;
    // Deallocate argument structure
    free(args);

    char first_frame[256];
    // Attempt to receive the first frame
    if (receive_frame(socket_fd, first_frame, sizeof first_frame) < 0)
    {
        // On error close the socket and exit thread
        close(socket_fd);
        return NULL;
    }

    // Check if the frame contains a CAR registration
    if (strncmp(first_frame, "CAR ", 4) == 0)
    {
        // Extract car name and floor range from the CAR frame
        char name[32] = {0}, car_lowest_floor[4] = {0}, car_highest_floor[4] = {0};
        (void)sscanf(first_frame, "CAR %31s %3s %3s", name, car_lowest_floor, car_highest_floor);
        // Manage the car connection and registration
        int index = car_connection_manager(socket_fd, name, car_lowest_floor, car_highest_floor);
        if (index < 0)
        {
            // on error close the socket and exit thread
            close(socket_fd);
            return NULL;
        }
        // Start the car TCP handler thread
        tcp_car_thread(socket_fd, name);
    }
    // Otherwise check if the frame contains a CALL request
    else if (strncmp(first_frame, "CALL ", 5) == 0)
    {
        // Invoke call thread
        tcp_call_thread(socket_fd, first_frame);
    }
    else
    // otherwise close the socket
    {
        close(socket_fd);
    }

    return NULL;
}


// ---------------- main ----------------

int main(int argc, char *argv[])
{
    (void)argc; 
    (void)argv;

    // Initialise the socket to IPv4
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == -1)
    {
        perror("Socket Error"); 
        return 1;
    }

    // Signal handling
    signal(SIGPIPE, SIG_IGN);

    
    int enable_reuse = 1;
    // Allow port reuse directly after termination
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &enable_reuse, sizeof(enable_reuse));

    // Initialise network address
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_port= htons(CTRL_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    // Attempt connect
    if (bind(s, (struct sockaddr*)&addr, sizeof addr) == -1) 
    {
        perror("Bind error");
        close(s);
        return 1;
    }
    if (listen(s, 16) == -1) {
        perror("Listening Erorr");
        close(s);
        return 1;
    }  signal(SIGPIPE, SIG_IGN);

    // Connection loop
    for (;;)
    {
        // Initialise the client client connection
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        // Wait for request and attempt accept
        int client_socket = accept(s, (struct sockaddr*)&client_addr, &client_len);
        if (client_socket == -1)
        {
            perror("Accepting Error");
            break;
        }

        // Allocate memory to pass arguement to thread
        tcp_args_t* args = (tcp_args_t*)malloc(sizeof *args);
        if(!args)
        {
            close(client_socket);
            continue;
        }

        // Store client file descriptor in memory
        args->socket_fd = client_socket;

        // Create thread for the client
        pthread_t th;
        if (pthread_create(&th, NULL, tcp_thread, args) != 0) 
        {
            perror("Pthread_create Error");
            close(client_socket);
            free(args);
            continue;
        }
        // Detach the thread
        pthread_detach(th);
    }
    // Gracefully close the socket
    close(s);
    //success
    return 0;
}
