// Author: Alexandros Sacranie
// Module: Safety Critical Monitor (internal.c)
// Project: Distributed Elevator Control System

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include "shared.h"

//                  Floor Handlers                  //
// floor_num_handler and index_handler reused from controller.c

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

int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        fprintf(stderr, "Usage: %s {car_name} {operation}\n", argv[0]);
        return 1;
    }

    // Map arguements
    char *car_name = argv[1];
    char *operation = argv[2];

    char shm_name[32];
    snprintf(shm_name, sizeof(shm_name), "/car%s", car_name);

    int fd = shm_open(shm_name, O_RDWR, 0666);
    if(fd == -1)
    {
        fprintf(stderr, "Unable to access car %s.\n", car_name);
        return 1;
    }

    car_shared_mem *shm_ptr = mmap(NULL, sizeof(car_shared_mem), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if(shm_ptr == MAP_FAILED)
    {
        perror("mmap failed");
        close(fd);
        return 1;
    }


    pthread_mutex_lock(&shm_ptr->mutex);

    if(strcmp(operation, "open") == 0)
    {
        shm_ptr->open_button = 1;
    }
    else if(strcmp(operation, "close") == 0)
    {
        shm_ptr->close_button = 1;
    }
    else if(strcmp(operation, "stop") == 0)
    {
        shm_ptr->emergency_stop = 1;
    }
    else if(strcmp(operation, "service_on") == 0)
    {
        shm_ptr->individual_service_mode = 1;
        shm_ptr->emergency_mode = 0;
    }
    else if(strcmp(operation, "service_off") == 0)
    {
        shm_ptr->individual_service_mode = 0;
    }
    else if(strcmp(operation, "up") == 0 || strcmp(operation, "down") == 0)
    {
        if(shm_ptr->individual_service_mode == 0)
        {
            pthread_mutex_unlock(&shm_ptr->mutex);
            fprintf(stderr, "Operation only allowed in service mode.\n");
            munmap(shm_ptr, sizeof(car_shared_mem));
            close(fd);
            return 1;
        }
        if(strcmp(shm_ptr->status, "Between") == 0)
        {
            pthread_mutex_unlock(&shm_ptr->mutex);
            fprintf(stderr, "Operation not allowed while elevator is moving.\n");
            munmap(shm_ptr, sizeof(car_shared_mem));
            close(fd);
            return 1;
        }
        if(strcmp(shm_ptr->status, "Closed") != 0)
        {
            pthread_mutex_unlock(&shm_ptr->mutex);
            fprintf(stderr, "Operation not allowed while doors are open.\n");
            munmap(shm_ptr, sizeof(car_shared_mem));
            close(fd);
            return 1;
        }


        int current_floor;
        if(floor_num_handler(shm_ptr->current_floor, &current_floor))
        {
            int next_floor = current_floor + ((strcmp(operation, "up") == 0) ? 1 : -1);
            if(next_floor == 0)
            {
                next_floor = ((strcmp(operation, "up") == 0)  ? 1:-1);
            }
            char dst_floor[8];
            index_handler(next_floor, dst_floor);
            strncpy(shm_ptr->destination_floor, dst_floor, sizeof shm_ptr->destination_floor -1);
            shm_ptr->destination_floor[sizeof shm_ptr->destination_floor - 1] = '\0';
        }

    }
    else
    {
        pthread_mutex_unlock(&shm_ptr->mutex);
        fprintf(stderr, "Invalid operation.\n");
        munmap(shm_ptr, sizeof(car_shared_mem));
        close(fd);
        return 1;
    }

  
     // Notify any waiting processes of changes
    pthread_cond_broadcast(&shm_ptr->cond);
    // Unlock mutex
    pthread_mutex_unlock(&shm_ptr->mutex);
    // Unmap shared memory and close file descriptor
    munmap(shm_ptr, sizeof(car_shared_mem));
    close(fd);
    // Success
    return 0;
}