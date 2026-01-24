// Author: Alexandros Sacranie
// Module: Safety Critical Monitor (safety.c)
// Project: Distributed Elevator Control System

/* Safety Critical Analysis and Design Constraints

    This component serves as the safety-critical monitor for the Elevator Control System.
    It is designed to oversee the shared memory state, enforce safety invariants, and trigger
    emergency modes upon detecting inconsistencies or critical faults (e.g., obstructions, 
    overloads, or emergency stops).

    MISRA-C Compliance & POSIX Adaptations:
    While the design principles align with safety-critical standards (MISRA-C-2012), 
    specific deviations were necessary to support the POSIX simulation environment:

    1. Standard Library Usage:
       Strict safety-critical systems typically limit standard library inclusion. However,
       libraries such as <stdio.h>, <stdlib.h>, <pthread.h>, and <sys/mman.h> are utilized 
       here to facilitate IPC (Shared Memory), threading primitives, and basic I/O within 
       the Linux environment. Return values for critical system calls (mmap, shm_open) are 
       checked to ensure initialization integrity.

    2. Diagnostic I/O:
       Blocking I/O (printf) is used strictly for operator feedback and diagnostics. In a 
       bare-metal deployment, this would be replaced by a deterministic logging mechanism 
       or memory-mapped I/O to avoid non-deterministic blocking behavior.

    3. Error Handling Strategy:
       Initialization errors are strictly handled (fail-safe). Runtime synchronization 
       primitives (mutex/cond) assume the fundamental reliability of the underlying Linux 
       kernel scheduler for this simulation scope. A production-grade implementation would 
       include redundant checks on all synchronization primitives to handle kernel-level 
       faults.
*/

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

//                  Macros                  //
#define CAR_LOCK(shm)   pthread_mutex_lock(&(shm)->mutex)
#define CAR_UNLOCK(shm) pthread_mutex_unlock(&(shm)->mutex)
#define CAR_NOTIFY(shm) pthread_cond_broadcast(&(shm)->cond)

// Shared Memory
char g_shm_name[32];
static car_shared_mem *g_shm_ptr = NULL;

//                  Status Flags                    //
static volatile sig_atomic_t g_shutdown = 0;

static void on_SIGINT(int sig)
{
    (void)sig;
    // Set shutdown flag
    g_shutdown = 1;

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


//                  Main                    //
int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s {car name}\n", argv[0]);
        return 1;
    }
    const char* car_name = argv[1];

    // Signal handling 
    signal(SIGPIPE, SIG_IGN);
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sigemptyset(&sa.sa_mask);
    sa.sa_handler = on_SIGINT;
    sigaction(SIGINT, &sa, NULL);

    // Create shared memory frame
    char shm_name[32];
    snprintf(shm_name, sizeof(shm_name), "/car%s", car_name);
    int fd = shm_open(shm_name, O_RDWR, 0666);
    if(fd == -1)
    {
        fprintf(stderr, "Unable to access car %s.\n", car_name);
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

    while(!g_shutdown)
    {
        CAR_LOCK(g_shm_ptr);

        pthread_cond_wait(&g_shm_ptr->cond, &g_shm_ptr->mutex);

        // If the safety system is a value other than 1
        if(g_shm_ptr->safety_system != 1)
        {
            // set safety system to 1
            g_shm_ptr->safety_system = 1;
            // Notify the system of status change
            CAR_NOTIFY(g_shm_ptr);
        }

        // Check for door obstructions while closing
        if(strcmp(g_shm_ptr->status,"Closing") == 0 && g_shm_ptr->door_obstruction == 1)
        {
            // If obstruction was detected swtich status to opening
            strcpy(g_shm_ptr->status, "Opening");
            // Notify the system of status change
            CAR_NOTIFY(g_shm_ptr);
        }

        if(g_shm_ptr->emergency_stop == 1 && g_shm_ptr->emergency_mode == 0)
        {
            // Set emergency mode and clear the stop
            g_shm_ptr->emergency_mode = 1;
            g_shm_ptr->emergency_stop = 0;
            // Notify the system of status chane
            CAR_NOTIFY(g_shm_ptr);
            // Unlock the mutex and print message to the operator
            CAR_UNLOCK(g_shm_ptr);
            printf("The emergency stop button has been pressed!\n");
            // Pass to wait for next signal
            continue;
        }

        if(g_shm_ptr->overload == 1 && g_shm_ptr->emergency_mode == 0)
        {
            // Set emergency mode
            g_shm_ptr->emergency_mode = 1;
            // Notify the system of status change
            CAR_NOTIFY(g_shm_ptr);
            // Unlock the mutex and print message to the operator
            CAR_UNLOCK(g_shm_ptr);
            printf("The overload sensor has been tripped!\n");
            // Elevator is to wait for a following signal to proceed
            continue;
        }

        // Check to make sure that the status is one of the 5 validated statuses
        int is_valid_status = (
            strcmp(g_shm_ptr->status,"Closed") == 0 ||
            strcmp(g_shm_ptr->status,"Opening") == 0 || 
            strcmp(g_shm_ptr->status,"Open") == 0 || 
            strcmp(g_shm_ptr->status,"Closing") == 0 ||
            strcmp(g_shm_ptr->status,"Between") == 0
        );

        int valid_floor;
        // Parse floors and store results
        int valid_cur_floor = floor_num_handler(g_shm_ptr->current_floor, &valid_floor);
        int valid_dst_floor = floor_num_handler(g_shm_ptr->destination_floor, &valid_floor);

        // Store if the fields in the struct are valid
        int are_valid_fields = (
            ((g_shm_ptr->open_button == 0) || (g_shm_ptr->open_button == 1)) &&
            ((g_shm_ptr->close_button == 0) || (g_shm_ptr->close_button == 1)) &&
            ((g_shm_ptr->door_obstruction == 0) || (g_shm_ptr->door_obstruction == 1)) &&
            ((g_shm_ptr->overload == 0) || (g_shm_ptr->overload == 1)) &&
            ((g_shm_ptr->emergency_stop == 0) || (g_shm_ptr->emergency_stop == 1)) &&
            ((g_shm_ptr->individual_service_mode == 0) || (g_shm_ptr->individual_service_mode == 1)) &&
            ((g_shm_ptr->emergency_mode == 0) || (g_shm_ptr->emergency_mode == 1))
        );

        // Store if obstruction status is a valid condition
        int is_valid_obstruction =  g_shm_ptr->door_obstruction == 1 || (strcmp(g_shm_ptr->status,"Closing") == 0 || strcmp(g_shm_ptr->status,"Opening") == 0);

        // check if emergency mode is active with any invalid circumstances
        if(g_shm_ptr->emergency_mode != 1 && 
        (!valid_cur_floor == 1 || 
         !valid_dst_floor == 1 || 
         !is_valid_status == 1 || 
         !are_valid_fields == 1 ||
         !is_valid_obstruction == 1))
        {
            // Set emergency mode
            g_shm_ptr->emergency_mode = 1;
            // Notify the system of status change
            CAR_NOTIFY(g_shm_ptr);
            // Unlock the mutex and print message to the operator
            CAR_UNLOCK(g_shm_ptr);
            printf("Data consistency error!\n");
            // Elevator is to wait for a following signal to proceed
            continue;
        }
        // Unlock mutex and proceed with next check
        CAR_UNLOCK(g_shm_ptr);
    }

    // Unmap memory
    munmap(g_shm_ptr, sizeof *g_shm_ptr);
    //success
    return 0;

}
