# Author: Alexandros Sacranie
# Module: Safety Critical Monitor (Makefile)
# Project: Distributed Elevator Control System

# Compiler and Flags
	
CC = gcc
CFLAGS = -Wall -Wextra -g -pthread

# 1. Typing 'make' builds all components

all: car controller call internal safety

# 2. Typing 'make car' builds the elevator car component

car: car.c
	$(CC) $(CFLAGS) -o car car.c

# 3. Typing 'make controller' builds the control system component

controller: controller.c
	$(CC) $(CFLAGS) -o controller controller.c

# 4. Typing 'make call' builds the call pad component

call: call.c
	$(CC) $(CFLAGS) -o call call.c

# 5. Typing 'make internal' builds the internal controls component

internal: internal.c
	$(CC) $(CFLAGS) -o internal internal.c

# 6. Typing 'make safety' builds the safety critical component

safety: safety.c
	$(CC) $(CFLAGS) -o safety safety.c

# Clean directory of all compiled executables and object files
	
clean: 
	rm -f car controller call internal safety