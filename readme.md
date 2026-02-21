# Distributed Elevator Control System

A multi-process elevator control simulation written in C. A central **controller** coordinates one or more elevator **cars** over TCP, while a separate **safety monitor** and **internal maintenance tool** interact with each car’s local state via POSIX shared memory and process-shared synchronization.

This project demonstrates systems programming fundamentals: IPC, synchronization across processes, TCP stream framing, and modular separation of safety monitoring.

## System Components

### Executables
- **`controller`** (TCP server, default port **3000**)
  Registers cars, receives floor/status updates, accepts ride requests, and dispatches cars.
- **`car`** (TCP client)
  Simulates a single elevator car and exposes its state via shared memory.
- **`call`** (TCP client CLI)
  Sends a pickup request to the controller.
- **`safety`** (local monitor)
  Attaches to a car’s shared memory and checks for invalid/unsafe state combinations.
- **`internal`** (local maintenance CLI)
  Attaches to a car’s shared memory to toggle service/emergency-related operations.

---

## Architecture

### High-level Data Flow

```text
       [Network / TCP]                          [Local / IPC]
                                       +-----------------------------+
call (TCP Client) -------------------> |      POSIX Shared Memory    |
                                       |   + process-shared sync     |
                                       +-----------------------------+
          |                                   ^               ^
          v                                   |               |
controller (TCP Server) <----------- car (TCP Client)         |
                                              |               |
                                              v               v
                                       safety (Monitor)  internal (Tool)
```

### IPC & Synchronization (Local)

- **Zero-Copy State:** Uses `shm_open` + `mmap` to expose a `car_shared_mem` state segment.
- **Cross-Process Locking:** Uses `pthread_mutex_t` and `pthread_cond_t` configured with `PTHREAD_PROCESS_SHARED` to coordinate the car and safety monitor.

### TCP Framing (Network)

- **Stream Reassembly:** Uses a 16-bit length-prefixed frame header (network byte order) to handle TCP fragmentation.
- **Payloads:** ASCII command strings (e.g., `FLOOR 5`) carried inside binary-framed transport.

---

## Build & Run

### Prerequisites

- GCC
- Make
- Linux/Unix environment (or WSL/macOS)

### Compile

```bash
make all
```

### Clean

```bash
make clean
```

---

## Quick Start

Start components in this order to establish both the TCP link and shared memory attachments.

1. **Start the controller** (default port 3000)
   ```bash
   ./controller
   ```
2. **Start a car**
   - Syntax: `./car <name> <min_floor> <max_floor> <delay_ms>`
   ```bash
   ./car Car1 1 10 1000
   ```
3. **Start the safety monitor** (must match the car name)
   ```bash
   ./safety Car1
   ```
4. **Send a ride request**
   - Syntax: `./call <source_floor> <destination_floor>`
   ```bash
   ./call 1 5
   ```
5. **(Optional) Use internal maintenance operations**
   - Syntax: `./internal <car_name> <operation>`
   ```bash
   ./internal Car1 service_on
   ./internal Car1 stop
   ```

---

## 📡 Protocol Overview

All TCP messages use length-prefixed framing to ensure integrity:

```text
[  Length (2 bytes)  ] [      Payload (N bytes)      ]
   uint16_t (BE)          ASCII Command String
```

- **Header:** unsigned 16-bit payload length, big-endian.
- **Payload:** command string (examples: `FLOOR 5`, `STATUS Opening 1 5`).

---

## Validation Evidence (Original Submission)

The project was validated during the original assessment using an automated tester suite.
In the tester output format, lines beginning with `###` are expected, and the indented lines below are actual output.

### Controller multi-car registration (sample)

```text
### CAR Alpha
    CAR Alpha
### RECV: FLOOR 1
    RECV: FLOOR 1
### CAR Beta
    CAR Beta
### RECV: FLOOR 1
    RECV: FLOOR 1
### CAR Gamma
    CAR Gamma
### RECV: FLOOR 3
    RECV: FLOOR 3
### UNAVAILABLE
    UNAVAILABLE
### UNAVAILABLE
    UNAVAILABLE
```

---

## Known Issues (Original Submission)

These issues were observed in automated tests / marking feedback and are included here to document current behaviour:

- **Car state transitions:** Under some timing/race conditions, intermediate “Between” floor updates can be skipped (“teleporting”), and the door-open sequence may not always be observed as expected.
- **Control flag clearing:** A control input flag (e.g., “open” behaviour) is not cleared immediately in at least one scenario, causing a mismatch during the Opening state.
- **Safety reporting coverage:** Some data consistency errors are not reported by the safety monitor in all cases.
- **Missing required car messages:** The car does not emit `INDIVIDUAL SERVICE` / `EMERGENCY` messages in all required scenarios.

---

## Engineering Roadmap

This project is currently in a functional prototype state. The following technical debt and optimization tasks would be planned for future iteration.

### Core Logic & Correctness

- **State Machine Refactoring:** Refactor the movement loop to enforce stepwise transitions, preventing race conditions where intermediate floors are skipped under high load.
- **Deterministic Flag Clearing:** Ensure control input flags (e.g., `open_button`) are cleared atomically immediately after consumption to prevent state oscillation.
- **Message Parity:** Ensure `INDIVIDUAL SERVICE` and `EMERGENCY` broadcast frames are emitted reliably in all edge-case scenarios.

### Maintainability & Refactoring

- **Modularization:** Extract duplicated networking logic into a static library:
  - `tcp_util.{c,h}` (frame encoding/decoding, read/write-all)
  - `parse_util.{c,h}` (floor parsing, index handling)
- **Unit Testing:** Decompose monolithic handlers (e.g., `tcp_thread`) into testable units to allow for integration testing without full network stack simulation.

### Safety-Critical Standards

- **Error Handling:** Replace `strtol` with robust, deterministic parsing patterns.
- **Type Safety:** Normalize numeric literals with strict typing (e.g., `U` suffixes) to align with MISRA-C guidelines.

---

## License

MIT License.

