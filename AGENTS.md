# AGENTS.md - SOEM Development Guidelines

## Project Overview

SOEM (Simple Open EtherCAT Master) is a C library for EtherCAT master communication. It supports multiple platforms (Linux, macOS, Windows, RTOS). The codebase uses CMake as its build system.

## Build Commands

### Initial Build (Linux/macOS)

```bash
mkdir build
cd build
cmake ..
make
```

### Initial Build (Windows with Visual Studio)

```bash
mkdir build
cd build
cmake .. -G "NMake Makefiles"
nmake
```

### Running Tests

The test executables are built as part of the normal build process. Tests require actual EtherCAT hardware to run meaningfully.

```bash
# Build tests (enabled by default when building from source root)
cd build
cmake ..
make

# Run specific test (example)
./test/linux/simple_test/simple_test eth0

# Other available tests:
./test/linux/slaveinfo/slaveinfo eth0
./test/linux/eepromtool/eepromtool eth0
./test/simple_ng/simple_ng eth0
```

### Clean Build

```bash
rm -rf build
mkdir build
cd build
cmake ..
make
```

### Single Test Rebuild

```bash
cd build
make simple_test  # Rebuild just simple_test
```

## Code Style Guidelines

### General Conventions

- **Language**: C (C99 standard)
- **License**: GPLv2 with exceptions (see LICENSE file)
- **Encoding**: UTF-8

### File Organization

- Headers (`.h`) go in the same directory as source or in appropriate OS-specific directories
- Source files (`.c`) grouped by module: `soem/`, `osal/`, `oshw/`
- Platform-specific code in subdirectories: `osal/linux/`, `oshw/linux/`, etc.

### Header Guards

```c
#ifndef _EC_ETHERCAT_H
#define _EC_ETHERCAT_H
/* ... */
#endif /* _EC_ETHERCAT_H */
```

### Include Order

1. Standard C headers (`<stdio.h>`, `<string.h>`, etc.)
2. OSAL headers (`"osal.h"`)
3. OSHW headers (`"oshw.h"`)
4. Local module headers (`"ethercat.h"`)
5. Other local headers

```c
#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"
#include "ethercat.h"
```

### Naming Conventions

- **Types**: `ec_slavet`, `ec_adaptert`, `ecx_contextt` (lowercase with `t` suffix)
- **Functions**: `ec_find_adapters()`, `ec_init()` (lowercase with underscores)
- **Macros/constants**: `EC_TIMEOUTMON`, `EC_STATE_OPERATIONAL` (uppercase with underscores)
- **Global variables**: `ec_slave[]`, `ec_slavecount` (lowercase with underscores)
- **Struct members**: `context->port`, `ec_slave[0].state` (lowercase)

### Data Types

The project defines its own portable types in `osal/osal.h`:

```c
typedef uint8_t             boolean;
typedef int8_t              int8;
typedef int16_t            int16;
typedef int32_t            int32;
typedef uint8_t            uint8;
typedef uint16_t           uint16;
typedef uint32_t           uint32;
typedef int64_t            int64;
typedef uint64_t           uint64;
typedef float              float32;
typedef double             float64;
```

Use these types instead of raw `int`/`unsigned` for EtherCAT-related data.

### Function Patterns

- **Context-based API**: New code should use `ecx_*` functions with explicit context (`ecx_contextt`)
- **Legacy API**: Old code uses global `ec_*` functions with global state (`ec_slave[]`, etc.)
- **Return values**: Functions typically return `int` (0=success, negative=error) or `boolean`
- **Pointer parameters**: Output parameters come last

### Error Handling

- Return `0` or `TRUE` for success, `-1` or `FALSE` for failure
- Use `EcatError` global flag for error state tracking
- Check return values from `ec_send_processdata()` and `ec_receive_processdata()`

### Packed Structures

Use `PACKED_BEGIN`/`PACKED_END` macros for structures that must be byte-aligned:

```c
PACKED_BEGIN
typedef struct PACKED
{
    uint16    comm;
    uint16    addr;
    uint16    d2;
} ec_eepromt;
PACKED_END
```

### Comments

- Doxygen-style for public APIs:
  ```c
  /** Create list over available network adapters.
   *
   * @return First element in list over available network adapters.
   */
  ```
- Brief descriptions for internal functions

### Code Formatting

- 4-space indentation (no tabs)
- Opening brace on same line for functions, new line for blocks
- Space after keywords (`if (`, `while (`)
- No space between function name and parentheses

### Testing Guidelines

- Tests are located in `test/linux/`, `test/simple_ng/`, etc.
- Each test is a standalone executable
- Tests accept network interface name as argument (e.g., `eth0`)
- Use `ecx_contextt` for new test code (see `test/simple_ng/simple_ng.c`)

### Platform-Specific Code

- OS-specific code in `osal/<OS>/` and `oshw/<OS>/`
- Supported OS: `linux`, `macosx`, `win32`, `rtk`, `rtems`
- Use `#ifdef` guards for platform-specific includes

### Common Build Options

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release    # Release build (default)
cmake .. -DCMAKE_BUILD_TYPE=Debug      # Debug build
cmake .. -DBUILD_SHARED_LIBS=ON        # Build shared library
cmake .. -DCMAKE_INSTALL_PREFIX=/usr   # Custom install path
```

### Additional Resources

- Full documentation: https://openethercatsociety.github.io/doc/soem/
- GitHub: https://github.com/OpenEtherCATsociety/SOEM
