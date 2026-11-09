/**
  ******************************************************************************
  * @file           : syscalls.c
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <board.h>


// Newlib system calls for this bare-metal target. The application routes
// output through communication.cpp; stdin has no byte-stream backend, and
// the firmware has no filesystem or process model.
int _read(int file, void *data, size_t len) {
    (void)data;
    if (file == STDIN_FILENO) return 0; // EOF: no libc stdin stream is connected.
    if (len == 0) return 0;
    errno = EBADF;
    return -1;
}

int _close(int file) {
    (void)file;
    errno = EBADF; // No file descriptors can be opened by this firmware.
    return -1;
}

_off_t _lseek(int file, _off_t offset, int dir) {
    (void)offset;
    (void)dir;
    if (file >= STDIN_FILENO && file <= STDERR_FILENO) {
        errno = ESPIPE; // Standard streams are not seekable.
    } else {
        errno = EBADF;
    }
    return (_off_t)-1;
}

int _fstat(int file, struct stat *st) {
    if (file < STDIN_FILENO || file > STDERR_FILENO || !st) {
        errno = EBADF;
        return -1;
    }
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    if (file >= STDIN_FILENO && file <= STDERR_FILENO) return 1;
    errno = EBADF;
    return 0;
}

pid_t _getpid(void) {
    return 1; // Single firmware process.
}

int _kill(pid_t pid, int sig) {
    (void)sig;
    errno = (pid == 1) ? ENOSYS : ESRCH;
    return -1; // Signals are not supported by the firmware runtime.
}

extern char _end; // provided by the linker script: it's end of statically allocated section, which is where the heap starts.
extern char _heap_end_max; // provided by the linker script
void* _end_ptr = &_end;
void* _heap_end_max_ptr = &_heap_end_max;
void* heap_end_ptr = 0;

/* @brief Increments the program break (aka heap end)
*
* This is called internally by malloc once it runs out
* of heap space. Malloc might expect a contiguous heap,
* so we don't call the FreeRTOS pvPortMalloc here.
* If this function returns -1, malloc will return NULL.
* Note that if this function returns NULL, malloc does not
* consider this as an error and will return the pointer 0x8.
*
* You should still be careful with using malloc though,
* as it does not guarantee thread safety.
*
* @return A pointer to the newly allocated block on success
*         or -1 otherwise.
*/
intptr_t _sbrk(size_t size) {
    intptr_t ptr;
	{
        uint32_t mask = cpu_enter_critical();
        if (!heap_end_ptr)
            heap_end_ptr = _end_ptr;
        if (heap_end_ptr + size > _heap_end_max_ptr) {
            ptr = -1;
        } else {
            ptr = (intptr_t)heap_end_ptr;
            heap_end_ptr += size;
        }
        cpu_exit_critical(mask);
	}
    return ptr;
}

// _write is defined in communication.cpp


