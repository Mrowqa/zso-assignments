#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/mman.h>
#include <assert.h>

#define PAYLOAD \
    "\x55\x48\x89\xe5\x48\x83\xec\x18\x48\x89\xfe\x48\xbf\x44\x44\x44" \
    "\x44\x44\x44\x44\x44\x48\xb9\x22\x22\x22\x22\x22\x22\x22\x22\xff" \
    "\xd1\x48\x83\xc4\x18\x5d\xc3"
#define PAYLOAD_SIZE 0x27
#define PRINTF_ADDR_OFFSET 0x17
#define FORMAT_ADDR_OFFSET 0x0d

void my_printf(void* fmt, long arg1) {
    printf(fmt, arg1);
}

typedef void (*formatter) (int);
formatter make_formatter (const char *format) {
    void *func = malloc(PAYLOAD_SIZE);
    memcpy(func, PAYLOAD, PAYLOAD_SIZE);

    // *((long*) func + offset) = ... doesnt write memory :(
    void *printf_addr = my_printf; // calling printf in assembly is tricky
    memcpy(func + PRINTF_ADDR_OFFSET, &printf_addr, 8);
    memcpy(func + FORMAT_ADDR_OFFSET, &format, 8);

    void *page_start = (void*) (((long)func) & (~0xfff));
    size_t len = func - page_start + PAYLOAD_SIZE;
    int ret = mprotect(page_start, len, PROT_READ | PROT_WRITE | PROT_EXEC);
    assert(ret == 0);

    return (formatter) func;
}

int main() {
    printf("a\n");
    formatter f = make_formatter("xd-%x-bx\n");
    f(42);
    printf("b\n");
    return 0;
}

