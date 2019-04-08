Each `TEST` is consisted of two binaries: `TEST-32` and `TEST-64`.
Test is run by invoking `./TEST-64`, which  executes internally `TEST-32` via `crossld_start`.
On success program prints "OK" and exits with code 0.

Available tests:
[ cleanup     ] validate if all parts of cleanup-32 are un-mapped after execution (requires exit() implementation)
[ exit        ] Test exit() implementation (return value)
[ long-ptr    ] Look for SIGABRT when pointer returned by 64-bit function isn't 32-bit
[ memsz       ] Test allocaton of segments MemSiz >> FileSiz
[ missing-sym ] Check proper handling of missing symbol(s)
[ mprotect    ] Try writing to read-only segment (and handle SIGSEGV)
[ mprotect2   ] Similar to `mprotect`, but try executing (and handle SIGSEGV)
[ multiargs   ] validate handling a large number of arguments with different types
[ multicall   ] Simple case: several 64<->32 switches in a loop
[ stacksize   ] Test if stack is big enough
