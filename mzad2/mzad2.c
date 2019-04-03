#define _GNU_SOURCE
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdio.h> // todo remove
#include <errno.h>
#include <linux/futex.h>
#include <stdatomic.h>

#define INIT_STACK_SIZE (1024*1024*4)
#define CHARS_AMOUNT 1000 // should be 1000

void main_thread();
int side_thread(void*);


long my_syscall6(long sys_num, long a, long b, long c, long d, long e, long f) {
    register long res __asm__("rax");
    register long sys_num_ __asm__("rax") = sys_num;
    register long a_ __asm__("rdi") = a;
    register long b_ __asm__("rsi") = b;
    register long c_ __asm__("rdx") = c;
    register long d_ __asm__("r10") = d;
    register long e_ __asm__("r8") = e;
    register long f_ __asm__("r9") = f;

    __asm__ volatile (
        "syscall"
        : "=g" (res)
        : "g" (a), "g" (b), "g" (c), "g" (d), "g" (e), "g" (f)
        : "cc", "memory", "rcx", "r11"
    );

    if (-4096 <= res && res <= -1) {
        errno = -res;
        return -1;
    }

    return res;
}

long my_syscall3(long sys_num, long a, long b, long c) {
    return my_syscall6(sys_num, a, b, c, 0, 0, 0);
}

long my_syscall1(long sys_num, long a) {
    return my_syscall6(sys_num, a, 0, 0, 0, 0, 0);
}

void print(const char *str, size_t count) {
    my_syscall3(SYS_write, STDOUT_FILENO, (long) str, count);
}

void *alloc_stack() {
    return (void*) my_syscall6(SYS_mmap, (long) NULL, INIT_STACK_SIZE, PROT_WRITE | PROT_READ,
                               MAP_PRIVATE | MAP_GROWSDOWN | MAP_ANONYMOUS, -1, 0);
}

long my_clone_raw(long a, long b, long c, long d, long e, long f, long g);
long my_clone(long a, long b, long c, long d, long e, long f, long g) {
    long res = my_clone_raw(a, b, c, d, e, f, g);

    if (-4096 <= res && res <= -1) {
        errno = -res;
        return -1;
    }

    return res;
}

int new_side_thread(void *stack_top, pid_t *child_tid) {
    return my_clone((long)side_thread, (long)stack_top,
        CLONE_THREAD | CLONE_SIGHAND | CLONE_VM | CLONE_CHILD_CLEARTID | CLONE_CHILD_SETTID,
        (long)NULL, (long)NULL, (long)NULL, (long)child_tid);
}


static int futex(int *uaddr, int futex_op, int val,
        const struct timespec *timeout, int *uaddr2, int val3) {
   return my_syscall6(SYS_futex, (long) uaddr, futex_op, val, (long) timeout, (long) uaddr, val3);
}

static void fwait(int *futexp, int val)
{
    int s;
    while (1) {
        /* Is the futex available? */
        const int zero = 0;
        if (atomic_compare_exchange_strong(futexp, &zero, 1))
            break;      /* Yes */

        /* Futex is not available; wait */
        s = futex(futexp, FUTEX_WAIT, val, NULL, NULL, 0);
        if (s == -1 && errno != EAGAIN)
            my_syscall1(SYS_exit_group, -1); // futex error
    }
}

int main() {
    void *child_stack = alloc_stack();
    if (child_stack == (void*) -1) {
        return 1;
    }
    pid_t child_tid;
    int pid = new_side_thread(child_stack + INIT_STACK_SIZE, &child_tid);
    if (pid == -1) {
        return 2;
    }
    main_thread();
    fwait(&child_tid, child_tid);
    my_syscall1(SYS_exit_group, 0);
}

void main_thread() {
    for (int i = 0; i < CHARS_AMOUNT; i++) {
        print("B\n", 2);
    }
}

int side_thread(void* _) {
    for (int i = 0; i < CHARS_AMOUNT; i++) {
        print("A\n", 2);
    }
    my_syscall1(SYS_exit, 0);
}

