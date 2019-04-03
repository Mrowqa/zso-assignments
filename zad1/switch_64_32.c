/* gcc -o switch_64_32 switch_64_32.c -fno-pic -no-pie */
#include <sys/mman.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


static void err(const char *err)
{
  fprintf(stderr, "%s: %s\n", err, strerror(errno));
  exit(1);
}

/* This code is called from 32-bit code */
_Noreturn void __code64()
{
	printf("Hello from 64-bit\n");
	_exit(0);
}

/* Sample 32-bit code */
__asm__ (
		".code32\n"

		"__code32:\n"
		"pushl $0x2b\n"
		"popl %ds\n"
		"pushl $0x2b\n"
		"popl %es\n"

		/* write message */
 		"movl $4, %eax\n"
		"movl $1, %ebx\n"
		"movl $msg, %ecx\n"
		"movl $(msg_end - msg), %edx\n"
		"int $0x80\n"

		/* jump to 64-bit code  */
		"pushl $0x33\n"
		"pushl $__code64\n"
		"lret\n"

		"msg:	.ascii \"Message from 32-bit code\\n\"\n"
		"msg_end:\n"

		".code64\n"
	);

int main()
{
  void* stack;
	
  
  if ((stack = mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_ANONYMOUS | MAP_PRIVATE | MAP_32BIT, -1, 0)) == MAP_FAILED)
    err("stack map");
  
  stack = (void*) (((uint64_t) stack) + 4096 - 4);
  
  
  /* Switch to 32-bit mode and jump into the 32-bit code */
  __asm__ volatile("movq %0, %%rsp;\n"
		   "subq $8, %%rsp\n"
		   "movl $0x23, 4(%%rsp);\n"
		   "movq $__code32, %%rax;\n"
		   "movl %%eax, (%%rsp);\n"
		   "lret" ::  "r"(stack) );
}
