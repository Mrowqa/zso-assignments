asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"
#define SIZE (1<<21)
void entry() {
	char test[SIZE];
	test[0] = 'O';
	test[1] = 'K';
	test[2] = 0;
	test[SIZE - 1] = 0x04;

	print(test);
	fake_exit(0);
}
