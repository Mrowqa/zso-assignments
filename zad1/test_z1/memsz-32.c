asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"

char empty[8192];

void entry() {
	for (int i = 0; i < 8192; i++)
		if (empty[i] != 0) {
			print("Empty is not clean");
			fake_exit(-1);
		}

	print("OK");
	fake_exit(0);
}
