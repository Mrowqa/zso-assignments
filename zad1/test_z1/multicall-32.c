asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"

void entry() {
	int val = 1;
	for (int i = 0; i < 10; i++)
		val *= get_int();
	
	if (val == 3628800) { 
		print("OK");
		fake_exit(0);
	} else {
		print("NOK");
		fake_exit(1);
	}
}
