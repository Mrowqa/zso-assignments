asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"

void entry()
{
	int val = get_int();
	exit(val);
}
