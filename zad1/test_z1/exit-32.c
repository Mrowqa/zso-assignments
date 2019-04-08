asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"


void entry() {
	exit(0x04134231);
}
