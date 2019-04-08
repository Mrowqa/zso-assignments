asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"

const char *str = "Some constant string";

void entry() {
	char *test = (char *) str;
	test[0] = 'A'; /* crash */

	exit(0);
}
