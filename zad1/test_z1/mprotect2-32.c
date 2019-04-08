asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"
char buf[10];
typedef void (*func) (void);

void entry() {
	buf[0] = '\xc3';

	func f = (func) buf;
	f(); /* sigsegv */
	while(1);
}
