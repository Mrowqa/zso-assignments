asm (
	".global _start\n"
	"_start:\n"
	"call entry\n"
	"hlt\n"
);

#include "fakelib.h"
#include "multiargs.h"

void entry() {
	long long int res = validate(I1, L1, LL1, U1, UL1, ULL1, I2, L2, LL2, U2, UL2, ULL2);
	if (res == LL_RET) {
		print("OK");
		fake_exit(0);
	}
	else {
		print("NOK");
		fake_exit(-1);
	}
}
