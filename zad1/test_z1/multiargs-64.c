#include "common.h"
#include "multiargs.h"

static long long validate(int i1, long l1, long long ll1, unsigned int ui1,
		unsigned long ul1, unsigned long long ull1, int i2, long l2,
		long long ll2, unsigned int ui2, unsigned long ul2,
		unsigned long long ull2) {
	if (i1 != I1 || l1 != L1 || ll1 != LL1 || ui1 != U1 || ul1 != UL1 || ull1 != ULL1 || i2 != I2 || l2 != L2 || ll2 != LL2 || ui2 != U2 || ul2 != UL2 || ull2 != ULL2)
		return -1;
	return LL_RET;
}


int main() {
	enum type validate_types[] = {TYPE_INT, TYPE_LONG, TYPE_LONG_LONG,
		TYPE_UNSIGNED_INT, TYPE_UNSIGNED_LONG, TYPE_UNSIGNED_LONG_LONG,
		TYPE_INT, TYPE_LONG, TYPE_LONG_LONG, TYPE_UNSIGNED_INT,
		TYPE_UNSIGNED_LONG, TYPE_UNSIGNED_LONG_LONG};


	struct function funcs[] = {
		print_function,
		fake_exit_function,
		{"validate", validate_types, 12, TYPE_LONG_LONG, validate},
	};
	
	return crossld_start("multiargs-32", funcs, 3);
}
