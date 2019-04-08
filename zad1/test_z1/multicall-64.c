#include "common.h"

static int get_int() {
	static int i = 1;
	return i++;
}

int main() {
	struct function funcs[] = {
		print_function,
		fake_exit_function,
		{"get_int", 0, 0, TYPE_INT, get_int},
	};
	
	return crossld_start("multicall-32", funcs, 3);
}
