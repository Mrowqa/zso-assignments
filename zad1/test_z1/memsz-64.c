#include "common.h"

int main() {
	struct function common_functions[] = {print_function, fake_exit_function};
	return crossld_start("memsz-32", common_functions, 2);
}
