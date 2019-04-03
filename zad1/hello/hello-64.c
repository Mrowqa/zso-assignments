#include <stdio.h>
#include "crossld.h"

static void print(char *data) {
	printf("%s\n", data);
}

int main(int argc, char *argv[]) {
	int res;
	enum type print_types[] = {TYPE_PTR};
	struct function funcs[] = {
		{"print", print_types, 1, TYPE_VOID, print},
	};
	
	 res = crossld_start(argc > 1 ? argv[1] : "hello-32", funcs, 1);
	 printf("Result: %d\n", res);
	 return res;
}
