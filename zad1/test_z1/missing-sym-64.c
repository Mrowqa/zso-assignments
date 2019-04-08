#include <stdio.h>
#include <stdlib.h>
#include "crossld.h"
static void obsolete_function() {
}

	
int main() {
	struct function funcs[] = {
		{"obsolete_function", 0, 0, TYPE_VOID, obsolete_function},
	};

	int v;
	if ((v = crossld_start("missing-sym-32", funcs, 1)) == -1) {
		printf("OK\n");
		return 0;
	} else {
		printf("Invalid: %d\n", v);
		return -1;
	}
}
