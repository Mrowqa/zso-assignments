#include <stdio.h>
#include <stdlib.h>
#include "crossld.h"

int main() {
	int v;
	if ((v = crossld_start("exit-32", 0, 0)) == 0x04134231) {
		printf("OK\n");
		return 0;
	} else {
		printf("Invalid: %d\n", v);
		return -1;
	}
}
