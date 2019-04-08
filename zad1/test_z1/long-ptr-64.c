#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "crossld.h"

static void *get_ptr64() {
	return (void*) (1LL<<63);
}

/* for programs w/o proper exit() */
void abort_handler(int i) {
	printf("OK\n");
	fflush(stdout);
	exit(0);
}

int main() {
	if (signal(SIGABRT, abort_handler) == SIG_ERR) {
		fprintf(stderr, "Couldn't set signal handler\n");
		return -1;
	}

	struct function funcs[] = {
		{"get_ptr64", 0, 0, TYPE_PTR, get_ptr64},
	};
	
	int v;
	if ((v = crossld_start("long-ptr-32", funcs, 1)) == -1) {
		printf("OK\n");
		return 0;
	} else {
		printf("Invalid response: %d\n", v);
		return -1;
	}
}
