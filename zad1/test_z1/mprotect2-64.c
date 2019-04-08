#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "crossld.h"


static void segv_handler(int sig, siginfo_t *si, void *unused) {
	printf("OK\n");
	exit(0);
}

int main() {
	struct sigaction sa;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = segv_handler;
	if (sigaction(SIGSEGV, &sa, NULL) == -1) {
		perror("sigaction");
		exit(1);
	}


	crossld_start("mprotect2-32", 0, 0); /* should crash */
	printf("NOK\n");
	return -1;
}
