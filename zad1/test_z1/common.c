#include <stdio.h>
#include <stdlib.h>
#include "common.h"

static void print(char *data) {
	printf("%s\n", data);
}

static void fake_exit(int res) {
	fflush(stdout);
	exit(res);
}

const static enum type print_types[] = {TYPE_PTR};
const static enum type fake_exit_types[] = {TYPE_INT};
const struct function print_function = {"print", print_types, 1, TYPE_VOID, print};
const struct function fake_exit_function = {"fake_exit", fake_exit_types, 1, TYPE_VOID, fake_exit};
	
