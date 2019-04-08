#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "crossld.h"
#define MAXPATH 4096

int main() {
	int v;
	char buf[MAXPATH];
	FILE *maps;

	if ((v = crossld_start("cleanup-32", 0, 0)) != 0) {
		printf("Invalid: %d\n", v);
		return -1;
	}

	if ((maps = fopen("/proc/self/maps", "r")) == NULL) {
		perror("fopen");
		exit(1);
	}

	while (fscanf(maps, "%*s %*s %*s %*s %*s%*[ ]%[^ \n]\n", buf) != EOF) {
		if(strcmp(buf + strlen(buf) - 11, "/cleanup-32") == 0) {
			printf("Found unmapped part of cleanup-32\n");
			fclose(maps);
			return -1;
		}
	}


	fclose(maps);
	printf("OK\n");
	return 0;
}
