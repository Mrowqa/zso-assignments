#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>

#define HELLO_IOCTL_SET_REPEATS _IO('H', 0x00)

int main(int argc, char **argv) {
	if (argc != 2) {
		fprintf(stderr, "One argument needed.\n");
		return 1;
	}
	int repeats;
	if (sscanf(argv[1], "%d", &repeats) != 1) {
		fprintf(stderr, "Number needed.\n");
		return 1;
	}
	int f = open("/dev/hello", O_RDWR);
	if (f < 0) {
		perror("open");
		return 1;
	}
	if (ioctl(f, HELLO_IOCTL_SET_REPEATS, repeats) < 0) {
		perror("ioctl");
		return 1;
	}
	return 0;
}
