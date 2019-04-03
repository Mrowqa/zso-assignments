#ifndef CROSSLD_H
#define CROSSLD_H

enum type {
	TYPE_VOID,
	TYPE_INT,
	TYPE_LONG,
	TYPE_LONG_LONG,
	TYPE_UNSIGNED_INT,
	TYPE_UNSIGNED_LONG,
	TYPE_UNSIGNED_LONG_LONG,
	TYPE_PTR,
};

struct function {
	const char *name;
	const enum type *args;
	int nargs;
	enum type result;
	void *code;
};

int crossld_start(const char *fname, const struct function *funcs, int nfuncs);

#endif
