#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <elf.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h> // TODO remove
#include "crossld.h"


#define GUEST_STACK_SIZE (1024 * 1024 * 4)
#define BREAKPOINT {__asm__ volatile (".byte 0xcc");} // todo remove


extern uint8_t call_guest_code_start;
extern uint8_t call_guest_code_end;
extern uint8_t call_guest_code_exit;
int call_guest_code(uint32_t entry_point, uint32_t new_stack);
typedef int (*call_guest_code_t)(uint32_t, uint32_t);

#define CALL_GUEST_SIZE (&call_guest_code_end - &call_guest_code_start)
#define CALL_GUEST_EP_OFFSET ((uint8_t*)(&call_guest_code) - &call_guest_code_start)
#define CALL_GUEST_EXIT_OFFSET (&call_guest_code_exit - &call_guest_code_start)

extern uint8_t trampoline_32to64_start;
extern uint8_t trampoline_32to64_end;
extern uint8_t trampoline_32to64;
extern uint8_t trampoline_32to64_func_ptr;
extern uint8_t trampoline_32to64_code_ptr;
extern uint8_t trampoline_32to64_required_stack_size;
extern uint8_t trampoline_32to64_conv_args_ptr;
extern uint8_t trampoline_32to64_conv_ret_val_ptr;

#define TRAMPOLINE_SIZE (&trampoline_32to64_end - &trampoline_32to64_start)
#define TRAMPOLINE_EP_OFFSET (&trampoline_32to64 - &trampoline_32to64_start)
#define TRAMPOLINE_FUNC_PTR_OFFSET (&trampoline_32to64_func_ptr - &trampoline_32to64_start)
#define TRAMPOLINE_CODE_PTR_OFFSET (&trampoline_32to64_code_ptr - &trampoline_32to64_start)
#define TRAMPOLINE_REQUIRED_STACK_SIZE_OFFSET (&trampoline_32to64_required_stack_size - &trampoline_32to64_start)
#define TRAMPOLINE_CONV_ARGS_PTR_OFFSET (&trampoline_32to64_conv_args_ptr - &trampoline_32to64_start)
#define TRAMPOLINE_CONV_RET_VAL_PTR_OFFSET (&trampoline_32to64_conv_ret_val_ptr - &trampoline_32to64_start)


typedef struct unmap_args {
    void *ptr;
    size_t len;
} unmap_args_t;

typedef struct mappings_info {
    unmap_args_t *args;
    size_t args_cap;
    size_t args_len;
} mappings_info_t;

// todo rename?
void init_new_mappings(mappings_info_t *minfo, size_t capacity);
void add_new_mapping(mappings_info_t *minfo, unmap_args_t args);
void clear_all_mappings(mappings_info_t *minfo, int *ret_code);

typedef struct trampolines_info {
    uint32_t base_addr;
    const struct function *funcs;
    size_t nfuncs;
} trampolines_info_t;

int generate_trampolines(trampolines_info_t *tinfo, mappings_info_t *minfo,
    const struct function *funcs, int nfuncs);
uint64_t calculate_required_stack_size(const struct function *fun);
uint32_t get_trampoline(trampolines_info_t *tinfo, const char *sym_name);

void convert_arguments(uint64_t *dst, uint32_t *src, struct function *fun);
uint32_t convert_return_value(uint64_t value, struct function *fun);

#define TRAMPOLINE_NOT_FOUND 0
#define TRAMPOLINE_GENERATION_FAILED 1
#define TRAMPOLINE_GENERATION_OK 0


// todo rename vars
// todo move declaration to beggining?
// todo jak duzo argsow? dotykanie stron pamieci po kolei?
// todo czy funcs args sa poprawne?


int crossld_start(const char *fname, const struct function *funcs, int nfuncs) {
    mappings_info_t minfo = {0};
    trampolines_info_t tinfo = {0};
    int ret_code = -1;

    // open & map file
    int fd = open(fname, O_RDONLY);
    if (fd == -1) {
        goto exit0;
    }
    struct stat sb;
    if (fstat(fd, &sb) == -1 || sb.st_size < sizeof(Elf32_Ehdr)) {
        goto exit1;
    }

    void *faddr = mmap(NULL, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (faddr == MAP_FAILED) {
        goto exit1;
    }

    // check elf header
    const Elf32_Ehdr *hdr = (Elf32_Ehdr*) faddr;
    if (memcmp(hdr->e_ident, ELFMAG, SELFMAG) != 0 ||
            hdr->e_ident[EI_CLASS] != ELFCLASS32 ||
            hdr->e_ident[EI_DATA] != ELFDATA2LSB ||
            hdr->e_ident[EI_OSABI] != ELFOSABI_NONE ||
            hdr->e_type != ET_EXEC ||
            hdr->e_machine != EM_386 ||
            hdr->e_version != EV_CURRENT) {
        goto exit2;
    }

    // map program segments
    if (hdr->e_phentsize != sizeof(Elf32_Phdr)) {
        goto exit2;
    }

    const Elf32_Phdr *segments = (Elf32_Phdr*) (faddr + hdr->e_phoff);
    size_t page_size = sysconf(_SC_PAGESIZE);
    if (page_size != 0x1000) {
        goto exit2;
    }

    init_new_mappings(&minfo, hdr->e_phnum + 1); // each segment + one mapping for trampolines

    int has_dyn_segment = 0;
    for (size_t i = 0; i < hdr->e_phnum; i++) {
        const Elf32_Phdr *phdr = &segments[i];

        if (phdr->p_type == PT_DYNAMIC) {
            has_dyn_segment = 1;
        }

        if (phdr->p_type != PT_LOAD) {
            continue;
        }

        if (page_size % phdr->p_align != 0) {
            goto exit3;
        }

        size_t page_offset = phdr->p_offset % page_size;
        size_t file_start = phdr->p_offset - page_offset;
        size_t vaddr_start = phdr->p_vaddr - page_offset;
        if (vaddr_start % page_size != 0 || file_start % page_size != 0) {
            goto exit3;
        }

        int prot = (phdr->p_flags & PF_X ? PROT_EXEC : 0) |
            (phdr->p_flags & PF_W ? PROT_WRITE : 0) |
            (phdr->p_flags & PF_R ? PROT_READ : 0);
        int flags = MAP_PRIVATE | MAP_32BIT | MAP_FIXED_NOREPLACE;
        size_t map_len = page_offset + phdr->p_memsz;
        void *segm = mmap((void*) vaddr_start, map_len,
            prot, flags, fd, file_start);
        
        if (segm == MAP_FAILED) {
            goto exit3;
        }
        add_new_mapping(&minfo, (unmap_args_t) {.ptr = segm, .len = map_len});

        if (segm != (void*)vaddr_start) {
            goto exit3;
        }

        // memset(segm, '\0', page_offset);
        memset(segm + page_offset + phdr->p_filesz, '\0',
            phdr->p_memsz - phdr->p_filesz);
    }

    // handle .dynamic section
    Elf32_Shdr *sections = (Elf32_Shdr*) (faddr + hdr->e_shoff);
    int has_dyn_section = 0;
    const char *str_tab = NULL;
    const Elf32_Sym *sym_tab = NULL;
    const Elf32_Rel *jmp_rel_tab = NULL;
    const Elf32_Rel *jmp_rel_tab_end = NULL;
    size_t plt_rel_sz = 0;

    for (size_t i = 0; i < hdr->e_shnum; i++) {
        const Elf32_Shdr *shdr = &sections[i];

        if (shdr->sh_type != SHT_DYNAMIC) {
            continue;
        }

        has_dyn_section = 1;
        
        const Elf32_Dyn *dyn_table = (Elf32_Dyn*)(uint64_t) shdr->sh_addr;
        const Elf32_Dyn *dyn_table_end = (Elf32_Dyn*)(uint64_t) (shdr->sh_addr + shdr->sh_size);
        
        for (const Elf32_Dyn *dyn_it = dyn_table; dyn_it < dyn_table_end; dyn_it++) {
            if (dyn_it->d_tag == DT_STRTAB) {
                if (str_tab) {
                    goto exit3;
                }
                str_tab = (char*)(uint64_t) dyn_it->d_un.d_ptr;
            }
            else if (dyn_it->d_tag == DT_SYMTAB) {
                if (sym_tab) {
                    goto exit3;
                }
                sym_tab = (Elf32_Sym*)(uint64_t) dyn_it->d_un.d_ptr;
            }
            else if (dyn_it->d_tag == DT_PLTRELSZ) {
                if (plt_rel_sz > 0) {
                    goto exit3;
                }
                plt_rel_sz = dyn_it->d_un.d_val;
            }
            else if (dyn_it->d_tag == DT_JMPREL) {
                if (jmp_rel_tab) {
                    goto exit3;
                };
                jmp_rel_tab = (Elf32_Rel*)(uint64_t) dyn_it->d_un.d_ptr;
            }
        }
        break;
    }

    if (has_dyn_section != has_dyn_segment) {
        goto exit3;
    }

    // relocation
    if (generate_trampolines(&tinfo, &minfo, funcs, nfuncs) != TRAMPOLINE_GENERATION_OK) {
        goto exit3;
    }

    if (has_dyn_section) {
        if (!sym_tab || !str_tab) {
            goto exit3;
        }

        if (jmp_rel_tab) {
            if (plt_rel_sz == 0) {
                goto exit3;
            }

            jmp_rel_tab_end = (Elf32_Rel*) (((char*)jmp_rel_tab) + plt_rel_sz);

            for (const Elf32_Rel *rel = jmp_rel_tab; rel < jmp_rel_tab_end; rel++) {
                const char* sym = str_tab + sym_tab[ELF32_R_SYM(rel->r_info)].st_name;
                switch(ELF32_R_TYPE(rel->r_info)) {
                    case R_386_JMP_SLOT: {
                        uint32_t addr = get_trampoline(&tinfo, sym);
                        if (addr != TRAMPOLINE_NOT_FOUND) {
                            *(Elf32_Word*)(uint64_t)(rel->r_offset) = addr;
                        }
                        else {
                            goto exit3;
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
        }
    }

    // alloc stack
    void *stack = mmap(NULL, GUEST_STACK_SIZE, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
    if (stack == MAP_FAILED) {
        goto exit3;
    }
    add_new_mapping(&minfo, (unmap_args_t) {.ptr = stack, .len = GUEST_STACK_SIZE});
    uint32_t stack32_top = (uint32_t)(uint64_t) (stack + GUEST_STACK_SIZE);

    // call the actual 32 bit program
    uint32_t guest_ep = get_trampoline(&tinfo, "_start");
    ret_code = ((call_guest_code_t)(uint64_t) guest_ep) (hdr->e_entry, stack32_top);

exit3:
    clear_all_mappings(&minfo, &ret_code);
exit2:
    munmap(faddr, sb.st_size);
exit1:
    close(fd);
exit0:
    return ret_code;
}


void init_new_mappings(mappings_info_t *minfo, size_t capacity) {
    assert(minfo);
    assert(minfo->args_cap == 0);
    assert(capacity > 0);
    
    minfo->args_len = 0;
    minfo->args_cap = capacity;
    minfo->args = (unmap_args_t*) malloc(
        minfo->args_cap * sizeof(unmap_args_t));
}

void add_new_mapping(mappings_info_t *minfo, unmap_args_t args) {
    assert(minfo);
    assert(minfo->args);
    assert(minfo->args_len < minfo->args_cap);

    minfo->args[minfo->args_len++] = args;
}

void clear_all_mappings(mappings_info_t *minfo, int *ret_code) {
    assert(minfo);
    assert(minfo->args);

    for (size_t i = 0; i < minfo->args_len; i++) {
        if (-1 == munmap(minfo->args[i].ptr, minfo->args[i].len)) {
            *ret_code = -1;
        }
    }
    free(minfo->args);
    memset(minfo, 0, sizeof(minfo));
}

int generate_trampolines(trampolines_info_t *tinfo, mappings_info_t *minfo,
        const struct function *funcs, int nfuncs) {
    assert(tinfo);
    assert(minfo);
    assert(funcs);
    assert(nfuncs > 0);
    
    // allocate memory
    size_t trampolines_size = nfuncs * TRAMPOLINE_SIZE + CALL_GUEST_SIZE;
    void *addr = mmap(NULL, trampolines_size, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
    if (addr == MAP_FAILED) {
        return TRAMPOLINE_GENERATION_FAILED;
    }
    add_new_mapping(minfo, (unmap_args_t) {.ptr = addr, .len = trampolines_size});

    tinfo->base_addr = (uint32_t)(uint64_t) addr;
    tinfo->funcs = funcs;
    tinfo->nfuncs = nfuncs;

    // prepare trampolines
    for (int i = 0; i < nfuncs; i++) {
        void *elem_addr = addr + i * TRAMPOLINE_SIZE;
        
        memcpy(elem_addr, &trampoline_32to64_start, TRAMPOLINE_SIZE);
        *(uint64_t*)(elem_addr + TRAMPOLINE_FUNC_PTR_OFFSET) = (uint64_t) &funcs[i];
        *(uint64_t*)(elem_addr + TRAMPOLINE_CODE_PTR_OFFSET) = (uint64_t) funcs[i].code;
        *(uint64_t*)(elem_addr + TRAMPOLINE_REQUIRED_STACK_SIZE_OFFSET) = calculate_required_stack_size(&funcs[i]);
        *(uint64_t*)(elem_addr + TRAMPOLINE_CONV_ARGS_PTR_OFFSET) = (uint64_t) convert_arguments;
        *(uint64_t*)(elem_addr + TRAMPOLINE_CONV_RET_VAL_PTR_OFFSET) = (uint64_t) convert_return_value;
    }

    // prepare _start & exit trampolines
    memcpy(addr + nfuncs * TRAMPOLINE_SIZE, &call_guest_code_start, CALL_GUEST_SIZE);

    // and make trampolines executable
    if (mprotect(addr, trampolines_size, PROT_READ | PROT_EXEC) != 0) {
        return TRAMPOLINE_GENERATION_FAILED;
    }

    return TRAMPOLINE_GENERATION_OK;
}

uint64_t calculate_required_stack_size(const struct function *fun) {
    assert(fun);

    uint64_t size = fun->nargs * 0x8;

    // make sure to pad to at least 6 arguments/registers
    if (size < 6 * 0x8) {
        size = 6 * 0x8;
    }
    // and make sure the stack is aligned to 0 mod 16
    else if (size & 0xF) {
        size += 0x8;
    }

    return size;
}

uint32_t get_trampoline(trampolines_info_t *tinfo, const char *sym_name) {
    assert(tinfo);
    assert(sym_name);

    if (strcmp(sym_name, "_start") == 0) {
        return tinfo->base_addr + tinfo->nfuncs * TRAMPOLINE_SIZE + CALL_GUEST_EP_OFFSET;
    }

    if (strcmp(sym_name, "exit") == 0) {
        return tinfo->base_addr + tinfo->nfuncs * TRAMPOLINE_SIZE + CALL_GUEST_EXIT_OFFSET;
    }

    for (size_t i = 0; i < tinfo->nfuncs; i++) {
        if (strcmp(sym_name, tinfo->funcs[i].name) == 0) {
            return tinfo->base_addr + TRAMPOLINE_SIZE * i + TRAMPOLINE_EP_OFFSET;
        }
    }

    return TRAMPOLINE_NOT_FOUND;
}

// todo test this!
void convert_arguments(uint64_t *dst, uint32_t *src, struct function *fun) {
    for (int i = 0; i < fun->nargs; i++) {
        switch (fun->args[i]) {
            case TYPE_INT:
            case TYPE_LONG:
                *(int64_t*) dst = *(int32_t*) src;
                break;
            case TYPE_LONG_LONG:
                *(int64_t*) dst = *(int64_t*) src;
                src++;
                break;
            case TYPE_UNSIGNED_INT:
            case TYPE_UNSIGNED_LONG:
            case TYPE_PTR:
                *dst = *src;
                break;
            case TYPE_UNSIGNED_LONG_LONG:
                *dst = *(uint64_t*) src;
                break;
            default:
                BREAKPOINT; // change to call exit(-1)
                break;
        }
        dst++;
        src++;
    }
}

// todo test this
uint32_t convert_return_value(uint64_t value, struct function *fun) {
    return (uint32_t) value;

    // switch (fun->result) {
    //     case TYPE_VOID:
    //         return 0;
    //     case TYPE_INT:
    //     case TYPE_LONG:
    //         *(int64_t*) dst = *(int32_t*) src;
    //         break;
    //     case TYPE_LONG_LONG: // how? :O
    //         *(int64_t*) dst = *(int64_t*) src;
    //         src++;
    //         break;
    //     case TYPE_UNSIGNED_INT:
    //     case TYPE_UNSIGNED_LONG:
    //     case TYPE_PTR:
    //         *dst = *src;
    //         break;
    //     case TYPE_UNSIGNED_LONG_LONG:
    //         *dst = *(uint64_t*) src;
    //         break;
    //     default:
    //         BREAKPOINT; // change to call exit(-1)
    //         break;
    // }
}
