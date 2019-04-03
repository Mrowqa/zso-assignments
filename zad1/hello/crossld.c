// czy crossld ma byc bezpiecznie wielowatkowo?
// czy mozemy zalozyc istnienie SHT_DYNAMIC?
// o co chodzi z wielokrotnymi definicjami DT_STRTAB i DT_PLTRELSZ?
// munmap a co ze stosem? (moze sie rozszerzyc i zwezyc w trakcie wykonania)
// jakie calling convention dla 32-bitowego i 64-bitowego kodu
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


extern uint8_t call_32bits_code_start;
extern uint8_t call_32bits_code_end;
extern uint8_t call_32bits_code;
// int call_32bits_code(uint32_t entry_point, uint32_t new_stack);

extern uint8_t trampoline_32to64_start;
extern uint8_t trampoline_32to64_end;
extern uint8_t trampoline_32to64;


struct unmap_args_t {
    void *ptr;
    size_t len;
};

struct {
    struct unmap_args_t *unmap_args;
    size_t unmap_args_cap;
    size_t unmap_args_len;
} g_call_context; // TODO make it local


int crossld_start(const char *fname, const struct function *funcs, int nfuncs) {
    // tmp code
    void *code = mmap(NULL, 4096, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
    if (code == MAP_FAILED) {
        exit(-1);
    }
    memcpy(code, &trampoline_32to64_start, &trampoline_32to64_end - &trampoline_32to64_start);
    void *code2 = mmap(NULL, 4096, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
    if (code2 == MAP_FAILED) {
        exit(-1);
    }
    memcpy(code2, &call_32bits_code_start, &call_32bits_code_end - &call_32bits_code_start);
    void *code2_offseted = code2 + (&call_32bits_code - &call_32bits_code_start);
    // tmp code end

    assert(g_call_context.unmap_args_len == 0);
    assert(g_call_context.unmap_args_cap == 0);
    assert(!g_call_context.unmap_args);
    
    int ret = -1;

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

    g_call_context.unmap_args_cap = hdr->e_phnum + nfuncs; // todo +1 dla trampoliny dla _start, +1 dla trampoliny dla _end
    g_call_context.unmap_args = (struct unmap_args_t*) malloc(
        g_call_context.unmap_args_cap * sizeof(struct unmap_args_t));

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
        void *segm = mmap((void*) vaddr_start, page_offset + phdr->p_memsz,
            prot, flags, fd, file_start);
        if (segm == MAP_FAILED) {
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
                printf("Found sym: %s, offset=%p\n", sym, rel->r_offset);
                switch(ELF32_R_TYPE(rel->r_info)) {
                    case R_386_JMP_SLOT:
                        *(Elf32_Word*)(uint64_t)(rel->r_offset) = (uint32_t)(uint64_t) code+(&trampoline_32to64 - &trampoline_32to64_start);//hdr->e_entry;//(Elf32_Word)0x24242424; // todo put trampoline here
                        break;
                    default:
                        break;
                }
            }
        }
    }

    // alloc stack
    void *stack = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_GROWSDOWN | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
    if (stack == MAP_FAILED) {
        goto exit3;
    }
    uint32_t stack32_top = (uint32_t)(uint64_t) (stack + page_size);

    // call the actual 32 bit program
    __asm__ volatile (".byte 0xcc");
    //ret = call_32bits_code(hdr->e_entry, stack32_top);
    ret = ((int(*)(uint32_t, uint32_t)) code2_offseted)(hdr->e_entry, stack32_top);

    // todo dealloc stack (kinda tricky, since it grows)

exit3:
    // todo add entries to this map! (mmap -> segment, functions)
    for (size_t i = 0; i < g_call_context.unmap_args_len; i++) {
        if (-1 == munmap(g_call_context.unmap_args[i].ptr, g_call_context.unmap_args[i].len)) {
            ret = -1;
        }
    }
    free(g_call_context.unmap_args);
    memset(&g_call_context, 0, sizeof(g_call_context));
exit2:
    munmap(faddr, sb.st_size);
exit1:
    close(fd);
exit0:
    return ret;
}


_Noreturn void exit(int status) {
    // todo
    __builtin_unreachable();
}

   /* __asm__ (
            ".code32\n"
            "__jmp_32_ep:\n"
    		"pushl $0x2b;\n"
            "popl %ds;\n"
            "pushl $0x2b;\n"
            "popl %es;\n"
            "ret;\n"

            ".code64\n"
    );*/

/*
__asm__ (
    ".code32\n"
    "func_ptr:\n"
        ".byte 0, 0, 0, 0\n"

    "trampoline_32to64:\n"
        "call 28(%eip)\n" // $__get_eip\n"
    "__get_eip:\n"
        "pop %eax\n"
		"pushl $0x33\n"
		//"pushl $trampoline_step2\n"
		//"movl $trampoline_step2 - $trampoline_32to64, %eax\n"
        //"pushl ($func_ptr)\n"
		"lret\n"
    ".code64\n"
    "trampoline_step2:\n"
        "\n"
        "\n"
        "\n"
        "\n"
        "\n"
        "\n"
        "\n"
        "\n"
        "\n"
        "\n"
        "\n"
);*/
