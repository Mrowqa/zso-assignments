.intel_syntax noprefix

.globl formatter_template
formatter_template:
    push   rbp
    mov    rbp,rsp
    push   r15
    sub    rsp,0x18
    lea    rdx,[rip+0xfffffffffffffff9]
    movabs r11,0x2e7b
    add    rdx,r11
    mov    DWORD PTR [rbp-0x14],edi
    mov    eax,DWORD PTR [rbp-0x14]
    mov    esi,eax
    movabs rax,0xffffffffffffe004
    lea    rdi,[rdx+rax*1]
    mov    r15,rdx
    mov    eax,0x0
    movabs rcx,0xffffffffffffd030
    add    rcx,rdx
    call   rcx
    nop
    add    rsp,0x18
    pop    r15
    pop    rbp
    ret

format_str:
    ; to be filled in runtime
