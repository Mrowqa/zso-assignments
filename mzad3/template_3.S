[bits 64]
;.intel_syntax noprefix
;.globl formatter

formatter:
    push   rbp
    mov    rbp,rsp
    sub    rsp,0x18
    mov    rsi,rdi                 ; 2nd arg
    mov    rdi,0x4444444444444444  ; format str code
    mov    rcx,0x2222222222222222  ; printf addr
    call   rcx
    add    rsp,0x18
    pop    rbp
    ret

