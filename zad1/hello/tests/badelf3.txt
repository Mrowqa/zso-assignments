/* ELF with misalign between file and vaddr - first mmap will fail */

PHDRS
{
  text PT_LOAD AT (0x08048000) ;
  rodata PT_LOAD AT (0x08048800) ;
}

SECTIONS
{
  . = 0x08048000;
  .text ALIGN(0x800) : { *(.text) } :text
  .rodata ALIGN(0x800) : { *(.rodata) } :rodata
}
