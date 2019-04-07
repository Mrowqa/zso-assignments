/* ELF with misaligned second segment - second mmap will fail */

PHDRS
{
  text PT_LOAD AT (0x08048000) ;
  rodata PT_LOAD AT (0x08048800) ;
}

SECTIONS
{
  .text 0x08048080 : { *(.text) } :text
  .rodata 0x08048884 : { *(.rodata) } :rodata
}
