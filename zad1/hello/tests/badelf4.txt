/* ELF with overlapping sections - mmap will succeed but map in the wrong place */

PHDRS
{
  text PT_LOAD AT (0x08048000) ;
  rodata PT_LOAD AT (0x08048800) ;
}

SECTIONS
{
  . = 0x08048000;
  .text ALIGN(0x800) : SUBALIGN(0x1000) { *(.text) } :text
  .bss1 ALIGN(0x800) (NOLOAD) : {
    . = ALIGN(0x1000) + 0x800;
  } :text
  .bss2 ALIGN(0x800) (NOLOAD) :{
    . = ALIGN(0x1000) + 0x800;
  } :rodata
  .rodata ALIGN(0x800) : SUBALIGN(0x1000) { *(.rodata) } :rodata
}
