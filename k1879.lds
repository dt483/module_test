/* Default linker script, for normal executables */
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm",
          "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SEARCH_DIR("=/usr/local/lib"); SEARCH_DIR("=/lib"); SEARCH_DIR("=/usr/lib");


MEMORY
{
	
	RAM				(rwx)	: ORIGIN = 0xC0000100, LENGTH = 0x7FEFF
	VECTORS			(rwx)	: ORIGIN = 0x00000000, LENGTH = 0xFF

	
}

  PROVIDE (_ram0 = 0xC0000000);
  PROVIDE (_ram1 = 0xC0040000);
  
 /* PROVIDE ( __stacktop = ORIGIN(RAM) + LENGTH(RAM) );*/
  
  /*PROVIDE (HEAP_SIZE				= 1024);*/

SECTIONS
{

  .init           :
  {
    KEEP (*(.init))
  }  > VECTORS 
  

  .text           :
  {
    *(.text.init)
    *(.text.main)
    *(.text.unlikely .text.*_unlikely)
    *(.text.exit .text.exit.*)
    *(.text.startup .text.startup.*)
    *(.text.hot .text.hot.*)
    *(.text .stub .text.* .gnu.linkonce.t.*)
  } 

  PROVIDE (__etext = .);
  PROVIDE (_etext = .);
  PROVIDE (etext = .);
 
  .rodata         : { *(.rodata .rodata.* .gnu.linkonce.r.*) } 
  .rodata1        : { *(.rodata1) }
 
   
  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } 
   PROVIDE_HIDDEN (__exidx_start = .); 
  .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } 
   PROVIDE_HIDDEN (__exidx_end = .);
   
 
  
  /*BSS DATA */
  .data           :
  {
      __data_start = . ;
    *(.data .data.* .gnu.linkonce.d.*)
  } 
  
  .data1          : { *(.data1) } 
  _edata = .; PROVIDE (edata = .);
 
  __bss_start = .;
  __bss_start__ = .;
  .bss            :
  {
   *(.dynbss)
   *(.bss .bss.* .gnu.linkonce.b.*)
   *(COMMON)
   /* Align here to ensure that the .bss section occupies space up to
      _end.  Align after .bss to ensure correct alignment even if the
      .bss section disappears because there are no input sections..  */
   . = ALIGN(. != 0 ? 32 / 8 : 1);
  } 
  
  _bss_end__ = . ; __bss_end__ = . ;
  . = ALIGN(32 / 8);
  . = ALIGN(32 / 8);
  __end__ = . ;
  _end = .; PROVIDE (end = .);
  
  /*.heap : 
  {
    __HeapBase = .;
    PROVIDE (__HeapBase = .);
    KEEP (*.heap);
    . += HEAP_SIZE;
    __HeapLimit = .;
    PROVIDE (__HeapLimit = .);
  } */
    
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) } 
  .line           0 : { *(.line) } 
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) } 
  .debug_sfnames  0 : { *(.debug_sfnames) } 
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) } 
  .debug_pubnames 0 : { *(.debug_pubnames) } 
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) } 
  .debug_abbrev   0 : { *(.debug_abbrev) } 
  .debug_line     0 : { *(.debug_line) } 
  .debug_frame    0 : { *(.debug_frame) } 
  .debug_str      0 : { *(.debug_str) } 
  .debug_loc      0 : { *(.debug_loc) } 
  .debug_macinfo  0 : { *(.debug_macinfo) } 
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) } 
  .debug_funcnames 0 : { *(.debug_funcnames) } 
  .debug_typenames 0 : { *(.debug_typenames) } 
  .debug_varnames  0 : { *(.debug_varnames) } 
  /* DWARF 3 */
  .debug_pubtypes 0 : { *(.debug_pubtypes) } 
  .debug_ranges   0 : { *(.debug_ranges) } 
    PROVIDE (__edbg = .);

  PROVIDE (__edbg = .);
  .comment :
  {
  	*(*.comment)
  }
 
  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) KEEP (*(.gnu.attributes)) } 
  .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) } 
  /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) } 
  PROVIDE (__earm_attr = .);
  
  /*TODO: change heap start pointer in function sbrk() in cstubs*/
  
   .stack ORIGIN(RAM) + LENGTH(RAM) - 4:
  {
  . = ALIGN(32 / 8);
    __stacktop = .;
    PROVIDE (__stacktop = .);
    *(.stack)
  } 
}
