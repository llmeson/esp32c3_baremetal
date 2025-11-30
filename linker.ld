/* linker.ld — ESP32-C3 didáctico (fix .srodata/.sdata/.sbss) */

ENTRY(_start)

MEMORY
{
  /* Código XIP */
  IROM (rx)  : ORIGIN = 0x42000000, LENGTH = 2M
  /* Constantes XIP por bus de datos */
  DROM (rx)  : ORIGIN = 0x3C000000, LENGTH = 512K
  /* DRAM interna */
  DRAM (rwx) : ORIGIN = 0x3FC80000, LENGTH = 0x00060000  /* hasta 0x3FCE0000 (fix stack guard) */
}

/* Tope de pila (lo publicamos igual que antes) */
PROVIDE(_stack_top = ORIGIN(DRAM) + LENGTH(DRAM));

SECTIONS
{
  /* --- Código/funciones en flash ejecutable --- */
  .text :
  {
    _stext = .;
    *(.init)
    *(.text .text.*)
    *(.gnu.linkonce.t.*)
    . = ALIGN(4);
    _etext = .;
  } > IROM

  /* --- Constantes en DROM (incluye .srodata) --- */
  .rodata : AT (LOADADDR(.text) + SIZEOF(.text))
  {
    . = ALIGN(4);
    _srodata = .;
    *(.rodata .rodata.*)
    *(.srodata .srodata.*)
    *(.gnu.linkonce.r.*)
    . = ALIGN(4);
    _erodata = .;
  } > DROM

  /* --- Datos inicializados en DRAM (incluye .sdata) --- */
  .data : AT (LOADADDR(.rodata) + SIZEOF(.rodata))
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data.*)
    *(.sdata .sdata.*)
    . = ALIGN(4);
    _edata = .;
  } > DRAM
  /* Dirección de carga en flash de .data (la usa tu startup) */
  PROVIDE(_sidata = LOADADDR(.data));

  /* --- BSS en DRAM (incluye .sbss y COMMON) --- */
  .bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sbss = .;
    *(.bss .bss.*)
    *(.sbss .sbss.*)
    *(COMMON)
    . = ALIGN(16);
    /* Guard pedagógico para evitar off-by-one en limpiado */
    . += 64;
    _ebss = .;
  } > DRAM

  /* --- Opcional: reservar pila dentro de DRAM (sugerido) ---
     Si querés pila explícita y símbolo fuerte __stack_top, descomentá:

  .stack (NOLOAD) :
  {
    . = ALIGN(16);
    __stack_bottom = .;
    . += 16K;
    __stack_top = .;
  } > DRAM
  PROVIDE(_stack_top = __stack_top);

  */
}