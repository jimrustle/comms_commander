
OUTPUT_FORMAT ("elf32-littlearm")

EXTERN(Reset_Handler)

/* STM32L071CB has 128K of Flash, 20K of RAM, 6K of EEPROM */

MEMORY {
    flash   (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    ram    (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
    eeprom   (r) : ORIGIN = 0x08080000, LENGTH = 6K
}

__stack = ORIGIN(ram) + LENGTH(ram);
_estack = __stack;

ENTRY(_start)

SECTIONS {
    . = ORIGIN(flash);

    .vectors : ALIGN(4) {
        KEEP(*( .vectors ))
    } > flash

    _text = .;

    .text : ALIGN(4) {
        *( .text )
        *( .text* )
    } > flash

    _etext = .;

    .rodata : ALIGN(4) {
        . = ALIGN(4);
        __rodata_start__ = .;
        *( .rodata )
        *( .rodata.* )
        . = ALIGN(4);
        __rodata_end__ = .;
    } > flash 

    .data : ALIGN(4) {
        FILL(0xA5)
        . = ALIGN(4);
        PROVIDE(__textdata__ = LOADADDR(.data));
        PROVIDE(__data_start__ = . );
        *(.data )
        *(.data.* )
        *(.ramtext )
        . = ALIGN(4);
        PROVIDE(__data_end__ = . );
    } > ram AT > flash

    .bss (NOLOAD) : ALIGN(4) {
        . = ALIGN(4);
        PROVIDE(_bss_start = . );
        __bss_start__ = .;
        *(.bss )
        *(.bss.* )
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = . );
        __bss_end__ = .;
        PROVIDE(end = . );
    } > ram

    .noinit (NOLOAD) : ALIGN(4) {
        __noinit_start__ = .;
        *(.noinit .noinit.* )
         . = ALIGN(4) ;
        __noinit_end__ = .;
    } > ram

    .ARM.extab : {
        *( .ARM.extab* .gnu.linkonce.armextab.* )
    } > flash

    .ARM.exidx : {
        __exidx_start = .;
        *( .ARM.exidx* .gnu.linkonce.armexidx.* )
        __exidx_end = .;
    } > flash

    .eh_frame_hdr : {
        *( .eh_frame_hdr )
    } > flash

    .eh_frame : ONLY_IF_RO {
        *( .eh_frame )
    } > flash

    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

}

