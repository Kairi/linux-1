/*
 *
 * Automatically generated file; DO NOT EDIT.
 * Linux/x86_64 3.4.0 Kernel Configuration
 *
 */
/*
 * Helper macros to use CONFIG_ options in C/CPP expressions. Note that
 * these only work with boolean and tristate options.
 */
/*
 * Getting something that works in C and CPP for an arg that may or may
 * not be defined is tricky.  Here, if we have "#define CONFIG_BOOGER 1"
 * we match on the placeholder define, insert the "0," for arg1 and generate
 * the triplet (0, 1, 0).  Then the last step cherry picks the 2nd arg (a one).
 * When CONFIG_BOOGER is not defined, we generate a (... 1, 0) pair, and when
 * the last step cherry picks the 2nd arg, we get a zero.
 */
/*
 * IS_ENABLED(CONFIG_FOO) evaluates to 1 if CONFIG_FOO is set to 'y' or 'm',
 * 0 otherwise.
 *
 */
/*
 * IS_BUILTIN(CONFIG_FOO) evaluates to 1 if CONFIG_FOO is set to 'y', 0
 * otherwise. For boolean options, this is equivalent to
 * IS_ENABLED(CONFIG_FOO).
 */
/*
 * IS_MODULE(CONFIG_FOO) evaluates to 1 if CONFIG_FOO is set to 'm', 0
 * otherwise.
 */
/*
 * wakeup.ld
 *
 * Linker script for the real-mode wakeup code
 */
/*
 * Definitions for the wakeup data structure at the head of the
 * wakeup code.
 */
/* Wakeup behavior bits */
OUTPUT_FORMAT("elf32-i386", "elf32-i386", "elf32-i386")
OUTPUT_ARCH(i386)
ENTRY(_start)
SECTIONS
{
 . = 0;
 .jump : {
  *(.jump)
 } = 0x90909090
 . = 8;
 .header : {
  *(.header)
 }
 . = ALIGN(16);
 .text : {
   *(.text*)
 } = 0x90909090
 . = ALIGN(16);
 .rodata : {
  *(.rodata*)
 }
 .videocards : {
  video_cards = .;
  *(.videocards)
  video_cards_end = .;
 }
 . = ALIGN(16);
 .data : {
   *(.data*)
 }
 . = ALIGN(16);
 .bss : {
  __bss_start = .;
  *(.bss)
  __bss_end = .;
 }
 .signature : {
  *(.signature)
 }
 _end = .;
 /DISCARD/ : {
  *(.note*)
 }
}
