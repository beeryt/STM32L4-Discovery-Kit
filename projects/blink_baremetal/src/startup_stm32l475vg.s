.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global _exit

.section .vector_table,"a",%progbits
vector_table:
  .word     _estack         /* Top of Stack */
  .word     Reset_Handler   /* Reset_Handler */
  .space    (14 * 14)       /* ARM handlers are left out */
  .space    (224 * 4)       /* Interrupts 0 .. 224 are left out */

.section .text,"a",%progbits
Reset_Handler:
    b       main            /* Simply branch to main. Everything is uninitialized!! */

_exit:
    b _exit                 /* Inifinite loop at exit */