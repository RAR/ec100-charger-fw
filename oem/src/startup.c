/* oem/src/startup.c */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _stack_top;
extern void (*__preinit_array_start[])(void);
extern void (*__preinit_array_end[])(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);
int main(void);
void SysTick_Handler(void);
void Reset_Handler(void);

#define REG(a) (*(volatile uint32_t *)(a))

/* Unexpected exception: stop feeding the watchdog, which resets the board within ~2 s. */
void Default_Handler(void) {
  for (;;) {
  }
}

/* Defined in hw/rs485.cpp; apps that don't link the RS485 driver never enable IRQ28. */
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

typedef void (*vector_t)(void);

__attribute__((section(".isr_vector"), used)) const vector_t g_vectors[16 + 48] = {
    (vector_t)&_stack_top, Reset_Handler, Default_Handler, Default_Handler,  /* SP, reset, NMI, HardFault */
    0, 0, 0, 0, 0, 0, 0, Default_Handler, 0, 0, Default_Handler, SysTick_Handler,  /* SVC 11, PendSV 14, SysTick 15 */
    [16 ... 43] = Default_Handler,
    [16 + 28] = USART1_IRQHandler,
    [45 ... 63] = Default_Handler,
};

void Reset_Handler(void) {
  REG(0xE000ED08) = (uint32_t)g_vectors; /* VTOR first: the bootloader's SysTick is still running */
  __asm volatile("dsb\n isb" ::: "memory");
  REG(0xE000E010) = 0;                   /* SysTick off until systick_init() */
  REG(0xE000E180) = 0xFFFFFFFFu;         /* NVIC: disable and un-pend whatever the bootloader left */
  REG(0xE000E184) = 0xFFFFFFFFu;
  REG(0xE000E280) = 0xFFFFFFFFu;         /* cleared again after the peripheral resets in takeover_from_bootloader() */
  REG(0xE000E284) = 0xFFFFFFFFu;
  REG(0x40003000) = 0xAAAAu;             /* feed the FWDGT the bootloader started */
  for (uint32_t *src = &_sidata, *dst = &_sdata; dst < &_edata;) *dst++ = *src++;
  for (uint32_t *dst = &_sbss; dst < &_ebss;) *dst++ = 0;
  for (void (**f)(void) = __preinit_array_start; f < __preinit_array_end; f++) (*f)();
  for (void (**f)(void) = __init_array_start; f < __init_array_end; f++) (*f)();
  __asm volatile("cpsie i" ::: "memory");
  main();
  for (;;) {
  }
}
