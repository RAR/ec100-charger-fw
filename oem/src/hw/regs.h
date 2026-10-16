// oem/src/hw/regs.h: register addresses used by the app. Values and sequences follow the stock v1 firmware;
// see docs/history/notes/oem-init-sequences.md for the evidence behind each one.
#pragma once
#include <cstdint>

namespace hw {

inline volatile uint32_t &reg(uint32_t addr) { return *reinterpret_cast<volatile uint32_t *>(addr); }
// Read-modify-write: not atomic. Only the main loop may use it on a register, except USART1_CTL0, which the USART1
// ISR and the main loop take turns on (the handoff is in rs485.cpp). SysTick touches no register.
inline void rmw(uint32_t addr, uint32_t mask, uint32_t value) { reg(addr) = (reg(addr) & ~mask) | (value & mask); }

constexpr uint32_t RCU_CTL = 0x40021000;
constexpr uint32_t RCU_CFG0 = 0x40021004;
constexpr uint32_t RCU_INT = 0x40021008;
constexpr uint32_t RCU_APB1RST = 0x40021010;
constexpr uint32_t RCU_AHBEN = 0x40021014;
constexpr uint32_t RCU_APB2EN = 0x40021018;
constexpr uint32_t RCU_APB1EN = 0x4002101C;
constexpr uint32_t RCU_BDCTL = 0x40021020;
constexpr uint32_t RCU_RSTSCK = 0x40021024;
constexpr uint32_t RCU_CFG1 = 0x4002102C;
constexpr uint32_t RCU_CFG2 = 0x40021030;
constexpr uint32_t DMA_INTC = 0x40020004;
constexpr uint32_t DMA_CH1CTL = 0x4002001C;  // DMA base 0x40020000 + 0x08 + 1 * 0x14
constexpr uint32_t DMA_CH1CNT = 0x40020020;
constexpr uint32_t DMA_CH1PADDR = 0x40020024;
constexpr uint32_t DMA_CH1MADDR = 0x40020028;
constexpr uint32_t DMA_CH3CTL = 0x40020044;  // DMA base 0x40020000 + 0x08 + 3 * 0x14
constexpr uint32_t DMAMUX_RM_CH1CFG = 0x40020804;
constexpr uint32_t FMC_WS = 0x40022000;
constexpr uint32_t FMC_KEY = 0x40022004;
constexpr uint32_t FMC_STAT = 0x4002200C;
constexpr uint32_t FMC_CTL = 0x40022010;
constexpr uint32_t FMC_ADDR = 0x40022014;

// docs/history/notes/oem-can-sequences.md §7
constexpr uint32_t CAN_CTL = 0x40006400;
constexpr uint32_t CAN_STAT = 0x40006404;
constexpr uint32_t CAN_TSTAT = 0x40006408;
constexpr uint32_t CAN_RFIFO0 = 0x4000640C;
constexpr uint32_t CAN_INTEN = 0x40006414;
constexpr uint32_t CAN_ERR = 0x40006418;  // not used by stock; bxCAN layout [I]
constexpr uint32_t CAN_BT = 0x4000641C;
constexpr uint32_t can_tmi(int n) { return 0x40006580u + 16u * uint32_t(n); }
constexpr uint32_t can_tmp(int n) { return 0x40006584u + 16u * uint32_t(n); }
constexpr uint32_t can_tmdata0(int n) { return 0x40006588u + 16u * uint32_t(n); }
constexpr uint32_t can_tmdata1(int n) { return 0x4000658Cu + 16u * uint32_t(n); }
constexpr uint32_t CAN_RFIFOMI0 = 0x400065B0;
constexpr uint32_t CAN_RFIFOMP0 = 0x400065B4;
constexpr uint32_t CAN_RFIFOMDATA00 = 0x400065B8;
constexpr uint32_t CAN_RFIFOMDATA10 = 0x400065BC;
constexpr uint32_t CAN_FCTL = 0x40006600;
constexpr uint32_t CAN_FMCFG = 0x40006604;
constexpr uint32_t CAN_FSCFG = 0x4000660C;
constexpr uint32_t CAN_FAFIFO = 0x40006614;
constexpr uint32_t CAN_FW = 0x4000661C;
constexpr uint32_t CAN_F0DATA0 = 0x40006640;
constexpr uint32_t CAN_F0DATA1 = 0x40006644;

// docs/history/notes/oem-adc-flash-sequences.md §2, §4
constexpr uint32_t ADC_STAT = 0x40012400;
constexpr uint32_t ADC_CTL0 = 0x40012404;
constexpr uint32_t ADC_CTL1 = 0x40012408;
constexpr uint32_t ADC_SAMPT0 = 0x4001240C;
constexpr uint32_t ADC_RSQ0 = 0x4001242C;
constexpr uint32_t ADC_RSQ2 = 0x40012434;
constexpr uint32_t ADC_RDATA = 0x4001244C;
constexpr uint32_t ADC_CCTL = 0x400124C0;
constexpr uint32_t PMU_CTL0 = 0x40007000;
// RTC backup registers from RTC + 0x50. BKP1 is the stock app's update flag, the bootloader writes BKP2 and BKP3
// (docs/history/notes/oem-hw-map.md §6.1); BKP4 is ours.
constexpr uint32_t RTC_BKP4 = 0x40002860;

// docs/history/notes/oem-usart-sequences.md §3.5, §7
constexpr uint32_t USART1_CTL0 = 0x40004400;
constexpr uint32_t USART1_CTL1 = 0x40004404;
constexpr uint32_t USART1_CTL2 = 0x40004408;
constexpr uint32_t USART1_BAUD = 0x4000440C;
constexpr uint32_t USART1_STAT = 0x4000441C;
constexpr uint32_t USART1_INTC = 0x40004420;
constexpr uint32_t USART1_RDATA = 0x40004424;
constexpr uint32_t USART1_TDATA = 0x40004428;

constexpr uint32_t FWDGT_CTL = 0x40003000;
constexpr uint32_t FWDGT_PSC = 0x40003004;
constexpr uint32_t FWDGT_RLD = 0x40003008;
constexpr uint32_t FWDGT_STAT = 0x4000300C;

constexpr uint32_t SLCD_CTL = 0x40002400;
constexpr uint32_t SLCD_CFG = 0x40002404;
constexpr uint32_t SLCD_STAT = 0x40002408;
constexpr uint32_t slcd_data(int com) { return 0x40002414u + 8u * uint32_t(com); }

constexpr uint32_t GPIOA = 0x48000000;
constexpr uint32_t GPIOB = 0x48000400;
constexpr uint32_t GPIOC = 0x48000800;
constexpr uint32_t GPIOD = 0x48000C00;
constexpr uint32_t GPIO_CTL = 0x00;
constexpr uint32_t GPIO_OMODE = 0x04;
constexpr uint32_t GPIO_OSPD = 0x08;
constexpr uint32_t GPIO_PUD = 0x0C;
constexpr uint32_t GPIO_ISTAT = 0x10;
constexpr uint32_t GPIO_BOP = 0x18;
constexpr uint32_t GPIO_AFSEL0 = 0x20;
constexpr uint32_t GPIO_AFSEL1 = 0x24;
constexpr uint32_t GPIO_BC = 0x28;

constexpr uint32_t SYST_CSR = 0xE000E010;
constexpr uint32_t SYST_RVR = 0xE000E014;
constexpr uint32_t SYST_CVR = 0xE000E018;
constexpr uint32_t NVIC_ISER0 = 0xE000E100;
constexpr uint32_t NVIC_ICER0 = 0xE000E180;
constexpr uint32_t SCB_AIRCR = 0xE000ED0C;

}  // namespace hw
