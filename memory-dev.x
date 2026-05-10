/* STM32H723ZG direct flash (no bootloader) for probe-rs debugging */
MEMORY
{
  FLASH  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM    : ORIGIN = 0x24000000, LENGTH = 320K
  CCMRAM : ORIGIN = 0x20000000, LENGTH = 128K
}
