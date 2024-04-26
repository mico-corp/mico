
#ifdef MICO_IS_RASPBIAN
/*
   tiny_gpio.c
   2016-04-30
   Public Domain
*/

/*
   gcc -o tiny_gpio tiny_gpio.c
   ./tiny_gpio
*/

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define GPSET0 7
#define GPSET1 8

#define GPCLR0 10
#define GPCLR1 11

#define GPLEV0 13
#define GPLEV1 14

#define GPPUD 37
#define GPPUDCLK0 38
#define GPPUDCLK1 39

unsigned piModel;
unsigned piRev;

static volatile uint32_t *gpioReg =
    static_cast<volatile uint32_t *>(MAP_FAILED);

#define PI_BANK (gpio >> 5)
#define PI_BIT (1 << (gpio & 0x1F))

/* gpio modes. */

#define PI_INPUT 0
#define PI_OUTPUT 1
#define PI_ALT0 4
#define PI_ALT1 5
#define PI_ALT2 6
#define PI_ALT3 7
#define PI_ALT4 3
#define PI_ALT5 2

void gpioSetMode(unsigned gpio, unsigned mode) {
  int reg, shift;

  reg = gpio / 10;
  shift = (gpio % 10) * 3;

  gpioReg[reg] = (gpioReg[reg] & ~(7 << shift)) | (mode << shift);
}

int gpioGetMode(unsigned gpio) {
  int reg, shift;

  reg = gpio / 10;
  shift = (gpio % 10) * 3;

  return (*(gpioReg + reg) >> shift) & 7;
}

/* Values for pull-ups/downs off, pull-down and pull-up. */

#define PI_PUD_OFF 0
#define PI_PUD_DOWN 1
#define PI_PUD_UP 2

void gpioSetPullUpDown(unsigned gpio, unsigned pud) {
  *(gpioReg + GPPUD) = pud;

  usleep(20);

  *(gpioReg + GPPUDCLK0 + PI_BANK) = PI_BIT;

  usleep(20);

  *(gpioReg + GPPUD) = 0;

  *(gpioReg + GPPUDCLK0 + PI_BANK) = 0;
}

int gpioRead(unsigned gpio) {
  if ((*(gpioReg + GPLEV0 + PI_BANK) & PI_BIT) != 0)
    return 1;
  else
    return 0;
}

void gpioWrite(unsigned gpio, unsigned level) {
  if (level == 0)
    *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
  else
    *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
}

void gpioTrigger(unsigned gpio, unsigned pulseLen, unsigned level) {
  if (level == 0)
    *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
  else
    *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;

  usleep(pulseLen);

  if (level != 0)
    *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
  else
    *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
}

/* Bit (1<<x) will be set if gpio x is high. */

uint32_t gpioReadBank1(void) { return (*(gpioReg + GPLEV0)); }
uint32_t gpioReadBank2(void) { return (*(gpioReg + GPLEV1)); }

/* To clear gpio x bit or in (1<<x). */

void gpioClearBank1(uint32_t bits) { *(gpioReg + GPCLR0) = bits; }
void gpioClearBank2(uint32_t bits) { *(gpioReg + GPCLR1) = bits; }

/* To set gpio x bit or in (1<<x). */

void gpioSetBank1(uint32_t bits) { *(gpioReg + GPSET0) = bits; }
void gpioSetBank2(uint32_t bits) { *(gpioReg + GPSET1) = bits; }

unsigned gpioHardwareRevision(void) {
  static unsigned rev = 0;

  FILE *filp;
  char buf[512];
  char term;
  int chars = 4; /* number of chars in revision string */

  if (rev)
    return rev;

  piModel = 0;

  filp = fopen("/proc/cpuinfo", "r");

  if (filp != NULL) {
    while (fgets(buf, sizeof(buf), filp) != NULL) {
      if (piModel == 0) {
        if (!strncasecmp("model name", buf, 10)) {
          if (strstr(buf, "ARMv6") != NULL) {
            piModel = 1;
            chars = 4;
          } else if (strstr(buf, "ARMv7") != NULL) {
            piModel = 2;
            chars = 6;
          } else if (strstr(buf, "ARMv8") != NULL) {
            piModel = 2;
            chars = 6;
          }
        }
      }

      if (!strncasecmp("revision", buf, 8)) {
        if (sscanf(buf + strlen(buf) - (chars + 1), "%x%c", &rev, &term) == 2) {
          if (term != '\n')
            rev = 0;
        }
      }
    }

    fclose(filp);
  }
  return rev;
}

int gpioInitialise(void) {
  int fd;

  piRev = gpioHardwareRevision(); /* sets piModel and piRev */

  fd = open("/dev/gpiomem", O_RDWR | O_SYNC);

  if (fd < 0) {
    fprintf(stderr, "failed to open /dev/gpiomem\n");
    return -1;
  }

  gpioReg =
      (uint32_t *)mmap(NULL, 0xB4, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  close(fd);

  if (gpioReg == MAP_FAILED) {
    fprintf(stderr, "Bad, mmap failed\n");
    return -1;
  }
  return 0;
}

#endif