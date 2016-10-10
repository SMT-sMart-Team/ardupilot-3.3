

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "GPIO_BBB.h"


LinuxGPIO_BBB::LinuxGPIO_BBB()
{}

void LinuxGPIO_BBB::init()
{
#if LINUX_GPIO_NUM_BANKS == 4
    int mem_fd;
    // Enable all GPIO banks
    // Without this, access to deactivated banks (i.e. those with no clock source set up) will (logically) fail with SIGBUS
    // Idea taken from https://groups.google.com/forum/#!msg/beagleboard/OYFp4EXawiI/Mq6s3sg14HoJ

    uint8_t bank_enable[3] = { 5, 65, 105 };
    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
        printf("open export done\n");
    if (export_fd == -1) {
        printf("error to export \n");
    }
    for (uint8_t i=0; i<3; i++) {
        dprintf(export_fd, "%u\n", (unsigned)bank_enable[i]);
    }
    close(export_fd);

        printf("before open mem\n");

    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
            printf("can't open /dev/mem \n");
            exit (-1);
    }

        printf("before offset \n");

    /* mmap GPIO */
    off_t offsets[LINUX_GPIO_NUM_BANKS] = { GPIO0_BASE, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE };
        printf("before bank init\n");
    for (uint8_t i=0; i<LINUX_GPIO_NUM_BANKS; i++) {
        printf("before mmap bank \n");
        gpio_bank[i].base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, offsets[i]);
        printf("after mmap bank \n");
        if ((char *)gpio_bank[i].base == MAP_FAILED) {
            printf("error to mmap bank \n");
        }
        gpio_bank[i].oe = gpio_bank[i].base + GPIO_OE;
        gpio_bank[i].in = gpio_bank[i].base + GPIO_IN;
        gpio_bank[i].out = gpio_bank[i].base + GPIO_OUT;
    }
        printf("after bank init\n");

    close(mem_fd);
#endif // LINUX_GPIO_NUM_BANKS
}

void LinuxGPIO_BBB::pinMode(uint8_t pin, uint8_t output)
{
}

int8_t LinuxGPIO_BBB::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t LinuxGPIO_BBB::read(uint8_t pin) {

    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return 0;
    }
    return *gpio_bank[bank].in & (1U<<bankpin) ? HIGH : LOW;

}

void LinuxGPIO_BBB::write(uint8_t pin, uint8_t value)
{
    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (value == LOW) {
        *gpio_bank[bank].out &= ~(1U<<bankpin);
    } else {
        *gpio_bank[bank].out |= 1U<<bankpin;
    }
}

void LinuxGPIO_BBB::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}


bool LinuxGPIO_BBB::usb_connected(void)
{
    return false;
}

