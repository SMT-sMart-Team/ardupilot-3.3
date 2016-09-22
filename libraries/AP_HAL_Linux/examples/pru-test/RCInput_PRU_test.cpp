
#define CONFIG_HAL_BOARD HAL_BOARD_LINUX

#include "RCInput_PRU_test.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>




// AB ZhaoYJ @2016-09-13 for testing multi-pwm
#ifdef MULTI_PWM
#define  TEST_MULTI_PWM
#endif


void LinuxRCInput_PRU::init(void*)
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd == -1) {
        ::printf("Unable to open /dev/mem");
    }
    ring_buffer = (volatile struct ring_buffer*) mmap(0, sizeof(ring_buffer) + 0x100, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCIN_PRUSS_SHAREDRAM_BASE);
    close(mem_fd);
    ring_buffer->ring_head = 0;
    _s0_time = 0;

    // no need any more for SMT
    // enable the spektrum RC input power
    // hal.gpio->pinMode(BBB_P8_17, HAL_GPIO_OUTPUT);
    // hal.gpio->write(BBB_P8_17, 1);
}

/*
  process a PPM-sum pulse of the given width
 */
void LinuxRCInput_PRU::_process_ppmsum_pulse(uint16_t width_usec)
{
}

void LinuxRCInput_PRU::_process_rc_pulse(uint16_t width_s0, uint16_t width_s1)
{
}

void LinuxRCInput_PRU::_timer_tick()
{
#ifdef TEST_MULTI_PWM
    static uint16_t test_cnt = 0;
    uint8_t rcin_multi_pwm = 1;
#endif
    {

        uint8_t chn_num = 0;

        // if we have reached the maximum supported channels then
        // mark as unsynchronised, so we wait for a wide pulse
        for (uint8_t i=0; i<MAX_RCIN_NUM; i++) {

            uint16_t width_usec = ring_buffer->multi_pwm_out[i].high;

            // valid pwm
            if (width_usec < 700 || width_usec > 2300) {
                // take a reading for the current channel
                // move to next channel
                continue;
            }
            chn_num++;
        }

#ifdef TEST_MULTI_PWM
            if(!((test_cnt++)%2000))
            {
                printf("dump pwm chs: \n");
                for (uint8_t i=0; i<MAX_RCIN_NUM; i++) {
                    printf("ch[%d]: %d\n", i, ring_buffer->multi_pwm_out[i].high);
                }
                printf("=========================== \n");
                sleep(2);
            }
#endif
    }
}

LinuxRCInput_PRU pruintest;



int main(void)
{
    unsigned int ii = 0;
    printf("enter pruin_test...\n");
    pruintest.init(NULL);

    // first write magic head to pwmpru, then wait for resp
    // then change mask for enable
    while(1)
    {

        pruintest._timer_tick();
        // usleep(200);
    }
}

