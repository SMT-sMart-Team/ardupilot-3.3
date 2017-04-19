#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI

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

#include "GPIO.h"
#include "RCInput.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

// AB ZhaoYJ @2016-09-13 for testing multi-pwm
#ifdef MULTI_PWM
// #define  TEST_MULTI_PWM
#endif

void LinuxRCInput_PRU::init(void*)
{
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (mem_fd == -1) {
        hal.scheduler->panic("Unable to open /dev/mem");
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
  called at 1kHz to check for new pulse capture data from the PRU
 */

void LinuxRCInput_PRU::_timer_tick()
{
    while (ring_buffer->ring_head != ring_buffer->ring_tail) 
    {
        if (ring_buffer->ring_tail >= NUM_RING_ENTRIES) 
        {
            // invalid ring_tail from PRU - ignore RC input
            return;
        }
        if (ring_buffer->buffer[ring_buffer->ring_head].pin_value == 1) 
        {
            // remember the time we spent in the low state
            _s0_time = ring_buffer->buffer[ring_buffer->ring_head].delta_t;
        } else {
            // the pulse value is the sum of the time spent in the low
            // and high states
            _process_rc_pulse(_s0_time, ring_buffer->buffer[ring_buffer->ring_head].delta_t);
        }
        // move to the next ring buffer entry
        ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;        
    }

#ifdef TEST_MULTI_PWM
    static uint16_t test_cnt = 0;
    uint8_t rcin_multi_pwm = 1;
#endif
#if ENABLE_COMMON_RCIN 
    uint8_t chn_num = 0;

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    for (uint8_t i=COMMON_RCIN_PIN; i<MAX_RCIN_NUM; i++) 
    {
        set_common_pwm(ring_buffer->multi_rc_in[i].high);
    }
#endif

#ifdef TEST_MULTI_PWM
    if(!((test_cnt++)%200))
    {
        printf("dump pwm chs: \n");
        for (uint8_t i=0; i<MAX_RCIN_NUM; i++) {
            printf("ch[%d]: %d\n", i, ring_buffer->multi_rc_in[i].high);
        }
        printf("=========================== \n");
    }
#endif
}


#endif // CONFIG_HAL_BOARD_SUBTYPE
