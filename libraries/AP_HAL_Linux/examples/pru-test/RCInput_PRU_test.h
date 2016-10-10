
#ifndef __AP_HAL_LINUX_RCINPUT_PRU_H__
#define __AP_HAL_LINUX_RCINPUT_PRU_H__


/*
  This class implements RCInput on the BeagleBoneBlack with a PRU
  doing the edge detection of the PPM sum input
 */


#define RCIN_PRUSS_SHAREDRAM_BASE   0x4a312000
// we use 300 ring buffer entries to guarantee that a full 25 byte
// frame of 12 bits per byte
//
// AB ZhaoYJ for trying to add PPMSUM decoding in PRU @2016-05-21

#define OK 0xbeef
#define KO 0x4110 /// !beef

// AB ZhaoYJ for multi-pwm to replace ppm-sum @2016-09-13
#define MULTI_PWM

#ifdef MULTI_PWM
#define MAX_RCIN_NUM 8
#define NUM_RCIN_BUFF 64
#endif

typedef unsigned char uint8_t;
typedef  unsigned short uint16_t;
typedef  unsigned int uint32_t;

class LinuxRCInput_PRU 
{
public:
    void init(void*);
    void _timer_tick(void);
    void _process_rc_pulse(uint16_t width_s0, uint16_t width_s1);
    void _process_ppmsum_pulse(uint16_t width_usec);
    /* AB ZhaoYJ@2016-09-13 for direct pwm*/
    void set_direct_pwm(uint8_t v) { _direct_pwm = v; };

 private:
    // static const unsigned int NUM_RING_ENTRIES=256;
    static const unsigned int NUM_RING_ENTRIES=300;
    // shared ring buffer with the PRU which records pin transitions
    struct ring_buffer {
        volatile uint16_t ring_head; // owned by ARM CPU
        volatile uint16_t ring_tail; // owned by the PRU
        struct {
               uint16_t pin_value;
               uint16_t delta_t;
        } buffer[NUM_RING_ENTRIES];
#ifdef MULTI_PWM 
    volatile struct {
        volatile uint16_t high;
        volatile uint16_t low;
    }multi_pwm_out[MAX_RCIN_NUM];
#endif
    };
    volatile struct ring_buffer *ring_buffer;



    // time spent in the low state
    uint16_t _s0_time;
#ifdef MULTI_PWM 
    uint8_t _direct_pwm;
#endif
};

#endif // __AP_HAL_LINUX_RCINPUT_PRU_H__
