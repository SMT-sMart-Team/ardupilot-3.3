
#ifndef __AP_HAL_LINUX_RCOUTPUT_PRU_H__
#define __AP_HAL_LINUX_RCOUTPUT_PRU_H__

#define RCOUT_PRUSS_SHAREDRAM_BASE     0x4a310000
#define MAX_PWMS                 12
#define PWM_CMD_MAGIC            0xf00fbaaf
#define PWM_REPLY_MAGIC          0xbaaff00f
#define PWM_CMD_CONFIG	         0	/* full configuration in one go */
#define PWM_CMD_ENABLE	         1	/* enable a pwm */
#define PWM_CMD_DISABLE	         2	/* disable a pwm */
#define PWM_CMD_MODIFY	         3	/* modify a pwm */
#define PWM_CMD_SET	         4	/* set a pwm output explicitly */
#define PWM_CMD_CLR	         5	/* clr a pwm output explicitly */
#define PWM_CMD_TEST	         6	/* various crap */

typedef unsigned char uint8_t;
typedef  unsigned short uint16_t;
typedef  unsigned int uint32_t;

// add By ZhaoYJ for keep alive with PRU
#define PWM_CMD_KEEP_ALIVE 0xbeef
#define PWM_REPLY_KEEP_ALIVE 0x2152 // ~0xdead

#define PRU_POWER_UP_TIME  30 // second
#define KEEP_ALIVE_TIME_OUT_HOST 50 // second 
#define KEEP_ALIVE_TIME_OUT_PRU 75 // 75*20ms

class LinuxRCOutput_PRU {
    public:
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

    void rcout_keep_alive(void);

    static const int TICK_PER_US=200;
    static const int TICK_PER_S=200000000;
    struct pwm_cmd {
        uint32_t magic;
	    uint16_t enmask;	/* enable mask */

	    uint32_t periodhi[MAX_PWMS][2];
        uint32_t hilo_read[MAX_PWMS][2];
        uint16_t keep_alive; // flag, add By ZhaoYJ 
        uint16_t time_out; // second, add By ZhaoYJ 
    };
    volatile struct pwm_cmd *sharedMem_cmd;

};

#endif // __AP_HAL_LINUX_RCOUTPUT_PRU_H__
