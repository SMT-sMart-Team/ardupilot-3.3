
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput_PRU.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <signal.h>
using namespace Linux;


#define PWM_CHAN_COUNT 12

static const uint8_t chan_pru_map[]= {10,8,11,9,7,6,5,4,3,2,1,0};                //chan_pru_map[CHANNEL_NUM] = PRU_REG_R30/31_NUM;
static const uint8_t pru_chan_map[]= {11,10,9,8,7,6,5,4,1,3,0,2};                //pru_chan_map[PRU_REG_R30/31_NUM] = CHANNEL_NUM;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
static void catch_sigbus(int sig)
{
    hal.scheduler->panic("RCOutput.cpp:SIGBUS error gernerated\n");
}
void LinuxRCOutput_PRU::init(void* machtnicht)
{
    uint32_t mem_fd;
    signal(SIGBUS,catch_sigbus);
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE, 
                                            MAP_SHARED, mem_fd, RCOUT_PRUSS_SHAREDRAM_BASE);
    if(MAP_FAILED == sharedMem_cmd)
    {
        hal.scheduler->panic("Failed to mmap PRU1 SHM\n");
    }
    close(mem_fd);

    // all outputs default to 50Hz, the top level vehicle code
    // overrides this when necessary
    set_freq(0xFFFFFFFF, 50);
}

void LinuxRCOutput_PRU::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    uint8_t i;
    unsigned long tick=TICK_PER_S/(unsigned long)freq_hz;

    for (i=0;i<PWM_CHAN_COUNT;i++) {
        if (chmask & (1U<<i)) {
            sharedMem_cmd->periodhi[chan_pru_map[i]][0]=tick;
        }
    }
#ifdef  SET_MAGIC_SYNC
    set_magic_sync();
#endif
}

uint16_t LinuxRCOutput_PRU::get_freq(uint8_t ch)
{
    return TICK_PER_S/sharedMem_cmd->periodhi[chan_pru_map[ch]][0];
}

void LinuxRCOutput_PRU::enable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void LinuxRCOutput_PRU::disable_ch(uint8_t ch)
{
    sharedMem_cmd->enmask &= !(1U<<chan_pru_map[ch]);
#ifdef  SET_MAGIC_SYNC
    set_magic_sync();
#endif
}

void LinuxRCOutput_PRU::write(uint8_t ch, uint16_t period_us)
{
    sharedMem_cmd->periodhi[chan_pru_map[ch]][1] = TICK_PER_US*period_us;
}

void LinuxRCOutput_PRU::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        write(ch+i,period_us[i]);
    }
#ifdef  SET_MAGIC_SYNC
    set_magic_sync();
#endif
}

uint16_t LinuxRCOutput_PRU::read(uint8_t ch)
{
    return (sharedMem_cmd->hilo_read[chan_pru_map[ch]][1]/TICK_PER_US);
}

void LinuxRCOutput_PRU::read(uint16_t* period_us, uint8_t len)
{
    uint8_t i;
    if(len>PWM_CHAN_COUNT){
        len = PWM_CHAN_COUNT;
    }
    for(i=0;i<len;i++){
        period_us[i] = sharedMem_cmd->hilo_read[chan_pru_map[i]][1]/TICK_PER_US;
    }
}


void LinuxRCOutput_PRU::set_magic_sync(void)
{
#if 0
	static char first_time = 1;
	if (first_time == 1)
	{
       sharedMem_cmd->magic = PWM_CMD_MAGIC;
       first_time = 0;
	}
	else
	{
		if (sharedMem_cmd->magic != PWM_REPLY_MAGIC )
			printf("11111\n");
	}
#endif
#ifdef  SET_MAGIC_SYNC
    sharedMem_cmd->magic = PWM_CMD_MAGIC;
#endif
}


// will be invoked 50Hz, meanwhile PRU will check alive 1Hz
void LinuxRCOutput_PRU::rcout_keep_alive(void)
{
#ifdef KEEP_ALIVE_WITH_PRU
    static unsigned int time_out = 0;
    static unsigned int wait_pru_time = 0;
    // check keep alive with PRU (first time: config timeout to PRU)
    if(time_out > 1)
    {
        // reply alive
        if(PWM_REPLY_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            // cmd alive
            sharedMem_cmd->keep_alive = PWM_CMD_KEEP_ALIVE; 
            time_out = 2;
        }
        else if(PWM_CMD_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            time_out++;
            // PRU should be dead
            if(time_out > (KEEP_ALIVE_TIME_OUT_HOST*50))
            {
                ::printf("Warning: PRU didn't reply for more than %d seconds (50Hz: %d), should be dead!\n", KEEP_ALIVE_TIME_OUT_HOST, time_out);
                time_out = 2;
            }
        }
        else
        {
            ::printf("Error: unknown PRU keep alive code 0x%08x!\n", sharedMem_cmd->keep_alive & 0xFFFF);
        }
    }
    else if(1 == time_out) // wait for 1st PRU reply (PRU wake up)
    {
        // reply alive
        if(PWM_REPLY_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            // cmd alive
            sharedMem_cmd->keep_alive = PWM_CMD_KEEP_ALIVE; 
            time_out = 2;
        }
        else if(PWM_CMD_KEEP_ALIVE == (sharedMem_cmd->keep_alive & 0xFFFF))
        {
            sharedMem_cmd->time_out = KEEP_ALIVE_TIME_OUT_PRU; 
            wait_pru_time++; 
            if(wait_pru_time > (PRU_POWER_UP_TIME*50))
            {
                wait_pru_time = 0;
                ::printf("Warning: PRU still not wakeup...\n");
            }
        }
        else
        {
            ::printf("Warning: unknown PRU keep alive code!\n");
        }
    }
    else // time_out == 0
    {
        sharedMem_cmd->time_out = KEEP_ALIVE_TIME_OUT_PRU; 
        sharedMem_cmd->keep_alive = PWM_CMD_KEEP_ALIVE; 
        time_out = 1;
    }
#endif
}

#endif
