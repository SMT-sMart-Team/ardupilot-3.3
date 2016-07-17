#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "ToneAlarmDriver.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#ifdef SMT_GPIO_ALARM
#include "GPIO.h"

#define TIME16_SUB(x, y) ((x >=y)?(x - y):(0xFFFF - y + x))

#define ALARM_GPIO BBB_P8_36
#define ALARM_ON  1
#define ALARM_OFF 0 

using namespace Linux;

extern const AP_HAL::HAL& hal;

//List of RTTTL tones 
uint16_t ToneAlarm::tunes[TONE_NUMBER_OF_TUNES] = { 
                                3, // "Startup:d=8,o=6,b=480:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7",
                                2, // "Error:d=4,o=6,b=400:8a,8a,8a,p,a,a,a,p",
                                1, // "notify_pos:d=4,o=6,b=400:8e,8e,a",
                                4, // "notify_neut:d=4,o=6,b=400:8e,e",
                                5, // "notify_neg:d=4,o=6,b=400:8e,8c,8e,8c,8e,8c",
                                6, // "arming_warn:d=1,o=4,b=75:g",
                                7, // "batt_war_slow:d=4,o=6,b=200:8a",
                                8, // "batt_war_fast:d=4,o=6,b=512:8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a",
                                9, //"GPS_war:d=4,o=6,b=512:a,a,a,1f#",
                                10, // "Arm_fail:d=4,o=4,b=512:b,a,p",
                                11, // "para_rel:d=16,o=6,b=512:a,g,a,g,a,g,a,g"
                                };

ToneAlarm::ToneAlarm()
{
    tune_num = -1;                    //initialy no tune to play
    tune_comp = true;
    first_time = true;
    prev_time = 0;
    duration = 0;
}

bool ToneAlarm::init()
{
    tune_num = 0;                    //play startup tune
    // setup GPIO
    hal.gpio->pinMode(ALARM_GPIO, HAL_GPIO_OUTPUT);
    hal.gpio->write(ALARM_GPIO, ALARM_OFF);
    //printf("***************init alarm\n");
    return true;
}

void ToneAlarm::set_tune(uint8_t tone)
{
    if(tone < TONE_NUMBER_OF_TUNES)
    {
        tune_num = tone;
    }
    //printf("set alarm\n");
}

bool ToneAlarm::is_tune_comp()
{
    return tune_comp;
}

void ToneAlarm::stop()
{
    hal.gpio->write(ALARM_GPIO, ALARM_OFF);
    //printf("stop alarm\n");
}

bool ToneAlarm::play()
{
    uint16_t cur_time = hal.scheduler->millis();

    if(first_time){
        first_time = false;
        prev_time = cur_time;
        hal.gpio->write(ALARM_GPIO, ALARM_ON);
        //printf("********    \n ********%d ms: start alarm\n", cur_time);
    }
    if(TIME16_SUB(cur_time, prev_time) > (duration*1000)){
        // printf("********    \n ********%d ms: stop alarm, duration:%d\n", cur_time, duration);
        stop();
        tune_comp = true;
        tune_num = -1;
        return true;
    }
    return false;
}

bool ToneAlarm::set_note(){
    if(tune_num <0 || tune_num > TONE_NUMBER_OF_TUNES){
        return false;
    }
    if(tune_num < TONE_NUMBER_OF_TUNES)
    {
        // first, get note duration, if available
        duration = tunes[tune_num];
        //printf("set note, tune_num: %d, duration: %d\n", tune_num, duration);
        return true;
    }

    return false;

}

bool ToneAlarm::init_tune(){

    if(tune_num <0 || tune_num > TONE_NUMBER_OF_TUNES){
        return false;
    }
    tune_comp = false;
    first_time = true;
    //printf("init tune\n");
    return true;
}
#else

using namespace Linux;

extern const AP_HAL::HAL& hal;
static uint16_t notes[] = { 0,
NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_AS4, NOTE_B4,
NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5, NOTE_A5, NOTE_AS5, NOTE_B5,
NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6, NOTE_A6, NOTE_AS6, NOTE_B6,
NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_F7, NOTE_FS7, NOTE_G7, NOTE_GS7, NOTE_A7, NOTE_AS7, NOTE_B7
};

//List of RTTTL tones 
const char* ToneAlarm::tune[TONE_NUMBER_OF_TUNES] = { 
                                "Startup:d=8,o=6,b=480:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7",
                                "Error:d=4,o=6,b=400:8a,8a,8a,p,a,a,a,p",
                                "notify_pos:d=4,o=6,b=400:8e,8e,a",
                                "notify_neut:d=4,o=6,b=400:8e,e",
                                "notify_neg:d=4,o=6,b=400:8e,8c,8e,8c,8e,8c",
                                "arming_warn:d=1,o=4,b=75:g",
                                "batt_war_slow:d=4,o=6,b=200:8a",
                                "batt_war_fast:d=4,o=6,b=512:8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a",
                                "GPS_war:d=4,o=6,b=512:a,a,a,1f#",
                                "Arm_fail:d=4,o=4,b=512:b,a,p",
                                "para_rel:d=16,o=6,b=512:a,g,a,g,a,g,a,g"};
//Tune Repeat true: play rtttl tune in loop, false: play only once
bool ToneAlarm::tune_repeat[TONE_NUMBER_OF_TUNES] = {false,true,false,false,false,false,true,true,false,false,false};

ToneAlarm::ToneAlarm()
{
    period_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/period",O_WRONLY);
    duty_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/duty",O_WRONLY);
    run_fd = open("/sys/devices/ocp.3/pwm_test_P8_36.12/run",O_WRONLY);
    
    tune_num = -1;                    //initialy no tune to play
    tune_pos = 0;
}

bool ToneAlarm::init()
{
    tune_num = 0;                    //play startup tune
    if((period_fd == -1) || (duty_fd == -1) || (run_fd == -1)){
        hal.console->printf("ToneAlarm: Error!! please check if PWM overlays are loaded correctly");
        return false;
    }
    return true;
}

void ToneAlarm::set_tune(uint8_t tone)
{
    tune_num = tone;
}

bool ToneAlarm::is_tune_comp()
{
    return tune_comp;
}

void ToneAlarm::stop()
{
    dprintf(run_fd,"0");
}

bool ToneAlarm::play()
{
    uint16_t cur_time = hal.scheduler->millis();
    if(tune_num != prev_tune_num){
        tune_changed = true;
        return true;
    }
    if(cur_note != 0){
        dprintf(run_fd,"0");
        dprintf(period_fd,"%u",1000000000/cur_note);    
        dprintf(duty_fd,"%u",500000000/cur_note);
        dprintf(run_fd,"1");
        cur_note =0;
        prev_time = cur_time;
    }
    if((cur_time - prev_time) > duration){
        stop();
        if(tune[tune_num][tune_pos] == '\0'){
            if(!tune_repeat[tune_num]){
                tune_num = -1;
            }

            tune_pos = 0;
            tune_comp = true;
            return false;
        }
        return true;
    }
    return false;
}

bool ToneAlarm::set_note(){
    // first, get note duration, if available
    uint16_t scale,note,num =0;
    duration = 0;

    while(isdigit(tune[tune_num][tune_pos])){                   //this is a safe while loop as it can't go further than 
                                                                //the length of the rtttl tone string
        num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
    }
    if(num){
        duration = wholenote / num;
    } else{
        duration = wholenote / 4;  // we will need to check if we are a dotted note after
    }        
    // now get the note
    note = 0;

    switch(tune[tune_num][tune_pos]){
        case 'c':
        note = 1;
        break;
        case 'd':
        note = 3;
        break;
        case 'e':
        note = 5;
        break;
        case 'f':
        note = 6;
        break;
        case 'g':
        note = 8;
        break;
        case 'a':
        note = 10;
        break;
        case 'b':
        note = 12;
        break;
        case 'p':
        default:
        note = 0;
    }

    tune_pos++;

    // now, get optional '#' sharp
    if(tune[tune_num][tune_pos] == '#'){
        note++;
        tune_pos++;
    }

    // now, get optional '.' dotted note

    if(tune[tune_num][tune_pos] == '.'){
        duration += duration/2;
        tune_pos++;
    }

    // now, get scale

    if(isdigit(tune[tune_num][tune_pos])){
        scale = tune[tune_num][tune_pos] - '0';
        tune_pos++;
    } else{
        scale = default_oct;
    }

    scale += OCTAVE_OFFSET;

    if(tune[tune_num][tune_pos] == ','){
        tune_pos++;       // skip comma for next note (or we may be at the end)
    }
    // now play the note

    if(note){
        if(tune_changed == true){
            tune_pos =0;
            tune_changed = false;
        }
        cur_note = notes[(scale - 4) * 12 + note];
        return true;
    } else{
        cur_note = 0;
        return true;
    }
    
}

bool ToneAlarm::init_tune(){

    uint16_t num;
    default_dur = 4;
    default_oct = 6;
    bpm = 63;
    prev_tune_num = tune_num;
    if(tune_num <0 || tune_num > TONE_NUMBER_OF_TUNES){
        return false;
    }
    
    tune_comp = false;
    while(tune[tune_num][tune_pos] != ':'){
        if(tune[tune_num][tune_pos] == '\0'){
            return false;
        }
        tune_pos++;
    }
    tune_pos++;                     
    
    if(tune[tune_num][tune_pos] == 'd'){
        tune_pos+=2;             
        num = 0;

        while(isdigit(tune[tune_num][tune_pos])){
            num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
        }
        if(num > 0){ 
            default_dur = num;
        }        
        tune_pos++;                   // skip comma
    }


    // get default octave

    if(tune[tune_num][tune_pos] == 'o')
    {
        tune_pos+=2;              // skip "o="
        num = tune[tune_num][tune_pos++] - '0';
        if(num >= 3 && num <=7){
            default_oct = num;
        }
        tune_pos++;                   // skip comma
    }

    // get BPM

    if(tune[tune_num][tune_pos] == 'b'){
        tune_pos+=2;              // skip "b="
        num = 0;
        while(isdigit(tune[tune_num][tune_pos])){
            num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
        }
        bpm = num;
        tune_pos++;                   // skip colon
    }

    // BPM usually expresses the number of quarter notes per minute
    wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)
    return true;
}

#endif

#endif
