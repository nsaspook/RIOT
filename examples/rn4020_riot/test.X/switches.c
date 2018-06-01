

#include "app.h"
#include "config.h"
#include "timers.h"

extern rn4020_appdata_t rn4020_appdata;

/* Switches state machine */

void rn4020_switch_tasks(void)
{
    //Check if switches have changed and debounce timers are expired
    if (rn4020_appdata.sw1changed && rn4020_timerdone(TMR_SW1_DEBOUNCE)) {

        rn4020_appdata.sw1changed = false;      //clear individual flag
        rn4020_appdata.sendswitches = true;     //set group flag to request TX
    }
    if (rn4020_appdata.sw2changed && rn4020_timerdone(TMR_SW2_DEBOUNCE)) {

        rn4020_appdata.sw2changed = false;
        rn4020_appdata.sendswitches = true;
    }
    if (rn4020_appdata.sw3changed && rn4020_timerdone(TMR_SW3_DEBOUNCE)) {

        rn4020_appdata.sw3changed = false;
        rn4020_appdata.sendswitches = true;
    }
    if (rn4020_appdata.sw4changed && rn4020_timerdone(TMR_SW4_DEBOUNCE)) {

        rn4020_appdata.sw4changed = false;
        rn4020_appdata.sendswitches = true;
    }
}

//Change notification interrupt
//Process and start debounce timers for switch changes
//The switches are well debounced in hardware
//Adding the software debounce limits unneeded switch update messages
//and groups together multiple switch presses that occur within the debounce period

void _cn_interrupt(void)
{}
