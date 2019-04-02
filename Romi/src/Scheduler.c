//
//  SimpleScheduler.c
//
//
//  Created by Rodrigo Moreno on 14/02/2019.
//

#include <Arduino.h>
#include "Scheduler.h"

static Task_t tasks[MAX_TASKS];

volatile unsigned long tick = 0;

volatile uint8_t update_task_flag = 0;
volatile uint8_t print_task_flag = 0;

volatile uint8_t tick_flag = 0;

void Scheduler() {
    if(tick_flag) {
        tick_flag = 0;
        for (uint8_t i=0; i < MAX_TASKS; ++i) {
            if (tasks[i].lastExecution + tasks[i].period < tick) { // Ready
                if(tasks[i].TickFct != NULL) {
                    tasks[i].TickFct(); //execute task tick
                    tasks[i].lastExecution = tick;
                }
            }
        }
    }
}

void initScheduler() {
    memset(tasks, 0, MAX_TASKS*sizeof(Task_t));

    setupTimer3();
}

uint8_t createTask(void (*TickFct)(void), unsigned long period) {
    for(uint8_t i=0; i<MAX_TASKS; i++) {
        if(tasks[i].TickFct == NULL) {
            tasks[i].TickFct = TickFct;
            tasks[i].period = period;

            return i;
        }
    }

    return 0xFF;
}

// Routine to setupt timer3 to run
void setupTimer3() {

    // disable global interrupts
    cli();

    // Reset timer3 to a blank condition.
    // TCCR = Timer/Counter Control Register
    TCCR3A = 0;     // set entire TCCR3A register to 0
    TCCR3B = 0;     // set entire TCCR3B register to 0

    // First, turn on CTC mode.  Timer3 will count up
    // and create an interrupt on a match to a value.
    // See table 14.4 in manual, it is mode 4.
    TCCR3B = TCCR3B | (1 << WGM32);

    // For a cpu clock precaler of 256:
    // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
    // Table 14.5 in manual.
    TCCR3B = TCCR3B | (1 << CS32);


    // set compare match register to desired timer count.
    // CPU Clock  = 16000000 (16mhz).
    // Prescaler  = 256
    // Timer freq = 16000000/256 = 62500
    // We can think of this as timer3 counting up to 62500 in 1 second.
    // compare match value = 62500 / 2 (we desire 2hz).
    OCR3A = 61;   //Aprox. 0.976 ms or 1024 Hz
    //OCR3A = 62500;    //Aprox. 1 s

    // enable timer compare interrupt:
    TIMSK3 = TIMSK3 | (1 << OCIE3A);

    // enable global interrupts:
    sei();

}

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {
    tick++;
    if(!tick_flag) {
        tick_flag = 1;
    }
}
