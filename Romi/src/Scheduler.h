//
//  SimpleScheduler.h
//
//
//  Created by Rodrigo Moreno on 1º4/02/2019.
//

#ifndef Scheduler_h
#define Scheduler_h

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TASKS   8

    typedef struct {
        unsigned long period;      // Rate at which the task should tick
        unsigned long lastExecution; // Time of task's last tick
        void (*TickFct)(void);     // Function to call for task's tick
    } Task_t;

    void initScheduler();
    void Scheduler();
    uint8_t createTask(void (*TickFct)(void), unsigned long period);

#ifdef __cplusplus
}
#endif

#endif /* SimpleScheduler_h */
