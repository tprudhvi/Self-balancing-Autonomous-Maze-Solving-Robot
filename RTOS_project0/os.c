#include <stdint.h>
#include "os.h"
#include "CortexM.h"
#include "BSP.h"
#include "../inc/tm4c123gh6pm.h"


#define NUMTHREADS  8        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread



void StartOS(void);	 // function definitions in osasm.s
void static runperiodicevents(void);



struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
	int32_t *blockpt;	 // nonzero if blocked on this semaphore
	uint32_t sleep;    // nonzero if this thread is sleeping
	uint8_t priority;	 // 0 is highest, 254 lowest
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

struct pertcb{
	void(*thread)(void);
	uint32_t period;
	int32_t counter;
};
typedef struct pertcb pertcbType;
pertcbType pertcbs[NUMPERIODIC];
uint32_t eventNumber; // Static counter to add new events in correct position of the array


// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void){
  //DisableInterrupts();
  BSP_Clock_InitFastest();// set processor clock to fastest speed
	BSP_PeriodicTask_Init(runperiodicevents, 1000, 3); // Initialize a HW interrupt to run periodic events at 1000 [Hz]
	eventNumber = 0; // Initially there are 0 periodic events
}

void SetInitialStack(int i){
	tcbs[i].sp = &Stacks[i][STACKSIZE - 16];	// thread stack pointer
	Stacks[i][STACKSIZE - 1] = 0x01000000;  // thumb bit
	Stacks[i][STACKSIZE - 3] = 0x14141414;	// R14
	Stacks[i][STACKSIZE - 4] = 0x12121212;  // R12
	Stacks[i][STACKSIZE - 5] = 0x03030303;	// R3
	Stacks[i][STACKSIZE - 6] = 0x02020202;	// R2
	Stacks[i][STACKSIZE - 7] = 0x01010101;	// R1
	Stacks[i][STACKSIZE - 8] = 0x00000000;	// R0
	Stacks[i][STACKSIZE - 9] = 0x11111111;	// R11
	Stacks[i][STACKSIZE - 10] = 0x10101010;	// R10
	Stacks[i][STACKSIZE - 11] = 0x09090909;	// R9
	Stacks[i][STACKSIZE - 12] = 0x08080808;	// R8
	Stacks[i][STACKSIZE - 13] = 0x07070707;	// R7
	Stacks[i][STACKSIZE - 14] = 0x06060606;	// R6
	Stacks[i][STACKSIZE - 15] = 0x05050505;	// R5
}

//******** OS_AddThreads ***************
// Add eight main threads to the scheduler
// Inputs: function pointers to eight void/void main threads
//         priorites for each main thread (0 highest)
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void), uint32_t p0,
                  void(*thread1)(void), uint32_t p1,
                  void(*thread2)(void), uint32_t p2,
                  void(*thread3)(void), uint32_t p3,
                  void(*thread4)(void), uint32_t p4,
                  void(*thread5)(void), uint32_t p5,
                  void(*thread6)(void), uint32_t p6,
                  void(*thread7)(void), uint32_t p7){
	
	int32_t status;
	status = StartCritical();
	
	// initialize TCB circular list
	tcbs[0].next = &tcbs[1];	// 0 points 1
	tcbs[1].next = &tcbs[2];	// 1 points 2
	tcbs[2].next = &tcbs[3];	// 2 points 3
	tcbs[3].next = &tcbs[4];	// 3 points 4
	tcbs[4].next = &tcbs[5];	// 4 points 5
	tcbs[5].next = &tcbs[6];	// 5 points 6
	tcbs[6].next = &tcbs[7];	// 6 points 7
	tcbs[7].next = &tcbs[0];	// 7 points 0

  // initialize threads as not blocked
  tcbs[0].blockpt = 0;
	tcbs[1].blockpt = 0;
	tcbs[2].blockpt = 0;
	tcbs[3].blockpt = 0;
	tcbs[4].blockpt = 0;
	tcbs[5].blockpt = 0;
	tcbs[6].blockpt = 0;
	tcbs[7].blockpt = 0;
													
	// initialize threads as not sleeping
	tcbs[0].sleep = 0;
	tcbs[1].sleep = 0;
	tcbs[2].sleep = 0;
	tcbs[3].sleep = 0;
	tcbs[4].sleep = 0;
	tcbs[5].sleep = 0;
	tcbs[6].sleep = 0;
	tcbs[7].sleep = 0;
	
	// initialize thread priorities
	tcbs[0].priority = p0;
	tcbs[1].priority = p1;
	tcbs[2].priority = p2;
	tcbs[3].priority = p3;
	tcbs[4].priority = p4;
	tcbs[5].priority = p5;
	tcbs[6].priority = p6;
	tcbs[7].priority = p7;
			
  // initialize stacks, including initial PC										
	SetInitialStack(0);	Stacks[0][STACKSIZE - 2] = (int32_t)(thread0);
	SetInitialStack(1);	Stacks[1][STACKSIZE - 2] = (int32_t)(thread1);
	SetInitialStack(2);	Stacks[2][STACKSIZE - 2] = (int32_t)(thread2);
	SetInitialStack(3);	Stacks[3][STACKSIZE - 2] = (int32_t)(thread3);
	SetInitialStack(4);	Stacks[4][STACKSIZE - 2] = (int32_t)(thread4);
	SetInitialStack(5);	Stacks[5][STACKSIZE - 2] = (int32_t)(thread5);
	SetInitialStack(6);	Stacks[6][STACKSIZE - 2] = (int32_t)(thread6);
	SetInitialStack(7);	Stacks[7][STACKSIZE - 2] = (int32_t)(thread7);
	
										
	// initialize RunPt
	RunPt = &tcbs[0];	  // Thread 0 will run first
	
	EndCritical(status);
 
  return 1;               // successful
}

//******** OS_AddPeriodicEventThread ***************
// Add one background periodic event thread
// Typically this function receives the highest priority
// Inputs: pointer to a void/void event thread function
//         period given in units of OS_Launch (Lab 3 this will be msec)
// Outputs: 1 if successful, 0 if this thread cannot be added
// It is assumed that the event threads will run to completion and return
// It is assumed the time to run these event threads is short compared to 1 msec
// These threads cannot spin, block, loop, sleep, or kill
// These threads can call OS_Signal
// In Lab 3 this will be called exactly twice
int OS_AddPeriodicEventThread(void(*thread)(void), uint32_t period){
	if (eventNumber >= NUMPERIODIC)	{ // pertcbs array full
		return 0; // thread cannot be added
	}
	else { 
		pertcbs[eventNumber].thread = thread;
		pertcbs[eventNumber].period = period;
		pertcbs[eventNumber].counter = period; // initially set to its period
		eventNumber++;
		
		return 1; // thread added successfully
	}
}

void static runperiodicevents(void){
	uint32_t i;
	for (i = 0; i < eventNumber; ++i) { // search periodic threads
		pertcbs[i].counter--; // decrease periodic counter
		
		if (pertcbs[i].counter == 0) { // run periodic tasks that are ready
			pertcbs[i].thread();
			pertcbs[i].counter = pertcbs[i].period; // reset its counter
		}
	}
	
	for (i = 0; i < NUMTHREADS; ++i) {
		if (tcbs[i].sleep != 0) { // search for sleeping threads
			tcbs[i].sleep--; // decrease sleeping counter
		}
	}
}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  STCTRL = 0;                  // disable SysTick during setup
  STCURRENT = 0;               // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = theTimeSlice - 1; // reload value
  STCTRL = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}

void Scheduler(void){      // every time slice
	int32_t max = 255;   // max
	tcbType *pt;
	tcbType *bestpt;
	
	pt = RunPt;   // search for highest thread not blocked or sleeping
	do {
		pt = pt->next;   // skip at least one
		if ((pt->blockpt == 0) && (pt->sleep == 0) && (pt->priority < max)) {
			max = pt->priority;
			bestpt = pt;
		}
	} while (RunPt != pt);   // look at all possible threads
	
	RunPt = bestpt;   // update running thread
}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void){
	STCURRENT = 0;        // any write to current clears it
  INTCTRL = 0x04000000; // trigger SysTick
												// next thread gets a full time slice
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
	RunPt->sleep = sleepTime; // set sleep parameter in TCB
	OS_Suspend(); // suspend, stops running
}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
	*semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
	DisableInterrupts();
	
	*semaPt = *semaPt - 1; // decrement semaphore
	if (*semaPt < 0) {
		RunPt->blockpt = semaPt; // assign reason why it is blocked
		EnableInterrupts();
		OS_Suspend(); // run thread switcher
	}
	
	EnableInterrupts();
}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
	tcbType *pt;
	DisableInterrupts();
	
	*semaPt = *semaPt + 1; // increment semaphore
	if (*semaPt <= 0) {
		pt = RunPt->next; // search for one blocked on this semaphore
		while (pt->blockpt != semaPt) {
			pt = pt->next;
		}
		pt->blockpt = 0; // wakeup found thread
	}
	
	EnableInterrupts();
}

#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  The "put" and "get" indices initially
// are equal, which means that the FIFO is empty.  Also
// initialize semaphores to track properties of the FIFO
// such as size and busy status for Put and Get operations,
// which is important if there are multiple data producers
// or multiple data consumers.
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
	PutI = GetI = 0; // initially empty
	OS_InitSemaphore(&CurrentSize, 0); // initialize semaphore for FIFO queue
	LostData = 0; // initially no lost data
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is putting data into the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
	if (CurrentSize == FSIZE) { // queue full
		LostData++;
		
		return -1; // failure
	}
	else { // queue available
		Fifo[PutI] = data; // put
		PutI = (PutI + 1) % FSIZE; // place to put next
		OS_Signal(&CurrentSize); // semaphore signal
		
		return 0; // success
	}
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.  Consider using a unique
// semaphore to wait on busy status if more than one thread
// is getting data from the FIFO and there is a chance that
// this function may interrupt itself.
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
	OS_Wait(&CurrentSize); // block if queue is empty
	data = Fifo[GetI]; // get
	GetI = (GetI + 1) % FSIZE; // place to get next
	
  return data;
}
