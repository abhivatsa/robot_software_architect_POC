#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>     /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h>    /* sched_setscheduler() */

#include "ecrt.h"

#include ""

#define PERIOD_NS (1000000)

//forces OS alot pages for this application so future fault does not happen
#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is  \
                                     guranteed safe to access without \
                                     faulting */

#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

// EtherCAT variables
static ec_master_t *master = NULL;

static ec_domain_t *domain1 = NULL;

// process data
static uint8_t *domain1_pd = NULL;

#define synapticon_circulo9 0x000022d2, 0x00000301 // Vendor Id, Product Code

#define synapticon_circulo9_pos0 0, 0 // Alias, Position

//Define a datastructure where we can access realtime information about the synapticon_circulo9
static struct 
{
    unsigned int statusword;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int controlword;
    unsigned int modes_of_operation;
    unsigned int target_position;
    unsigned int target_velocity;
} offset

//List record type for mass registering in the first domain
//And also passing memory address for further use
const static ec_pdo_entry_reg_t domain1_registers[] = {
	{synapticon_circulo9_pos0, synapticon_circulo9, 0x6041, 0, &offset.statusword},             // 6041 0 statusword
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x6064, 0, &offset.position_actual_value},  // 6064 0 pos_act_val
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x606C, 0, &offset.velocity_actual_value},  // 606C 0 vel_act_val
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x6040, 0, &offset.controlword},            // 6040 0 control word
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x6060, 0, &offset.modes_of_operation},     // 6060 0 mode_of_operation
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x607A, 0, &offset.target_position},        // 607A 0 target position
    {synapticon_circulo9_pos0, synapticon_circulo9, 0x60FF, 0, &offset.target_velocity},        // 60FF 0 target velocity
}

//Group these pdo entry for first slave so that they can later be grouped under Transfer or receive PDO
ec_pdo_entry_info_t slave1_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x6060, 0x00, 8}, /* Modes of operation */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    
};

//assume receving and transmit data with respect to slave
//PDO entries grouped under 0x1600 PDO will used by slave for receiving data
//PDO entries grouped under 0x1a00 PDO will used by slave for transmitting data to master
ec_pdo_info_t slave1_pdos[] = {
    {0x1600, 3, slave1_pdo_entries + 0}, /* Receive PDO1 mapping */
    {0x1601, 1, slave1_pdo_entries + 3}, /* Receive PDO2 mapping */
    
    {0x1a00, 2, slave1_pdo_entries + 4}, /* Transmit PDO1 mapping */
    {0x1a01, 1, slave1_pdo_entries + 6}, /* Transmit PDO2 mapping */
    
};

//assume syncmaster lies with master
//Configuring the synmanagers with respect to slave1_pdos 
//EC_DIR_INPUT->slave to master
//EC_DIR_OUTPUT->master to slave
ec_sync_info_t slave1_syncsM[] = {
	{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
    {2, EC_DIR_OUTPUT, 2, slave1_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave1_pdos + 2, EC_WD_DISABLE},//no need for watchdog for receiving data , only concerned with sending data within time limit.
    {0xff}
};

//Watchdog Enabled: With the watchdog enabled, the synchronization manager
//will monitor the timing of data exchanges. If the master does not send
//data within the expected interval, the watchdog will trigger an 
//error or take predefined safety actions.






void cyclic_task(){




	//Do work with domain1_pd(address of all data mapped to this domain 1)
	//basically receiving the PDO from slaves and then sending PDO to slaves
}
void stack_prefault(){
	//forcing the OS to use extra pages for the initialzing of the 
	//stack defined in this func. which will locked due to use of
	//mlockall(MCL_CURRENT | MCL_FUTURE) , This (MCL_FUTURE) .
}





//initialing the ethercat protocol and executing it.
int main(int argc, char **argv)
{
	//slave reference
    ec_slave_config_t *slave_pointer;
    //used for time management
    struct timespec time_pointer;
    int ret = 0;

    //reserving the first master (0th index) for use in this application
    master = ecrt_request_master(0);
    if (!master)
    {
        return -1;
    }

    //creates a slave config. object using the data of the circulo 
    if (!(slave_pointer = ecrt_master_slave_config(
              master, synapticon_circulo9_pos0, synapticon_circulo9)))
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    //Now we config. the slave object such that we can modify the slaves
    //with our config. by sending SDOs when the master is activated.
    //This is done by wiping out the default PDO of the slaves and replacing
    //it with our own PDOs

    //Config for RxPDO  ---------------------------------------
    //clearing RxPDO 
    ecrt_slave_config_sdo8( slave_pointer, 0x1C12, 0, 0 ); /* clear sm pdo 0x1c12 (SM index '2' in ec_sync_info_t) */
    ecrt_slave_config_sdo8( slave_pointer, 0x1600, 0, 0 ); /* clear RxPdo 0x1600 */
    ecrt_slave_config_sdo8( slave_pointer, 0x1601, 0, 0 ); /* clear RxPdo 0x1601 */
    ecrt_slave_config_sdo8( slave_pointer, 0x1602, 0, 0 ); /* clear RxPdo 0x1602 */
    ecrt_slave_config_sdo8( slave_pointer, 0x1603, 0, 0 ); /* clear RxPdo 0x1603 */

    //Assigning RxPDO
    ecrt_slave_config_sdo32( slave_pointer, 0x1600, 1, 0x6040 ); /* 0x6040:0/16bits, control word */
    ecrt_slave_config_sdo32( slave_pointer, 0x1600, 2, 0x6060); /* 0x6060:1/8bits Modes of operation xxx */
    ecrt_slave_config_sdo8( slave_pointer, 0x1600, 0, 2 ); /* set number of PDO entries for 0x1600 */

    ecrt_slave_config_sdo32( slave_pointer, 0x1601, 1, 0x607a ); /* 0x607a:0/32bits, target position */
    ecrt_slave_config_sdo32( slave_pointer, 0x1601, 2, 0x60ff ); /* 0x60ff:1/32bits target velocity xxx */
    ecrt_slave_config_sdo8( slave_pointer, 0x1601, 0, 2 ); /* set number of PDO entries for 0x1601 */

    //inputting 0x1600 and 0x1601 into SM 0x1C12
    ecrt_slave_config_sdo16( slave_pointer, 0x1C12, 1, 0x1600 ); /* list all RxPdo in 0x1C12:1-4 */
    ecrt_slave_config_sdo16( slave_pointer, 0x1C12, 2, 0x1601 ); /* list all RxPdo in 0x1C12:1-4 */
    ecrt_slave_config_sdo8( slave_pointer, 0x1C12, 0, 2 ); /* set number of RxPDO */

    //Config for TxPDO  ---------------------------------------
    //clearing TxPDO  
    ecrt_slave_config_sdo8( sc_akd, 0x1C13, 0, 0 ); /* clear sm pdo 0x1c13  (SM index '3' in ec_sync_info_t) */
    ecrt_slave_config_sdo8( sc_akd, 0x1A00, 0, 0 ); /* clear TxPdo 0x1A00 */
    ecrt_slave_config_sdo8( sc_akd, 0x1A01, 0, 0 ); /* clear TxPdo 0x1A01 */
    ecrt_slave_config_sdo8( sc_akd, 0x1A02, 0, 0 ); /* clear TxPdo 0x1A02 */
    ecrt_slave_config_sdo8( sc_akd, 0x1A03, 0, 0 ); /* clear TxPdo 0x1A03 */

    //Assigning TxPDO
    ecrt_slave_config_sdo32( sc_akd, 0x1A00, 1, 0x6041); /* 0x6041:0/16bits, status word */
    ecrt_slave_config_sdo32( sc_akd, 0x1A00, 2, 0x6066 );  /* 0x6066:0/32bits, position actual value */
    ecrt_slave_config_sdo8( sc_akd, 0x1A00, 0, 2 ); /* set number of PDO entries for 0x1A00 */

    ecrt_slave_config_sdo32( sc_akd, 0x1A01, 1, 0x606c );  /* 0x606c:0/32bits, Velocity actual value */
    ecrt_slave_config_sdo8( sc_akd, 0x1A01, 0, 1 ); /* set number of PDO entries for 0x1A01 */

    //inputting 0x1A00 and 0x1A01 into SM 0x1C13
    ecrt_slave_config_sdo16( sc_akd, 0x1C13, 1, 0x1A00 ); /* list all TxPdo in 0x1C13:1-4 */
    ecrt_slave_config_sdo16( sc_akd, 0x1C13, 2, 0x1A01 ); /* list all TxPdo in 0x1C13:1-4 xxxx */
    ecrt_slave_config_sdo8( sc_akd, 0x1C13, 0, 2 ); /* set number of TxPDO */ 

    //Registring group of PDO(from domain1_registers) for a domain1(allow data transfer within that domain) 
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_registers))
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    //Activating the master 
    if (ecrt_master_activate(master))
    {
        return -1;
    }

    //Calling the func. to get the mapped domain process data memory(basically address of data mapped to domain_1)
    if (!(domain_pd = ecrt_domain_data(domain)))
    {
        return -1;
    }

    //define struct sched_param to store the maximum priority number from
    //sched_get_priority_max().
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);

    //Now we tell OS how to handle the execution process of this application (In FIFO manner)
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */
    //Now we tell the OS how to manage memory management of this application
    //MCL_CURRENT: Lock all pages that are currently mapped into the address space of the process.
    //MCL_FUTURE: Lock all pages that will be mapped into the address space of the process in the future.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }
    //ensure that all pages of the stack are mapped into memory before
    //the application enters its real-time loop. This prevents page faults 
    //during the critical section of code, which could introduce 
    //unpredictable delays.
    stack_prefault();

    //Configure the clock as monotonic(non-resettable)
    clock_gettime(CLOCK_MONOTONIC, &time_pointer);
    time_pointer.tv_sec += 1; /* start in future */
    time_pointer.tv_nsec = 0;

    //cyclic data receive and send
    while (1)
    {
        //When timer_abstine is mentioned , it basically deals with 
        //absolute time , sleeps until the absolute time )
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                              &time_pointer, NULL);
        if (ret)
        {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        //calculate the time and modify time_pointer so that it can sleep
        //for the certain period for cyclic responce to happen
        
        time_pointer.tv_nsec += PERIOD_NS;
        while (time_pointer.tv_nsec >= NSEC_PER_SEC)
        {
            time_pointer.tv_nsec -= NSEC_PER_SEC;
            time_pointer.tv_sec++;
        }
        return ret
    }