#include "typedefs.h"                                                                                                                                
#include "command_interface_parameter.h"   
#include "command_interface_setting.h"

/* Command table structure used by the monitor:                                
 */                                                                            
typedef struct   {                                                           
    char    *name;          /* Name of command seen by user. */                
    int     (*func)();      /* Function called when command is invoked. */     
} cmdlist_t;                                                                             
                                                                               
typedef struct
{
    uint8_t operation;  /* this is to check wether cli task is operating or not*/
    uint8_t flag;       /* check input data whether there is '\r' or '\n'                       */
    uint32_t open_cnt;  /* count '\r' or '\n' : cli task will be created when open_cnt = 3;     */
    uint8_t close;      /* if this value is 1, cli task will be deleted                         */
    uint8_t len;        /* this value indicate length of cmdline                                */
    uint8_t cmdline[CMDLINESIZE]; // this is buffer to save input data                          */
} cli_info_t;

void cli_task_control(void);
static int cli_set_close(int argc, char *argv[]);
static void cli_writeprompt(void);
static int cli_docommand(char *cmdline);
extern void cli_commnadloop(void);
static int cli_tokenize(char *string,char *argv[]);
static int cli_getopt(int argc, char *argv[], char *opts);
static void cli_getoptinit(void);
static int cli_can(int argc, char *argv[]);
extern uint8_t *get_uart1_buffer(void);
extern void cli_initialize(void);

