#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "components.h"
#include "can_lld_cfg.h"
#include "PDK_RTOS_manager.h"
#include "vehicle_communication.h"
#include "command_interface.h"
#include "diagnosis.h"
#include "serial_lld_cfg.h"

/* Variables used by getopt: */
static int optindex;         /* argv index to first cmd line arg that is not part of the option list.*/
static char *p_optarg;       /* pointer to argument associated with an option */
static int sp;

static cli_info_t cli_info;
can_desc_t _tmp_can_desc;  

uint8_t uart1_buffer;


uint8_t *get_uart1_buffer(void)
{
    return (&uart1_buffer);
}

cmdlist_t cmdlist[COMMAND_LIST] = {
    { "can",        cli_can},
    { "reboot",     cli_reboot},
	{ "close",      cli_set_close}
};

/*  The irq_getByte is callback function from isr handler of uart
 *  This function is to store the user's input data as they type.
 */
void cli_irq_getByte(SerialDriver *sdp) 
{
    if(cli_info.len >= CMDLINESIZE-1) 
    {
        cli_info.len = CLI_INIT_VARIABLE;
    }    
    //pal_lld_togglepad(PORT_LED_R_STATE, LED_R_STATE);
    sd_lld_write(sdp, &uart1_buffer, UART1_BUFFER_SIZE);
    edmaChannelStart(sdp->rx_channel);
    EDMA_1.CH[sdp->rx_channel].TCD_CITER.R = true;	             // Counter Clear
    EDMA_1.CH[sdp->rx_channel].TCD_DADDR.R = &uart1_buffer;  // DMA Buffer Point Clear	    

    cli_info.cmdline[cli_info.len] = uart1_buffer;
    if((cli_info.cmdline[cli_info.len] == '\r') || (cli_info.cmdline[cli_info.len] == '\n'))
    {
        cli_info.flag = ENTER_KEY_PRESSED;   
        cli_info.open_cnt++;               
    }
    cli_info.len++;     


}

/* 
 *  The writeprompt will dislpay text like the following whenever the user presses the Enter key.
 */
static void cli_writeprompt(void)
{
    char prompt[]="PDK>";

    for(int i=0;i<sizeof(prompt);i++)
    {
        sd_lld_write(&SD1, &prompt[i], 1);
    }    
}

static int cli_StringToHexa(const char *param)
{
    int hex = 0;
    int count = strlen(param), i = 0;
 
    for(i = 0; i < count; i++){
        if(*param>= '0' && *param<= '9') hex = hex *16 + *param- '0';
        else if(*param>= 'A' && *param<= 'F') hex = hex *16 + *param- 'A' + 10;
        else if(*param>= 'a' && *param<= 'f') hex = hex *16 + *param- 'a' + 10;
        param++;
    }
    return hex;
}

static int cli_StringToDecimal(const char *param)
{
    int decimal = 0;
    int count = strlen(param), i = 0;
 
    for(i = 0; i < count; i++)
    {
        if(*param>= '0' && *param<= '9') decimal = decimal *10 + *param- '0';
        param++;
    }
    return decimal;
}


static int cli_docommand(char *cmdline)
{
    int ret, argc;
    cmdlist_t *cmdptr;
    char    *ps, *argv[ARGCNT], cmdcpy[CMDLINESIZE];

    /* Just in case an upper-level interpreter has the same command as
     * this citor, if the first character of the incoming command
     * string is an underscore, just throw it away.  This allows the
     * user to get to the monitor command that is duplicated in an
     * upper level interpreter by preceding it with an underscore.
     */
    if (cmdline[0] == '_')
        cmdline++;

    /* Copy the incoming command line to a local buffer so that we can 
     * be sure that the line is writeable; plus we do not alter the 
     * incoming buffer.
     */
    strcpy(cmdcpy,cmdline);

    /* Scan through the string, if '#' is found, terminate line if
     * it is not preceded by a backslash.  If it is preceded by a
     * backslash, then remove the backslash from the line and leave
     * the rest of the line as is.
     */
    ps = strchr(cmdcpy,'#');
    if (ps)
    {
        if (*(ps-1) != '\\')
            *ps = 0;
        else
            strcpy(ps-1,ps);
    }

    /* Build argc/argv structure based on incoming command line. */
    argc = cli_tokenize(cmdcpy,argv);
//    if (argc == 0)
//        return(CMD_LINE_ERROR);
//    if (argc < 0) {
//        return(CMD_LINE_ERROR);
//    }
    if(argc <= 0) return(CMD_LINE_ERROR);
    else{}

    // /* Initialize static data used by getopt(). */
    cli_getoptinit();

    /* Step through the command table looking for a match between
     * the first token of the command line and a command name.
     */
    cmdptr = cmdlist;
    while(cmdptr->name)
    {
        if (strcmp(argv[0],cmdptr->name) == 0)
            break;
        cmdptr++;
    }
    
    if (cmdptr->name)
    {
        if ((argc > 1) && (strcmp(argv[argc-1],"?") == 0))
        {
            return(CMD_PREFILL);
        }
        ret = cmdptr->func(argc,argv);

        return(ret);
    }

    /* If we get here, then the first token does not match on any of
     * the command names in ether of the command tables.  As a last
     * resort, we look to see if the first token matches an executable
     * file name in TFS.
     */
    ret = CMD_NOT_FOUND;

    return(ret);
}

/* tokenize():
 *  Take the incoming string and create an argv[] array from that.  The
 *  incoming string is assumed to be writeable.  The argv[] array is simple
 *  a set of pointers into that string, where the whitespace delimited
 *  character sets are each NULL terminated.
 */
static int cli_tokenize(char *string, char *argv[])
{
    int argc, done;

    /* Null out the incoming argv array. */
    for(argc=0;argc<ARGCNT;argc++)
        argv[argc] = (char *)0;

    argc = 0;
    while(1) {
        while ((*string == ' ') || (*string == '\t'))
            string++;
        if (*string == NULL)
            break;
        argv[argc] = string;
        while ((*string != ' ') && (*string != '\t')) {
            if ((*string == '\\') && (*(string+1) == '"')) {
                strcpy(string,string+1);
            }
            else if (*string == '"') {
                strcpy(string,string+1);
                while(*string != '"') {
                    if ((*string == '\\') && (*(string+1) == '"'))
                        strcpy(string,string+1);
                    if (*string == NULL)
                        return(-1);
                    string++;
                }
                strcpy(string,string+1);
                continue;
            }
            if (*string == NULL)
                break;
            string++;
        }
        if (*string == NULL)
            done = 1;
        else {
            done = NULL;
            *string++ = NULL;
        }
        argc++;
        if (done)
            break;
        if (argc >= ARGCNT) {
            argc = -1;
            break;
        }
    }
    return(argc);
}

static int cli_getopt(int argc, char *argv[], char *opts)
{    
    int c;
    char *cp;

    if(sp == 1) {
        if(optindex >= argc) {
            return(-1);
        }
        else if (argv[optindex][0] != '-') {
            return(-1);
        }
        else if (argv[optindex][1] == '\0') {
            return(-1);
        }    
        else if(strcmp(argv[optindex], "--") == 0) {
            optindex++;
            return(-1);
        }
    }
    c = argv[optindex][sp];
    if(c == ':' || (cp=strchr(opts, c)) == 0) {
        if(argv[optindex][++sp] == '\0') {
            optindex++;
            sp = 1;
        }
        return('?');
    }
    if(*++cp == ':') {
        if(argv[optindex][sp+1] != '\0')
            p_optarg = &argv[optindex++][sp+1];
        else if(++optindex >= argc) {
            sp = 1;
            return('?');
        } else {
            p_optarg = argv[optindex]; //here
            optindex++;           
        }    
        sp = 1;
    } else {
        if(argv[optindex][++sp] == '\0') {
            sp = 1;
            optindex++;
        }
        p_optarg = 0;
    }
    return(c);
}
/* getoptinit():
 * Since getopt() can be used by every command in the monitor
 * it's variables must be reinitialized prior to each command
 * executed through docommand()...
 */
static void cli_getoptinit(void)
{
   sp = 1;
   optindex = 1;
}

static int cli_can(int argc, char *argv[])
{
    int i;
    int opt=0;      
	char *arg1, *arg2, *end, *str;
    uint16_t val=0;
    uint32_t can_id=0;
    uint8_t  start=0, len=0;
    uint32_t size;
    can_list_t *pCan;               
    can_desc_t *pCanDesc;               

    pCan = get_can_list(); //this function is used to retrieve the address of the can structure that is currently in use. 
    pCanDesc = get_can_desc();    

    if (argc < 2) {
        return(0);
    }

    while((opt=cli_getopt(argc,argv,"v:i:s:l:n:")) != -1) {
        switch(opt) { 
        case 'l' : //0x6C: 
            if(p_optarg > 0)
                len = cli_StringToDecimal(p_optarg);
            break;
        case 'v' : //0x76
            if(p_optarg > 0)
                val = (uint16_t)strtoul(p_optarg,0,0);
            break;
        case 'i' : //0x69
            if(p_optarg > 0)
                can_id = cli_StringToHexa(p_optarg);
            break;
        case 's': //0x73:
            if(p_optarg > 0)
                start = cli_StringToDecimal(p_optarg); 
            break;
        case 'n': 
            if(p_optarg > 0)
                str = p_optarg;
            break;
        default:
            return(CMD_PARAM_ERROR);
        }
    }    
    arg1 = argv[optindex];
    arg2 = argv[optindex+1];

    if(arg1==NULL) {
        printf("No arguments, exiting... use help!\n");
        return 0;
    }    

	if(strcmp(arg1, "help") == 0) 
    {
        // for(int i=0; i<12;i++) //12
        // {
        //     printf("%s",HelpCan[i]);
        // }    
    }
	else if(strcmp(arg1, "info") == 0) 
	{
        for(int i=0; i<TOTAL_CAN_PROTOCOL; i++)
        {
            if(pCanDesc->can_list[i].can_id_info[0].id == 0)
            {
                pCanDesc->total = i;
                break;
            }
        }
		printf("|------CAN Info.------|\r\n");
		printf("|Protocol Total:%6d|\r\n",pCanDesc->total);
		printf("|Protocol SEL:%8d|\r\n",pCanDesc->select);
		printf("|---------------------|\r\n");
        for(int i=0; i<pCanDesc->total; i++)
        {
		    printf("|---------------------|\r\n");
            printf("|  Protocol Number:%d  |\r\n",i);
		    printf("|---------------------|\r\n");
		    printf("|manufacturer:%8s|\r\n",pCanDesc->can_list[i].manufacture);
		    printf("|model:%15s|\r\n",pCanDesc->can_list[i].model);
		    printf("|year:%16d|\r\n",pCanDesc->can_list[i].year);
		    printf("|Baud Rate:%11d|\r\n",pCanDesc->can_list[i].baudrate[0]);
		    printf("|Sample Point:%8d|\r\n",pCanDesc->can_list[i].samplepoint[0]);
            for(int j=0;j<8;j++)
            {
                if(pCanDesc->can_list[i].can_id_info[j].id == 0)
                {
                    break;
                }    
		        printf("|ID:0x%16x|\r\n",pCanDesc->can_list[i].can_id_info[j].id);
		        printf("|Flag:%16x|\r\n",pCanDesc->can_list[i].can_id_info[j].item_idx);
		        printf("|Start Bit:%11d|\r\n",pCanDesc->can_list[i].can_id_info[j].start_bit);
		        printf("|Bit Len:%13d|\r\n",pCanDesc->can_list[i].can_id_info[j].len);
            }
        }    
        printf("|---------------------|\r\n");        
	}
	if(strcmp(arg1, "curr") == 0) 
	{
        printf("|---------------------|\r\n"); 
		printf("|    Current Info.    |\r\n");
        printf("|---------------------|\r\n"); 
		printf("|Protocol SEL:%8d|\r\n",pCanDesc->select);
		printf("|---------------------|\r\n");
        printf("|manufacturer:%8s|\r\n",pCan->manufacture);
        printf("|model:%15s|\r\n",pCan->model);
        printf("|year:%16d|\r\n",pCan->year);
        printf("|Baud Rate:%11d|\r\n",pCan->baudrate[0]);
        printf("|Sample Point:%8d|\r\n",pCan->samplepoint[0]);
        for(int j=0;j<8;j++)
        {
            if(pCan->can_id_info[j].id == 0)
            {
                continue;
            }    
            printf("|---------------------|\r\n");        
            printf("|ID:0x%16x|\r\n",pCan->can_id_info[j].id);
            printf("|Flag:%16x|\r\n",pCan->can_id_info[j].item_idx);
            printf("|Start Bit:%11d|\r\n",pCan->can_id_info[j].start_bit);
            printf("|Bit Len:%13d|\r\n",pCan->can_id_info[j].len);
        }        
        printf("|---------------------|\r\n");        
	}
    else if(strcmp(arg1, "sel") == 0) 
    {
        if(val > TOTAL_CAN_PROTOCOL-1)
        {
            val = TOTAL_CAN_PROTOCOL-1;
        }
        pCanDesc->select = val;
        //
        printf("current protocol:%d\r\n",pCanDesc->select);
    }
    else if(strcmp(arg1, "del") == 0) 
    {
        if(val > TOTAL_CAN_PROTOCOL-1)
        {
            val = TOTAL_CAN_PROTOCOL-1;
        }
        if(pCanDesc->select !=val)
        {
            can_desc_t temp_can_info;
            memcpy(&temp_can_info,pCanDesc,sizeof(can_desc_t));
            memset(pCanDesc,0x0,sizeof(can_desc_t));
            
            for(i=0; i<val; i++)
            {
                if(i>=0) memcpy(&pCanDesc->can_list[i], &temp_can_info.can_list[i], sizeof(can_list_t));
            }
            for(i=val+1; i<TOTAL_CAN_PROTOCOL; i++) 
            {
                if(i>0) memcpy(&pCanDesc->can_list[i-1], &temp_can_info.can_list[i], sizeof(can_list_t));
            }   

            for(i=0; i<TOTAL_CAN_PROTOCOL; i++)
            {
                if(pCan->can_id_info[0].id==0)
                {
                    pCanDesc->total = i+1;
                    break;
                }
            }
            printf("The protocl(%d) has been deleted \r\n",val);            
        }
        else
        {
            printf("The cuurent protocl(%d) can't be deleted \r\n",pCanDesc->select);
        }
    }
    else if(strcmp(arg1, "save") == 0) 
    {
        eeprom_write(&EEPROMD, 0, sizeof(can_desc_t), pCanDesc);
        // printf("Save Can Information...\r\n");
    }
    else if(strcmp(arg1, "load") == 0) 
    {
        eeprom_read(&EEPROMD, 0, &size, &_tmp_can_desc);
        // printf("Load Can Information...\r\n");
    }
    else if(strcmp(arg1, "config") == 0) 
    {           
        vc_set_control_CAN_task(0); //suspend can task and can driver
        vc_set_CAN_filter();
        vc_set_CAN_bitRate(0);
        vc_set_CAN_bitRate(1);
        eeprom_write(&EEPROMD, 0, sizeof(can_desc_t), pCanDesc);
        eeprom_write(&EEPROMD, 1, sizeof(CANConfig), &can_config_mcanconf);     
        vc_set_control_CAN_task(1); //resume can task and reconfigure can driver     
    }        
    else if(strcmp(arg1, "close") == 0) 
    {           
        cli_info.close = CLI_END;
    }        
    else if(strcmp(arg1, "add") == 0) 
    {
        if(arg2==NULL)
            return 0;

       if((strcmp(arg2, "menu") == 0))
        {    
            strcpy(pCan->manufacture,str);
            printf("menu:%s\r\n",pCan->manufacture);
        } 
        else if((strcmp(arg2, "year") == 0))
        {    
            pCan->year = val;
            printf("year:%d\r\n",pCan->year);
        } 
        else if((strcmp(arg2, "baud") == 0))
        {
            pCan->baudrate[0] = val;
            printf("baudrate:%d\r\n",pCan->baudrate[0]);
        } 
        else if((strcmp(arg2, "sample") == 0))
        {
            pCan->samplepoint[0] = val;
            printf("samplepoint:%d\r\n",pCan->samplepoint[0]);
        } 
        else if((strcmp(arg2, "ign") == 0))
        {
            pCan->can_id_info[FLAG_IGN].id = can_id;            
            pCan->can_id_info[FLAG_IGN].item_idx = FLAG_IGN;
            pCan->can_id_info[FLAG_IGN].start_bit = start;
            pCan->can_id_info[FLAG_IGN].len = len;

            printf("ign id:%04x\r\n",pCan->can_id_info[FLAG_IGN].id);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_IGN].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_IGN].len);
            //printf("item flag:%04x\r\n",pCan->can_id_info[0].item_idx);
        } 
        else if((strcmp(arg2, "gear") == 0))
        {
            pCan->can_id_info[FLAG_GEAR].id = can_id;
            pCan->can_id_info[FLAG_GEAR].item_idx = FLAG_GEAR;
            pCan->can_id_info[FLAG_GEAR].start_bit = start;
            pCan->can_id_info[FLAG_GEAR].len = len;

            printf("gear id:%04x\r\n",pCan->can_id_info[FLAG_GEAR].id);
            printf("item flag:%04x\r\n",pCan->can_id_info[FLAG_GEAR].item_idx);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_GEAR].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_GEAR].len);
        }
        else if((strcmp(arg2, "speed") == 0))
        {
            pCan->can_id_info[FLAG_SPEED].id = can_id;
            pCan->can_id_info[FLAG_SPEED].item_idx = FLAG_SPEED;
            pCan->can_id_info[FLAG_SPEED].start_bit = start;
            pCan->can_id_info[FLAG_SPEED].len = len;

            printf("speed id:%04x\r\n",pCan->can_id_info[FLAG_SPEED].id);
            printf("item flag:%04x\r\n",pCan->can_id_info[FLAG_SPEED].item_idx);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_SPEED].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_SPEED].len);
        }
        else if((strcmp(arg2, "hazard") == 0))
        {
            pCan->can_id_info[FLAG_HAZARD].id = can_id;
            pCan->can_id_info[FLAG_HAZARD].item_idx = FLAG_HAZARD;
            pCan->can_id_info[FLAG_HAZARD].start_bit = start;
            pCan->can_id_info[FLAG_HAZARD].len = len;

            printf("hazard id:%04x\r\n",pCan->can_id_info[FLAG_HAZARD].id);
            printf("item flag:%04x\r\n",pCan->can_id_info[FLAG_HAZARD].item_idx);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_HAZARD].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_HAZARD].len);
        }
        else if((strcmp(arg2, "wheel") == 0))
        {
            pCan->can_id_info[FLAG_WHEEL].id = can_id;
            pCan->can_id_info[FLAG_WHEEL].item_idx = FLAG_WHEEL;
            pCan->can_id_info[FLAG_WHEEL].start_bit = start;
            pCan->can_id_info[FLAG_WHEEL].len = len;

            printf("hazard id:%04x\r\n",pCan->can_id_info[FLAG_WHEEL].id);
            printf("item flag:%04x\r\n",pCan->can_id_info[FLAG_WHEEL].item_idx);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_WHEEL].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_WHEEL].len);
        }
        else if((strcmp(arg2, "right") == 0))
        {
            pCan->can_id_info[FLAG_TURN_RIGHT].id = can_id;
            pCan->can_id_info[FLAG_TURN_RIGHT].item_idx = FLAG_TURN_RIGHT;
            pCan->can_id_info[FLAG_TURN_RIGHT].start_bit = start;
            pCan->can_id_info[FLAG_TURN_RIGHT].len = len;

            printf("turn right light id:%04x\r\n",pCan->can_id_info[FLAG_TURN_RIGHT].id);
            printf("item flag:%04x\r\n",pCan->can_id_info[FLAG_TURN_RIGHT].item_idx);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_TURN_RIGHT].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_TURN_RIGHT].len);
        }
        else if((strcmp(arg2, "left") == 0))
        {
            pCan->can_id_info[FLAG_TURN_LEFT].id = can_id;
            pCan->can_id_info[FLAG_TURN_LEFT].item_idx = FLAG_TURN_LEFT;
            pCan->can_id_info[FLAG_TURN_LEFT].start_bit = start;
            pCan->can_id_info[FLAG_TURN_LEFT].len = len;

            printf("hazard id:%04x\r\n",pCan->can_id_info[FLAG_TURN_LEFT].id);
            printf("item flag:%04x\r\n",pCan->can_id_info[FLAG_TURN_LEFT].item_idx);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_TURN_LEFT].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_TURN_LEFT].len);
        }
        else if((strcmp(arg2, "brake") == 0))
        {
            pCan->can_id_info[FLAG_BRAKE].id = can_id;
            pCan->can_id_info[FLAG_BRAKE].item_idx = FLAG_BRAKE;
            pCan->can_id_info[FLAG_BRAKE].start_bit = start;
            pCan->can_id_info[FLAG_BRAKE].len = len;

            printf("hazard id:%04x\r\n",pCan->can_id_info[FLAG_BRAKE].id);
            printf("item flag:%04x\r\n",pCan->can_id_info[FLAG_BRAKE].item_idx);
            printf("start:%04x\r\n",pCan->can_id_info[FLAG_BRAKE].start_bit);
            printf("len:%04x\r\n",pCan->can_id_info[FLAG_BRAKE].len);
        }

    }
}

// char *HelpCan[] = {
// 	"[can command list]\r\n",
// 	"  can info, can curr, can sel, can del, can config\r\n",
// 	"  can add menu, year, id, start..\r\n",
// 	"[Description]\r\n",
// 	"  -can info: it will show all of can protocol info.\r\n",
// 	"  -can curr: it will show the currently used protocol.\r\n",
// 	"  -can sel: it is used to change current protocol\r\n",
// 	"  -can del: it is used to delete selected protocol\r\n",
// 	"[Options]\r\n",
// 	"  -v : this is to input decimal data for command like can sel, can del\r\n",
// 	"  -i : this is to input can id(hexsa)\r\n",
// 	"  -s : this is to input start bit(decimal) for parsing of can data\r\n",
// 	"  -l : this is to input length(decimal) for parsing of can data\r\n",
// 	0,
// };

/*
 * The manager_cli is to control task creation and deletion
 */
void cli_task_control(void)
{
    task_handle_t *taskhandle;
    taskhandle = get_task_handle();

    if((cli_info.open_cnt > CLI_START_COUNT ) && (cli_info.operation == NULL))
    {
        cli_info.operation = CLI_OPERATING;
        create_task_command_line_interface();
    }
    if(cli_info.close == CLI_END)
    {
        cli_info.open_cnt = CLI_INIT_VARIABLE;
        cli_info.close = CLI_INIT_VARIABLE;
        cli_info.operation= CLI_NOT_OPERATING;
        vTaskDelete(taskhandle->task_command_interface_handle);
    }
}

static cli_set_close(int argc, char *argv[])
{
    cli_info.close = CLI_END;
}


void cli_commnadloop(void)
{
    cli_writeprompt();                  			/* Issue prompt */	

    console:	
    vTaskDelay(50); 
    //pal_lld_togglepad(PORT_E, LED_R_STATE);
    /* Calling the function will have used some stack space, we would 
        therefore now expect uxTaskGetStackHighWaterMark() to return a 
        value lower than when it was called on entering the task. */
    if(cli_info.flag)
    {            
        printf("\r\n");
        cli_info.flag = 0;
        cli_info.cmdline[cli_info.len-1] = CLI_INIT_VARIABLE;    // it needs to delete '\r' or '\n' before conveying docommand function.
        //taskENTER_CRITICAL();
        cli_docommand((char *)cli_info.cmdline);   // docommand is a role of parsing string to execute cli function
        //taskEXIT_CRITICAL();
        memset(cli_info.cmdline,0x0,sizeof(cli_info.cmdline));
        cli_info.len = CLI_INIT_VARIABLE;          
#if CLI_STATIC_STACK                      
        cliWaterMark = uxTaskGetStackHighWaterMark( NULL );
#endif            
    }
    else 
    {
        goto console;
    }               
}

void cli_initialize(void)
{
    sd_lld_start(&SD1, &serial_config_cli); // uart debug
    sd_lld_read(&SD1, &uart1_buffer,UART1_BUFFER_SIZE);
}    
