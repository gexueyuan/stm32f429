/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_osal_debug.c
 @brief  : This is the global debug realization.
 @author : wangyf
 @history:
           2014-11-13    wangyf    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#include "cv_osal_dbg.h"
//#include "components.h"
#include "cv_drv_file.h"

#if (OSAL_GLOBAL_DEBUG)
/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
/**
 *  [block begin] Debug level configuration
 *  You can add your module here and DONOT modify any other code in the block.
 */
const debug_entry_t debug_entry_table_start = {NULL, NULL};
OSAL_DEBUG_ENTRY_DECLARE(sysc)
//OSAL_DEBUG_ENTRY_DECLARE(vam)
OSAL_DEBUG_ENTRY_DECLARE(vsa)
//OSAL_DEBUG_ENTRY_DECLARE(voc)


// add your module here...

const debug_entry_t debug_entry_table_end = {NULL, NULL};
/**
 *  [block end]
 */


#define OSAL_DBG_LOG_TASK_PRIOPRITY (OSAL_TASK_PRIOPRITY_LOWEST)

/** 
 * Configure of output buffer
 */
#define OSAL_DBG_LOG_WIDTH  128
#define OSAL_DBG_LOG_LINES  64
char osal_dbg_log_buffer[OSAL_DBG_LOG_LINES][OSAL_DBG_LOG_WIDTH] __attribute__((at(CCMDATARAM_BASE)));





/* BEGIN: Added by wanglei, 2015/5/12 */
int g_dbg_print_type = 0;
/* END:   Added by wanglei, 2015/5/12 */


unsigned char log_media_sd = FALSE;


osal_sem_t*  log_sem;

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/
int osal_dbg_set_level(char *module, int level)
{
	debug_entry_t *cmdtp;

	/* Search command table - Use linear search - it's a small table */
	for (cmdtp = (debug_entry_t *)(&debug_entry_table_start + 1); cmdtp->module; cmdtp++) {
		if (strcmp (module, cmdtp->module) == 0) {
		    if (cmdtp->handler){
                (cmdtp->handler)(level);
                return TRUE;
		    }
		}
	}
    return FALSE;
}

void osal_dbg_dump_data(uint8_t *p, uint32_t len)
{
    int i;
    #define LINE_WIDTH 16

    if (len > 0){
        osal_printf("====================== dump data ======================\n");
        osal_printf(" Addr| ");

        for (i=0; i<LINE_WIDTH; i++) {
            osal_printf("%02x ", i);
        }
        osal_printf("\n -----------------------------------------------------");
        
        for (i=0;i<len;i++) {
            if ((i%LINE_WIDTH) == 0) {
                osal_printf("\n %04x| ", i);
            }
            osal_printf("%02x ", *(p+i));
        }
        osal_printf("\n======================== end ==========================\n");
    }
}

void osal_log(const char *fmt, ...)
{
    static uint32_t line = 0;
    char *outbuf;
    va_list arg_ptr;
    int err;
    static uint8_t flag =1;

    err = osal_sem_take(log_sem, OSAL_WAITING_NO);
    if (err != OSAL_STATUS_SUCCESS) {
        return;
    }
    outbuf = osal_dbg_log_buffer[line];
    if (*outbuf > 0) {
        //Notice, the buffer is full.
        if (flag) {
            osal_printf("!!!Log buffer is full!!!\n");
            flag =0;
        }
        return;
    }

    va_start(arg_ptr, fmt);
    *outbuf = vsnprintf(outbuf+1, OSAL_DBG_LOG_WIDTH-3, fmt, arg_ptr);

    /* To avoid memory error, string will be truncated when it is too long. */
    if (*outbuf > OSAL_DBG_LOG_WIDTH-4) { /* String is so long that it has been truncated. */
        *(outbuf+OSAL_DBG_LOG_WIDTH-4) = '!';
        *(outbuf+OSAL_DBG_LOG_WIDTH-3) = '#';
        *(outbuf+OSAL_DBG_LOG_WIDTH-2) = '\n';
        *(outbuf+OSAL_DBG_LOG_WIDTH-1) = 0;
        *outbuf = OSAL_DBG_LOG_WIDTH - 1;
    }
    va_end(arg_ptr);

    if (++line >= OSAL_DBG_LOG_LINES) {
        line = 0;
    }

    osal_sem_release(log_sem);
}


static void output_to_uart(char *data, uint32_t length)
{
    #if (defined(OS_RT_THREAD) && defined(RT_USING_DEVICE))
    rt_device_t console = rt_console_get_device();
    if (console) {
        rt_uint16_t old_flag = console->flag;
        console->flag |= RT_DEVICE_FLAG_STREAM;
        rt_device_write(console, 0, data, length);
        console->flag = old_flag;
    }
    #else
    while(length--) {
        putchar(*data++);
    }
    #endif
}


static void output_to_sdcard(char *data, uint32_t length)
{
    if (log_store.sd_state) {
        log_file_write(data,length);
    }
}

void osal_dbg_thread_entry(void *parameter)
{
    static uint32_t line = 0;
    char *outbuf = osal_dbg_log_buffer[0];
	
        rtc_init();
        if (log_store.log_media & LOG_SD_CARD) {
            if (log_sd_init() == TRUE) {
                log_store.sd_state = TRUE;
                log_media_sd = TRUE;
                osal_printf("file system init ok, log will store in sd card\n");
            }
            else {
                log_store.log_media =  LOG_UART;
                log_store.sd_state = FALSE;
                osal_printf("log will output to uart\n");
            }
        }
	while(1){

	    outbuf = osal_dbg_log_buffer[line];
	    if (*outbuf) {
                if (log_store.log_media & LOG_UART)  {
                    output_to_uart(outbuf+1, (uint32_t)*outbuf);
                }
                if (log_store.log_media & LOG_SD_CARD) {
                    output_to_sdcard(outbuf+1, (uint32_t)*outbuf);
                }
                
                *outbuf = 0; /* clear the contant */
            
            if (++line >= OSAL_DBG_LOG_LINES) {
                line = 0;
            }
	    }
	    else {
            osal_delay(1);
	    }
	}
}

void osal_dbg_init(void)
{
    osal_task_t *task;

    memset(osal_dbg_log_buffer, 0, OSAL_DBG_LOG_LINES*OSAL_DBG_LOG_WIDTH);
    
    /* log output to sd card default */
    memset(&log_store,0,sizeof(log_store));
    log_store.log_media = LOG_SD_CARD;
    memset(&share_spi,0x00,sizeof(share_spi));
    if (log_store.log_media & LOG_SD_CARD) {
        /* creat spi mutex manage */
        share_spi.spi_mutex = osal_sem_create("spi_m",1);
    }

    log_sem = osal_sem_create("log_s", 1);
    osal_assert(log_sem != NULL);

    task = osal_task_create("tdbg", osal_dbg_thread_entry, NULL, 2048, OSAL_DBG_LOG_TASK_PRIOPRITY);
    osal_assert(task != NULL);
}

#ifdef OS_RT_THREAD
/**
 * Export to finsh of RT-thread
 */
void debug(char *module, int level)
{
	if (osal_dbg_set_level(module, level)) {
        osal_printf("success.\n");
	}
	else {
        osal_printf("cannot find module \"%s\" !\n", module);
	}
}
FINSH_FUNCTION_EXPORT(debug, 0-off 1-err 2-warn 3-info 4-trace 5-loud);

void dump(uint32_t addr, uint32_t len)
{
    osal_dbg_dump_data((uint8_t *)addr, len);
}
FINSH_FUNCTION_EXPORT(dump, printf the raw data);


/* BEGIN: Added by wanglei, 2015/5/12 */
void dbg_print(int type)
{
    if (type < 0 || type > 3){
        return;
    }
    else{
        g_dbg_print_type = type;
    }
}
FINSH_FUNCTION_EXPORT(dbg_print, 0-disable 1-print wnet rxtx 2-print peer status 3-both);
/* END:   Added by wanglei, 2015/5/12 */

#endif /* OS_RT_THREAD */
#endif /* OSAL_GLOBAL_DEBUG */
