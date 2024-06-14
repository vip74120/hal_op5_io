/********************************************************************
* Description:  hal_opi5_gpio.c
*               Driver for the Orange Pi 5 GPIO pins
*
* Author: Andrea Guglielmi
* License: GPL Version 2
* Copyright (c) 2023.
*
* made work for Orange Pi 5 Guglielmi Andrea
* 
* version 1.00 first version
*********************************************************************/

#include "rtapi.h"			/* RTAPI realtime OS API */
#include "rtapi_ctype.h"	/* isspace() */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"			/* HAL public API decls */
#include <rtapi_io.h>

#include <termios.h>
#include <unistd.h>
#include <sys/types.h>

#include "./opi5/wiringPi/piHiPri_c.h"
#include "./opi5/wiringPi/wiringPi.h"
#include "./opi5/wiringPi/wiringPi_c.h"

#define RTAPI_BIT(nr) (1UL << (nr))

MODULE_AUTHOR("Andrea Guglielmi");
MODULE_DESCRIPTION("Driver for Orange Pi 5 GPIO pins");
MODULE_LICENSE("GPL");

// This driver uses wiringPi released officially for opi5: https://github.com/orangepi-xunlong/wiringOP

// how to build the driver:
// 1. clone linuxcnc branch on opi5
// 2. copy this file inside linuxcnc/src/hal/drivers
// 3. clone next branch of wiringOP
// 4. create a folder linuxcnc/src/hal/drivers/opi5
// 5. copy wiringPi folder and version.h file included into wiringOP package into opi5
// 6. inside opi5/wiringPi find and rename file: wiringPi.c --> wiringPi_c.h
// 7. inside opi5/wiringPi find and rename file: piHiPri.c --> piHiPri_c.h
// 8. build the driver "sudo halcompile --install /home/USER/WhereLinuxCNCWasCloned/linuxcnc/src/hal/drivers/hal_op5_io.c"

/* How to use in hal:
copy and paste this piece of text

# ========= GPIO -> WiringPI pins =======================
# The number of pin correspond to the number of wPi in the table calling "gpio readall", for opi5 0-16
# 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1 
# 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
#
# 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 1 0 0 0 0 1 0 1 	--> 51333 // dir 	 [bin to dec]  mask (0 = IN, 	1 = OUT)
# 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 	-->    24 // exclude [bin to dec]  mask (0 = used, 	1 = Excluded)
# excluded pins for serial (modbus)
# using uart1 for modbus, wpi 3-4 should be excluded.
loadrt hal_op5_io dir=51333 exclude=24

# --- begin addf s
addf hal_op5_io.read base-thread			# <== read task
addf stepgen.make-pulses base-thread
addf stepgen.capture-position servo-thread
addf motion-command-handler servo-thread
addf motion-controller servo-thread
addf stepgen.update-freq servo-thread
addf hal_op5_io.write base-thread			# <== write task
# --- end addf s

# The number of pin correspond to the number of wPi in the table calling "gpio readall"
net xstep stepgen.0.step => hal_op5_io.pin-02-out
net ystep stepgen.1.step => hal_op5_io.pin-11-out
net zstep stepgen.2.step => hal_op5_io.pin-15-out
*/

// for any question you can try to write me: mail@guglielmi.eu

// port direction bits, 1=output
static char *dir = "0"; // all input
RTAPI_MP_STRING(dir, "port direction, 1=output");
static unsigned dir_map;

// exclusion bits, 1=excluded
static char *exclude = "0"; // all input enabled
RTAPI_MP_STRING(exclude, "port excusion, 1=excluded");
static unsigned exc_map;

hal_bit_t **port_data;

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/
static int comp_id;		    /* component ID */
static int npins;        	/* number of gpio in/out */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
*/
static void write_port(void *arg, long period);
static void read_port(void *arg, long period);
int getGpioNum(int model);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
    int n, retval, pin_no;
	int model = -1;
	char *endptrD;
	char *endptrE;
	int fd;
	 
	piBoardId (&model);
		
	rtapi_print_msg(RTAPI_MSG_INFO,
	    "OPI5 GPIO: model found: %d \n", model);
	
	npins = getGpioNum(model);
    if (npins == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "OPI5 GPIO: ERROR: no GPIO pins configured\n");
	return -1;
    }
	rtapi_print_msg(RTAPI_MSG_INFO,
	    "OPI5 GPIO: number of pins found: %d \n", npins);

    /* STEP 1: initialise the driver */
    comp_id = hal_init("hal_op5_io");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "OPI5 GPIO: ERROR: hal_init() failed\n");
	return -1;
    }
    /* STEP 2: allocate memory */
	if (system("sudo chmod 777 /dev/mem") != 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL_OP5_IO: ERROR: setting dev/mem permissions\n");
		hal_exit(comp_id);
		return -1;
	}	
    port_data = hal_malloc(npins * sizeof(void *));	
    if (port_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "OPI5 GPIO: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }
		
	wiringPiSetup();
	
	// Debug serial port could be used. it is called ttyFIQ0 and it should be renamed to ttyS2
	// in this way it could be configured in classic ladder. The port should have root permissions
	if (system("sudo mv /dev/ttyFIQ0 /dev/ttyS2") == 0) {
		if (system("sudo chmod 777 /dev/ttyS2") < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"OPI5 GPIO: Serial ttyS2 error (not present?) \n");
			return -1;
		}
	}

	/* STEP 3: check gpio direction */
    dir_map = strtoul(dir, &endptrD,0);
    if (*endptrD) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"OPI5 GPIO: dir=%s - trailing garbage: '%s'\n",
			dir, endptrD);
	return -1;
    }
	rtapi_print_msg(RTAPI_MSG_INFO,
	    "OPI5 GPIO: dir: %d \n", dir_map);	
	if (dir_map == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "OPI5 GPIO: ERROR: no dir string parameter\n");
	return -1;
    }
	
    /* STEP 4: check gpio exclusion */
    exc_map = strtoul(exclude, &endptrE,0);
    if (*endptrE) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"OPI5 GPIO: exclude=%s - trailing garbage: '%s'\n",
			exclude, endptrE);
	return -1;
    }
	rtapi_print_msg(RTAPI_MSG_INFO,
	    "OPI5 GPIO: exclude: %d \n", exc_map);	
	if (exc_map < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "OPI5 GPIO: ERROR: no exclude string parameter\n");
	return -1;
    }

	// Export pins
	for (n = 0; n < npins; n++) {
	  if (!(exc_map & RTAPI_BIT(n))) {
		  if (dir_map & RTAPI_BIT(n)) {		  
			  pinMode (n, OUTPUT);
			  if ((retval = hal_pin_bit_newf(HAL_IN, &port_data[n],
						   comp_id, "hal_op5_io.pin-%02d-out", n)) < 0)
			break;
		  } 
		  else {
			pinMode (n, INPUT);
			pullUpDnControl(n, PUD_DOWN);
			if ((retval = hal_pin_bit_newf(HAL_OUT, &port_data[n],
						   comp_id, "hal_op5_io.pin-%02d-in", n)) < 0)
			break;
		  }
	  }
    }
	
	// Export write function
    retval = hal_export_funct("hal_op5_io.write", write_port, 0,
			      0, 0, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_OP5_IO: ERROR: write funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
	
	// Export read function
    retval = hal_export_funct("hal_op5_io.read", read_port, 0,
			      0, 0, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_OP5_IO: ERROR: read funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
		
    rtapi_print_msg(RTAPI_MSG_INFO,
	"HAL_OP5 IO: installed driver\n");
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}

/**************************************************************
* REALTIME PORT WRITE FUNCTION                                *
**************************************************************/

static void write_port(void *arg, long period)
{
    for (int n = 0; n < npins; n++) {
	  if (!(exc_map & RTAPI_BIT(n))) {
		  if (dir_map & RTAPI_BIT(n)) {
			if (*port_data[n])
				digitalWrite (n, HIGH);	// On
			else
				digitalWrite (n, LOW);	// Off
		  }
	  }
	}
}

static void read_port(void *arg, long period)
{
    for (int n = 0; n < npins; n++) {
	  if (!(exc_map & RTAPI_BIT(n))) {
		  if ((dir_map & RTAPI_BIT(n)) == 0) {
			if (digitalRead (n) == HIGH)
				*port_data[n] = 1;
			else
				*port_data[n] = 0;
		  }
      }
	}
}

int getGpioNum(int model) {
	switch (model)
		{
			case PI_MODEL_5B:
				return 16;
//				break;
			case PI_MODEL_3:
			case PI_MODEL_LTIE_2:
			case PI_MODEL_ZERO:
			case PI_MODEL_ZERO_PLUS_2:
			case PI_MODEL_ZERO_PLUS:
			case PI_MODEL_4_LTS:
			case PI_MODEL_800:
			case PI_MODEL_5:
				return 17;
				break;
			case PI_MODEL_ZERO_2:
				return 21;
				break;
			case PI_MODEL_WIN:
			case PI_MODEL_PRIME:
			case PI_MODEL_PC_2:
			case PI_MODEL_H3:
			case PI_MODEL_RK3399:
			case PI_MODEL_4:
			case PI_MODEL_5_PLUS:
			case PI_MODEL_CM4:
				return 28;
				break;
			case PI_MODEL_R1_PLUS:
				return 8;
				break;
			default:
				rtapi_print_msg(RTAPI_MSG_ERR,
					"HAL_OP5_IO: ERROR: model not recognized getting gpio number\n");
				hal_exit(comp_id);
				return -1;
			break;
		}
}


