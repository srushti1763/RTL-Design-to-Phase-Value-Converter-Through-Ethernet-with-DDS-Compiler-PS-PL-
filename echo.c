/*
 * Copyright (C) 2009 - 2019 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
#include"xparameters.h"
#include <stdio.h>
#include <string.h>
#include "xgpio.h"
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "sleep.h"

#include "xil_printf.h"

#define DDS_GPIO_DEVICE_ID      XPAR_AXI_GPIO_0_DEVICE_ID
#define DDS_GPIO_CHANNEL        1
#define PHASE_WIDTH             24
#define DDS_CLK_FREQ            100000000.0
#define BANDWIDTH_CHANNEL 1
#define RAMP_TIME_CHANNEL 2

#define GPIO_DEVICE_ID  XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_CHANNEL    1
#define TCP_PORT        7
XGpio Gpio;


int transfer_data() {
	return 0;
}

void print_app_header()
{
#if (LWIP_IPV6==0)
	xil_printf("\n\r\n\r-----lwIP TCP echo server ------\n\r");
#else
	xil_printf("\n\r\n\r-----lwIPv6 TCP echo server ------\n\r");
#endif
	xil_printf("TCP packets sent to port 6001 will be echoed back\n\r");

	xil_printf("\n\r----- lwIP TCP Bandwidth-RampTime Server -----\n\r");
	xil_printf("Send: BW:5;RAMP:2000 (BW in MHz, RAMP in ms)\n\r");
}

err_t recv_callback(void *arg, struct tcp_pcb *tpcb,
                               struct pbuf *p, err_t err)
{
	/* do not read the packet if we are not in ESTABLISHED state */
	if (!p) {
		tcp_close(tpcb);
		tcp_recv(tpcb, NULL);
		return ERR_OK;
	}


	if (p->len > 0) {
		char recv_buf[64];
		int bandwidth = 0, ramp_time = 0;
		u32 phase_incr = 0;

		memcpy(recv_buf, p->payload, p->len);
		recv_buf[p->len] = '\0';

		xil_printf("Received: %s\r\n", recv_buf);

		// Parse format: BW:5;RAMP:2000
		if (sscanf(recv_buf, "BW:%d;RAMP:%d", &bandwidth, &ramp_time) == 2) {
			xil_printf("Parsed BW = %d MHz, RAMP = %d ms\r\n", bandwidth, ramp_time);

			// Calculate phase increment
			float bw_hz = (float)bandwidth * 1000000.0;
			phase_incr = (u32)((bw_hz / DDS_CLK_FREQ) * ((u64)1 << PHASE_WIDTH));

			// Send to GPIO channels
			XGpio_DiscreteWrite(&Gpio, BANDWIDTH_CHANNEL, phase_incr);
			XGpio_DiscreteWrite(&Gpio, RAMP_TIME_CHANNEL, ramp_time);

			xil_printf("Wrote to GPIO: PhaseIncr=0x%08X, RampTime=%d\n\r", phase_incr, ramp_time);
		} else {
			xil_printf("Invalid format. Expected: BW:5;RAMP:2000\n\r");
		}

		tcp_recved(tpcb, p->len);
	}
	pbuf_free(p);
	return ERR_OK;
}

err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	static int connection = 1;


	tcp_recv(newpcb, recv_callback);


	tcp_arg(newpcb, (void*)(UINTPTR)connection);


	connection++;

	return ERR_OK;
}


int start_application()
{
	struct tcp_pcb *pcb;
	err_t err;
	unsigned port = 7;

	/* create new TCP PCB structure */
	pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
	if (!pcb) {
		xil_printf("Error creating PCB. Out of Memory\n\r");
		return -1;
	}

	/* bind to specified @port */
	err = tcp_bind(pcb, IP_ANY_TYPE, port);
	if (err != ERR_OK) {
		xil_printf("Unable to bind to port %d: err = %d\n\r", port, err);
		return -2;
	}

	/* we do not need any arguments to callback functions */
	tcp_arg(pcb, NULL);

	/* listen for connections */
	pcb = tcp_listen(pcb);
	if (!pcb) {
		xil_printf("Out of memory while tcp_listen\n\r");
		return -3;
	}

	/* specify callback to use for incoming connections */
	tcp_accept(pcb, accept_callback);

	xil_printf("TCP echo server started @ port %d\n\r", port);

	return 0;
}

int init_gpio()
{
	int status;
	XGpio_Config *cfg;

	cfg = XGpio_LookupConfig(GPIO_DEVICE_ID);
	if (cfg == NULL) {
		xil_printf("ERROR: XGpio_LookupConfig failed.\n\r");
		return XST_FAILURE;
	}

	status = XGpio_CfgInitialize(&Gpio, cfg, cfg->BaseAddress);
	if (status != XST_SUCCESS) {
		xil_printf("ERROR: XGpio_CfgInitialize failed.\n\r");
		return XST_FAILURE;
	}

	XGpio_SetDataDirection(&Gpio, BANDWIDTH_CHANNEL, 0x00);  // Output
	XGpio_SetDataDirection(&Gpio, RAMP_TIME_CHANNEL, 0x00);  // Output
	return XST_SUCCESS;
}

void setup_tcp_server()
{
	struct tcp_pcb  *pcb = tcp_new();
	tcp_bind(pcb , IP_ADDR_ANY  , TCP_PORT);
	pcb = tcp_listen(pcb);
	tcp_accept(pcb, accept_callback);
}
