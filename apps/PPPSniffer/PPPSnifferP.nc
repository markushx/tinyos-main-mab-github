/*
 * Copyright (c) 2008 The Regents of the University  of California.
 * All rights reserved."
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*									tab:4
 * Copyright (c) 2000-2005 The Regents of the University  of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2002-2005 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */

/*
 * @author Markus Becker
 * @author Phil Buonadonna
 * @author Gilman Tolle
 * @author David Gay
 */

/*
 * PPPSnifferP bridges snooped packets between the radio and PPP over
 * serial/USB.
 */

#include <Tasklet.h>

module PPPSnifferP {
  uses {
    interface Boot;

    interface Packet;
    interface RadioReceive;
    interface RadioState;

    interface SplitControl as PppSplitControl;
    interface LcpAutomaton as Ipv6LcpAutomaton;
    interface PppIpv6;

    interface Leds;
  }
}

implementation
{
  enum {
    PPP_QUEUE_LEN = 10,
  };

  uint16_t serial_read;
  uint16_t radio_read;
  uint16_t serial_fail;
  uint16_t radio_fail;

  bool echo_busy;
  message_t echo_buf;

  message_t  pppQueueBufs[PPP_QUEUE_LEN];
  message_t  *pppQueue[PPP_QUEUE_LEN];
  uint8_t    pppIn, pppOut;
  bool       pppBusy, pppFull;
  bool       ppp_link_up;

  event void Ipv6LcpAutomaton.transitionCompleted (LcpAutomatonState_e state) { }
  event void Ipv6LcpAutomaton.thisLayerUp () { }
  event void Ipv6LcpAutomaton.thisLayerDown () { }
  event void Ipv6LcpAutomaton.thisLayerStarted () { }
  event void Ipv6LcpAutomaton.thisLayerFinished () { }

  event void PppSplitControl.startDone (error_t error) { }
  event void PppSplitControl.stopDone (error_t error) { }

  event void PppIpv6.linkUp ()
  {
      atomic {
	  ppp_link_up = TRUE;
      }
  }

  event void PppIpv6.linkDown ()
  {
      atomic {
          ppp_link_up = FALSE;
      }
  }

  event error_t PppIpv6.receive (const uint8_t* message,
                                 unsigned int len)
  {
    return SUCCESS;
  }

  event void Boot.booted() {
    uint8_t i;
    error_t rc;

    atomic {
	for (i = 0; i < PPP_QUEUE_LEN; i++)
	    pppQueue[i] = &pppQueueBufs[i];
	pppIn = pppOut = 0;
	pppBusy = FALSE;
	pppFull = TRUE;
	ppp_link_up = FALSE;
    }

    rc = call Ipv6LcpAutomaton.open();
    rc = call PppSplitControl.start();
    call RadioState.turnOn();
  }

  tasklet_async event void RadioState.done() {
  }

  /*
  event void RadioSplitControl.startDone(error_t error) {}
  event void RadioSplitControl.stopDone(error_t error) {}

  message_t* receive(message_t* msg, void* payload, uint8_t len);
  */
  // Set TI CC24xx FCS format in Wireshark:
  // Edit -> Preferences -> Protocols -> IEEE 802.15.4 ->
  // TI CC24xx FCS format
  /*
  event message_t *ActiveReceive.receive(message_t *msg,
					 void *payload,
					 uint8_t len) {
    printf("AM\n");
    //printf("active radio message received (%i)\n", len);
    // try everything to make wireshark think CRC is OK.
    // FIXME:
    ((uint8_t*)msg)[len+sizeof(cc2420_header_t)-2] = 0xff;
    ((uint8_t*)msg)[len+sizeof(cc2420_header_t)-1] = 0xff;

    return receive(msg, payload, len);
  }

  event message_t *BareReceive.receive(message_t *msg,
					void *payload,
					uint8_t len) {

    printf("15.4\n");
    // try everything to make wireshark think CRC is OK.
    // FIXME:
    ((uint8_t*)msg)[len+sizeof(cc2420_header_t)-2] = 0xff;
    ((uint8_t*)msg)[len+sizeof(cc2420_header_t)-1] = 0xff;

    return receive(msg, payload, len);
  }

  message_t* receive(message_t *msg, void *payload, uint8_t len) {
    message_t *ret = msg;
    //CHECK_NODE_ID NULL;

    // -1 remove length field
    // +2 add empty crc
    // = +1
    if (ppp_link_up == TRUE) {
	if (call PppIpv6.transmit((uint8_t*)msg+1, len-1+sizeof(cc2420_header_t)) == SUCCESS) {
	    //printf("ppp tx success\n");
	} else {
	    printf("ppp tx failed\n");
	}
    }

    return ret;
  }
  */

  task void ppptransmit() {
      message_t* msg;

      atomic {
	  if (pppIn == pppOut && !pppFull) {
	      pppBusy = FALSE;
	      return;
	  }

	  msg = pppQueue[pppOut];
      }

      if (call PppIpv6.transmit((uint8_t*)msg, //skip to cc2420header or ieee802154header
				128 // ((uint8_t*)(msg))[0] //figure out length, including 154 header
				) == SUCCESS) {
	  atomic {
	      if (msg == pppQueue[pppOut]) {
		  if (++pppOut >= PPP_QUEUE_LEN)
		      pppOut = 0;
		  if (pppFull)
		      pppFull = FALSE;
	      }
	  }
	  post ppptransmit();
      } else {
	  post ppptransmit();
      }
  }

  tasklet_async event bool RadioReceive.header(message_t* msg) {
    return TRUE;
  }

  tasklet_async event message_t* RadioReceive.receive(message_t* msg) {
    if (ppp_link_up == TRUE) {
      atomic {
	  if (!pppFull) {
	      pppQueue[pppIn] = msg;

	      pppIn = (pppIn + 1) % PPP_QUEUE_LEN;

	      if (pppIn == pppOut)
		  pppFull = TRUE;

	      if (!pppBusy) {
		  call Leds.led1Toggle();
		  post ppptransmit();
		  pppBusy = TRUE;
	      } else {
		  call Leds.led2Toggle();
	      }
	  } else {
	      post ppptransmit();
	      call Leds.led0Toggle();
	  }
      }
    }
    return msg;
  }
}
