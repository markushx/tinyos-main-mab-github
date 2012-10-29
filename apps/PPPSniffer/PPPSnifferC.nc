/*
 * Copyright (c) 2008 The Regents of the University  of California.
 * Copyright (c) 2002-2003 Intel Corporation
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
 */

/**
 * The TinyOS 2.x PPPSniffer snoops AM and bare 802.15.4 packets and
 * delivers them over ppp to the host computer. On the host wireshark
 * can pick the packets up on the ppp interface.
 *
 * Restructured according to apps/tests/rfxlink/RadioSniffer
 *
 * @author Markus Becker
 * @author Phil Buonadonna
 * @author Gilman Tolle
 * @author David Gay
 * @author Philip Levis
 * @date January 3 2011
 */

configuration PPPSnifferC
{
}

implementation {

#define UQ_METADATA_FLAGS       "UQ_METADATA_FLAGS"
#define UQ_RADIO_ALARM          "UQ_RADIO_ALARM"

    components MainC, PPPSnifferP, LedsC;//, AssertC;

    PPPSnifferP.Boot -> MainC;
    PPPSnifferP.RadioState -> RadioDriverLayerC;
    PPPSnifferP.RadioReceive -> RadioDriverLayerC;

    PPPSnifferP.Leds -> LedsC;

    // just to avoid a timer compilation bug
    components new TimerMilliC();

// -------- TimeStamping

    components new TimeStampingLayerC();
    TimeStampingLayerC.LocalTimeRadio -> RadioDriverLayerC;
    TimeStampingLayerC.SubPacket -> MetadataFlagsLayerC;
    TimeStampingLayerC.TimeStampFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];

// -------- MetadataFlags

    components new MetadataFlagsLayerC();
    MetadataFlagsLayerC.SubPacket -> RadioDriverLayerC;

// -------- RadioAlarm

    components new RadioAlarmC();
    RadioAlarmC.Alarm -> RadioDriverLayerC;

// -------- RadioDriver

#if defined(PLATFORM_IRIS) || defined(PLATFORM_MULLE) || defined(PLATFORM_MESHBEAN)
    components RF230DriverLayerC as RadioDriverLayerC;
    components RF230RadioP as RadioP;
#elif defined(PLATFORM_MESHBEAN900)
    components RF212DriverLayerC as RadioDriverLayerC;
    components RF212RadioP as RadioP;
#elif defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSA) || defined(PLATFORM_TELOSB)
    components CC2420XDriverLayerC as RadioDriverLayerC;
    components CC2420XRadioP as RadioP;
#elif defined(PLATFORM_UCMINI)
    components RFA1DriverLayerC as RadioDriverLayerC;
    components RFA1RadioP as RadioP;
#endif

    RadioDriverLayerC.PacketTimeStamp -> TimeStampingLayerC;
    RadioDriverLayerC.Config -> RadioP;

    RadioDriverLayerC.TransmitPowerFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];
    RadioDriverLayerC.RSSIFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];
    RadioDriverLayerC.TimeSyncFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];
    RadioDriverLayerC.RadioAlarm -> RadioAlarmC.RadioAlarm[unique(UQ_RADIO_ALARM)];

    /* Serial/PPP stack */
    components PppDaemonC;
    PPPSnifferP.PppSplitControl -> PppDaemonC;

#if defined(PLATFORM_TELOSB) || defined(PLATFORM_EPIC)
    components PlatformHdlcUartC as HdlcUartC;
#else
    components DefaultHdlcUartC as HdlcUartC;
#endif
    PppDaemonC.HdlcUart -> HdlcUartC;
    PppDaemonC.UartControl -> HdlcUartC;

    /* Link in RFC5072 support for both the control and network protocols */
    components PppIpv6C;
    PppDaemonC.PppProtocol[PppIpv6C.ControlProtocol] -> PppIpv6C.PppControlProtocol;
    PppDaemonC.PppProtocol[PppIpv6C.Protocol] -> PppIpv6C.PppProtocol;
    PppIpv6C.Ppp -> PppDaemonC;
    PppIpv6C.LowerLcpAutomaton -> PppDaemonC;
    PPPSnifferP.Ipv6LcpAutomaton -> PppIpv6C;
    PPPSnifferP.PppIpv6 -> PppIpv6C;
}
