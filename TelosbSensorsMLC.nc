/*
 * Copyright (c) 2009, Columbia University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the Columbia University nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COLUMBIA UNIVERSITY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
  * Fennec Fox TelosB Sensors Application
  *
  * @author: Marcin K Szczodrak
  * @updated: 02/04/2014
  */

generic configuration TelosbSensorsMLC(process_t process) {
provides interface SplitControl;

uses interface TelosbSensorsMLParams;

uses interface AMSend as SubAMSend;
uses interface Receive as SubReceive;
uses interface Receive as SubSnoop;
uses interface AMPacket as SubAMPacket;
uses interface Packet as SubPacket;
uses interface PacketAcknowledgements as SubPacketAcknowledgements;

uses interface PacketField<uint8_t> as SubPacketLinkQuality;
uses interface PacketField<uint8_t> as SubPacketTransmitPower;
uses interface PacketField<uint8_t> as SubPacketRSSI;
}

implementation {
components new TelosbSensorsMLP(process);
SplitControl = TelosbSensorsMLP;

TelosbSensorsMLParams = TelosbSensorsMLP;

SubAMSend = TelosbSensorsMLP.SubAMSend;
SubReceive = TelosbSensorsMLP.SubReceive;
SubSnoop = TelosbSensorsMLP.SubSnoop;
SubAMPacket = TelosbSensorsMLP.SubAMPacket;
SubPacket = TelosbSensorsMLP.SubPacket;
SubPacketAcknowledgements = TelosbSensorsMLP.SubPacketAcknowledgements;

components SerialActiveMessageC;
components new SerialAMSenderC(100);
components new SerialAMReceiverC(100);
TelosbSensorsMLP.SerialAMSend -> SerialAMSenderC.AMSend;
TelosbSensorsMLP.SerialAMPacket -> SerialAMSenderC.AMPacket;
TelosbSensorsMLP.SerialPacket -> SerialAMSenderC.Packet;
TelosbSensorsMLP.SerialSplitControl -> SerialActiveMessageC.SplitControl;
TelosbSensorsMLP.SerialReceive -> SerialAMReceiverC.Receive;

SubPacketLinkQuality = TelosbSensorsMLP.SubPacketLinkQuality;
SubPacketTransmitPower = TelosbSensorsMLP.SubPacketTransmitPower;
SubPacketRSSI = TelosbSensorsMLP.SubPacketRSSI;

components new TimerMilliC();
TelosbSensorsMLP.Timer -> TimerMilliC;

components LedsC;
TelosbSensorsMLP.Leds -> LedsC;

#ifndef TOSSIM 

components new SensirionSht11C();
TelosbSensorsMLP.ReadHumidity -> SensirionSht11C.Humidity;
TelosbSensorsMLP.ReadTemperature -> SensirionSht11C.Temperature;

components new HamamatsuS10871TsrC();
TelosbSensorsMLP.ReadLight -> HamamatsuS10871TsrC;

#else

/* Sensor Number 0 */
components new CapeInputC() as CapeHumidityC;
TelosbSensorsMLP.ReadHumidity -> CapeHumidityC.Read16;

/* Sensor Number 1 */
components new CapeInputC() as CapeTemperatureC;
TelosbSensorsMLP.ReadTemperature -> CapeTemperatureC.Read16;

/* Sensor Number 2 */
components new CapeInputC() as CapeLightC;
TelosbSensorsMLP.ReadLight -> CapeLightC.Read16;

#endif

}
