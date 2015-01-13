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
  * Fennec Fox empty application driver
  *
  * @author: Marcin K Szczodrak
  * @updated: 02/04/2014
  */


#include <Fennec.h>
#include "TelosbSensorsML.h"

generic module TelosbSensorsMLP(process_t process) {
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

/* Serial Interfaces */
uses interface AMSend as SerialAMSend;
uses interface AMPacket as SerialAMPacket;
uses interface Packet as SerialPacket;
uses interface Receive as SerialReceive;
uses interface SplitControl as SerialSplitControl;

uses interface Read<uint16_t> as ReadHumidity;
uses interface Read<uint16_t> as ReadTemperature;
uses interface Read<uint16_t> as ReadLight;

uses interface Timer<TMilli> as Timer;
uses interface Leds;
}

implementation {

/* TelosbSensors app
 * Takes two parameters:
 * uint16_t dest : the address of the mote to which the sensor
 *			measurements should be send to
 *			default value: 0
 * uint16_t sampling_rate : the millisecond delay between consecutive
 *			rounds of sensors' sampling
 *			default value: 1024
 */

telosb_sensors_t *data = NULL;
void *serial_data = NULL;
message_t network_packet;
message_t serial_packet;
uint16_t dest;
uint16_t the_src;
telosb_sensors_t *d= NULL;
uint16_t* array;

task void report_measurements() {
	//call Leds.led1Toggle();
	//dbgs(F_APPLICATION, S_NONE, data->hum, data->temp, data->light, 0, 0);

	if (call SubAMSend.send(dest, &network_packet,
			sizeof(telosb_sensors_t)) != SUCCESS) {
		call Leds.led1Toggle();
		call Leds.led0Toggle();
		signal SubAMSend.sendDone(&network_packet, FAIL);
	}
}



task void send_serial_message() {
	//call Leds.led2Toggle();
	if (call SerialAMSend.send(BROADCAST, &serial_packet, sizeof(telosb_sensors_t)) != SUCCESS) {
		signal SerialAMSend.sendDone(&serial_packet, FAIL);
		call Leds.led0On();
	}
}

command error_t SplitControl.start() {
	array=(uint16_t *)malloc(4*sizeof(uint16_t));
	
	data = (telosb_sensors_t*)call SubAMSend.getPayload(&network_packet,
							sizeof(telosb_sensors_t));
	data->seq = 0;
	data->src = TOS_NODE_ID;

	call SerialSplitControl.start();

	serial_data = (void*) call SerialAMSend.getPayload(&serial_packet,
							sizeof(telosb_sensors_t));
	if (call TelosbSensorsMLParams.get_dest()) {
		dest = call TelosbSensorsMLParams.get_dest();
	} else {
		dest = TOS_NODE_ID;
	}
	dbg("Application", "TelosbSensors SplitControl.start() width dest: %d", dest);
	call Timer.startPeriodic(call TelosbSensorsMLParams.get_sampling_rate());
	signal SplitControl.startDone(SUCCESS);
	return SUCCESS;
}

command error_t SplitControl.stop() {
	dbg("Application", "TelosbSensors SplitControl.start()");
	signal SplitControl.stopDone(SUCCESS);
	return SUCCESS;
}

event void SubAMSend.sendDone(message_t *msg, error_t error) {}

event message_t* SubReceive.receive(message_t *msg, void* payload, uint8_t len) {
#ifdef TOSSIM
	telosb_sensors_t *d = (telosb_sensors_t*)payload;
	dbg("Application", "TelosbSensors Humidity: %d, Temperature: %d, Light: %d",
					d->hum, d->temp, d->light);
#endif

	d = (telosb_sensors_t*)payload;
	the_src=d->src;
	if(the_src==2){
		call Leds.led2Toggle();
	}

	memcpy(serial_data, payload, len);

	post send_serial_message();
	
	return msg;
}

event message_t* SubSnoop.receive(message_t *msg, void* payload, uint8_t len) {
	return msg;
}

event void Timer.fired() {
	data->seq++;
	call ReadHumidity.read();
}

event void ReadHumidity.readDone(error_t error, uint16_t val) {
        dbg("Application", "Application TelosbSensors ReadHumidity.readDone(%d %d)",
                                                        error, val);
	data->hum = val;
	call ReadTemperature.read();
}

event void ReadTemperature.readDone(error_t error, uint16_t val) {
        dbg("Application", "Application TelosbSensors ReadTemperature.readDone(%d %d)",
                                                        error, val);
	data->temp = val;
	call ReadLight.read();
}

event void ReadLight.readDone(error_t error, uint16_t val) {
        dbg("Application", "Application TelosbSensors ReadLight.readDone(%d %d)",
                                                        error, val);
	data->light = val;
	post report_measurements();
}

event void SerialSplitControl.startDone(error_t error) {
}

event void SerialSplitControl.stopDone(error_t error) {
}

event message_t* SerialReceive.receive(message_t *msg, void* payload, uint8_t len) {
	call Leds.led0Toggle();
	dbg("Application", "Application TelosbSensors SerialReceive()");
	
	d = (telosb_sensors_t*)payload;
	array[0]=d->src;
	array[1]=d->temp;
	array[2]=d->hum;
	array[3]=d->light;

	if(array[0]==0&&array[1]==0&&array[2]==0&&array[3]==0){
		//SWITCH TO NEW STATE!!
	}

	//memcpy(serial_data, payload, len);	
	
	//post send_serial_message();
	
	call Leds.led0Toggle();
	return msg;
}

event void SerialAMSend.sendDone(message_t *msg, error_t error) {
}

}
