#!/usr/bin/python
#
# Copyright (c) 2014 Columbia University. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the
#   distribution.
# - Neither the name of the copyright holder nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
# THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Read Telosb Sensor Measurements from Serial (UART)
#
# @author Marcin Szczodrak, Sameer Lal
# @date   December 2014

import os
import sys
import time
import struct
from SensorRegress import Node
import SensorRegress

import TelosbMsg
from CollectSensorData import write_reading_to_file
from collections import defaultdict
from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial

#ensures that the minimum amount of data a node has
#is above the required training level
def check_minimum_training(nodes, training):
    minimum = 1000000
    for node_id in nodes.keys():
        if len(nodes[node_id].measurement_list) < minimum:
             minimum= len(nodes[node_id].measurement_list)

    if minimum % 1000 == 0:
        print "COLLECTED: "+str(minimum)+" SAMPLES FROM NODES..."

    if minimum < training:
        return False
    else:
        return True


class TelosbSerial:
    def __init__(self, uart):
        self.nodes = defaultdict()
        self.training_size = 50
        self.mif = MoteIF.MoteIF()
        self.tos_source = self.mif.addSource(uart)
        self.mif.addListener(self, TelosbMsg.TelosbMsg)
        self.predict_state = False

    def reset(self):
        for node in self.nodes:
            node.clear()

    def send_coefficients(self,node_id,coeffs):
        for co in coeffs:
            self.send(int(node_id),int(co[0]*1000),int(co[1]*1000),int(co[2]*1000),int(co[3]*1000))

    def run_regressions_send_coefficients(self):

        #send initial
        if not self.predict_state:
            self.send(0,0,0,0,0)

        #Conditional to check that network has switched to prediction state
        #then sends the coefficients to the root
        if self.predict_state:

            #send confirmation
            #self.send(999,999,999,999,999)

            #send coefficients for all nodes
            for node_id in self.nodes.keys():
                coefficients=SensorRegress.findNodeCoefficients(self.nodes[node_id])
                print coefficients
                self.send_coefficients(node_id,coefficients)


    def receive(self, src, msg):
        if msg.get_amType() == TelosbMsg.AM_TYPE:
            print msg
            src = msg.get_src()
            seq = msg.get_seq()
            hum = msg.get_hum()
            temp = msg.get_temp()
            light = msg.get_light()

            print time.time(), src, seq, hum, temp, light
            #write_reading_to_file(src, temp, light, hum)

            reset=False
            #Reset signal from root due to too high error
            if src==0 and seq==0 and hum==0 and temp==0 and light==0:
                reset=True
                self.predict_state=False
                self.reset()


            #Checking if predict_state
            if src==999 and seq==999 and hum==999 and temp==999 and light==999:
                self.predict_state=True

            #If not in the predict state then add measurement to the sensor
            if not self.predict_state and not reset:
                self.nodes = SensorRegress.addMeasurementToNode(self.nodes, src, temp, light, hum)

            if not self.predict_state and check_minimum_training(self.nodes, self.training_size):
                self.run_regressions_send_coefficients()

            #sys.stdout.flush()

    def send(self,id,src,hum,temp,light):
        #node_id = 1
        msg = TelosbMsg.TelosbMsg()
        #msg.set_rx_timestamp(time.time())
        msg.set_src(id)
        msg.set_seq(src)
        msg.set_hum(hum)
        msg.set_temp(temp)
        msg.set_light(light)
        self.mif.sendMsg(self.tos_source, id,
                         msg.get_amType(), 0, msg)

    def run(self):
        while 1:
            pass


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "\n\nusage %s <serial_device:boundrate>"
        print "\n\nex. $ %s serial@/dev/ttyUSB0:115200\n"
        sys.exit(1)
    g = TelosbSerial(sys.argv[1])

    try:
        g.run()
    except KeyboardInterrupt:
        pass

