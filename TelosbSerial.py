#!/usr/bin/python
#
# Copyright (c) 2012 Columbia University. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# - Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
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
# @author Marcin Szczodrak
# @date   June 05 2012

import os
import sys
import time
import struct

import TelosbMsg
from CollectSensorData import write_reading_to_file
from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial

class TelosbSerial:
	def __init__(self, uart):
		self.mif = MoteIF.MoteIF()
		self.tos_source = self.mif.addSource(uart)
		self.mif.addListener(self, TelosbMsg.TelosbMsg)
		self.received=True

	def receive(self, src, msg):
		if msg.get_amType() == TelosbMsg.AM_TYPE:
			print msg
	                src = msg.get_src()
	                seq = msg.get_seq()
        	        hum = msg.get_hum()
        	        temp = msg.get_temp()
        	        light = msg.get_light()

			print time.time(), src, seq, hum, temp, light
			write_reading_to_file(src,temp,light,hum)
			if(self.received):
				self.received=False
				#self.send()
			else:
				self.received=True

		#sys.stdout.flush()

	def send(self):
		node_id = 1
		msg = TelosbMsg.TelosbMsg()
		#msg.set_rx_timestamp(time.time())
		msg.set_src(5)
		msg.set_seq(5)
		msg.set_hum(5)
		msg.set_temp(5)
		msg.set_light(5)
		self.mif.sendMsg(self.tos_source, node_id,
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
