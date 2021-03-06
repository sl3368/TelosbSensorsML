#
# This class is automatically generated by mig. DO NOT EDIT THIS FILE.
# This class implements a Python interface to the 'TelosbMsg'
# message type.
#

import tinyos.message.Message

# The default size of this message type in bytes.
DEFAULT_MESSAGE_SIZE = 12

# The Active Message type associated with this message.
AM_TYPE = 100

class TelosbMsg(tinyos.message.Message.Message):
    # Create a new TelosbMsg of size 12.
    def __init__(self, data="", addr=None, gid=None, base_offset=0, data_length=12):
        tinyos.message.Message.Message.__init__(self, data, addr, gid, base_offset, data_length)
        self.amTypeSet(AM_TYPE)
    
    # Get AM_TYPE
    def get_amType(cls):
        return AM_TYPE
    
    get_amType = classmethod(get_amType)
    
    #
    # Return a String representation of this message. Includes the
    # message type name and the non-indexed field values.
    #
    def __str__(self):
        s = "Message <TelosbMsg> \n"
        try:
            s += "  [seq=0x%x]\n" % (self.get_seq())
        except:
            pass
        try:
            s += "  [src=0x%x]\n" % (self.get_src())
        except:
            pass
        try:
            s += "  [hum=0x%x]\n" % (self.get_hum())
        except:
            pass
        try:
            s += "  [temp=0x%x]\n" % (self.get_temp())
        except:
            pass
        try:
            s += "  [light=0x%x]\n" % (self.get_light())
        except:
            pass
        return s

    # Message-type-specific access methods appear below.

    #
    # Accessor methods for field: seq
    #   Field type: long
    #   Offset (bits): 0
    #   Size (bits): 32
    #

    #
    # Return whether the field 'seq' is signed (False).
    #
    def isSigned_seq(self):
        return False
    
    #
    # Return whether the field 'seq' is an array (False).
    #
    def isArray_seq(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'seq'
    #
    def offset_seq(self):
        return (0 / 8)
    
    #
    # Return the offset (in bits) of the field 'seq'
    #
    def offsetBits_seq(self):
        return 0
    
    #
    # Return the value (as a long) of the field 'seq'
    #
    def get_seq(self):
        return self.getUIntElement(self.offsetBits_seq(), 32, 1)
    
    #
    # Set the value of the field 'seq'
    #
    def set_seq(self, value):
        self.setUIntElement(self.offsetBits_seq(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'seq'
    #
    def size_seq(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'seq'
    #
    def sizeBits_seq(self):
        return 32
    
    #
    # Accessor methods for field: src
    #   Field type: int
    #   Offset (bits): 32
    #   Size (bits): 16
    #

    #
    # Return whether the field 'src' is signed (False).
    #
    def isSigned_src(self):
        return False
    
    #
    # Return whether the field 'src' is an array (False).
    #
    def isArray_src(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'src'
    #
    def offset_src(self):
        return (32 / 8)
    
    #
    # Return the offset (in bits) of the field 'src'
    #
    def offsetBits_src(self):
        return 32
    
    #
    # Return the value (as a int) of the field 'src'
    #
    def get_src(self):
        return self.getUIntElement(self.offsetBits_src(), 16, 1)
    
    #
    # Set the value of the field 'src'
    #
    def set_src(self, value):
        self.setUIntElement(self.offsetBits_src(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'src'
    #
    def size_src(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'src'
    #
    def sizeBits_src(self):
        return 16
    
    #
    # Accessor methods for field: hum
    #   Field type: int
    #   Offset (bits): 48
    #   Size (bits): 16
    #

    #
    # Return whether the field 'hum' is signed (False).
    #
    def isSigned_hum(self):
        return False
    
    #
    # Return whether the field 'hum' is an array (False).
    #
    def isArray_hum(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'hum'
    #
    def offset_hum(self):
        return (48 / 8)
    
    #
    # Return the offset (in bits) of the field 'hum'
    #
    def offsetBits_hum(self):
        return 48
    
    #
    # Return the value (as a int) of the field 'hum'
    #
    def get_hum(self):
        return self.getUIntElement(self.offsetBits_hum(), 16, 1)
    
    #
    # Set the value of the field 'hum'
    #
    def set_hum(self, value):
        self.setUIntElement(self.offsetBits_hum(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'hum'
    #
    def size_hum(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'hum'
    #
    def sizeBits_hum(self):
        return 16
    
    #
    # Accessor methods for field: temp
    #   Field type: int
    #   Offset (bits): 64
    #   Size (bits): 16
    #

    #
    # Return whether the field 'temp' is signed (False).
    #
    def isSigned_temp(self):
        return False
    
    #
    # Return whether the field 'temp' is an array (False).
    #
    def isArray_temp(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'temp'
    #
    def offset_temp(self):
        return (64 / 8)
    
    #
    # Return the offset (in bits) of the field 'temp'
    #
    def offsetBits_temp(self):
        return 64
    
    #
    # Return the value (as a int) of the field 'temp'
    #
    def get_temp(self):
        return self.getUIntElement(self.offsetBits_temp(), 16, 1)
    
    #
    # Set the value of the field 'temp'
    #
    def set_temp(self, value):
        self.setUIntElement(self.offsetBits_temp(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'temp'
    #
    def size_temp(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'temp'
    #
    def sizeBits_temp(self):
        return 16
    
    #
    # Accessor methods for field: light
    #   Field type: int
    #   Offset (bits): 80
    #   Size (bits): 16
    #

    #
    # Return whether the field 'light' is signed (False).
    #
    def isSigned_light(self):
        return False
    
    #
    # Return whether the field 'light' is an array (False).
    #
    def isArray_light(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'light'
    #
    def offset_light(self):
        return (80 / 8)
    
    #
    # Return the offset (in bits) of the field 'light'
    #
    def offsetBits_light(self):
        return 80
    
    #
    # Return the value (as a int) of the field 'light'
    #
    def get_light(self):
        return self.getUIntElement(self.offsetBits_light(), 16, 1)
    
    #
    # Set the value of the field 'light'
    #
    def set_light(self, value):
        self.setUIntElement(self.offsetBits_light(), 16, value, 1)
    
    #
    # Return the size, in bytes, of the field 'light'
    #
    def size_light(self):
        return (16 / 8)
    
    #
    # Return the size, in bits, of the field 'light'
    #
    def sizeBits_light(self):
        return 16
    
