#!/usr/bin/python3
import struct
import time
import usb.core
import usb.util as uu


VENDOR_ID=0xcafe
PRODUCT_ID=0xcaff

class MyDac(object):
    def __init__(self):
        self.dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        print(self.dev)
        print(self.dev.set_configuration())
        self.rt = uu.CTRL_OUT | uu.CTRL_RECIPIENT_INTERFACE | uu.CTRL_TYPE_VENDOR

    def enable(self, a, b):
        self.dev.ctrl_transfer(self.rt, 4, 0, a == True)
        self.dev.ctrl_transfer(self.rt, 4, 1, b == True)

    def sin(self, chan, freq=1000, ampl=0.5, offset=1):
        sformat = "<III"
        dat = struct.pack("<III", int(freq * 1000), int(ampl * 1000), int(offset * 1000))
        print(self.dev.ctrl_transfer(self.rt, 1, chan, 0, dat))

    def triangle(self, chan, freq=1000, ampl=0.5, offset=1):
        sformat = "<III"
        dat = struct.pack("<III", int(freq * 1000), int(ampl * 1000), int(offset * 1000))
        print(self.dev.ctrl_transfer(self.rt, 6, chan, 0, dat))

    def user_setup(self, chan, samps):
        """This just sets up wavetables, frequency is separate"""
        print(self.dev.ctrl_transfer(self.rt, 8, chan, 0, struct.pack("<I", len(samps))))
        print(self.dev.write(1, struct.pack("<%dh" % len(samps), *samps)))

    def user(self, chan, freq=1000):
        dat = struct.pack("<I", int(freq * 1000))
        print(self.dev.ctrl_transfer(self.rt, 9, chan, 0, dat))

    def led(self, on):
        if on:
            self.dev.ctrl_transfer(self.rt, 3, 1, 0)
        else:
            self.dev.ctrl_transfer(self.rt, 3, 0, 0)
    def buffer(self, a, b):
        self.dev.ctrl_transfer(self.rt, 5, 0, a == True)
        self.dev.ctrl_transfer(self.rt, 5, 1, b == True)

    def sync(self):
        self.dev.ctrl_transfer(self.rt, 7, 0, 0)


def test():
    d = MyDac()
    for i in range(20):
        d.led(True)
        time.sleep((100 + (10*i))/1000)
        d.led(False)
        time.sleep((100 + (1*i))/1000)

def test2():
    d = MyDac()
    d.sin(0, 500, 1, 0.5)
    d.user_setup(1, [200,400,100,400,600,400,200,250,250,250,800,400,300,250])
    d.user(1, 750)
        
if __name__ == "__main__":
    test2()
    
