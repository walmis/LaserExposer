import usb.core
import usb.util

from PySide.QtCore import *
from PySide.QtGui import *

class UsbDevice(QObject):
	
	onOpened = Signal(bool)
	
	open = Property(bool, notify=onOpened)
	
	def __init__(self, vendor, product):
		QObject.__init__(self)
		
		self._opened = False
		
		self.vendor = vendor
		self.product = product
		
		self.interface = 2
		self.out_endpoint = 0x05
		self.in_endpoint = 0x82
		
		self.out_ep = None
		self.in_ep = None
		
		self.startTimer(500)
		
	def timerEvent(self, event):
		if self.opened and not self.isAttached():
			self.close()
				
			
	@Property(bool)	
	def opened(self):
		return self._opened
		
	def isAttached(self):
		dev = usb.core.find(idVendor=self.vendor, idProduct=self.product)
		return dev is not None
		
	def close(self):
		self._opened = False
		self.onOpened.emit(False)		
		
	def open(self):
		
		dev = usb.core.find(idVendor=self.vendor, idProduct=self.product)
		
		if dev is None:
			raise IOError("Device not found")
			
		try:
			if dev.is_kernel_driver_active(1):
				dev.detach_kernel_driver(1)			
		except:
			pass			
		
		dev.set_configuration()
		
		cfg = dev.get_active_configuration()

		intf = usb.util.find_descriptor(cfg, bInterfaceNumber = self.interface)
		
		print "USB Device opened successfully"
		for ep in intf:
			if ep.bEndpointAddress == self.out_endpoint:
				self.out_ep = ep
			
			if ep.bEndpointAddress == self.in_endpoint:
				self.in_ep = ep
		
		self._opened = True
		self.onOpened.emit(True)
				
	def read(self, length, **kwargs):
		if self._opened:
			return self.in_ep.read(length, **kwargs).tostring()
		else:
			return 0
		
	def write(self, data, **kwargs):
		if self._opened:
			return self.out_ep.write(data, **kwargs)
