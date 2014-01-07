from PySide.QtUiTools import QUiLoader
from PySide.QtCore import *
from PySide.QtGui import *
import sys
from math import *
import struct
from array import array
import atexit
from collections import deque
from UsbDevice import UsbDevice
import time

from pdfrender import renderPdf

from graphics import GfxScene, GfxView

#from popplerqt4 import Poppler

px_mm = 23.62 # pixels per mm at 600 dpi

try:
  import matplotlib 
  import numpy as np

  matplotlib.use('Qt4Agg')
  matplotlib.rcParams['backend.qt4']='PySide'

  from matplotlib.figure import Figure
  from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

  class MatplotlibWidget(FigureCanvas):

      def __init__(self, parent=None,xlabel='x',ylabel='y',title='Title'):
	  super(MatplotlibWidget, self).__init__(Figure())

	  self.setParent(parent)
	  self.figure = Figure()
	  self.canvas = FigureCanvas(self.figure)
	  self.axes = self.figure.add_subplot(111)

	  self.axes.set_xlabel(xlabel)
	  self.axes.set_ylabel(ylabel)
	  self.axes.set_title(title)
except:
  print "matplotlib is not available"


class Math(QObject):
  
  scan_angle = 0
  t_rad = 0
  min_pos = 0
  max_pos = 0  
  
  def __init__(self, settings):
    QObject.__init__(self)
    
    self.settings = settings
    
    self.onSettingsChanged()
    
    self.settings.changed.connect(self.onSettingsChanged)
    
  def onSettingsChanged(self):
    s = self.settings
    
    self.scan_angle = ((4 * pi) / s.facets) * s.mirror_duty_cycle
    
    print "Scan angle:", degrees(self.scan_angle), "deg"

    #time per radian
    self.t_rad = s.scan_period / self.scan_angle
  
    #scan line angle
    print "Time per radian:", self.t_rad, "us"
    
    coef = 0.5 + s.pos_offset / 2

    alpha = (self.scan_angle * coef)
    alpha2 = (self.scan_angle * (1 - coef))
    
    self.min_pos = s.distance*tan(-alpha)
    self.max_pos = s.distance*tan(alpha2) - self.min_pos


  def position(self, x):
    return atan((x + self.min_pos) / self.settings.distance)

  def tdiff(self, x1, x2):
    return (self.position(x2) - self.position(x1)) * self.t_rad
  
  
  def updateGraph(self, plot):
    try:
      plot.axes.clear()
      
      x = np.arange(0, self.max_pos)
      
      y = []
      for _x in x:
	y.append(self.tdiff(_x, _x+1))
      
      plot.axes.plot(x, y)
      
      plot.draw()
      
      print self.tdiff(0, self.max_pos)
      print "sum", np.sum(y)    
      
    except Exception, e:
      print e
    
  pos = 0  
  data = array("H")
  
  def reset(self):
    self.pos = 0
    self.data = array("H")
    self.totaleError = 0
    
  def add(self, move, laser):
    
    singlePeriod = 41.67/1000
   
    t = self.tdiff(self.pos, self.pos+move)
    
    p = t/singlePeriod
    
    self.totaleError += (p - round(p))
    
    self.data.append( (int(round(p)) << 1) | (int(laser) & 1) )
    
    self.pos += move

  def getPattern(self):
      
    return self.data
  
  
  def genTestPattern(self, dist):
    s = self.settings

    laser = False

    totalTime = 0
    
    d = 0
    
    self.reset()
    
    self.add(dist, False)
    
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    self.add(0.2, True)
    self.add(9.8, False)
    print "error", self.totaleError
      
    return self.getPattern()

class Device(UsbDevice):
  period = 0    
  
  def __init__(self):
    UsbDevice.__init__(self, 0xffff, 0x2012)
    
    self.interface = 1
    self.out_endpoint = 0x02
    self.in_endpoint = 0x82
    
    self.open()

    self.write("\n")
    
    self.scanning = False

    self.scansPerLine = 1 
    
    self.autoPositionIncrement(False)

  def isScanning(self):
    return self.scanning
    
  def isReady(self):
    return self.period != 0
    
  def start(self, freq=1000):
    self.period = 0
    self.write("start %d\n" % freq)
    
  periodChanged = Signal(int)
  
  scanStarted = Signal()
  currentScanLine = Signal(int)
  scanFinished = Signal()
  
    
  def updatePeriod(self):
    self.write("period\n")
    self.period = int(self.read(20, timeout=50))
    if self.period != 0:
      self.periodChanged.emit(self.period)    
    
  def autoPositionIncrement(self, enabled):
    self.write("stepper autoinc %d\n" % (1 if enabled else 0))
    
    
  def uploadImage(self, image, scansPerLine = 0):

    class imageProcessor(QThread):
      def __init__(self, image):
	QThread.__init__(self)
	
	self.image = image
	self.data = deque()
	self.sem = QSemaphore()
	
	self.start()
      
      def run(self):
	for i in xrange(0, self.image.height()):
	  line = self.image.getLine(i)
	  self.data.append(line)
	  self.sem.release(1)
	  
      def getLine(self):
	self.sem.acquire(1)
	return self.data.popleft()

    class Thread(QThread):
     
      def __init__(self, parent, image):
	QThread.__init__(self)
	self.device = parent
	self.image = image
      
      def run(self):
	self.device.clearLine()
	self.device.setStepSpeed(2)
	
	proc = imageProcessor(self.image)
	
	if self.image.marginTop != 0:
	  self.device.moveTo(int(px_mm * self.image.marginTop))
	  self.device.wait()
	else:
	  self.device.home()
	  self.device.wait()

	speed = int((self.device.scansPerLine * (self.device.period/1000)))
	if speed < 2:
	  speed = 2
	self.device.setStepSpeed(speed)

	for i in xrange(0, self.image.height()):
	 if not self.device.scanning:
	   break
	 
	 self.device.currentScanLine.emit(i)
	 line = proc.getLine()
	 self.device.sendPattern(line)
	 
	self.device.autoPositionIncrement(False)
	self.device.scanFinished.emit()
	self.device.clearLine()
	self.device.home()
	self.device.scanning = False
	
	proc.wait()
	 
    self.scanning = True
    self.scanStarted.emit()
    self.autoPositionIncrement(True)
    self.setScansPerLine(scansPerLine)

    self.upload_thread = Thread(self, image)
    self.upload_thread.start()
    
  def moveTo(self, stepPos):
    self.write("stepper move %d\n" % stepPos)
    #self.serial.flush()
    
  def wait(self):
    while 1:
      self.write("stepper isbusy\n")
      busy = int(self.read(2))
      if not busy:
	return
      time.sleep(0.05)

   
  def cancelScan(self):
    self.scanning = False

  def sendPattern(self, pattern):
    buf = array("B")
    
    buf.fromstring("linedata\n")
    buf.fromstring(struct.pack("H", len(pattern)))
    buf.fromstring(pattern.tostring())
    
    self.write(buf.tostring())
    
  def setStepSpeed(self, stepDelay):  
    print "Setting speed", stepDelay
    self.write("stepper speed %d\n" % stepDelay)
    
  def home(self):
    self.setStepSpeed(2)
    self.write("stepper home\n")
    
  def setLaserPower(self, power):
    if power > 200:
      power = 200
    self.write("laser power %d\n" % power)
    
  def laserFocus(self, pos):
    self.write("laser focus %d\n" % pos)
  
  def laserFocusHome(self):
      self.write("laser focus -100\n")
      
  def setScansPerLine(self, value):
    print "Scans/Line", value
    self.scansPerLine = value
    self.write("numscans %d\n" % value)
    
  def timerEvent(self, arg):
    if self.period == 0:
      self.updatePeriod()
      
    super(Device, self).timerEvent(arg)
      
  def clearLine(self):
    self.write("clear\n")
    
  def disable(self):
    self.write("mm off\n")
    

class Settings(QObject):
  
  facets = 7
  mirror_duty_cycle = 0.956
  distance = 82.5
  motor_clk = 500
  f_systick = 96000000
  scan_period = 860
  pos_offset = 0.0
  
  def __init__(self):
    QObject.__init__(self)
    
    self.settings = QSettings("walmis", "LaserExposer")
    
    self.facets = float(self.settings.value("facets", Settings.facets))
    self.mirror_duty_cycle = float(self.settings.value("mirror_duty_cycle", Settings.mirror_duty_cycle))
    self.distance = float(self.settings.value("distance", Settings.distance))
    self.motor_clk = int(self.settings.value("motor_clk", Settings.motor_clk))
    self.f_systick = int(self.settings.value("f_systick", Settings.f_systick))
    self.scan_period = int(self.settings.value("scan_period", Settings.scan_period))
    self.pos_offset = float(self.settings.value("pos_offset", Settings.pos_offset))
    
  def getStore(self):
    return self.settings
  
  @Signal
  def changed(self): pass

  def set(self, prop, value):
    print "set", prop, value
    self.__dict__[prop] = value
    self.settings.setValue(prop, value)
    self.changed.emit()




class Image(QObject, QImage):
  processProgress = Signal(int)
  
  marginLeftChanged = Signal(int)
  marginTopChanged = Signal(int)
  
  def __init__(self, parent=None):
    QObject.__init__(self, parent=parent)
    QImage.__init__(self, parent=parent)
    
    self.imageData = []
    
    self.marginLeft = 0
    self.marginTop = 0
    
  def load(self, filename):
    
    if filename.endswith(".pdf"):
      
      class PdfDialog(QDialog):
	def __init__(self):
	  QDialog.__init__(self)

	  self.setWindowTitle("PDF Render")
	  self.setSizeGripEnabled(False)
	  
	  self.crop = QCheckBox()
	  self.page = QSpinBox()
	  form = QFormLayout(self)
	  form.addRow(QLabel("PDF Render"))
	  form.addRow(QLabel("Render page:"), self.page)
	  form.addRow("Crop to image:", self.crop)
	  
	  self.btn = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
	  self.btn.accepted.connect(self.accept)
	  self.btn.rejected.connect(self.reject)
	  form.addRow(self.btn)
	  
	def getPage(self):
	  return self.page.value()
	  
	def getCrop(self):
	  return self.crop.isChecked()
	  
      
      dialog = PdfDialog()
      
      if dialog.exec_():
	try:
	  pdfimg = renderPdf(str(filename), dialog.getPage(), 600, 600)
	except Exception, e:
	  msg = QMessageBox.critical(dialog, "Import error", str(e))
	  return

	img = QImage(pdfimg.data(), pdfimg.width(), pdfimg.height(), QImage.Format_ARGB32)
	
	if dialog.getCrop():
	  (x, y, width, height) = pdfimg.getCropBounds()
	  
	  img = img.copy(x, y, width, height)
	  self.swap(img)
	  
	else:
	  self.swap(img)
      
    else:
      super(Image, self).load(filename)
    
    if self.format() != QImage.Format_Mono:
      print "Image is not 1bit, converting"
      self.swap(self.convertToFormat(QImage.Format_Mono, flags=Qt.MonoOnly))
    
    self.math = self.parent().math
    
    self.setColorTable([(0x20 << 24), (0xFF << 24) | 0x0066FF])
    print self.color(0)

    
  def setScale(self, scale):
    pass
  
  def setMarginTop(self, margin):
    self.marginTop = margin
    self.marginTopChanged.emit(margin)
    
  def setMarginLeft(self, margin):
    self.marginLeft = margin
    self.marginLeftChanged.emit(margin)
    
    
  def getLine(self, line, margin=0):
      invert = False
      
      self.math.reset()
      #distances.insert(0, (40, False)) 
      self.math.add(self.marginLeft, False)
      print "line %d/%d" % (line, self.height())
      
      dotmm = 1.0/px_mm
      
      dist = 0.0
      
      line = self.constScanLine(line)
      #print repr(str(line))
      
      dot = ord(line[0])>>7
      
      bits = range(7, -1, -1)
      count = 0
      
      for data in line:
	num = ord(data)
	
	for b in bits:
	  if count == self.width():
	    break
	  count+=1
	  
	  #print b,
	  #print (num >> b) & 1,
	  if (num >> b) & 1:
	    if dot:
	      dist+= dotmm
	    else:
	      self.math.add(dist, invert)
	      #distances.append((dist, not invert))
	      dist = dotmm
	      dot = True
	      
	  else:
	    if dot:
	      self.math.add(dist, not invert)
	      #distances.append((dist, invert))
	      dist = dotmm	
	      dot = False
	    else:
	      dist += dotmm
      
      self.math.add(dist, dot)      
      #print dist, dot, count      
      #print self.math.getPattern()
	      
      return self.math.getPattern()


class App(QApplication):
	def __init__(self):
		QApplication.__init__(self, sys.argv)

		loader = QUiLoader()
		
		self.window = loader.load("interface.ui")
		self.window.progressBar.hide()
		
		self.settings = Settings()
		self.math = Math(self.settings)
		
		try:
		  self.plot = MatplotlibWidget()
		  self.window.plot.addWidget(self.plot)
		except:
		  pass
		
		self.settings.changed.connect(self.onSettingsChanged)
		
		try:
		  self.device = Device()
		except IOError, e:
		  print e
		  QMessageBox.warning(self.window, "LaserExposer", "Device not connected")
		  return
		
		self.device.math = self.math
				
		self.onSettingsChanged()

		
		self.image = Image(parent=self)

		self.gfxView = GfxView()
		
		self.window.verticalLayout.insertWidget(0, self.gfxView)
		
		self.scene = GfxScene()
		
		self.image.setMarginLeft(self.window.marginValue.value())
		self.image.setMarginTop(self.window.marginTop.value())
		
		self.gfxView.setScene(self.scene)

		
		self.scene.onLineHover.connect(self.onLineHover)
		self.scene.scanLineChanged.connect(lambda line: self.window.line.setValue(line))
		
	
		self.device.start(self.settings.motor_clk)
		
		self.device.periodChanged.connect(lambda x: self.settings.set("scan_period", x))		
		
		self.window.facets.setValue(self.settings.facets)
		self.window.dutycycle.setValue(self.settings.mirror_duty_cycle)
		self.window.distance.setValue(self.settings.distance)
		self.window.m_clock.setValue(self.settings.motor_clk)
		self.window.posOffset.setValue(self.settings.pos_offset)
		
		self.window.facets.valueChanged.connect(lambda x: self.settings.set("facets", x))	
		self.window.dutycycle.valueChanged.connect(lambda x: self.settings.set("mirror_duty_cycle", x))
		self.window.distance.valueChanged.connect(lambda x: self.settings.set("distance", x))
		self.window.m_clock.valueChanged.connect(lambda x: self.settings.set("motor_clk", x) or self.device.start(x))
		self.window.posOffset.valueChanged.connect(lambda x: self.settings.set("pos_offset", x))
		
		self.window.homeBtn.clicked.connect(lambda: self.device.home())
		
		self.window.line.valueChanged.connect(self.onLine)
		
		self.window.b_testpatt.clicked.connect(self.onTestPattern)
		self.window.testDist.valueChanged.connect(self.onTestPattern)
		
		self.window.focusHome.clicked.connect(lambda: self.device.laserFocusHome())
		self.window.laserFocus.valueChanged.connect(lambda val: self.device.laserFocus(val))
		
		
		self.window.scansLine.valueChanged.connect(lambda val: self.device.setScansPerLine(val))
		self.window.scansLine.setValue(int(self.settings.getStore().value("scansPerLine", 32)))
		self.window.scansLine.valueChanged.connect(lambda val: self.settings.getStore().setValue("scansPerLine", val))
		
		self.window.scanBtn.clicked.connect(self.onScanPressed)
		
		### INVERT IMAGE ###
		self.window.invertImg.stateChanged.connect(self.onInvertChanged)
		
		### LOAD IMAGE ###
		self.window.loadImage.clicked.connect(self.onLoadImage)
		
		## MARGINS
		self.window.marginValue.valueChanged.connect(lambda margin: self.image.setMarginLeft(margin))
		self.window.marginTop.valueChanged.connect(lambda margin: self.image.setMarginTop(margin))
		
		## IMAGE SCALE
		self.window.imgScale.valueChanged.connect(lambda scale: self.image.setScale(scale/100.0))
		
		self.device.currentScanLine.connect(self.onDeviceScanLineChanged)
		self.device.scanStarted.connect(self.onScanStarted)
		self.device.scanFinished.connect(self.onScanFinished)
		
		### LASER POWER ###		
		self.window.laserPower.valueChanged.connect(lambda value: self.device.setLaserPower(value))
		self.window.laserPower.setValue(int(self.settings.getStore().value("laserPower", 70)))
		
		self.window.laserPower.valueChanged.connect(lambda value: self.settings.getStore().setValue("laserPower", value))
		####
		
		atexit.register(lambda: self.device.disable())
		
		self.device.home()
		self.device.wait()
		
		self.window.show()
		self.exec_()
		
	def onScanStarted(self):
	  self.window.progressBar.show()
	  self.window.progressBar.setRange(0, self.image.height())
	  self.window.scanBtn.setText("Cancel")
	  
	def onScanFinished(self):
	  self.window.progressBar.hide()
	  self.window.scanBtn.setText("Scan")
		
	def onDeviceScanLineChanged(self, line):
	  self.scene.setActiveLine(line)
	  self.window.progressBar.setValue(line)
		
	def onScanPressed(self):
	  if self.device.isScanning():
	    self.device.cancelScan()
	  else:
	    self.device.uploadImage(self.image, self.window.scansLine.value())
		
	def onInvertChanged(self, state):
	  self.image.invertPixels()
	  self.scene.setImage(self.image)
		
	def onLoadImage(self):
	  dialog = QFileDialog()
	  dialog.setFileMode(QFileDialog.ExistingFile)
	  
	  filt = "Images ("
	  
	  for fmt in QImageReader.supportedImageFormats():
	    filt += "*.%s " % str(fmt)
	  
	  try:
	    if renderPdf:
	      filt += "*.pdf "  
	  except NameError:
	    pass
	  
	  filt += ")"
	  dialog.setNameFilter(filt)
	    
	  dialog.restoreState(self.settings.getStore().value("fileDialogState"))
	  
	  dialog.exec_()
	  
	  self.settings.getStore().setValue("fileDialogState", dialog.saveState())
	  
	  f = dialog.selectedFiles()
	  if len(f):
	    self.image.load(f[0])
	    
	    if self.window.invertImg.checkState():
	      self.image.invertPixels()
	    
	    self.scene.setImage(self.image)
	    
	  self.window.lblWidth.setText("%.2f mm" % (self.image.width()/px_mm))
	  self.window.lblHeight.setText("%.2f mm" % (self.image.height()/px_mm))
	
	def onLineHover(self, line):
	  dotmm = px_mm
	  
	  self.window.lineInfo.setText("> Line: %d (+%.2f mm)" % (line, line/dotmm))
	
		
	def onLine(self, d):
	  print "line", d
	  
	  self.device.clearLine()
	  self.scene.setActiveLine(d)
	  line = self.image.getLine(d)
	  
	  if self.window.goBtn.isChecked():
	    self.device.moveTo(d + int(round(self.image.marginTop*px_mm)))
	  
	  self.device.wait()
	  #turn off limited exposure
	  self.device.setScansPerLine(0)
	  self.device.sendPattern(line)

	  
		
	def onTestPattern(self):
	  if self.device.isReady():
	    #turn off limited exposure
	    self.device.setScansPerLine(0)
	    
	    pat = self.math.genTestPattern( self.window.testDist.value() )
	    print pat 
	    
	    self.device.sendPattern(pat)
	  
		
	def onSettingsChanged(self):
	  
	  self.window.scanAngle.setText("%.3f deg" % degrees(self.math.scan_angle))	
	  self.window.period.setText("%d us" % self.settings.scan_period)
	  
	  self.window.tdeg.setText("%.3f us" % (self.math.t_rad * pi/180.0))
	  self.window.maxPos.setText("%.2f mm" % self.math.max_pos)
	  self.window.minPos.setText("%.2f mm" % self.math.min_pos)
	  
	  self.onTestPattern()
	  
	  try:
	    self.math.updateGraph(self.plot)
	  except:
	    pass
		
app = App()