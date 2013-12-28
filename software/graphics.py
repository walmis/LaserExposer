from PySide.QtCore import *
from PySide.QtGui import *

class GfxScene(QGraphicsScene):
  scanLineChanged = Signal(int)
  onLineHover = Signal(int)
  
  def __init__(self):
    QGraphicsScene.__init__(self)
    
    self.pixmapItem = None
    
    self.image = None
    self.scanLine = QGraphicsLineItem()
    self.scanLine.setZValue(10)
   
    pen = QPen()
    pen.setColor(QColor(0, 0x99, 0xFF, 240))
    pen.setWidth(4)
    
    self.scanLine.setPen(pen)
    
    
    self.previewLine = QGraphicsLineItem()
    
    self.platformOutline = QGraphicsRectItem()
    
    pen = QPen()
    pen.setWidth(5);

    brush = QBrush()
    brush.setColor(QColor(0, 0x99, 0xff, 20))
    brush.setStyle(Qt.SolidPattern)
    self.platformOutline.setPen(pen)
    self.platformOutline.setBrush(brush)
        
    maxy_mm = 500
    maxx_mm = 250
    
    dotmm = 23.62
    self.platformOutline.setRect(0, 0, int(maxx_mm*dotmm), int(maxy_mm*dotmm))
    self.addItem(self.platformOutline)
    
    pen = QPen()
    pen.setColor(QColor(0xFF, 0x10, 0, 255))
    pen.setWidth(1)
    self.previewLine.setPen(pen)
    self.previewLine.setZValue(20)
    self.addItem(self.previewLine)
    
    self.setSceneRect(-400, -400, 8000, 10000)

    self.addItem(self.scanLine)
    #self.setActiveLine(20)
    
  def setImage(self, image):
    self.image = image
    self.image.marginTopChanged.connect(self.onMarginTopChanged)
    self.image.marginLeftChanged.connect(self.onMarginLeftChanged)
    
    pixmap = QPixmap(image)
    
    if self.pixmapItem:
      self.removeItem(self.pixmapItem)

    self.pixmapItem = self.addPixmap(pixmap)
    self.pixmapItem.setAcceptHoverEvents(True)
    
    self.pixmapItem.setCursor(Qt.CrossCursor)
    self.pixmapItem.setFlag(QGraphicsItem.ItemIsSelectable)

    self.onMarginLeftChanged(self.image.marginLeft)
    self.onMarginTopChanged(self.image.marginTop)
    
    self.zoomOnImage()
    
  def onMarginTopChanged(self, margin):
    self.pixmapItem.setY(23.62 * margin)
    
  def onMarginLeftChanged(self, margin):
    self.pixmapItem.setX(23.62 * margin)
    
  def zoomOnImage(self):
    for view in self.views():
      view.fitInView(self.pixmapItem, Qt.KeepAspectRatio)
      view.scale(0.9, 0.9)
      
  def setActiveLine(self, line):
    line += 23.62 * self.image.marginTop
    self.scanLine.setLine(23.62*self.image.marginLeft, line, 23.62*self.image.marginLeft + self.image.width(), line)
    self.onLineHover.emit(line - int(23.62 * self.image.marginTop))
    
  def setPreviewLine(self, line):
    #print "line", line
    #line += 23.62 * self.image.marginTop
    self.previewLine.setLine(23.62*self.image.marginLeft, int(line), 23.62*self.image.marginLeft + self.image.width(), int(line))
    self.onLineHover.emit(line - int(23.62 * self.image.marginTop))
    
  def onScanLineSelected(self, line):
    self.scanLineChanged.emit(self.previewLine.line().y1() - int(23.62 * self.image.marginTop))
    
  def mouseMoveEvent(self, event):
    pos = event.lastScenePos()
    self.setPreviewLine(pos.y())
    
    super(GfxScene, self).mouseMoveEvent(event)
    
  def drawBackground(self, painter, rect):
    #print "bg", rect
    pen = QPen()
    
    painter.fillRect(rect, Qt.white)
    
    dotmm = 23.62 # pixels per mm at 600 dpi
    
    pen.setWidth(1);
    pen.setColor(QColor(170,170,170)) 
    painter.setPen(pen);
    
    pos = rect.top() - rect.top() % dotmm
    while pos < rect.bottom():
      painter.drawLine(QPoint(rect.left(), int(pos)), QPoint(rect.right(), int(pos)))
      pos += dotmm  
      
    pen.setWidth(2);
    pen.setColor(QColor(130,130,130))  
    painter.setPen(pen);
    
    pos = rect.top() - rect.top() % (dotmm*5)
    while pos < rect.bottom():
      painter.drawLine(QPoint(rect.left(), int(pos)), QPoint(rect.right(), int(pos)))
      pos += dotmm*5
      
    pen.setWidth(3);
    pen.setColor(QColor(100,100,100))
    painter.setPen(pen);     
    
    pos = rect.top() - rect.top() % (dotmm*10)
    while pos < rect.bottom():
      painter.drawLine(QPoint(rect.left(), int(pos)), QPoint(rect.right(), int(pos)))
      pos += dotmm*10   
    
    ####### Draw vertical lines #######
    pen.setWidth(1);
    pen.setColor(QColor(170,170,170)) 
    painter.setPen(pen)

    pos = rect.left() - rect.left() % (dotmm)
    while pos < rect.right():
      painter.drawLine(QPoint(int(pos), rect.top()), QPoint(int(pos), rect.bottom()))
      pos += dotmm  
      
    pen.setWidth(2);
    pen.setColor(QColor(130,130,130))  
    painter.setPen(pen);
    
    pos = rect.left() - rect.left() % (dotmm*5)
    while pos < rect.right():
      painter.drawLine(QPoint(int(pos), rect.top()), QPoint(int(pos), rect.bottom()))
      pos += dotmm*5
      
    
    pen.setWidth(3);
    pen.setColor(QColor(100,100,100))
    painter.setPen(pen);     
    pos = rect.left() - rect.left() % (dotmm*10)
    while pos < rect.right():
      painter.drawLine(QPoint(int(pos), rect.top()), QPoint(int(pos), rect.bottom()))
      pos += dotmm*10 
    
class GfxView(QGraphicsView):
  def __init__(self):
    QGraphicsView.__init__(self)
    
    self.setDragMode(QGraphicsView.ScrollHandDrag)
    self.setRenderHint(QPainter.Antialiasing)
    self.setRenderHint(QPainter.SmoothPixmapTransform)
    self.setCacheMode(QGraphicsView.CacheBackground)
 
    #self.setInteractive(False)
    
  def mousePressEvent (self, event):
    #print "press", event
    pos = event.pos()
    items = self.items(pos)
    for item in items:
      if item.flags() & QGraphicsItem.ItemIsSelectable:
	pos = self.mapToScene(pos)
	self.scene().onScanLineSelected(int(pos.y()))
	return

    super(GfxView, self).mousePressEvent(event)
    
  def wheelEvent(self, event):
    if event.delta() > 0:
      self.scale(1.1, 1.1)
    else:
      self.scale(0.9, 0.9)