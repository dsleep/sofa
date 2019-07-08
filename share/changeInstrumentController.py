import Sofa
import sys
from collections import namedtuple

HapticInstrument = namedtuple('HapticInstrument', 'activated node currentInstrument instrumentNodes')

class ChangeInstrumentController(Sofa.PythonScriptController):

	# called once the script is loaded
	def initGraph(self,node):
		self.r = self.getContext()
		self.haptics = []
		self.Hp1Down = 0;
		self.Hp2Down = 0;
		self.Tool_Target_1 = 0;
		self.Tool_Target_2 = 0;
		
		print 'ChangeInstrumentController: initGraph (python side)'
		sys.stdout.flush()
				
		childrenNodes = self.r.findChildNodes('haptic')
		for child in childrenNodes:
		
			InstrumentChildren = child.findChildNodes('instrument')
			currentHaptic = HapticInstrument( activated = False, node = child, currentInstrument = 0, instrumentNodes = InstrumentChildren )		
			
			i = 0
			for hapticChild in currentHaptic.instrumentNodes:
				hapticChild.setActive(i == 0)
				print('ChangeInstrumentController: initGraph: Instrument: ' + hapticChild.name)
				i += 1
			
			self.haptics.append(currentHaptic)
						
		sys.stdout.flush()		
		return 0			

	def changeInstrument(inHaptic):
		inHaptic.instrumentNodes[inHaptic.currentInstrument].setActive(0)
		inHaptic.currentInstrument = (inHaptic.currentInstrument+1)%len(inHaptic.currentInstrument)		
		print('Changing instrument TO: ' + inHaptic.node.name + 'Instr: ' + inHaptic.instrumentNodes[h.currentInstrument].name)
		inHaptic.haptics[inHaptic.currentInstrument].setActive(1)
		inHaptic.haptics[inHaptic.currentInstrument].init()	
		return 0		
	
	def IsValidIndex(Array,idx):
		if idx >= 0 and idx < len(Array):
			return True
		else:	
			return False		
			
	def changeInstrumentTo(inHaptic, Idx):
		if inHaptic.currentInstrument != Idx and IsValidIndex(inHaptic.instrumentNodes, Idx ):
			inHaptic.instrumentNodes[inHaptic.currentInstrument].setActive(0)
			inHaptic.currentInstrument = Idx
			print('Changing instrument TO: ' + inHaptic.node.name + 'Instr: ' + inHaptic.instrumentNodes[h.currentInstrument].name)
			inHaptic.haptics[inHaptic.currentInstrument].setActive(1)
			inHaptic.haptics[inHaptic.currentInstrument].init()	
		else:
			print('Instrument already changed')
			
		return 0		
		
		
	# key and mouse events; use this to add some user interaction to your scripts 
	def onKeyPressed(self,k):
	
		if c == 'C':
			if IsValidIndex(0,self.haptics):
				self.changeInstrument(self.haptics[0])		
		elif c == 'V':
			if IsValidIndex(1,self.haptics):
				self.changeInstrument(self.haptics[1])				
				
		print 'onKeyPressed '+k
		sys.stdout.flush()
		return 0 
		
		
	def onHaptic(self, c, d, px, py, pz):
		# (c,d,e) means button state: c means deviceID(0~1), d means ButtonState(0)
		# c means deviceID(0~1)
		# d means ButtonState(0->noButton; 1->FirstButtonDown; 2->SecondButtonDown; 
		# 3~10 means second button down and the user wants to switch to the "d-2"th instrument
		# px,py,pz means device position
		
		print 'onHaptic'
		sys.stdout.flush()		
		
		if c == 0 and d >= 1 and d!=2:
			self.Hp1Down = 1;
			self.Tool_Target_1 = d-2;		  
		elif c == 0 and d == 0 and self.Hp1Down == 1:
			self.Hp1Down = 0;
			if IsValidIndex(0,self.haptics) and self.Tool_Target_1>0:
				self.changeInstrumentTo(self.haptics[0],self.Tool_Target_1)
				self.Tool_Target_1 = 0
			elif IsValidIndex(0,self.haptics):
				self.changeInstrument(self.haptics[0])
				print('xyz:' + px + py + pz)		  
		elif c == 1 and d >= 1 and d!=2:
			self.Hp2Down = 1
			self.Tool_Target_2 = d-2		
		elif c == 1 and d == 0 and self.Hp2Down == 1:
			self.Hp2Down = 0;
			if IsValidIndex(self.haptics,1) and self.Tool_Target_2>0:
			  self.changeInstrumentTo(self.haptics[1],self.Tool_Target_2)
			  self.Tool_Target_2 = 0
			elif IsValidIndex(self.haptics,1):
			  self.changeInstrument(self.haptics[1])				
  
		return 0;
		
	def onKeyReleased(self,k):
		print 'onKeyReleased '+k
		sys.stdout.flush()
		return 0 

	def onMouseButtonLeft(self,x,y,pressed):
		#print 'onMouseButtonLeft x='+str(x)+' y='+str(y)+' pressed='+str(pressed)
		#sys.stdout.flush()
		return 0

	def onMouseButtonRight(self,x,y,pressed):
		#print 'onMouseButtonRight x='+str(x)+' y='+str(y)+' pressed='+str(pressed)
		#sys.stdout.flush()
		return 0

	def onMouseButtonMiddle(self,x,y,pressed):
		#print 'onMouseButtonMiddle x='+str(x)+' y='+str(y)+' pressed='+str(pressed)
		#sys.stdout.flush()
		return 0

	def onMouseWheel(self,x,y,delta):
		#print 'onMouseButtonWheel x='+str(x)+' y='+str(y)+' delta='+str(delta)
		#sys.stdout.flush()
		return 0

	def onMouseMove(self,x,y):
		#print 'onMouseMove x='+str(x)+' y='+str(y)
		#sys.stdout.flush()
		return 0

	
		