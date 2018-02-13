

from Lima import Core,Fli

cam=Fli.Camera('/dev/fliusb0')
hwint = Fli.Interface(cam)
ct=Core.CtControl(hwint)
acq=ct.acquisition()
image=ct.image()
display=ct.display()
display.setNames('fli', 'fli')
display.setActive(True)
Core.DebParams.setModuleFlags(Core.DebModCamera)
Core.DebParams.setTypeFlags(Core.DebTypeTrace)

acq.setAcqNbFrames(3)

ct.prepareAcq()
ct.startAcq()


