The instructions are quite simple. You just have to download and extract this (https://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.21/libusb-1.0.21.7z/download) in the c:/opt folder (you will have to create a new folder named libusb inside the opt folder). 

After this in the cmake GUI just select the AAOmni plugin, and click configure. 2 new variables should come up LIBUSB_INCLUDE_DIR, which should be set to C:/opt/libusb/include/libusb-1.0 and LIBUSB_LIBRARY_DIR which should be set to C:\opt\libusb-1.0.21\MS64\static. Then generate using the cmake gui.
The project should build without any problems.
 
In the scene file, change the NewOmniDriver to AAOmniDriver and add AAOmni as a required plugin as shown in the snippet below:
  <RequiredPlugin pluginName="AAOmni" />
  <RequiredPlugin pluginName="SurfLabHaptic" />
  <RequiredPlugin pluginName="SaLua" />
  <LuaController listening="1" source="changeInstrumentController.lua" />
  <Node name="PHANToM 1" tags="haptic">
    <Node name="RigidLayer">
      <AAOmniDriver alignOmniWithCamera="false" desirePosition="-1.2699999809265137 0.052796244621276855 3.5999996662139893 " deviceName="PHANToM 1" forceScale="0.0010000000474974513" listening="true" name="driver" orientationBase="0.7071067690849304 0.0 0.0 0.7071067690849304 " permanent="true" positionBase="-1.2699999809265137 0.052796244621276855 3.5999996662139893 " scale="72.0" tags="PHANToM 1__omni" />


