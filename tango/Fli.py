############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2011
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
#=============================================================================
#
# file :        Fli.py
#
# description : Python source for the Fli and its commands.
#                The class is derived from Device. It represents the
#                CORBA servant object which will be accessed from the
#                network. All commands which can be executed on the
#                Pilatus are implemented in this file.
#
# project :     TANGO Device Server
#
# copyleft :    European Synchrotron Radiation Facility
#               BP 220, Grenoble 38043
#               FRANCE
#
#=============================================================================
#         (c) - Bliss - ESRF
#=============================================================================
#
import PyTango
from Lima import Core
from Lima import Fli as FliAcq
from Lima.Server import AttrHelper


class Fli(PyTango.Device_4Impl):

    Core.DEB_CLASS(Core.DebModApplication, 'LimaCCDs')


#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------
    def __init__(self,*args) :
        PyTango.Device_4Impl.__init__(self,*args)
        
        self.__ExtTriggerLevel = {'LOW':0,
                                  'HIGH':1}       
        
        self.__Attribute2FunctionBase = {
            'temperature_sp': 'TemperatureSP',
            'temperature_ccd': 'TemperatureCCD',
            'temperature_base': 'TemperatureBase',
        }
        
        self.init_device()


#------------------------------------------------------------------
#    Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        pass

#------------------------------------------------------------------
#    Device initialization
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def init_device(self):
        self.set_state(PyTango.DevState.ON)
        self.get_device_properties(self.get_device_class())

        if self.temperature_sp:
            _FliInterface.setTemperatureSP(self.temperature_sp)

        if self.ext_trigger_level:
            _FliInterface.setExtTriggerLevel(self.__ExtTriggerLevel[self.ext_trigger_level])

#------------------------------------------------------------------
#    getAttrStringValueList command:
#
#    Description: return a list of authorized values if any
#    argout: DevVarStringArray   
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        #use AttrHelper
        return AttrHelper.get_attr_string_value_list(self, attr_name)
#==================================================================
#
#    Fli read/write attribute methods
#
#==================================================================
    def __getattr__(self,name) :
        #use AttrHelper
        return AttrHelper.get_attr_4u(self,name,_FliInterface)


#==================================================================
#
#    FliClass class definition
#
#==================================================================
class FliClass(PyTango.DeviceClass):

    class_property_list = {}

    device_property_list = {
        # define one and only one of the following 4 properties:
        'camera_path':
        [PyTango.DevString,
         "Camera device path", []],
        'temperature_sp':
        [PyTango.DevDouble,
         'Temperature set point in Celsius', []],
        'ext_trigger_level':
        [PyTango.DevString,
         'level of external trigger input ("LOW"/"HIGH")', []],                 
    }

    cmd_list = {
        'getAttrStringValueList':
        [[PyTango.DevString, "Attribute name"],
         [PyTango.DevVarStringArray, "Authorized String value list"]],
    }

    attr_list = {
        'cooler_power':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'unit': '%',
             'format': '%1f',
             'description': 'cooler power (%)',
         }],
        'ext_trigger_level':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'unit': 'N/A',
             'format': '',
             'description': 'external trigger input level, see manual for usage LOW or HIGH',
         }],
        'temperature_sp':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ_WRITE],
         {
             'unit': 'C',
             'format': '%1f',
             'description': 'temperature set-point (C)',
         }],
        'temperature_ccd':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'unit': 'C',
             'format': '%1f',
             'description': 'sensor temperature (C)',
         }],
        'temperature_base':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ],
         {
             'unit': 'C',
             'format': '%1f',
             'description': 'base (external) temperature (C)',
         }],       
    }

    def __init__(self,name) :
        PyTango.DeviceClass.__init__(self,name)
        self.set_type(name)

#----------------------------------------------------------------------------
# Plugins
#----------------------------------------------------------------------------
_FliCam = None
_FliInterface = None

def get_control(camera_path='/dev/fliusb0',**keys) :
    global _FliCam
    global _FliInterface

    print ("FLI camera path: ", camera_path)

    if _FliCam is None:
        _FliCam = FliAcq.Camera(camera_path)
        _FliInterface = FliAcq.Interface(_FliCam)
    return Core.CtControl(_FliInterface)

def get_tango_specific_class_n_device():
    return FliClass,Fli
