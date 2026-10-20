classdef MTIAvailableDevices
    properties
	DeviceName = cell(1,12);			%read only
	FirmwareName = cell(1,12);		%read only
	CommType = cell(1,12);			%read only
	USARTBaudRate = cell(1,12);		%read only
	CommPortNumber = cell(1,12);		%read only
	CommPortName = cell(1,12);		%read only
	VmaxMEMSDriver = cell(1,12);		%read only
	NumDevices;			%read only
    end
end
