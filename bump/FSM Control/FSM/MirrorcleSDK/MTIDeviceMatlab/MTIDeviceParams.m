classdef MTIDeviceParams
    properties
	Success;								%read only
	DeviceName;                             %read only
	FirmwareName;                           %read only
	CommType;                           	%read only
	BluetoothMAC;                   		%read only
	HardwareFilterOn;						%read only
	VmaxMEMSDriver;                         %read only
	DeviceState;							%read only
	DeviceErrorRegister;					%read only
	USARTBaudRate;
	SampleRate;
	HardwareFilterBw;
	VdifferenceMax;
	Vbias;
	XOffset;
	YOffset;
	DataScale;
    DataRotation;
	MEMSDriverEnable;
	DigitalOutputEnable;
    LaserModulationEnable;
	BufferOffset;
	DeviceAxes;
	BootSetting;
	DataMode;
	SyncMode;
    DataFormat;
	DeviceLimits;
    end
end
