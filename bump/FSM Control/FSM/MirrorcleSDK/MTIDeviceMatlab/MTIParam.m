classdef MTIParam
    enumeration
		HardwareFilterOn,			% read only
		VmaxMEMSDriver,				% read only
		DeviceState,				% read only
		DeviceErrorRegister,		% read only
		USARTBaudRate,
		SampleRate,
		HardwareFilterBw,
		VdifferenceMax,
		Vbias,
		OutputOffsets,
		DataScale,
        DataRotation,
		MEMSDriverEnable,
		DigitalOutputEnable,
        LaserModulationEnable,
		BufferOffset,
		DeviceAxes,
		BootSetting,
		DataMode,
		SyncMode,
        DataFormat,
		FramesPerSecond,
		InterpolationType,
		WaveformType,
    end
end