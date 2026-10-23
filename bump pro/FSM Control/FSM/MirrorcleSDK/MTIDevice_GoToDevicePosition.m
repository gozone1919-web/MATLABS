%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mirrorcle Technologies, Inc. 2011-2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MTIDevice version 10.6.0.0
% Compatible with Firmware 4.x or newer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MTIDevice_GoToPosition demonstrates the use of the GoToDevicePosition function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
path(path,'./MTIDeviceMatlab'); % Add the MTIDeviceMatlab library to the path
clear; clf reset; clc;

% Create an object of the MTIDevice class to access its functions,
% variables, and the target device at serial port
mMTIDevice = MTIDevice;

try
%mMTIDevice.ConnectDevice('COM3'); % Fastest way to connect if device COM port is known
mMTIDevice.ConnectDevice(); % Easiest way (no argument) will autoconnect 
% Then we can check if connection is successful
err = mMTIDevice.GetLastError();
if (err ~= MTIError.MTI_SUCCESS)
    disp(err);
    mMTIDevice.DisconnectDevice();
    delete(mMTIDevice);
    return;
end

mMTIDevice.ResetDevicePosition(); % Send mirror to origin and zero offsets

% Now apply new device parameters as desired by user at beginning of the session
% It is recommended to load the Mirrorcle-provided ini file for a
% specific device and set those parameters
lParams = mMTIDevice.LoadDeviceParams('mtidevice.ini');
mMTIDevice.SetDeviceParams(lParams);

% Any additional parameters can be set directly as follows:
% This overrides any previously set parameters
mMTIDevice.SetDeviceParam( MTIParam.DataScale, 1.0 );
mMTIDevice.SetDeviceParam( MTIParam.SampleRate, 20000 );
err = mMTIDevice.GetLastError();
% Turn MEMS driver on
mMTIDevice.SetDeviceParam( MTIParam.MEMSDriverEnable, 1 )

answer=0;
display(' ');
display('Input position the device should step and settle to.');
display('Use normalized positions from 0 to 1 for 1-Quadrant devices.');
display('Use normalized positions from -1 to 1 for 4-Quadrant devices.');
display('Program will exit when you enter a value outside those limits');
display(' ');
while answer<=1
    s=input('Input go to position in xpos,ypos format: ','s');
    Pos = sscanf(s, '%f , %f');
    xpos = Pos(1,1);
    if xpos>1 
        break;
    end
    ypos = Pos(2,1);
    answer=max(abs(xpos),abs(ypos));
    if answer<=1
        % Go to new position with a 10ms step from current position
        tstep = 10; % Step time in ms
        mpos = 255; % Have the digital output 255 (laser on) in new pos.
        mMTIDevice.GoToDevicePosition(xpos,ypos,mpos,tstep);
    end
end

% NOTE - for fastest response, which minimizes comm overhead and step time
% user can enter step time (tstep) of 0.  In this case the step will not
% have interpolated samples to get the device from stopped position to new
% position but will go in a single step sample and therefore if the
% hardware filter is not set properly to protect the MEMS device from
% overshoot and ringing, the device may be damaged
% mMTIDevice.GoToDevicePosition(xpos,ypos,mpos,0);
clc;
display(' ');
display('A sequence of fastest steps GoToDevicePosition with parameter tstep of 0');
display('Example should be run with recommended HardwareFilterBw to protect device');
display('Example uses small incremental steps where tstep of 0 is also OK');
% Go point to point as quickly as possible with PC in the loop
for i = 1:100 % make a spiral example
    r = i / 100;
    x = r.*cos( 2 * pi * (i/25) );
    y = r.*sin( 2 * pi * (i/25) );
    mMTIDevice.GoToDevicePosition( x, y, 255, 0 );
end

% Return device to origin and close the session nicely
mMTIDevice.ResetDevicePosition();
% After the device is back to origin - disable the driver
mMTIDevice.SetDeviceParam( MTIParam.MEMSDriverEnable, 0 );
% Disconnect and delete the object
mMTIDevice.DisconnectDevice();
delete(mMTIDevice);
display('Closed successfully..');

catch
    display('Application failed to run properly!');
    mMTIDevice.ClearInputBuffer();
    mMTIDevice.DisconnectDevice();
    delete(mMTIDevice);
    display('Closed in final catch..');
end