%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mirrorcle Technologies, Inc. 2011-2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MTIDevice version 10.6.0.0
% Compatible with Firmware 4.x or newer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MTIDeviceSerial_Test demonstrates the use of various functions in producing
% analog (X,Y) and digital (M) output from the USB Controller and driving 
% a Mirrorcle MEMS Mirror to follow a desired trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
path(path,'./MTIDeviceMatlab'); % Add the MTIDeviceMatlab library to the path
clear; clf reset; clc;

% Create an object of the MTIDevice class to access its functions,
% variables, and the target device at serial port
mMTIDevice = MTIDevice;

% Data can be negative and positive, representing negative and positive
% deflections of the mirror for each axis
% The data must be normalized to -1 to +1 floating point range
t1 =linspace(0,2*pi,256)'; 
t2 =linspace(0,2*pi,256)'; 
x =sin(2.*t1(1:end-1)); y = sawtooth(t2(1:end-1),0.5);   
xy=[x,y];
figure(1);
plot(x,y,'bo-'); xlabel('X axis'); ylabel('Y axis'); axis([-1 1 -1 1]);
% Digital data for the 8-bit digital port of the controller
m=mod((0:length(xy)-1),256);   % Prepare every possible combination from 0-255

try
% Check for any available devices to connect to
availableDevices = mMTIDevice.GetAvailableDevices();
% Display the available devices and their ports
mMTIDevice.ListAvailableDevices(availableDevices);
% If no devices detected exit the demo
if (availableDevices.NumDevices < 1) return; end;
% If the user knows the comport can proceed this way
comport = 'COM3'; % Serial COM port where the USB-SL Controller is connected
% If the user does not know the port or has multiple devices attached
comport = cell2mat(availableDevices.CommPortName(1)); % Choose the first Controller found
% Connect to USB-SL Controller at the specified comport
mMTIDevice.ConnectDevice( comport );
% Then we can check if connection is successful
err = mMTIDevice.GetLastError();
if (err ~= MTIError.MTI_SUCCESS)
    disp(err);
    mMTIDevice.DisconnectDevice();
    delete(mMTIDevice);
    return;
end

mMTIDevice.ResetDevicePosition();	% Send mirror to origin and zero offsets
mParams = mMTIDevice.GetDeviceParams();	% Get device info and device parameters

%check if RGB Capable module and if yes assign some color to the content
if (( mParams.DataFormat == MTIDataFormat.Reduced_XYRGB )||( mParams.DataFormat == MTIDataFormat.Standard_XYRGB )||( mParams.DataFormat == MTIDataFormat.Standard_XYMRGB ))
    rgbCapable = 1;
    RGBColor = hex2dec('55DD44'); %color in hex, as RED(0-255) GREEN(0-255) BLUE(0-255)
    mMTIDevice.SetRGBData( RGBColor );
else
    rgbCapable = 0;
end

% Now apply new device parameters as desired by user at beginning of the session
% It is recommended to load the Mirrorcle-provided ini file for a
% specific device and set those parameters
lParams = mMTIDevice.LoadDeviceParams('mtidevice.ini');
mMTIDevice.SetDeviceParams(lParams);

% Any additional parameters can be set directly as follows:
% This overrides any previously set parameters
mMTIDevice.SetDeviceParam( MTIParam.DataScale, 1.0 );
mMTIDevice.SetDeviceParam( MTIParam.DeviceAxes, MTIAxes.Normal );
mMTIDevice.SetDeviceParam( MTIParam.OutputOffsets, 0, 0 );
err = mMTIDevice.GetLastError();    % Optional, checking errors after a setting
mMTIDevice.SetDeviceParam( MTIParam.SampleRate, 8000 );
err = mMTIDevice.GetLastError();

% Turn MEMS driver on
mMTIDevice.SetDeviceParam( MTIParam.MEMSDriverEnable, 1 );
% Turn Digital Output on (optional)
mMTIDevice.SetDeviceParam( MTIParam.DigitalOutputEnable, 1 );
% Finally, optionally can check all the device params to see if set
mParams = mMTIDevice.GetDeviceParams();


% Send data to Controller
adjustDigitalOutputDelay = 1;  % Tells the function to adjust digital output for LPF group delays
mMTIDevice.SendDataStream(x,y,m,length(x),adjustDigitalOutputDelay,1); % Send data to Controller and run automatically

disp('Press any key to stop device'); 
pause;
mMTIDevice.StopDataStream();
xStopPosition = mMTIDevice.GetXStop();
yStopPosition = mMTIDevice.GetYStop();
iStopPosition = mMTIDevice.GetIStop();
fprintf('Mirror stopped by user.  Last sample position is %3.3f, %3.3f\n', xStopPosition, yStopPosition); 
fprintf('Mirror stopped by user.  Last sample index is %i\n\n', iStopPosition); 
disp('Press any key to return reset device position to origin (with laser OFF)');
pause;
mMTIDevice.ResetDevicePosition();

% Another example to run a single cycle but two distinct times when user presses a key
mMTIDevice.SetDeviceParam( MTIParam.SampleRate, 3000 ); % Run slower this time
disp('Press any key to run only 5 times'); 
pause; 
mMTIDevice.StopDataStream();
if (rgbCapable) %if RGB Module include some color variations into next runs
    cl = colormap();
    cl = cl(floor(linspace(1,64,length(x))),:);
    RGBColor = floor(cl(:,1)*255*65536 + cl(:,2)*255*256 + cl(:,3)*255); %color in hex, as RED(0-255) GREEN(0-255) BLUE(0-255)
    mMTIDevice.SetRGBData( RGBColor, length(x) );
end
mMTIDevice.SendDataStream(x,y,m,length(x),adjustDigitalOutputDelay,1); % Send to Controller again (can be skipped)
mMTIDevice.StartDataStream(5);  % Run 5 times, this is a blocking call, returns when the sampling/running is completed
disp('Press any key to run once more at a tenth of that rate');
mMTIDevice.SetDeviceParam( MTIParam.SampleRate, mMTIDevice.GetDeviceParam( MTIParam.SampleRate )/10 );
pause;
mMTIDevice.StartDataStream(1);  % Run once, this is a blocking call, returns when the sampling/running is completed
disp('Press any key to run once more at the slow rate and keep checking when it will complete');
pause;
mMTIDevice.StartDataStream(1, 0);  % Run once, this is a non-blocking call, no confirm when complete is received
samplesRemaining = 1e7;
while( (samplesRemaining > 0) && (samplesRemaining~=length(x)) ) % Firmware before 5.2 gives length(x) when complete
    samplesRemaining = mMTIDevice.GetSamplesRemaining();
    fprintf('%i samples remain to complete\n', samplesRemaining); 
    pause(0.05); %perform some work while running
end

% Return device to origin and close the session nicely
mMTIDevice.ResetDevicePosition();
% After the device is back to origin, disable the driver
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