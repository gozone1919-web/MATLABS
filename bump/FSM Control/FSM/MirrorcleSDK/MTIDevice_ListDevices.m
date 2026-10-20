%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mirrorcle Technologies, Inc. 2011-2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MTIDevice version 10.6.0.0
% Compatible with Firmware 4.x or newer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MTIDevice_ListDevices demonstrates the use of GetAvailableDevices and 
% ListAvailableDevices to display a list of the connected devices, and
% provide detailed information about each device.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
path(path,'./MTIDeviceMatlab'); % Add the MTIDeviceMatlab library to the path
clear; clf reset; clc;

% Create an object of the MTIDevice class to access its functions,
% variables, and the target device at serial port
mMTIDevice = MTIDevice;


try

availableDevices = mMTIDevice.GetAvailableDevices();

if( availableDevices.NumDevices == 0 )
    fprintf('\nThere are no devices available. Press any key to exit...\n');
    pause();
else
    portnumber = 999;
    while (portnumber > 0)
        clc;
        mMTIDevice.ListAvailableDevices( availableDevices );
        fprintf('\n');
        portnumber = input('\nEnter valid COM port number for detailed info or 0 to quit: ');
        comPortName = ['COM' num2str(portnumber)];
        if ( portnumber > 0 )
            mMTIDevice.ConnectDevice( comPortName );
            err = mMTIDevice.GetLastError();
            if (err ~= MTIError.MTI_SUCCESS)
                disp(err);
                mMTIDevice.DisconnectDevice();
                delete(mMTIDevice);
                return;
            end
            params = mMTIDevice.GetDeviceParams();
            tparams = mMTIDevice.GetTrackParams();
            fprintf('\n COM Port Name: \t%s', comPortName);
            fprintf('\n DeviceName: \t\t%s',params.DeviceName);
            fprintf('\n FirmwareName: \t\t%s',params.FirmwareName);
            [temp, temps] = enumeration('MTIDataFormat');
            fprintf('\n DataFormat: \t\t%s', cell2mat(temps(params.DataFormat+1)));
            fprintf('\n CommType: \t\t\t%s',params.CommType);
            fprintf('\n BluetoothMAC: \t\t%s',params.BluetoothMAC);
            fprintf('\n VmaxMEMSDriver: \t%i',params.VmaxMEMSDriver);
            if (tparams.Success)
                fprintf('\n Tracking Add-On: \tEnabled');
            else
                fprintf('\n Tracking Add-On: \tN/A');
            end;
            fprintf('\n\n');
            disp('Press any key to continue...'); pause;
            mMTIDevice.DisconnectDevice();
        end;
    end;
end;

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