function [spc5Settings] = spc5InitFile

% Header revision
spc5Settings.HeaderInfo.szMagic = 'spc5';
spc5Settings.HeaderInfo.uRev = 1;
spc5Settings.HeaderInfo.uMajor = 0;
spc5Settings.HeaderInfo.uMinor = 0;

% General Settings
spc5Settings.uTargetMotor = 0;
spc5Settings.uFlags = 0;
spc5Settings.uControlMode = 0;
spc5Settings.uFlxWeakBusVolt = 50;

% Communication Type with SPC56EL micro
spc5Settings.Peripheral.szType = 'RS232 Serial';
spc5Settings.Peripheral.ubaudRate = 9600;
spc5Settings.Peripheral.uParity = 'None';
spc5Settings.Peripheral.uStopBits = 1;

% Speed PID Settings
spc5Settings.SpeedPID.uKp = 20;
spc5Settings.SpeedPID.uKi = 5;
spc5Settings.SpeedPID.uKd = 0;

% Torque PID Settings
spc5Settings.TorquePID.uKp = 20;
spc5Settings.TorquePID.uKi = 5;
spc5Settings.TorquePID.uKd = 0;

% Flux PID Settings
spc5Settings.FluxPID.uKp = 20;
spc5Settings.FluxPID.uKi = 5;
spc5Settings.FluxPID.uKd = 0;

% Flux PID Settings
spc5Settings.FluxWeakPID.uKp = 0;
spc5Settings.FluxWeakPID.uKi = 0;
spc5Settings.FluxWeakPID.uKd = 0;

% PLL PID Settings
spc5Settings.PllPID.uKp = 0;
spc5Settings.PllPID.uKi = 0;
spc5Settings.PllPID.uKd = 0;




end

