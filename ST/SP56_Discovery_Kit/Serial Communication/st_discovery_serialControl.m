clear all; close all; clc;

%% Set up Serial Port

%Create serial port object
ComPortName = 'COM4';
ComPort = serial(ComPortName);

%Configure serial port
set(ComPort,'BaudRate',38400,'Parity','none');
ComPort.StopBit = 1;
ComPort.DataBits = 8;
ComPort.Terminator = 'CR';

fprintf('Current Serial Port Settings : \n');
get(ComPort);

%Open Serial Port
fopen(ComPort);

%% Commands to Motor Controller

%Motor Bit
frameStartByte = uint8(35);
payloadLength = uint8(1);
payloadID = uint8(2);
% crc = uint8(8);

fwrite(ComPort, [frameStartByte, payloadLength, payloadID]);
get(ComPort);

%% Delete serial port object
fclose(ComPort);
clear ComPort;
delete(instrfind);