function heaterobj=heater_init(com)
% function heaterobj=heater_init(com)
%
% input: com     comport 'COM3'
%
% output: heaterobj
%
% example: heaterobj=heater_init('COM3')

heaterobj = serial(com);
set(heaterobj,'BaudRate',9600,'DataBits',7,'StopBits',1,'Parity','even','Timeout',3);
fopen(heaterobj);
heaterobj.terminator='CR/LF';
set(heaterobj,'readasyncmode','continuous');

