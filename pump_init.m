function pumpobj=pump_init(com,diam)
% pump_init
%
% Inputs:
%   com      com port 'COM11' for bwc laptop
% Outputs:
%   pumpobj  this object gets passed to all pump commands
% Examples:
%   pumpobj=pump_init('COM11',11.989)
%diameter=11.989
%force=50%

pumpobj=serial(com);
set(pumpobj,'baudrate',115200);
set(pumpobj,'databits',8);
set(pumpobj,'parity','none');
set(pumpobj,'stopbits',2);
pumpobj.Terminator = 'CR';
fopen(pumpobj);
pause(.1)

%- setup syring info
fprintf(pumpobj,'diam %.3f\r',diam);
pause(.1)
char(fread(pumpobj,pumpobj.BytesAvailable))';%for debugging


%-- quick test for communications
fwrite(pumpobj,['version' char(13)]);
pause(.1)
versioninfo=char(fread(pumpobj,pumpobj.BytesAvailable))';
if ~isempty(versioninfo)
    disp('Pump: Initialized!')
    %disp(versioninfo)
else
    disp('Pump: Initialization Failed!')
end
