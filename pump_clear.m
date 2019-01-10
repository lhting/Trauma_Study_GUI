function ok=pump_clear(pumpobj)
%
% ok=pump_clear(pumpobj)
%

%--- clear all
fwrite(pumpobj,['ctvolume' char(13)]) %clear
pause(.1)
char(fread(pumpobj,pumpobj.BytesAvailable))'
fwrite(pumpobj,['cttime' char(13)]) %clear
pause(.1)
char(fread(pumpobj,pumpobj.BytesAvailable))'
fwrite(pumpobj,['cvolume' char(13)]) %clear
pause(.1)
char(fread(pumpobj,pumpobj.BytesAvailable))'
fwrite(pumpobj,['ctime' char(13)]) %clear
pause(.1)
char(fread(pumpobj,pumpobj.BytesAvailable))'

ok = 1;
