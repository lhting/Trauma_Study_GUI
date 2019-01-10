function heater_close(heaterobj)
% heater_close(heaterobj)
%
% input: heaterobj
%
% output:
%
% example: heater_close(heaterobj)

fclose(heaterobj)
delete(heaterobj)
