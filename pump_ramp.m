function ok=pump_ramp(pumpobj,ramp1_start,ramp1_end,ramp1_time)
% pump_ramp
%
% for a 5ml syring, min. rate is 20.365nl/min max ratae 21.539 ml/min
%Inputs:
%
%Outputs:
%
%Example:
%
debug=1;
tic
fprintf(pumpobj,'iramp %.2f ul/min %.2f ul/min %.2d \r',[ramp1_start, ramp1_end, ramp1_time])
pause(.05)
out = char(fread(pumpobj,pumpobj.bytesavailable))';
if debug,disp(out);end
fwrite(pumpobj,['irun' char(13)]);
pause(.05)
out = char(fread(pumpobj,pumpobj.bytesavailable))';
if debug,disp(out);end
disp('pump:rampfcn - ok') %for debugging
toc
ok =1;
