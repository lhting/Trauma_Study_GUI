function ok=pump_close(pumpobj)

fclose(pumpobj)
delete(pumpobj)
clear pumpobj
ok =1;