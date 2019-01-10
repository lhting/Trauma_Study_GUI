%prototype_main_gui.m
%v.2.0
%Authored by: Lucas Ting & Bryan Cunitz

clear all
close all


cd('C:\Users\Prototype1\Documents\stasys\src')
%cd('C:\Users\Bryan\Documents\stasys')
%--- New Experiment or Process old data ---%
runmode= questdlg('Run Mode', ...
                         'runmode', ...
                         'Acquire', 'Post-Process', 'Acquire');
runmode=strcmp(runmode,'Post-Process');


if runmode ==1
    dirname = uigetdir('C:\Users\Prototype1\Documents\stasys\src\data');
    dirlist=dir([dirname '\*.tif']);
    [garbage dirsort]=sort([dirlist.datenum]);    %correct filenaming order bug
end
%--- Experiment Settings ---%
%Open dialog box to ask  the conversion from pixel to displacement, and to ask for
%spring constant of micropost
prompt = {'Enter micron/px conversion:','Stiffness (note h=15, d=4.5default)'};
dlg_title = 'Input for Force Calculations';
num_lines = 1;
def = {'0.16','44.7'};
answer2 = inputdlg(prompt,dlg_title,num_lines,def);
%convert inputted string to numbers and store as seperate variables
answer2 = str2double(answer2);
MicronsPerPixel = answer2(1);
PostStiffness = answer2(2);
%--- Processing Settings ---%
multisensor = 1;
contrastthresh = 0.4;  %Default value
num_pairs = 7;      %<-- set for number of block-post pairs, in the future it will autodetect
box_w = 160;        % ROI width for each block-post pair
box_h = 220;        % ROI height for each block-post pair
minBlockArea=900;   % minimum area for a block
maxBlockArea=5000;
minPostArea=30;     % minimum area for a post
minDistance = 50;
maxDistance = 90;
%--- Camera Settings ---%
fps=1;                  %image acquisition rate (frames/sec)
exposure = 200;         %Default in ms
resolution=[2560,1920];
ROT = 0; %rotation factor (ROT*90deg)
%-- do not touch--%
Bin = 0;        % (1) 1:2 decimation
BinMode = 0;    % (1) Bin/average
hblanking = 12;%  %0-48
vblanking = 40; %the time in 50us unit for vblanking, e.g. 20 means 1000us.
sensorFrequency = 96;%96; %24, 48, or 96MHz
%--- Pump Settings ---%
pumpdelay = 10; %# frames to collect before starting the pump

ramp1_start     = 1;  % ul/min
ramp1_end       = 48; % ul/min
ramp1_time      = 15; % sec

constant_rate   = 48; % ul/min
constant_time   = 30; % sec

ramp2_start     = 48;
ramp2_end       = 1;
ramp2_time      = 15;

pump_params = [ramp1_start, ramp1_end, ramp1_time;...
    constant_rate, constant_rate, constant_time;...
    ramp2_start, ramp2_end, ramp2_time];

pumptimer=timer('TimerFcn',{@pumptimerFcn, pump_params} ,'ExecutionMode','fixedRate','Period',.1);

%--- Debugging ---%
if runmode ==1
    debugmode = 4;
    %ImageDebug=uint16(imread('.\Chip1_0001.tiff')).*2^4;    %for debugging import image and make it 12bit
    ImageDebug=uint16(imread([dirname '\' dirlist(dirsort(1)).name]))./2^4;
else
    debugmode = 0;
end
%0 all HW
%1 just camera
%2 just pump
%3 just heater
%4 no HW


%%  1 - Initialize HW
%-- 0 clear all instruments
w=instrfind;
if ~isempty(w)
    fclose(w)
    delete(w)
    clear w
end

%-- 1.1 Mightex Camera
if (debugmode ==0)||(debugmode ==1)
    if libisloaded('mightex')
        mightex_SDK_close(1);
    end
    [ok, deviceID, framePtr]=mightex_SDK_init(resolution, Bin, BinMode, hblanking, vblanking, sensorFrequency);
    pause(.1)
    ok = mightex_SDK_setexposure(deviceID,exposure);    %start at the set default
end
%-- 1.2 Harvard Apparatus Pump
if (debugmode ==0)||(debugmode ==2)
    clear pumpobjF pumpstep
    pumpobj=pump_init('COM6',11.989);
    ok=pump_clear(pumpobj);
end
%-- 1.3 Heating Unit
if (debugmode ==0)||(debugmode ==3)
    clear heaterobj
    heaterobj=heater_init('COM4');
    PVlog=[];
    PVtlog=[];
    Tplot = figure;
    Taxes = gca;
    set(Tplot,'Position',[960 246 640 728])
    heatertimer=timer('TimerFcn','evalin(''base'',''PVlog=[PVlog heater_readPV(heaterobj)];plot(Taxes,PVlog);drawnow'')','ExecutionMode','fixedRate','Period',.1);
end

%% 2 - Adjust Camera Settings
%  (runs in a loop while user selects exposure and threshold)
%


%- Setup preview figure
previewF=figure;
%set(previewF,'Position',[650 246 687 728])
set(previewF,'Position',[1 41 1920 963])
subplot(122)
histA=gca;
subplot(121)
previewA=gca;
%- Setup buttons
backFbutton=uicontrol('Style','togglebutton','string','Illumination Gradient Correction','Position',[20 130 200 40],'Callback','background = imopen(Image,strel(''disk'',100));');    %obtain background shading
cropbutton=uicontrol('Style','togglebutton','string','crop','Position',[20 90 200 40],'Callback','imagesc(rot90(ImageF,ROT),''Parent'',previewA);[I2 rect]=imcrop;contrastthresh=graythresh(I2)');
%--
findthresh=0;
objectsfound=0;
findObjsbutton=uicontrol('Style','togglebutton','string','find objects','Position',[20 50 200 40],'Callback','findthresh = 1');    %obtain background shading
donebutton=uicontrol('Style','togglebutton','string','done','Position',[20 10 200 40]);
%--
exposuretext=uicontrol('Style','text','String','Camera Exposure (ms)','Position',[320 30 260 20]);
exposureslider=uicontrol('Style','slider','Position',[320 10 200 20],'Min',50,'Max',750,'Value',exposure);
exposurevalue=uicontrol('Style','edit','Position',[540 10 40 20],'String',num2str(exposure));


deviceID=1; %for debugging
set(exposureslider,'callback','set(exposurevalue, ''String'',num2str(round(get(exposureslider,''value''))));exposure=str2num(get(exposurevalue,''string''));ok = mightex_SDK_setexposure(deviceID,exposure);');
set(exposurevalue, 'callback','set(exposureslider,''Value'', str2num(get(exposurevalue,''String'')));       exposure=str2num(get(exposurevalue,''string''));ok = mightex_SDK_setexposure(deviceID,exposure);');
if ~exist('rect')
    if debugmode<=1
        rect = [1 1 resolution(2) resolution(1)];
    else
        rect=[1 1 size(ImageDebug,2) size(ImageDebug,1)];
    end
end
%disp('debugging pause')
%keyboard
while ~get(donebutton,'Value')
    if (debugmode ==0)||(debugmode ==1),
        ImageRaw = mightex_SDK_acquire(1, deviceID, framePtr);
        Image=mightex_conv_RAW_image(ImageRaw,resolution,Bin);
    else
        Image=ImageDebug;   %for debugging
        
    end
    if exist('background','var')
        ImageF=Image-background;
    else
        ImageF=Image;
    end
    rect=round(rect);
    ImageC=ImageF(rect(2):rect(2)+rect(4)-1,rect(1):rect(1)+rect(3)-1);
    
    
    %subplot(211)
    
    %imagesc(rot90(Segout',ROT),'Parent',previewA,[0 2^16])
    imagesc(rot90(ImageC,ROT),'Parent',previewA)
    
    %colormap(gray)
    axis equal tight
    title('Set Exposure ')
    %subplot(212)
    %hist(histA,reshape(single(ImageC)/2^12,1,rect(3)*rect(4)),100)
    drawnow
    
    contrastthresh = graythresh(ImageC.*2^4);   %starting threshold
    
    while findthresh
        [BWstats,BWfinal]=find_objects(ImageC.*2^4,contrastthresh); %find blocks and posts
        metric = 4*pi*[BWstats.Area]./[BWstats.Perimeter].^2;
        %centroids = cat(1, BWstats(([BWstats.Area]>minPostArea)&[BWstats.Area]<maxBlockArea&metric<1&metric>.4).Centroid);  %filter out non-blocks
        centroids = cat(1, BWstats(([BWstats.Area]>minBlockArea)&[BWstats.Area]<maxBlockArea&metric<1&metric>.4).Centroid);  %just find the blocks
        %                roicheck = centroids(:,1)>rect(1)&centroids(:,1)<rect(2)&centroids(:,2)>rect(3)&centroids(:,2)<rect(4);
        if length(centroids)>=num_pairs
            findthresh = 0;
            centroids=[centroids ; [centroids(:,1) centroids(:,2)+80]]; %for just finding blocks, always assume that the posts are 80 pixels below
            %--- Find center of pairs
            [XYI]=nearestneighbour(centroids');
            centers=(centroids+centroids(XYI,:))/2;
            [a,b]=sort(centers);
            centersS=centers(b(:,1),:);
            centersN=round(centersS(1:2:length(centersS),:));
            objectsfound=1;
        else
            contrastthresh = contrastthresh -.01
        end
        
    end
    if objectsfound
        hold(previewA,'on')
        disp([num2str(length(centroids)) ' objects found'])
        %--- fix ROI so that all the mini roi's are contained
        
        if min(centersN(:,1))-(box_w/2)<0
            rect(1)=rect(1)+(min(centersN(:,1))-(box_w/2)-1);
            centersN(:,1)=centersN(:,1)-(min(centersN(:,1))-(box_w/2)-1);
        end
        if max(centersN(:,1))+(box_w/2)> rect(3)
            rect(3)=max(centersN(:,1))+(box_w/2)+1;
        end
        if min(centersN(:,2))-(box_h/2)<0
            rect(2)=rect(2)+(min(centersN(:,2))-(box_h/2)-1);
            centersN(:,2)=centersN(:,2)-(min(centersN(:,2))-(box_h/2)-1);
        end
        if max(centersN(:,2))+(box_h/2)> rect(4)
            rect(4)=max(centersN(:,2))+(box_h/2)+1;
        end
        %---
        for a = 1:length(centroids)
            plot(centroids(XYI(a),1),centroids(XYI(a),2),'ro','Parent',previewA)
        end
        for a = 1:length(centersN)
            plot(centersN(a,1),centersN(a,2),'rx','Parent',previewA)
            text(centersN(a,1)-55,centersN(a,2)+13, num2str(a),'color',[1 0 0],'Parent',previewA)
            line([centersN(a,1)-box_w/2 centersN(a,1)+box_w/2],[centersN(a,2)-box_h/2 centersN(a,2)-box_h/2],'Color',[1 0 0],'Parent',previewA)
            line([centersN(a,1)-box_w/2 centersN(a,1)+box_w/2],[centersN(a,2)+box_h/2 centersN(a,2)+box_h/2],'Color',[1 0 0],'Parent',previewA)
            line([centersN(a,1)-box_w/2 centersN(a,1)-box_w/2],[centersN(a,2)-box_h/2 centersN(a,2)+box_h/2],'Color',[1 0 0],'Parent',previewA)
            line([centersN(a,1)+box_w/2 centersN(a,1)+box_w/2],[centersN(a,2)-box_h/2 centersN(a,2)+box_h/2],'Color',[1 0 0],'Parent',previewA)
        end
        drawnow
    end
end
close(previewF)
rect=round(rect);
%imagesc(rot90(Image,ROT),[0 2^12])

%close

%% 3 - Start Experiment

%--> dialog box to ask to start experiment or quit
choice = questdlg('Ready to start the experiment?', ...
    'start experiment', ...
    'start','cancel','start');
switch choice
    case 'start'
        %-- Create New Experiment directory
        if runmode==0
            workingDir=([pwd '\data\exp' datestr(now,30)]);
            mkdir(workingDir);
        else
            workingDir=dirname;
        end
        
        %-- Initialize graphics
        % - Image acquisition window
        experimentF=figure;
        imageA=gca;
        %axis(imageA,[0 rect(1) 0 rect(3)])
        %axis equal tight
        %set(imageA,'nextplot','replacechildren')
        set(experimentF,'Position',[300 246 640 728])
        donebutton=uicontrol('Style','togglebutton','string','done','Position',[20 10 80 40]);
        % - Processed data figure
        nNplot = figure;
        nNaxes = gca;
        set(nNplot,'Position',[960 246 640 728])
        %set(nNaxes,'nextplot','replacechildren')
        
        %-- Initialize experiment
        framecounter=1;
        crop_frame_start = 1;
        datarack = zeros(10,1);
        if ~exist('rect')
            rect = [1 1 resolution(1) resolution(2)];
        end
        %profile on
        starttime=tic;
        if (debugmode ==0)||(debugmode ==3)
            start(heatertimer)
        end
        while ~get(donebutton,'Value')
            frametime=tic;
            
            %-- Acquire Image
            if (debugmode ==0)|(debugmode ==1)
                ImageRaw = mightex_SDK_acquire(1, deviceID, framePtr);
                Image=mightex_conv_RAW_image(ImageRaw,resolution,Bin);
            else
                
                if runmode==1
                    Image=imread([dirname '\' dirlist(dirsort(framecounter)).name]);    %load in an image
                end
                %Image=ImageDebug;
            end
            if exist('background','var')
                ImageF=Image-background;
            else
                ImageF=Image;
            end
            
            ImageC=ImageF(rect(2):rect(2)+rect(4)-1,rect(1):rect(1)+rect(3)-1);
            
%             if framecounter == pumpdelay
%                 disp('Pump: Starting pump timer')
%                 if (debugmode ==0) | (debugmode ==2)
%                     %start(pumptimer)
%                 end
%             end
%             
            %--- Save Image
            if runmode ==0
                imwrite(uint16(2^4*Image), [workingdir '\img' num2str(framecounter) '.tif'],'tiff');%note teh 2^4 factor corrects the 12 bit image to a 16bit file format in the color scaling
            end
            for a = 1:num_pairs
                %--- Function to process image here
                contrastthresh(a) = graythresh(ImageC(round(centersN(a,2)-box_h/2:centersN(a,2)+box_h/2),round(centersN(a,1)-box_w/2:centersN(a,1)+box_w/2)));
                findthresh=1;
                while findthresh
                    [BWstats,BWfinal]=find_objects(ImageC(round(centersN(a,2)-box_h/2:centersN(a,2)+box_h/2),round(centersN(a,1)-box_w/2:centersN(a,1)+box_w/2)),contrastthresh(a));
                    if (length(BWstats)==2&&BWstats(1).Area>minBlockArea),
                        findthresh = 0;
                    else
                        contrastthresh(a) = contrastthresh(a) -.01;
                        if (contrastthresh(a) <=0)
                            findthresh =0;
                        end
                    end
                end
                if contrastthresh(a)<=0
                    disp(['could not autocontrast box :' num2str(a)])
                    if 0
                        figure(21)
                        imagesc(frameF(round(centersN(a,2)-box_h/2:centersN(a,2)+box_h/2),round(centersN(a,1)-box_w/2:centersN(a,1)+box_w/2)));
                        drawnow
                        pause
                    end
                    centroids = NaN;
                    distances(a) = NaN;
                    Bdisplacement(a) = NaN;
                    Pdisplacement(a) = NaN;
                else
                    centroids = cat(1, BWstats.Centroid);
                    distances(a) = sqrt(sum((centroids(:,1)-centroids(:,2)).^2));
                    if framecounter==1,
                        origin(a)=sqrt(sum(BWstats(1).Centroid.^2));
                        Bdisplacement(a)=0;
                        Pdisplacement(a)=0;
                    else
                        Bdisplacement(a)=sqrt(sum(BWstats(1).Centroid.^2))-origin(a);  %look at displacement of Block
                        Pdisplacement(a)=sqrt(sum(BWstats(2).Centroid.^2))-origin(a);  %look at displacement of Block
                    end
                    
                end
            end
            
            %--- Accumulate measurements
            alldistances(framecounter,:)=distances;
            allcontrastthreshs(framecounter,:)=contrastthresh;
            allBdisplacements(framecounter,:)=Bdisplacement;
            allPdisplacements(framecounter,:)=Pdisplacement;
            allForces(framecounter,:)=alldistances(framecounter,:)*MicronsPerPixel*PostStiffness;
            meanallForces(framecounter)=sum(distances(~isnan(distances)))/sum(~isnan(distances))*MicronsPerPixel*PostStiffness;
            
            %-- Display realtime data    %Plot a graph based on nN
            %- Acquired Image
            elapsedtime=toc(starttime);
            %imagesc(rot90(Segout',ROT),'parent',imageA,[0 2^16])
            imagesc(rot90(ImageC,ROT),'parent',imageA)
            title(imageA,['Elapsed time: ' num2str(round(elapsedtime)) ' (s)'])
            %-
            plot(1:framecounter,allForces,'x',1:framecounter,meanallForces);
            xmin = 0;
            xmax = framecounter;
            ymin = -50;
            ymax = 150;
            
            xlabel(nNaxes,'Frame Number, (1 Frame = 1 sec)');
            ylabel(nNaxes,'Force (nN)');
            title(nNaxes,'nN on Post');
            drawnow
            if framecounter==1,
                set(imageA,'nextplot','replacechildren');
            end
            %-- Control framerate
            framecounter=framecounter+1;
            if runmode==0
            while toc(frametime)<(1/fps) %loop until frame takes 1 sec
                %disp('framerate wait') %for debugging
            end
            end
            %-- Increment Frame Count
            
            %profile viewer
            disp(['framerate: ' num2str(1/toc(frametime))])
            % stop if there are now more files to load in runmode==1
            if runmode==1&framecounter>length(dirlist)
                set(donebutton,'Value',1)
            end
        end
        %% --- Experiment Complete ---%
        if (debugmode ==0)||(debugmode ==3)
            stop(heatertimer)
        end
        %close(experimentF)
        
        %% 4 - Save Data
        %--- Save plot as .fig file into the directory named after the video filename
        %--- Save another copy as a png for quick extraction
        saveas(nNplot,fullfile(workingDir,'nN vs Frame'), 'fig');
        saveas(nNplot,fullfile(workingDir,'nN vs Frame'), 'png');
        if (debugmode ==0)||(debugmode ==3)
            saveas(Tplot,fullfile(workingDir,'Temperature Log'), 'fig');
            saveas(Tplot,fullfile(workingDir,'Temperature Log'), 'png');
        end
        %--- Save all the variables to a workspace inside the directory
        save(fullfile(workingDir,sprintf('alldata.mat')));
        
        %Write the displacement (in microns) vs. frame data to an excel sheet
        xlswrite(fullfile(workingDir,'Displacement vs Frame'), alldistances...
            , 'displacements.xls','A1')
        
        %Write the force vs. frame data to an excel sheet
        xlswrite(fullfile(workingDir,'Force vs Frame'), allForces...
            , 'forces.xls','A1')
        
        
    case 'cancel'
        disp('Experiment Aborted!')
end
%% 5 - Close HW
%close camera HW
if (debugmode == 0)|(debugmode == 1)
    mightex_SDK_close(deviceID);
end
if (debugmode == 0)|(debugmode == 2)
    pump_close(pumpobj);
end
if (debugmode == 0)|(debugmode == 3)
    heater_close(heaterobj);
end