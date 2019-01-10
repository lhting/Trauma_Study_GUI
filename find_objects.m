function [BWstats,BWfinal]=find_objects(Image,contrastthresh)


BW = im2bw(Image,contrastthresh);

%Fill in enclosed holes
BWdfill = imfill(BW, 'holes');

%Remove single unconnected pixels to clean up image
BWisolated = bwmorph(BWdfill,'clean');

%Remove border connected objects such as a half-block on the edge
BWnobord = imclearborder(BWdfill, 4);
%BWnobord = BWdfill;

%Erode twice using diamond structure
seD = strel('diamond',2);
BWfinal = imerode(BWnobord,seD);
BWfinal = imerode(BWfinal,seD);

BWstats = regionprops(BWfinal, 'Area','centroid','Perimeter');
