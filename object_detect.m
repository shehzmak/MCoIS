function obj_I = object_detect(I)
%% Pre-process the image
% colour correction
I_lin = rgb2lin(I);
illuminant = illumpca(I_lin);
I_lin = chromadapt(I_lin,illuminant,'ColorSpace','linear-rgb');
I = lin2rgb(I_lin);
% grayscale conversion
Ig=rgb2gray(I);
% gaussian filter to normalize abrupt changes in pixels
sigma = 0.1;
Ig = (imgaussfilt(Ig,sigma));
%Increase the contrast of the image
Ig_c = adapthisteq(Ig);

%% Design Array of Gabor Filters
imageSize = size(I);
numRows = imageSize(1);
numCols = imageSize(2);
%Sample wavelength in increasing powers of two starting 
% from 4/sqrt(2) up to the hypotenuse length of the input image.[Jain,1991]
wavelengthMin = 4/sqrt(2);
wavelengthMax = hypot(numRows,numCols);
n = floor(log2(wavelengthMax/wavelengthMin));
wavelength = 2.^(0:(n-2)) * wavelengthMin;
% Regularly sample orientations between [0,150] degrees in 
% steps of 30 degrees. [Jain,1991]
deltaTheta = 30;
orientation = 0:deltaTheta:(180-deltaTheta);
% Design an array of Gabor Filters which are tuned to different frequencies 
% and orientations.
g = gabor(wavelength,orientation);

%% Extract Gabor magnitude features from source image.
gabormag = imgaborfilt(Ig_c,g);

%% Post-process the Gabor Magnitude Images into Gabor Features.
% Smooth local variations
for i = 1:length(g)
    sigma = 0.5*g(i).Wavelength;
    K = 3;
    gabormag(:,:,i) = imgaussfilt(gabormag(:,:,i),K*sigma); 
end
% Add a map of spatial location information in both X and Y to allow
% classifier prefer groupings which are close together spatially.
X = 1:numCols;
Y = 1:numRows;
[X,Y] = meshgrid(X,Y);
featureSet = cat(3,gabormag,X);
featureSet = cat(3,featureSet,Y);
% Reshape data into a matrix |X| of the form expected by
% the |kmeans| function.
X = reshape(featureSet,numRows*numCols,[]);
% Normalize the features to be zero mean, unit variance.
X = bsxfun(@minus, X, mean(X));
X = bsxfun(@rdivide,X,std(X));

%% Classify Gabor Texture Features using kmeans
km=10;
L = kmeans(X,km,'Replicates',4);

%% Separate segments along the centre
ver_c=size(Ig)/2;
L = reshape(L,[numRows numCols]);
% figure
% imshowpair(I,label2rgb(L),'blend')
lab_cen=unique(L(ver_c(1),:));
Aseg = zeros(size(Ig_c),'like',Ig_c);
BW = zeros(numRows,numCols);
for i=lab_cen
    BW1 = L == i;
    BW = BW + BW1;
end
BW = logical(BW);
Aseg(BW) = Ig_c(BW);
%% binary image from canny
A_bw = edge(Aseg, 'canny', [0.15 0.25], 1.5);
A_bw=bwareaopen(A_bw,40);

%% connected components
CC = bwconncomp(A_bw);
L = labelmatrix(CC);

%%
% keep only those which are along the centre
lab_cen=unique(L(ver_c(1),:));
BW = zeros(numRows,numCols);
for i=lab_cen
    if i>0
        BW1 = L == i;
        BW = BW + BW1;
    end
end
BW = logical(BW);
%%
mask = boundarymask(BW);
%figure
%imshow(mask);
%obj_I=imoverlay(I,BW,'yellow');
%figure; imshow(obj_I)
obj_I=imoverlay(I,mask,'yellow');
%figure; imshow(obj_I)
end
