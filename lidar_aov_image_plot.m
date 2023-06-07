% File name: lidar_aov_image_plot.m
% Author: Shehzeen Malik
% Description: This is the main file that takes text file with LiDAR Data and images from the folder, 
% runs the object_detect on the images to find objects along the centre and maps it to LIDAR based map.

% Lidar aov mapping
clear
close all
clc
%% lidar x-z data
rdata=readtable('out.txt');
rdata=table2array(rdata);

%% lidar y-z data
ydata=readtable('outv.txt');
ydata=table2array(ydata);
iyz_scan=lidarScan(ydata(:,2),ydata(:,1)*pi/180);
i_y=iyz_scan.Cartesian(:,1);
i_zy=iyz_scan.Cartesian(:,2);
y_length=(round(range(i_y))/10)*2/3;
zy_length=round(range(i_zy))/10;
%%
rdata=sortrows(rdata);
% keeping the largest range value for same angle and replacing smaller ones
%with NaN
for i=1:length(rdata)-1%k
    if rdata(i+1,1)==rdata(i,1)
       if rdata(i+1,2)>rdata(i,2)
          rdata(i,:)=NaN;
          rdata(i+1,:)= rdata(i+1,:);
       else
           rdata(i+1,:)= rdata(i,:);
           rdata(i,:)=NaN;
       end
    end
end
%removing Nan values
rdata=rmmissing(rdata);
li_ang=rdata(:,1);
li_rang=rdata(:,2)/10;% for cm
%% if zero and 360 not there add those points so interpolation can work
if li_ang(1) ~= 0
    li_ang=[0;li_ang;360];
    li_rang=[li_rang(1);li_rang;li_rang(1)];
else
    li_ang=[li_ang;360];
    li_rang=[li_rang;li_rang(1)];
end
%% replacing zero range points with values between previous and 
%next non zero values
nz_ind=find(li_rang);
z_ind=setdiff(1:length(li_rang),nz_ind);
j=li_rang(nz_ind);
s=interp1(nz_ind,j,z_ind);
li_rang(z_ind)=s;
%% image data
aov=45.5;
ds=imageDatastore('indoor_scene');
I=readimage(ds,1); 
I=imrotate(I,-90);
I1=impyramid(I,'reduce');  I=impyramid(I1,'reduce');
I1=impyramid(I,'reduce'); I=impyramid(I1,'reduce');
I=permute(I,[2,1,3]);
Ig=(rgb2gray(I));
[img_len,img_height]=size(Ig);

%% fixed data for full plotting
y1=ones(img_len,1);y2=linspace(1,img_height,img_height);%y_length
y=y1*y2;
x2=ones(1,img_height);
z2=ones(1,img_height);
% angle per pixel
ang_pix=aov/img_len;
%% motor angle
% step of motor equal to AOV
m_ang = [0,315,270,225,180,135,90,45];
%% angle of view so +-half of aov mapped per image.
h_aov=(aov/2); 
im_ang=[];
for j=1:length(m_ang)
    ang=m_ang(j)-h_aov;
    im_ang=[im_ang;ang+(1:img_len)'*ang_pix];
    im_ang(im_ang<0)=360+im_ang(im_ang<0);
end
im_rang=interp1(li_ang,li_rang,im_ang);
im_scan=lidarScan(im_rang,im_ang*pi/180);
i_x=im_scan.Cartesian(:,1);
i_z=im_scan.Cartesian(:,2);

%% point cloud
figure
for j=0:(length(m_ang)-1)%:-1:6
    I=readimage(ds,j+1);
    I=imrotate(I,-90);
    I1=impyramid(I,'reduce');  I=impyramid(I1,'reduce');
    I1=impyramid(I,'reduce'); I=impyramid(I1,'reduce');
    obj_I = object_detect(I);
    I=permute(obj_I,[2,1,3]);
    x1=i_x(j*img_len+1:img_len*(j+1));
    x=x1*x2;
    z1=i_z(j*img_len+1:img_len*(j+1));
    z=z1*z2;
    b(:,:,1)=x;
    b(:,:,2)=y;
    b(:,:,3)=z;
    for m=1:size(x,1)
        for n=1:size(x,2)
            out_x(m,n)=x(m,n)+sign(x(m,n));
            out_y(m,n)=y(m,n)+sign(y(m,n));
            out_z(m,n)=z(m,n)+sign(z(m,n));
        end
    end
    out(:,:,1)=out_x;
    out(:,:,2)=out_y;
    out(:,:,3)=out_z;
    colour=I(:,1,:);
    colour = repmat(colour,[1 size(x,2)]);
    pt1=pointCloud(out,'Color', colour);
    pcshow(pt1, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
    hold on
    pt=pointCloud(b,'Color' ,I);
    pcshow(pt, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
    xlabel('X cm')
    ylabel('Y pixels')
    zlabel('Z cm')
    hold on
end
