clear all
close all

% Add opcodemesh folders
folder_path = pwd;
path_opcodemesh = [folder_path '\opcodemesh'];
if isempty(strfind(path,path_opcodemesh))
    disp('Adding opcodemesh path...\n')
    addpath(genpath(path_opcodemesh))
end 

fileName = 'mug.stl';

% Import model to a geometry
model = createpde('structural');
gm = importGeometry(model, fileName);
%figure
%pdegplot(gm)

% Get the normals
[~, ~, N] = stlread2(fileName);

[F, V] = gm.allDisplayFaces();% V for vertices and F for the facets

V = V'; % Vertices
F = F'; % Faces
N = N'; % Normals
V(3,:) = V(3,:) + 600; % Give some distance

DpthImg = KinectSimulator_Depth(V,F,N,...
    'simple','default',[],'imgrng',[204 4000],'subray',[5 9]);
%figure, imshow(DpthImg,[170 320])
%title('Noisy depth image of xbox controller in front of nothing')


% Kinect parameters -------------------------------------------------------
% Resolution of real outputted Kinect IR image (rows x cols)
ImgRes = [480 640];   % pix
% Field of view of transmitter/receiver (vertFOV x horzFOV)
ImgFOV = [45.6 58.5]; % deg
% Minimum and maximum operational depths of the Kinect sensor (min x max)
ImgRng = [800 4000];  % mm
% Determine horizontal and vertical focal lengths
ImgFOV = ImgFOV*(pi/180); % rad
FocalLength = [ImgRes(2)/(2*tan(ImgFOV(2)/2)); ImgRes(1)/(2*tan(ImgFOV(1)/2))]; % pix

K = [FocalLength(1) 0 0.5; 0 FocalLength(2) 0.5];

%depth is depth image in double format
Sd = size(DpthImg);
[X,Y] = meshgrid(1:Sd(2),1:Sd(1));
%K is calibration matrix
X = X - K(1,3) + 0.5;
Y = Y - K(2,3) + 0.5;
XDf = DpthImg/K(1,1);
YDf = DpthImg/K(2,2);
X = X .* XDf;
Y = Y .* YDf;
XY = cat(3,X,Y);
cloud = cat(3,XY,DpthImg);
cloud = reshape(cloud,[],3) / 1000.0;
% if you can use matlab point cloud library
cloud = pointCloud(cloud);
pcshow(cloud);


%obj = imaq.VideoDevice('kinect', 1);
% ptCloud = pcfromkinect(obj,DpthImg)
% 
% player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
% 	'VerticalAxis','y','VerticalAxisDir','down');
% 
% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');