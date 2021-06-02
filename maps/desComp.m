clc;
clear;
close all
fileDes = "office/BLC/descriptor128dim_60kp.txt";
fileKp = "office/BLC/keypoints60.txt";
fid = fopen(fileDes,'r');
desSize = 128;
desNum = 60;
dataDes = zeros(desSize/8,desNum);
for i=1:desNum
    dataDes(:,i) = fread(fid,desSize/8);
end