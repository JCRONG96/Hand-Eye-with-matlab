%toolbox˫Ŀ�궨 ����Calib_Results_stereo.mat

%%
% Create a set of calibration images.
% close all;
% clear;
% clc;o
%load('D:\rongjc���ļ���\˫Ŀ�궨\eyeinhand\eyeinhand\���۱궨5.7\Calib_Results_stereo.mat')
path ='���۱궨5.7';
num_of_images = 12;
%% ������λ�˱��浽һ��������
% �����е�ֵ�λ������
bHg = zeros(4, 4, num_of_images); % ��ʼ��,��Ҫ�Ǵ�С
jiaodu = zeros(3, num_of_images);
positions = importdata([path '/����6.txt']);
for i = 1 : num_of_images
    %��������6����ת��Ϊ4X4����ϵת������
    pos = positions( i, 1:end);
    location = pos(1:3)* 1000;
    % 1
    q = pos(4:7);
    %orientation = quat2rotm(q);
    orientation=[ 2*q(1).^2-1+2*q(2)^2    2*(q(2)*q(3)-q(1)*q(4)) 2*(q(2)*q(4)+q(1)*q(3));
        2*(q(2)*q(3)+q(1)*q(4)) 2*q(1)^2-1+2*q(3)^2     2*(q(3)*q(4)-q(1)*q(2));
        2*(q(2)*q(4)-q(1)*q(3)) 2*(q(3)*q(4)+q(1)*q(2)) 2*q(1)^2-1+2*q(4)^2];
    % 1
    jiaodu(1:3,i)=pos(4:6)*pi/180.0;
    %orientation = eul2rotm(jiaodu(1:3,i)','XYZ');
    figure(3);
    plotCamera('Location',location,'Orientation',orientation','Size',20);
    hold on
    pcshow([0,0,0], [1.0,0,0], 'VerticalAxisDir','down','MarkerSize',40);
    bHg(1:3, 1:3, i) = orientation;
    bHg(1:3, 4,   i) = location';
    bHg(4, :, i) = [0 0 0 1];
end
%% ������λ�˱��浽һ��������
cHw = zeros(4, 4, num_of_images);
for i = 1 : num_of_images
  %% ֱ��ȡ�궨����е�������Ϊ���
    rotation_vector = rotationVectorToMatrix(cameraParams.RotationVectors(i,:));
    translation_vector =  cameraParams.TranslationVectors(i,:);
  %% Compute camera pose ->�����λ����ʾ�ڱ궨������ϵ��
    [orientation, location] = extrinsicsToCameraPose(rotation_vector, ...
      translation_vector);
figure(1);
    plotCamera('Location',location,'Orientation',orientation,'Size',20); hold on
    pcshow([0,0,0], [1.0,0,0], 'VerticalAxisDir','down','MarkerSize',40); hold on
  %% �������λ��
    cHw(1:3, 1:3, i) = rotation_vector';
    cHw(1:3, 4, i) = translation_vector';
    cHw(4, 1:4, i) = [0 0 0 1];
end


  %% ���۱궨
gHc = handEye(bHg, cHw);
% bHgΪ����ʱ��¼�����꣬Ϊĩ����Ի����˻������ת������
% cHwΪ˫Ŀ�궨�������Ա궨���λ��ת���������롮Calib_Results_stereo.mat������
% ʹ��handEye�������������۱궨
% gHc = handEye(bHg, cHw);
% ��ȡgHc�������ڻ�����ĩ�˵�λ��ת������
disp(gHc);
rot_vect = -rotationMatrixToVector(gHc(1:3, 1:3));
theta = norm(rot_vect);
r = rot_vect / theta;
disp('��ת����:');
disp(r);
disp(['��ת��' num2str(theta * 180 / pi)]);
  %% ��ʾ����ڻ���������λ��
bHc= bHg(:, :, 1)*gHc;
figure(4);
plotCamera('Location', bHg(1:3,4,1)','Orientation', bHg(1:3, 1:3, 1)','Size',20);
hold on
plotCamera('Location',bHc(1:3, 4),'Orientation',  bHc(1:3, 1:3)','Size',20);
hold on
pcshow([0,0,0], [1.0,0,0], 'VerticalAxisDir','down','MarkerSize',40);

