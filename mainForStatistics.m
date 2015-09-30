%进行统计的.m文件，其中需要用户输入的参数均在第一部分
%因为保留了中间结果，所以每一部分都可以单独运行（注释掉其它部分）
%这个文件进行唯一性判断，即判断每个空间中的小隔间中可以有哪些自由度对应的手表的位置

% ------------------------------------------------------------------------------------------------------------------------------
%1.准备工作
% ------------------------------------------------------------------------------------------------------------------------------
clc;clear;
%1.1.输入参数
%胳膊数值参数：大臂长，小臂长，小臂半径
val_arm=[1 1 0.1];
a=armModel(val_arm);
%5个自由度遍历的步长，单位：度
step_angle=[15 15 15 15 45];
%5个自由度的遍历范围，单位：度
range_angle=[
0 90;
-45 180;
0 135;
0 135;
-90 90
];
%格子的大小
step_pos=[0.1 0.1 0.1];
%1.2.定义data数据中的数据（data中存放重要数据）
%(1) N*5的数组，存放遍历的5个自由度的度数
data.angle=[];
%(2) N*3的数组，存放遍历的手表在空间中的位置，每一个data.angle对应一个data.pos
data.pos=[];
%(3) data.cell N*3的数组，在后面定义，代表了这个格子在人坐标系中的位置，存放格子的范围下界（下界加上格子的大小就可以得到上界）
%(4) data.fre N*1的数组，在后面定义，代表了这个格子中出现手表的次数，存放格子中手表位置出现的频率
%(5) data.item cell(N,1),在后面定义，代表了哪些状态（5个自由度大小）下手表会出现在这个格子中，存放格子中的手臂的状态
%以下三个参数(6)(7)(8)去掉了未出现手表的格子，只统计那些出现过手表的格子
%(6) data.notZeroCell
%(7) data.notZeroFre
%(8) data.notZeroItem
%(9) data.freCluster这个参数还没写
save('D:\用户目录\Documents\MATLAB\process1.mat');

% -------------------------------------------------------------------------------------------------------------------------------
%2.遍历姿态，得到所有姿态对应的手的位置（在人坐标系中）,结果存放在data.angle和data.pos中
% -------------------------------------------------------------------------------------------------------------------------------
clear;clc
load('D:\用户目录\Documents\MATLAB\process1.mat');
tic
for i=range_angle(1,1):step_angle(1):range_angle(1,2)
    for j=range_angle(2,1):step_angle(2):range_angle(2,2)
%         %输出方便查看进度
%         fprintf('i=%d,j=%d',i,j);
%         fprintf('\r\n');
        for k=range_angle(3,1):step_angle(3):range_angle(3,2)
            for m=range_angle(4,1):step_angle(4):range_angle(4,2)
                for n=range_angle(5,1):step_angle(5):range_angle(5,2)
                    data.angle=[data.angle;[i j k m n]];
                    temp=degreeFreedomToWatchOrientation2(a,[i j k m n],0,1);
                    data.pos=[data.pos;temp];
                end
            end
        end
    end
end
% hold off
clear i j k m n a
toc
%保存中间数据
save('D:\用户目录\Documents\MATLAB\process2.mat');

% -------------------------------------------------------------------------------------------------------------------------------
%3.划分成格子，进行统计，结果存放在data.notZeroCell，data.notZeroFre，data.notZeroItem中
% -------------------------------------------------------------------------------------------------------------------------------
% 格子左闭右开
clc;clear;
load('D:\用户目录\Documents\MATLAB\process2.mat');
tic
%得到格子的范围（在x,y,z三个方向上的范围）
minData=min(data.pos);
maxData=max(data.pos);
range_pos=[minData;maxData];
range_pos=range_pos';
%len记录遍历的状态的个数
len=size(data.pos);
%生成格子
range_posAppro(:,1)=floor(range_pos(:,1)./step_pos').*step_pos';
range_posAppro(:,2)=floor(range_pos(:,2)./step_pos').*step_pos';
stepa=range_posAppro(1,1):step_pos(1):range_posAppro(1,2);
stepb=range_posAppro(2,1):step_pos(2):range_posAppro(2,2);
stepc=range_posAppro(3,1):step_pos(3):range_posAppro(3,2);
stepa=stepa';
stepb=stepb';
stepc=stepc';
temp1=kron(stepc,ones(1,length(stepb)*length(stepa)));
temp1=reshape(temp1,[],1);
temp2=kron(stepb,ones(1,length(stepa)));
temp2=reshape(temp2,[],1);
temp2=kron(temp2,ones(length(stepc),1));
temp3=kron(stepa,ones(length(stepc)*length(stepb),1));
temp4=[temp3,temp2,temp1];
%定义一些存放中间结果的变量
data.cell=temp4;
data.fre=zeros(length(temp3),1);
data.item=cell(length(temp3),1);
%遍历所有状态，将他们一一放入格子中
for i=1:1:len(1)
%     %输出进度
%     fprintf('i=%d,len=%d\r\n',i,len);
    temp=data.pos(i,:);
    temp=temp./step_pos;
    temp=floor(temp); 
    num=(temp(1)-range_posAppro(1,1)/step_pos(1))*length(stepb)*length(stepc)+(temp(2)-range_posAppro(2,1)/step_pos(2))*length(stepc)+(temp(3)-range_posAppro(3,1)/step_pos(3))+1;
    data.fre(num)=data.fre(num)+1;
    data.item{num}=[data.item{num};data.angle(i,:)];
end
%对中间结果扫描，去掉无用项
notZeroFre=find(data.fre~=0);
data.notZeroCell=data.cell(notZeroFre,:);
data.notZeroFre=data.fre(notZeroFre);
data.notZeroItem=data.item(notZeroFre);
clear i temp len num a b c minData maxData stepa stepb stepc temp1 temp2 temp3 temp4
toc
save('D:\用户目录\Documents\MATLAB\process3.mat');

% % -------------------------------------------------------------------------------------------------------------------------------
% %4.判断每个格子中出现的状态是不是近似的属于一个状态，结果存放在data.freCluster
% % -------------------------------------------------------------------------------------------------------------------------------
% clear;clc;
% load('D:\用户目录\Documents\MATLAB\process3.mat');