%����ͳ�Ƶ�.m�ļ���������Ҫ�û�����Ĳ������ڵ�һ����
%��Ϊ�������м���������ÿһ���ֶ����Ե������У�ע�͵��������֣�
%����ļ�����Ψһ���жϣ����ж�ÿ���ռ��е�С�����п�������Щ���ɶȶ�Ӧ���ֱ��λ��

% ------------------------------------------------------------------------------------------------------------------------------
%1.׼������
% ------------------------------------------------------------------------------------------------------------------------------
clc;clear;
%1.1.�������
%�첲��ֵ��������۳���С�۳���С�۰뾶
val_arm=[1 1 0.1];
a=armModel(val_arm);
%5�����ɶȱ����Ĳ�������λ����
step_angle=[15 15 15 15 45];
%5�����ɶȵı�����Χ����λ����
range_angle=[
0 90;
-45 180;
0 135;
0 135;
-90 90
];
%���ӵĴ�С
step_pos=[0.1 0.1 0.1];
%1.2.����data�����е����ݣ�data�д����Ҫ���ݣ�
%(1) N*5�����飬��ű�����5�����ɶȵĶ���
data.angle=[];
%(2) N*3�����飬��ű������ֱ��ڿռ��е�λ�ã�ÿһ��data.angle��Ӧһ��data.pos
data.pos=[];
%(3) data.cell N*3�����飬�ں��涨�壬���������������������ϵ�е�λ�ã���Ÿ��ӵķ�Χ�½磨�½���ϸ��ӵĴ�С�Ϳ��Եõ��Ͻ磩
%(4) data.fre N*1�����飬�ں��涨�壬��������������г����ֱ�Ĵ�������Ÿ������ֱ�λ�ó��ֵ�Ƶ��
%(5) data.item cell(N,1),�ں��涨�壬��������Щ״̬��5�����ɶȴ�С�����ֱ���������������У���Ÿ����е��ֱ۵�״̬
%������������(6)(7)(8)ȥ����δ�����ֱ�ĸ��ӣ�ֻͳ����Щ���ֹ��ֱ�ĸ���
%(6) data.notZeroCell
%(7) data.notZeroFre
%(8) data.notZeroItem
%(9) data.freCluster���������ûд
save('D:\�û�Ŀ¼\Documents\MATLAB\process1.mat');

% -------------------------------------------------------------------------------------------------------------------------------
%2.������̬���õ�������̬��Ӧ���ֵ�λ�ã���������ϵ�У�,��������data.angle��data.pos��
% -------------------------------------------------------------------------------------------------------------------------------
clear;clc
load('D:\�û�Ŀ¼\Documents\MATLAB\process1.mat');
tic
for i=range_angle(1,1):step_angle(1):range_angle(1,2)
    for j=range_angle(2,1):step_angle(2):range_angle(2,2)
%         %�������鿴����
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
%�����м�����
save('D:\�û�Ŀ¼\Documents\MATLAB\process2.mat');

% -------------------------------------------------------------------------------------------------------------------------------
%3.���ֳɸ��ӣ�����ͳ�ƣ���������data.notZeroCell��data.notZeroFre��data.notZeroItem��
% -------------------------------------------------------------------------------------------------------------------------------
% ��������ҿ�
clc;clear;
load('D:\�û�Ŀ¼\Documents\MATLAB\process2.mat');
tic
%�õ����ӵķ�Χ����x,y,z���������ϵķ�Χ��
minData=min(data.pos);
maxData=max(data.pos);
range_pos=[minData;maxData];
range_pos=range_pos';
%len��¼������״̬�ĸ���
len=size(data.pos);
%���ɸ���
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
%����һЩ����м����ı���
data.cell=temp4;
data.fre=zeros(length(temp3),1);
data.item=cell(length(temp3),1);
%��������״̬��������һһ���������
for i=1:1:len(1)
%     %�������
%     fprintf('i=%d,len=%d\r\n',i,len);
    temp=data.pos(i,:);
    temp=temp./step_pos;
    temp=floor(temp); 
    num=(temp(1)-range_posAppro(1,1)/step_pos(1))*length(stepb)*length(stepc)+(temp(2)-range_posAppro(2,1)/step_pos(2))*length(stepc)+(temp(3)-range_posAppro(3,1)/step_pos(3))+1;
    data.fre(num)=data.fre(num)+1;
    data.item{num}=[data.item{num};data.angle(i,:)];
end
%���м���ɨ�裬ȥ��������
notZeroFre=find(data.fre~=0);
data.notZeroCell=data.cell(notZeroFre,:);
data.notZeroFre=data.fre(notZeroFre);
data.notZeroItem=data.item(notZeroFre);
clear i temp len num a b c minData maxData stepa stepb stepc temp1 temp2 temp3 temp4
toc
save('D:\�û�Ŀ¼\Documents\MATLAB\process3.mat');

% % -------------------------------------------------------------------------------------------------------------------------------
% %4.�ж�ÿ�������г��ֵ�״̬�ǲ��ǽ��Ƶ�����һ��״̬����������data.freCluster
% % -------------------------------------------------------------------------------------------------------------------------------
% clear;clc;
% load('D:\�û�Ŀ¼\Documents\MATLAB\process3.mat');