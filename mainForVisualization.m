%���п��ӻ���.m�ļ�
%input����5�����ɶȵĶ���
clc;clear;
val_arm=[1 1 0.1];
input=[30 15 60 20 10];
a=armModel(val_arm);
temp=degreeFreedomToWatchOrientation2(a,input,1,1);