%进行可视化的.m文件
%input输入5个自由度的度数
clc;clear;
val_arm=[1 1 0.1];
input=[30 15 60 20 10];
a=armModel(val_arm);
temp=degreeFreedomToWatchOrientation2(a,input,1,1);