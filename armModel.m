classdef armModel
    %Establish a arm model.
    
    properties
%         %1. degree of freedom
%         %暂时无用
%         degfree_elbowPos=[-90,-180;-45,-270];%[-90,-225;-45,-270];
%         degfree_shoulderSpin=[0 -135];
%         degfree_elbow=[-180 -315];
        %2. para for arm
        length_bigArm=1;%大臂长度
        length_smallArm=1;%小臂长度
        radius_smallArm=0.1;%小臂半径，实际上是手表位置和手腕质心的距离
    end
    
    methods
        function obj=armModel(val)
        %构造函数：输入手臂参数
            obj.length_bigArm=val(1);
            obj.length_smallArm=val(2);
            obj.radius_smallArm=val(3);
        end
        
        function pos_wrist=degreeFreedomToWatchOrientation(obj,a,b,c,d)
        %测试函数：计算手肘和手腕在人坐标系中的坐标，只考虑4个自由度（还未未测试正确性，==。,不过这个函数没啥用）
            %caculate the pos of elbow
            %original point is shoulder
            y_elbow=roundn(obj.length_bigArm*cos(a*pi/180),-4);
            x_elbow=roundn(-obj.length_bigArm*sin(a*pi/180)*cos(b*pi/180),-4);
            z_elbow=roundn(-obj.length_bigArm*sin(a*pi/180)*sin(b*pi/180),-4);
            pos_elbow=[x_elbow y_elbow z_elbow];
            %caculate the pos of watch
            %original point is elbow
            x_wrist=roundn(obj.length_smallArm*cos(d*pi/180),-4);
            y_wrist=roundn(obj.length_smallArm*sin(d*pi/180)*cos(c*pi/180),-4);
            z_wrist=roundn(obj.length_smallArm*sin(d*pi/180)*sin(c*pi/180),-4);
            pos_wrist=[x_wrist y_wrist z_wrist];
            %spin the coordinate system of elbow
            dcm1=angle2dcm(pi/2,0,0,'YZX');
            dcm2=angle2dcm(0,0,(-90-a)*pi/180,'YZX');
            dcm3=angle2dcm(-(-90-b)*pi/180,0,0,'YZX');
            dcm=dcm3*dcm2*dcm1;
            pos_wristShoulder=pos_wrist*inv(dcm);
            %add the two corrdinate system and get the final pos of watch
            pos_wrist=pos_wristShoulder+pos_elbow;
            %orientation=[];
        end

        function returnData=degreeFreedomToWatchOrientation2(obj,angle,graphOK,oriOK)
        %功能函数：给定5个自由度此时的度数，得到手表坐标系中点在人坐标系中的位置，
        %返回手表在人坐标系的坐标
        %其中obj是实例化的对象，angle是一个1*5的向量，5个数据分别表示五个自由度的度数，graphOK是可视化开关，oriOK是orientation开关
            %0. 进行参数修正，使得输入参数的大小和正负与人的习惯保持统一
            angle(1)=90+angle(1);
            angle(2)=90+angle(2);
            angle(1)=-angle(1);
            angle(2)=-angle(2);
            angle(3)=-angle(3);
            angle(4)=180-angle(4);
            angle(5)=angle(5)+90;
            
            
            %1. caculate the pos in relatively coordinate system
            %计算手肘在人坐标系中的位置，计算手腕在手肘坐标系中的位置，列出手腕坐标系的法向量，原点和手表在手腕坐标系的位置
            %1.1 caculate the pos of elbow in the people coordinate system
            %original point is shoulder
            %计算手肘在人坐标系中的位置，放在pos_elbow_people中
            y_elbow_people=obj.length_bigArm*cos(angle(1)*pi/180);
            x_elbow_people=-obj.length_bigArm*sin(angle(1)*pi/180)*cos(angle(2)*pi/180);
            z_elbow_people=-obj.length_bigArm*sin(angle(1)*pi/180)*sin(angle(2)*pi/180);
            pos_elbow_people=[x_elbow_people y_elbow_people z_elbow_people];
            pos_elbow_people=roundn(pos_elbow_people,-4);
            %1.2 caculate the pos of watch in the elbow coordinate system
            %original point is elbow
            %计算手腕在手肘坐标系中的位置，放在pos_wrist_elbow中
%             x_wrist_elbow=obj.length_smallArm*cos(angle(4)*pi/180);
%             y_wrist_elbow=obj.length_smallArm*sin(angle(4)*pi/180)*cos(angle(3)*pi/180);
%             z_wrist_elbow=obj.length_smallArm*sin(angle(4)*pi/180)*sin(angle(3)*pi/180);
%             pos_wrist_elbow=[x_wrist_elbow y_wrist_elbow z_wrist_elbow];
%             pos_wrist_elbow=roundn(pos_wrist_elbow,-4);
            %1.3 the normal vector of watch coordinate and the pos of watch
            %original point is wrist
            %计算手腕坐标系中的信息，放在pendingVector_wrist中
            normal=eye(3);
            pos_watch_wrist=[0,0,-obj.radius_smallArm];
            pos_originPoint_wrist=[0,0,0];
            pos_plane1=[obj.radius_smallArm,obj.radius_smallArm,-obj.radius_smallArm];
            pos_plane2=[obj.radius_smallArm,-obj.radius_smallArm,-obj.radius_smallArm];
            pos_plane3=[-obj.radius_smallArm,-obj.radius_smallArm,-obj.radius_smallArm];
            pos_plane4=[-obj.radius_smallArm,obj.radius_smallArm,-obj.radius_smallArm];
            pos_plane=[pos_plane1;pos_plane2;pos_plane3;pos_plane4];
            x_watch=[0,-1,-obj.radius_smallArm];
            y_watch=[-1,0,-obj.radius_smallArm];
            z_watch=[0,0,-obj.radius_smallArm-1];
            coordinateSystem_watch=[x_watch;y_watch;z_watch];
            %pendingVector_wrist的前三行是手腕坐标系的法向量，第四行是手腕坐标系中手表的位置，
            %第五行是手腕坐标系的原点，6-9行四个点构成了手表平面,10-12行是手表坐标系的三个法向量
            pendingVector_wrist=[normal; pos_watch_wrist; pos_originPoint_wrist;pos_plane;coordinateSystem_watch];
            
            
            %2 translate the coordinate
            %2.1 先算出pendingVector在手肘坐标系中的位置
            %进行坐标系旋转
            dcm1=angle2dcm(0,0,(90-angle(5))*pi/180,'ZYX');
            dcm2=angle2dcm(0,-(180-angle(4))*pi/180,0,'ZYX');
            dcm=dcm2*dcm1;
            pendingVector_elbow=pendingVector_wrist*inv(dcm);
            %进行坐标系平移
            pendingVector_elbow(:,1)=pendingVector_elbow(:,1)-obj.length_smallArm*cos((180-angle(4))*pi/180);
            pendingVector_elbow(:,2)=pendingVector_elbow(:,2)+obj.length_smallArm*0;
            pendingVector_elbow(:,3)=pendingVector_elbow(:,3)+obj.length_smallArm*sin((180-angle(4))*pi/180);
            pendingVector_elbow=roundn(pendingVector_elbow,-4);
            %2.2 再算出pendingVector在人坐标系中的位置
            %spin the coordinate system of elbow
            %进行坐标系旋转
            dcm1=angle2dcm(0,pi/2,0,'ZYX');
            dcm2=angle2dcm(pi/2,0,0,'ZYX');
            dcm3=angle2dcm(0,0,(-90-angle(1))*pi/180,'ZYX');
            dcm4=angle2dcm(0,-(-90-angle(2))*pi/180,0,'ZYX');
            dcm5=quat2dcm([cos(-angle(3)*pi/360),sin(-angle(3)*pi/360)*pos_elbow_people]);
            dcm=dcm5*dcm4*dcm3*dcm2*dcm1;
            pendingVector_people=pendingVector_elbow*inv(dcm);
            %进行坐标系平移
            pendingVector_people=pendingVector_people+kron(pos_elbow_people,ones(12,1));
            pendingVector_people=roundn(pendingVector_people,-4);
            
            
            %3. 可视化
            if(graphOK==1)
                %3.1 画出身体平面
                figure
                hold on
                grid on
                fill3([0 0 0 0],[0 1 1 0],[-1 -1 0 0],'w');
                axis([-2.2 2.2 -2.2 2.2 -2.2 2.2]);
                xlabel('x');
                ylabel('y');
                zlabel('z');
                %3.2 画出大臂
                temp1=posPlotNode([0 0 0],pos_elbow_people,50);
                x=temp1(1,:);
                y=temp1(2,:);
                z=temp1(3,:);
                plot3(x,y,z,'k.');
                %3.3 画出小臂
                temp2=posPlotNode(pos_elbow_people,pendingVector_people(5,:),50);
                x=temp2(1,:);
                y=temp2(2,:);
                z=temp2(3,:);
                plot3(x,y,z,'b.');
                %3.4 画出手表位置
                plot3(pendingVector_people(4,1),pendingVector_people(4,2),pendingVector_people(4,3),'b*');
                %3.5 画出手表平面
                temp3=pendingVector_people(6:1:9,:);
                temp3=temp3';
                fill3(temp3(1,:),temp3(2,:),temp3(3,:),'r.');
%                 %3.6 手表坐标系的法向量，用来验证第四部分所要用的法向量是否正确
%                 temp4=posPlotNode(pendingVector_people(4,:),pendingVector_people(10,:),50);
%                 x=temp4(1,:);
%                 y=temp4(2,:);
%                 z=temp4(3,:);
%                 plot3(x,y,z,'LineWidth',1);
%                 temp5=posPlotNode(pendingVector_people(4,:),pendingVector_people(11,:),50);
%                 x=temp5(1,:);
%                 y=temp5(2,:);
%                 z=temp5(3,:);
%                 plot3(x,y,z,'LineWidth',1);
%                 temp6=posPlotNode(pendingVector_people(4,:),pendingVector_people(12,:),50);
%                 x=temp6(1,:);
%                 y=temp6(2,:);
%                 z=temp6(3,:);
%                 plot3(x,y,z,'LineWidth',1);   
                hold off
            end
            
            
            %4. 计算方向
            if(oriOK==1)
                %4.1 计算出手表坐标系在人坐标系原点的姿态
                xori_watch=pendingVector_people(10,:);
                yori_watch=pendingVector_people(11,:);
                zori_watch=pendingVector_people(12,:);
                origin_watch=pendingVector_people(4,:);
                %origin_people=[0,0,0];
                xori_people=xori_watch-origin_watch;
                yori_people=yori_watch-origin_watch;
                zori_people=zori_watch-origin_watch;
                %4.2 画图验证
%                 hold on
%                 temp7=posPlotNode(xori_people,origin_people,50);
%                 temp8=posPlotNode(yori_people,origin_people,50);
%                 temp9=posPlotNode(zori_people,origin_people,50);
%                 x=temp7(1,:);
%                 y=temp7(2,:);
%                 z=temp7(3,:);
%                 plot3(x,y,z,'LineWidth',1);
%                 x=temp8(1,:);
%                 y=temp8(2,:);
%                 z=temp8(3,:);
%                 plot3(x,y,z,'LineWidth',1);
%                 x=temp9(1,:);
%                 y=temp9(2,:);
%                 z=temp9(3,:);
%                 plot3(x,y,z,'LineWidth',1);
%                 hold off
                %4.3 计算两个坐标系之间旋转矩阵
                watch=[xori_people;yori_people;zori_people];
                people=eye(3);
                
            end
            
%             return
            returnData=pendingVector_people(4,:);
            
            
        end
    end
end

