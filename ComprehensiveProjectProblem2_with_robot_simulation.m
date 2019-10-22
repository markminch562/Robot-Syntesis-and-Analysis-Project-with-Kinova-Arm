clear;

%----------- **** Importaint ****--------%
%--- If You want the robot on the the scatter plot make R_scatter = 1 %
%--- If not set R_scatter = 0
%----------- ******************** -------%
R_scatter = 1;

%----------- **** Importaint ****--------%
%--- control Robot Ploting speed with R_speed %
%--- sugested speed is between  5 - 50 ---%
%----------- ******************** -------%
R_speed = 20;
fig_num = 1;

%--- if you dont want a robot simulation set 'no_Robot_graph' to true
no_Robot_graph = false;

%-----Initial Parameters----- 
qs=[0; 0;]; %Initial joint angles
qf=[pi/2; pi/2;]; %Desired joint angles
obstacle=[2; 0.5;]; %Obstacle position
%Calculations to determine final joint positions
fO1=[cos(qf(1,1)); sin(qf(1,1))];                                                                 %Final joint 1 position
fO2=[cos(qf(1,1)) + cos([qf(2,1)+qf(1,1)]); sin(qf(1,1)) + sin([qf(2,1)+qf(1,1)]);];              %Final joint 2 position
%Gain Parameters
xi1=1; %Attractive force gain
xi2=1; %Attractive force gain
alpha=[.01;.01;]; %Joint rotation gains
eta=1; %Repulsive force gain
rho=.5; %Distance of influence for obstacles in meters

%L1= Link([0 1 0 pi/2 ], 'standard');
L1 = Link('d', 0, 'a', 1, 'alpha', 0);
L2 = Link('d', 0, 'a', 1, 'alpha', -pi/2);
%L2= Link([0 1 0 pi], 'standard');
links=[L1 L2];
R=SerialLink(links);

q0=[-.125 0];
q1=[pi pi];

%t = [0:.05:2]'; 	% generate a time vector
%qt1 = jtraj(q0, q1, t);
%R.plot(qt1);

%------------------------------Path Planning------------------------------
iterations=0;
posO1x(1)=cos(qs(1,1));
posO1y(1)=sin(qs(1,1));
posO2x(1)=cos(qs(1,1)) + cos([qs(2,1)+qs(1,1)]);
posO2y(1)=sin(qs(1,1)) + sin([qs(2,1)+qs(1,1)]); 
i= 0;

while(abs(qf(1,1)-qs(1,1))>.01 || abs(qf(2,1)-qs(2,1))>.01)
if no_Robot_graph == false
    
if rem(i, R_speed) == 0
    qb = qs';    
    R.plot(qb)
    hold on

if i == 0
    scatter(-1,1,1000)
    scatter(-1,1,10,'MarkerFaceColor', 'b' )
    scatter(2,0.5,1000)
    scatter(2,0.5,10,'MarkerFaceColor','r')
end 
    
figure(1);
if rem(i, R_speed) == 0 && R_scatter == 1 && iterations > 0
    scatter(posO1x(iterations),posO1y(iterations),'MarkerEdgeColor', 'b')
    scatter(posO2x(iterations),posO2y(iterations),'MarkerEdgeColor','r')
end
end
end



%Determining initial joint positions...
iO1=[cos(qs(1,1)); sin(qs(1,1))];                                                                 %Initial joint 1 position
iO2=[cos(qs(1,1)) + cos([qs(2,1)+qs(1,1)]); sin(qs(1,1)) + sin([qs(2,1)+qs(1,1)]);];              %Initial joint 2 position

%Determine the attractive forces for the joints to desired positions
fAtt1=-xi1*(iO1-fO1);
fAtt2=-xi2*(iO2-fO2);

%Determine the repulsive forces for the joints to obstacle
fRep1=0; %Joint 1 will be uninfluenced due to distance from obstacle, this can be changed if influence is desired
proximityO2=sqrt((obstacle(1,1)-iO2(1,1))^2+(obstacle(2,1)-iO2(2,1))^2); %Determine the distance from joint 2 to the obstacle
fRep2=eta*[(1/proximityO2)-(1/rho)]*(1/(proximityO2^2))*((iO2-obstacle)/abs(iO2-obstacle)); %Repulsive force of joint 2 to obstacle (2x2 matrix)
fRep2=[fRep2(1,2); fRep2(2,2);]; %Repulsive force of joint 2 to obstacle (2x1 matrix)

%Determine the total forces on each joint
fTotal1=fAtt1+fRep1;
fTotal2=fAtt2+fRep2;

%Determine the current jacobian matrix
j1=[-sin(qs(1,1)) 0; cos(qs(1,1)) 0;];
j2=[-sin(qs(1,1))-sin(qs(1,1)+qs(2,1)) -sin(qs(1,1)+qs(2,1)); cos(qs(1,1))+cos(qs(1,1)+qs(2,1)) cos(qs(1,1)+qs(2,1))];

%Determine the joint torques
torque=j1'*fTotal1 + j2'*fTotal2;
qs=torque.*alpha+qs;
iterations=iterations+1;
posO1x(iterations)=iO1(1,1);
posO1y(iterations)=iO1(2,1);
posO2x(iterations)=iO2(1,1);
posO2y(iterations)=iO2(2,1);
torque1(iterations)=torque(1,1);
torque2(iterations)=torque(2,1);
i= i+1;
end
if R_scatter == ~1
    hold off
    fig_num = fig_num + 1;
end

figure(fig_num);
hold on
scatter(posO1x,posO1y,20,1:iterations)
scatter(2,0.5,1000)
scatter(-1,1,1000)
scatter(posO2x,posO2y,20,1:iterations)
title('Joint Positions Over Iterations');
hold off
grid on
axis([-1.5 2 -1.5 2])
pbaspect([1 1 1])
figure(fig_num+1);
subplot(2,1,1);
plot(1:iterations,torque1);
title('Joint 1 Torque');
subplot(2,1,2);
plot(1:iterations,torque2);
title('Joint 2 Torque');
