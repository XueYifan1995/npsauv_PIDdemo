clc;
clear;
clear ILOSpsi;
close all;
ts=0.5;  %采样时间=0.5s
wpt.pos.x   = [60  140 250 200 0];
wpt.pos.y   = [50  160 110 20 0 ];

Delta=6*1.5;
kappa=0.01;
R_switch=10;
e_1=0;      %前一时刻的偏差      
Ee=0;       %累积偏差
e_2=0;      %前一时刻的偏差      
Eu=0;       %累积偏差
% u_1=0.0;    %前一时刻的控制量
% y_1=0;       %前一时刻的输出
x_1 = [0.1;0;0;0;0;0;0;0;10;0;0;0];
u_1 = [0;0;0;0;0;0];
%PID参数
kp=12;    
ki=0.001;
kd=50;
kpu=10000;    
kiu=20;
kdu=200;
u=zeros(6,1000);%预先分配内存
time=zeros(1,1000);%时刻点（设定1000个）
x(:,1) = x_1(1:12);
u(:,1) = u_1(1:6);
for k=1:1:1000
    time(k)=k*ts;   %时间参数
    r(k) = ILOSpsi(x(7,k),x(8,k),Delta,kappa,ts,R_switch,wpt);
    ru(k)=1.2;
    [xdot(:,k),U(:,k)] = npsauv(x(:,k),u(:,k));
    st = x(:,k);  con =xdot(:,k);
    st_next  = st+ (ts*con);
    x(:,k+1) = st_next;
    e(k)=r(k)-x(12,k+1);   %误差信号
    u(1,k)=-(kp*e(k)+ki*Ee+kd*(e(k)-e_1)); %系统PID控制器输出序列
    Ee=Ee+e(k);    %误差的累加和
    u(1,k+1)=u(1,k); %前一个的控制器输出值
    e_1=e(k);		%前一个误差信号的值
    eu(k)=ru(k)-x(1,k+1);   %误差信号
    u(6,k)=(kpu*eu(k)+kiu*Eu+kdu*(eu(k)-e_2)); %系统PID控制器输出序列
    Eu=Eu+eu(k);    %误差的累加和
%     u(1,k+1)=u(1,k); %前一个的控制器输出值
    u(6,k+1)=u(6,k);
%     y_1=y(k);    	%前一个的系统响应输出值
    e_2=e(k);		%前一个误差信号的值
end
%（仅绘制过渡过程的曲线，x坐标限制为[0,1]）
figure(1)
subplot(3,1,1);plot(time,x(1,2:end),'g');legend('u');
subplot(3,1,2);plot(time,x(2,2:end),'r');legend('v');
subplot(3,1,3);plot(time,x(3,2:end),'b');legend('w');
figure(2)
subplot(3,1,1);plot(time,x(4,2:end),'g');legend('p');
subplot(3,1,2);plot(time,x(5,2:end),'r');legend('q');
subplot(3,1,3);plot(time,x(6,2:end),'b');legend('r');
figure(3)
subplot(3,1,1);plot(time,x(10,2:end),'g');legend('roll');
subplot(3,1,2);plot(time,x(11,2:end),'r');legend('pitch');
subplot(3,1,3);plot(time,x(12,2:end),'b');legend('yaw');
figure(4)
plot(x(7,2:end),x(8,2:end),'m');
str={'*','*','*','*','*'};
text(wpt.pos.x,wpt.pos.y,str);
legend('position x y');
figure(5)
p1=plot(time,r,'-.');xlim([0,200]);hold on;%指令信号的曲线（即期望输入）
p2=plot(time,x(12,2:end),'r');xlim([0,200]);%PID曲线
legend('heading');
figure(6)
p1=plot(time,ru,'-.');xlim([0,200]);hold on;%指令信号的曲线（即期望输入）
p2=plot(time,x(1,2:end),'r');xlim([0,200]);%PID曲线
legend('u');
hold on;