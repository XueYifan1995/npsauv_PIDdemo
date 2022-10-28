clc;
clear;
close all;
ts=0.5;  %采样时间=0.5s
e_1=0;      %前一时刻的偏差      
Ee=0;       %累积偏差
x_1 = [0.1;0;0;0;0;0;0;0;0;0;0;0];
u_1 = [0;0;0;0;0;0];
%PID参数
kp=2000;    
ki=15;
kd=100;
u=zeros(6,1000);%预先分配内存
time=zeros(1,1000);%时刻点（设定1000个）
x(:,1) = x_1(1:12);
u(:,1) = u_1(1:6);
for k=1:1:1000
    time(k)=k*ts;   %时间参数
%     r(k)=0.8;      %期望值
    r(k)=0.6;
%     y(k)=-1*den(2)*y_1+num(2)*u_1+num(1)*u(k);%系统响应输出序列
    [xdot(:,k),U(:,k)] = npsauv(x(:,k),u(:,k));
    st = x(:,k);  con =xdot(:,k);
    st_next  = st+ (ts*con);
    x(:,k+1) = st_next;
    e(k)=r(k)-x(1,k+1);   %误差信号
    u(6,k)=(kp*e(k)+ki*Ee+kd*(e(k)-e_1)); %系统PID控制器输出序列
    Ee=Ee+e(k);    %误差的累加和
%     u(1,k+1)=u(1,k); %前一个的控制器输出值
    u(6,k+1)=u(6,k);
%     y_1=y(k);    	%前一个的系统响应输出值
    e_1=e(k);		%前一个误差信号的值
end

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
legend('position x y');
figure(5)
p1=plot(time,r,'-.');xlim([0,400]);hold on;%指令信号的曲线（即期望输入）
p2=plot(time,x(1,1:1000),'r');xlim([0,400]);%PID曲线
legend('heading');
hold on;
