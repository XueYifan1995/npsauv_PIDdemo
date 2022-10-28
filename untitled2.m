clc;
clear;
close all;
ts=0.5;  %采样时间=0.005s
sys=tf(0.998,[0.021,1]);   %建立被控对象传递函数，即式4.1
dsys=c2d(sys,ts,'z');      %离散化
[num,den]=tfdata(dsys,'v');   %
e_1=0;      %前一时刻的偏差      
Ee=0;       %累积偏差
u_1=0.0;    %前一时刻的控制量
y_1=0;       %前一时刻的输出
%PID参数
kp=0.22;    
ki=0.13;
kd=0;
u=zeros(6,100);%预先分配内存
time=zeros(1,100);%时刻点（设定1000个）
for k=1:1:100
    time(k)=k*ts;   %时间参数
    r(k)=1500;      %期望值
    y(k)=-1*den(2)*y_1+num(2)*u_1+num(1)*u(k);%系统响应输出序列
    e(k)=r(k)-y(k);   %误差信号
    u(k)=kp*e(k)+ki*Ee+kd*(e(k)-e_1); %系统PID控制器输出序列
    Ee=Ee+e(k);    %误差的累加和
    u_1=u(k);    	%前一个的控制器输出值
    y_1=y(k);    	%前一个的系统响应输出值
    e_1=e(k);		%前一个误差信号的值
end
%（仅绘制过渡过程的曲线，x坐标限制为[0,1]）
p1=plot(time,r,'-.');xlim([0,50]);hold on;%指令信号的曲线（即期望输入）
p2=plot(time,y,'--');xlim([0,50]);%不含积分分离的PID曲线
hold on;
