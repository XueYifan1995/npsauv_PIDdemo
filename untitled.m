clear, 
clc;
close all;

ts=0.001; %采样时间=0.001s

sys=tf(5.235e005,[1,87.35,1.047e004,0]); %建立被控对象传递函数

dsys=c2d(sys,ts,'z'); %把传递函数离散化

[num,den]=tfdata(dsys,'v'); % 离散化后提取分子、分母

u_1=0.0;u_2=0.0;u_3=0.0; %输入向量 的初始状态

y_1=0.0;y_2=0.0;y_3=0.0; %输出的初始状态

x=[0,0,0]; %PID的3个参数Kp Ki Kd组成的数组

error_1=0; %初始误差

S=input('2');

for k=1:1:500

time(k)=k*ts; % 仿真时间500ms

if S==1;

kp=1.50;ki=0.01;kd=0.01;

yd(k)=1; % 指令为阶跃信号

elseif S==2

kp=0.50;ki=0.001;kd=0.001;

yd(k)=sign(sin(2*2*pi*k*ts)); % 指令为方波信号

elseif S==3

kp=1.5;ki=1.0;kd=0.01; % 指令为正弦信号

yd(k)=0.5*sin(2*2*pi*k*ts);

end

u(k)=kp*x(1)+kd*x(2)+ki*x(3); % PID控制器

% 限制控制器的输出

if u(k)>=10

u(k)=10;

end

if u(k)<=-10

u(k)=-10;

end

% 近似线性模型

y(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(2)*u_1+num(3)*u_2+num(4)*u_3;

error(k)=yd(k)-y(k);

% 返回pid参数

u_3=u_2;u_2=u_1;u_1=u(k);

y_3=y_2;y_2=y_1;y_1=y(k);

x(1)=error(k); % 计算 P

x(2)=(error(k)-error_1)/ts; % 计算 D

x(3)=x(3)+error(k)*ts; % 计算 I

error_1=error(k);

end

figure(1);

set(0,'defaultfigurecolor','w') % 设置图像背景为白色

plot(time,yd,'r',time,y,'b','linewidth',2);

xlabel('time(s)');ylabel('信号输出');

legend('理想信号','追踪信号');
hold on