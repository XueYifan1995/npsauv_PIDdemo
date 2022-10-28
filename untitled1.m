clc;
clear, 
close all;
% x = [u; v; w; p; q; r; x; y; z; phi; theta; psi ]';
% ui = [ delta_r delta_s delta_b delta_bp delta_bs n ]';
t=1;
x0 = [1;0;0;0;0;0;0;0;10;0;0;0];
ui = [-pi/9;0;0;0;0;1200];
x(:,1) = x0(1:12);
% ui(:,1)=ui(1:6);
for k=1:200
    [xdot(:,k),U(:,k)] = npsauv(x(:,k),ui);
    st = x(:,k);  con =xdot(:,k);
    st_next  = st+ (t*con);
    x(:,k+1) = st_next;
end
% for k=1:80
%     [xdot,U] = npsauv(x(:,k),ui);
%     st = x(:,k);  con =xdot;
%     st_next  = st+ (t*con);
%     x(:,k+1) = st_next;
% end
y=x(12,:)*180/pi;
figure(1)
plot(y,'m');
legend('heading');
figure(2)
plot(x(11,:),'g');
legend('pitch');
figure(3)
plot(x(10,:),'r');
legend('roll');
figure(4)
plot(x(9,:),'b');
legend('z');
figure(5)
plot(x(7,:),x(8,:),'m');
legend('x y');
figure(6)
plot(x(6,:),'g');
legend('yaw velocity');
figure(7)
plot(x(5,:),'r');
legend('pitch velocity');
figure(8)
plot(x(4,:),'b');
legend('roll velocity');
figure(9)
plot(x(3,:),'g');
legend('w');
figure(10)
plot(x(2,:),'r');
legend('v');
figure(11)
plot(x(1,:),'b');
legend('u');