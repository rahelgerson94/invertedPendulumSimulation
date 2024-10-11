m  = .164948;
m = .3;
M = 1.035052;
l = 0.2425;
kdx = 1;
kv = 1;
kE = 1;
kx = 1/100;
g = 9.804;

% kdx = kdx*10;
% kv = kv*10;
% kE = kE*10;
% kx = kx*10;
% 
% kdx = kdx*1000;  
% kv = kv*1000;
% kE = kE*1000;
% kx = kx*1000;

x0 = 0;
dx0 = 0;
theta0 = 23*D2R;
dtheta0 = 0;

ml = m*l;
ml2 = ml*l;


phi = 25; %error from upright pos, in degrees
R2D = 180/pi;
D2R = pi/180;


A = 1/(M*l)*[0 , 1, 0, 0;
          0, 0, -m*l*g, 0;
          0, 0, 0, 1;
          0, 0, (M+m)*g, 0];
B = 1/(M*l)*[0;
            l;
            0;
            -1];

C = [1,0,0,0;
    0,0,1,0];

D = [0;0];


%%%%% LQR %%%%%
%[44 23 74 11].
Q = eye(4); 
weights = [60, 50, 100, 40];
R = 15;
for i = 1:4
    Q(i,i) = weights(i);
end
K = lqr(A,B,Q,R);

[n,d] = ss2tf(A,B,C,D);

Gp_x = tf(n(1,:), d)
Gp_theta = tf(n(2,:), d)
fprintf("-------Gp_x----------\n")
fprintf("Gp_x zeros");
disp(roots(n(1,:)));
fprintf("\n")
fprintf("Gp_x poles:");
disp(roots(d)');
