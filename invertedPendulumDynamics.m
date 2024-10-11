%% define the state space model
% x' = f([x]) = [A]x + [B]u
l = 0.3; %meters to com of pendulum
M = 1; %kg
m = .3; %kg
I = (m*l^2)/3; %kg m^2
g = 9.81; %m/s^2
Amp = 4;
r_wheel = l/3; % wheel radius (m)
GR = 2/3; % gear ratio 
F2T = GR*r_wheel; %force to torque conversion (T = F* GR* r_wheel) 
T2F = 1/(GR*r_wheel);
x = 0; %x: cart pos (m)
x_dot = 0; %x': cart velo (m/s)
theta = pi; %θ: pend angle (rad)
theta_dot = 0; %θ': pend angular velo (rad/s)
thetaDesired = 0;
wc = .4; %cutoff frequency
wn = wc; %sqrt(g/l)/4;
k1 = 2;
zeta = 1; %damping ratio
states0 = [x; x_dot; theta; theta_dot];
states = [x; x_dot; theta; theta_dot];
statesDesired = [0, 0, thetaDesired, 0];
Ed = m*g*l*cos(thetaDesired); %desired energy is purely potential, when upright
bTheta = .03; % viscous friction coeff N/m/sec
bX = 0.6;
l2 = l*l; ml = m*l;  ml2 = m*l^2;

%% my new calcs %%
d = (((4/3)*m*l2) * (M+m)) - (ml)^2;


a22 = -(4/3)*m*l^2*bX; %x'
a23 = -(ml)^2*g; %θ
a24 = -m*l*bTheta; %θ'

a42 = -m*l*bX; %x'
a43 = ml*g*(M+m); %θ
a44 = -(M+m)*bTheta; %θ'


b21 = (4/3)*m*l2;%F
b41 = -m*l; %F
%          x  x'   θ     θ'
A = (1/d)*[0, 1,   0,    0;    %x'
           0, a22, a23,  a24;  %x''
           0, 0,   0,    1;    %θ'
           0, a42, a43,  a44]; %θ''
B = (1/d)*[0;b21;0;b41];


C = [1, 0,0,0;
    0,0,1,0];

D = [0;0];

%sys = ss(A,B,C,D);

%%%%%%% designing the LQR controller %%%%%%%

Q = eye(4); 
weights = [1,2,1,1];
for i = 1:4
    Q(i,i) = weights(i);
end
R = 1/10;

K = [-31.6228, -22.4894, 87.1251, 18.1589];

k = 100; %%gain of the control law

n = 2.1;

phi = 25; %error from upright pos, in degrees
R2D = 180/pi;
D2R = pi/180;

%% motor dynamics  
Jm = 0.01; %(J)     moment of inertia of the rotor     0.01 kg.m^2
bm = 0.1; %motor viscous friction constant             0.1 N.m.s
Km = 0.01; %electromotive force constant               0.01 V/rad/sec
Rm = 1;%electric resistance                            1 Ohm
Lm = 0.5;%electric inductance                          0.5 H

Am = [-bm/Jm,   Km/Jm;
    -Km/Lm   -Rm/Lm];
Bm = [0
    1/Lm];
Cm = [1   0];
Dm = 0;

%%%% EKF additions 
R = .01; %measurement noise
Q = 0.02; %process noise
Ts = .01; %sample time;

initCovariance = (1/(10^7))*[1 0 0 0;
                  0 1 0 0;
                  0 0 1 0;
                  0 0 0 1];

modelCovariance = [.002 0 0 0;
                  0 .002 0 0;
                  0 0 .002 0;
                  0 0 0 .002];

measurementCovariance = [.001 0 0 0;
                          0 .001 0 0;
                          0 0 .001 0;
                          0 0 0 .001];
%%%% Partial Feedback Gains %%%%%% 
K_E = .6;
K_x = .2;
K_xdot = .2;

