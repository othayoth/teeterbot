%% Theory based on Katariya (2010)
% https://smartech.gatech.edu/bitstream/handle/1853/33465/katariya_ashish_s_201005_ro.pdf
% PDF included in the repository


clear 

%% system parameters for teeterbot (symbols according to the convention in Katariya (2010)

M_w  = 1.0    ;% kg      -- mass of wheel
M_c  = 10.0   ;% kg      -- mass of chassis
I_w  = 0.02   ;% kgm^2   -- moment of inertia of wheel
I_c  = 0.608  ;% kgm^2   -- moment of inertia of chassis (pitch)
I_y  = 0.15   ;% kg^2    -- moment of inertia of chassis (yaw)
r    = 0.2    ;% m       -- wheel radius
l    = 0.4    ;% m       -- wheel axle to CoM distance
d    = 0.5    ;% m       -- wheel base
g    = 9.81   ;% ms^2    -- acceleration due to gravity

%% substitutions -- from Katariya (2010) , Page 36 , Equations (53) to (56)
s_mu    = 2*M_w*r*r + 2*I_w + M_c*r*r;
s_gamma = I_c + M_c*l*l;
s_beta  = M_w*d*d*r*r + I_w*d*d + 2*I_y*r*r;
s_psi   = M_c*l*r;

%% state space (linearised) -- from Katariya (2010) , Page 40 , Equation 69 
% system matrix --- consider full 6x6 matrix first
A      = zeros(6,6);
A(1,2) = 1;
A(2,3) = -(s_psi*s_psi*g)/(s_mu*s_gamma-s_psi*s_psi);
A(3,4) = 1;
A(4,3) =  (s_mu*M_c*g*l)/(s_mu*s_gamma-s_psi*s_psi);
A(5,6) = 1;

% control matrix --- consider full 6x2  matrix first
B = zeros(6,2);
B(2,1) = r*(s_gamma+s_psi)/(s_mu*s_gamma-s_psi*s_psi);
B(2,2) = r*(s_gamma+s_psi)/(s_mu*s_gamma-s_psi*s_psi);
B(4,1) = -(s_mu+s_psi)/(s_mu*s_gamma-s_psi*s_psi);
B(4,2) = -(s_mu+s_psi)/(s_mu*s_gamma-s_psi*s_psi);
B(6,1) =  d*r/s_beta;
B(6,2) = -d*r/s_beta;


%% Simplified state

% states:  x = [forward velocity, tilt, tilt velocity]
% control: u = [wheel torque] (both wheels have the same torque)
% output:  y = [forward velocity]
%-------------------
% state space model
% x_dot = Ax + Bu
%     y = Cx
A = A([2,3,4,6],[2,3,4,6]);
B = B([2,3,4,6],:)
C = [1 0 0 0; ...
     0 0 0 1]; 
D = zeros(2,2);
sys_ol = ss(A,B,C,D); % state space of open loop system
% ------------------

% ensure controllability of open loop system
if(rank(ctrb(A,B))==size(A,1))
    disp('open loop system is controllable')
else
    disp('open loop system is not controllable')
end
%% state space feedback control with integral action
% Goal -- drive the output y to a reference value r by feedback of the entire state
% For the linearised system, input is of the form
%  u = -K*x = K_r*r
% where
%   -->  K is the gain matrix
%   -->  r is the  reference value of the output (constant)
%   -->  K_r

% tracking with feedback
Ai = [A,zeros(4,2);-C zeros(2,2)]
Bi = [B;zeros(2,2)]
Ci = [C zeros(2,2)]
if(rank(ctrb(Ai,Bi)))
    disp('augmented system is controllable')
else
    disp('augmented system is not controllable')
end

% place the poles using LQR feedback
Qi = diag([10 100 100 10 50 50]);
% rationale for choosing gains
Ri = 0.1*eye(2)
Ki = lqr(Ai,Bi, Qi,Ri)

% desired state
xid = [0.1 0 0 0 0 0]';

% Construct the closed loop system (including integrator)
Hi = ss(Ai-Bi*Ki,Bi*Ki*xid - [0; 0; 0; 0;Ci*xid],Ci,0);
[yout,t,x]=step(Hi, 10);


figure(1)
clf
hold on
plot(t,x(:,5))
plot(t,x(:,2)*180/pi)

% s^2 + 32s + 400
% K = place(A, B, [-2 -36+2j -36-2j] )

% the closed loop system is then
% x_dot = (A-B*K)*x + B*k_r*r
% We will find feedback gain to suiotably place the poles of A-BK
% %  input penalty
% K_r = -1/(C*inv(A-B*K)*B)

% R = eye(2,2);
% rho = 10
% rho=10
% % state penalty
% Q = rho*(C'*C);
% Q = diag([10 2000 20 10]) 
% K = lqr(A,B,Q,R)
% %  K = place(A,B,[-.2 -9 -19 -81])
% eig(A-B*K)
% % g = [K eye(2,2)]*inv([A B; C D])*[zeros(4,2); eye(2,2)]
% % 
% % 
% 
% t = 0:0.01:2;
% u = zeros(numel(t),1);
% % sys_cl = ss(Ai-Bi*K,B,C,D);
% % 
% % K= place( [A 0;-C 0]-[B;0]*K,[B;0]);
% % sys_cl = ss( [A 0;-C 0]-[B;0]*K,[B;0]);
% % 
% x0 = [.1 50*pi/180 0 1 ];
% % lsim(sys_cl,u,t,x0);
% [yy,tt,xx]=lsim(Hi,u,t,x0);
% % 
% % figure(1)
% % clf
% % 
% % hold on
% % % plot(tt,yy)
% % plot(tt,xx(:,1))
% % plot(tt,xx(:,2)*180/pi)
% % plot(tt,xx(:,3))
% % legend({'v','\theta','\omega'})
% 
% % % Co = ctrb(A,B)
% % unco = length(A)-rank(Co)