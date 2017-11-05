function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
persistent D_O_phi_prev D_O_theta_prev time_prev


% Store Variables to Plot 
global time Error1 Error2 Error3 M1 M2 M3 Force 

if(isempty(time))
	time = [];
	Error1 = [];	Error2 = [];	Error3 = [];  M1=[]; M2=[]; M3=[]; Force =[];
end

% Thrust
F = 0;

% Moment
M = zeros(3,1);

% System parameters
Mass = params.mass; % Kg
Inertia =  params.I; % 3x3 Matrix 
InvInertia = params.invI; % Inverse of Inertia
ArmLength  = params.arm_length;	% meters


Gravity = params.gravity; % m/sec^2



% Controller Gains - Moment 
Chunkphi = 50; Chunktheta = 40; Chunkpsi = 10;
Tdphi = (11.59114 - 11.5772)/8;	Tdtheta = (12.17326 - 12.16080657)/8; Tdpsi = (12.6022 - 12.588)/8;
Kpphi = 0.8*Chunkphi; Kptheta = 0.8*Chunktheta; Kppsi = 0.8*Chunkpsi;
Kdphi = 0; Kdtheta = 0; Kdpsi = 0;	% Initalize - Can remove if required. 
Kdphi = Kpphi*Tdphi; Kdtheta = Kptheta*Tdtheta;  Kdpsi = Kppsi*Tdpsi;

% Current Angles 
C.A.phi = state.rot(1);    C.A.theta = state.rot(2);  C.A.psi = state.rot(3);
% Current Omega
C.O.phi = state.omega(1);	C.O.theta = state.omega(2); C.O.psi = state.omega(3);

D.A.psi = des_state.yaw;	
D.O.psi = des_state.yawdot;

% Gains
KpPosphi = 5;	KvPosphi = 2;	
KpPostheta = 5;	KvPostheta = 2;

% Error in Position
ep.phi = des_state.pos(1) - state.pos(1); ep.theta =des_state.pos(2) - state.pos(2);	
% Error in Velocity
ev.phi = des_state.vel(1) - state.vel(1); ev.theta = des_state.vel(2) - state.vel(2);  	

RequiredAccphi = des_state.acc(1) + ( KpPosphi*(ep.phi) + KvPosphi*(ev.phi));
RequiredAcctheta = des_state.acc(2) + ( KpPostheta*(ep.theta) + KvPostheta*(ev.theta));

D.A.phi = (1/Gravity)*(RequiredAccphi*sin(D.A.psi) - RequiredAcctheta*cos(D.A.psi));
D.A.theta = (1/Gravity)*(RequiredAccphi*cos(D.A.psi) + RequiredAcctheta*sin(D.A.psi));

if(isempty(D_O_phi_prev))
    D.O.phi=0; D.O.theta=0;
else
        D.O.phi = (D.A.phi - D_O_phi_prev)/0.01; %(t-time_prev);
        D.O.theta = (D.A.theta - D_O_theta_prev)/0.01; %; (t-time_prev);
end

M(1) = Kpphi*(D.A.phi - C.A.phi) + Kdphi*(D.O.phi - C.O.phi); 
M(2) = Kptheta*(D.A.theta - C.A.theta) + Kdtheta*(D.O.theta - C.O.theta);
M(3) = Kppsi*(D.A.psi - C.A.psi) + Kdpsi*(D.O.psi - C.O.psi);

% Gains thrust 
ChunkForce = 20;
Td = (11.16 - 9.75)/8;
kp_Thrust = 0.8*ChunkForce; kd_Thrust = kp_Thrust*Td;
% Thrust 
F = Mass*Gravity - Mass*(kd_Thrust*(state.vel(3) - des_state.vel(3)) + kp_Thrust*(state.pos(3) - des_state.pos(3)));

% =================== Your code ends here ===================
% Store for Next turn 
D_O_phi_prev = D.A.phi;
D_O_theta_prev = D.A.theta;
time_prev = t;


% ---- Storing ---- %
time = [time;t];
Error1 = [Error1;D.A.phi - C.A.phi]; 
Error2 = [Error2;D.A.theta - C.A.theta]; 
Error3 = [Error3;D.A.psi - C.A.psi]; 

M1 = [M1;M(1)];
M2 = [M2;M(2)];
M3 = [M3;M(3)];

Force = [Force;F];

end
