clear variables
clc
%% solver parameters
%RK4
ST = 0.01; % time step [s]

%% initial conditions

Xe_0 = [ 0 0 -20 ]'; % initial position in NED (north east down) frame [m]
V0 = [ 0 0 0 ]'; % initial velocity in body frame [m/s]
euler0 = [ 0 0 0 ]'; % initial attitude (roll, pitch, yaw, angles) [rad] 
omega0 = [ 0 0 0 ]'; % initial rotational velocity (roll, pitch, yaw, rate) [rad/s]

latref = 44.200626; % reference latitude for earth fixed-origin [deg]
lonref = 12.063822; % reference longitude for earth fixed-origin [deg]
href = 0; % reference altitude for earth-fixed origin (zero if on surface but not necessarely at sea level!) [m]

%% inertial parameters

m = 2.15; % take-off mass [kg]
J = diag([ 0.0319 0.0287 0.0633 ]); % matrix of inertia in body frame [kg*m^2]
STACG = 0; % station line of CG [m]
BLCG = 0; % buttline of CG [m]
WLCG = -0.02; % water line of CG [m]

%% environmental conditions

wind_velocity_e = [ 0 0 0 ]'; % wind speed in NED frame [m/s]
wind_rate = [ 0 0 0 ]'; % wind rate in body frame [rad/s]
dyn_viscosity = 17.89e-6; % reference air dynamic viscosity [Pa*s]

%% main configuration parameters

b = 0.55/2; % half radial rotor-to-rotor distance [m]

STAH1 = -b*cosd(30); % station line of rotor 1 [m]
BLH1 = b*sind(30); % butt line of rotor 1 [m]
WLH1 = 32e-3; % water line of rotor 1 [m]
chi1 = 1; % 1 for anti-clockwise rotation, -1 for clockwise

STAH2 = 0; % station line of rotor 2 [m]
BLH2 = b; % butt line of rotor 2 [m]
WLH2 = 32e-3; % water line of rotor 2 [m]
chi2 = -1; % 1 for anti-clockwise rotation, -1 for clockwise

STAH3 = b*cosd(30); % station line of rotor 3 [m]
BLH3 = b*sind(30); % butt line of rotor 3 [m]
WLH3 = 32e-3; % water line of rotor 3 [m]
chi3 = 1; % 1 for anti-clockwise rotation, -1 for clockwise

STAH4 = b*cosd(30); % station line of rotor 4 [m]
BLH4 = -b*sind(30); % butt line of rotor 4 [m]
WLH4 = 32e-3; % water line of rotor 4 [m]
chi4 = -1; % 1 for anti-clockwise rotation, -1 for clockwise

STAH5 = 0; % station line of rotor 5 [m]
BLH5 = -b; % butt line of rotor 5 [m]
WLH5 = 32e-3; % water line of rotor 5 [m]
chi5 = 1; % 1 for anti-clockwise rotation, -1 for clockwise

STAH6 = -b*cosd(30); % station line of rotor 6 [m]
BLH6 = -b*sind(30); % butt line of rotor 6 [m]
WLH6 = 32e-3; % water line of rotor 6 [m]
chi6 = -1; % 1 for anti-clockwise rotation, -1 for clockwise

%% propeller parameters 

BLADES = 2; % number of blades 
ROTOR = 4*0.0254; % rotor radius [m]
CHORD = 17.55e-3; % mean aerodynamic chord [m]
CHORD75 = 21e-3; % blade chord at 75% radius [m]
ASLOPE = 5.9; % blade lift curve slope [1/rad]
Theta0 = 45*pi/180; % theoretical blade pitch angle at rotor hub (linear twist) [rad]
THETT = -38.0973*pi/180; % amount of twist from tip to root (theta_tip-theta_hub) [rad]
k_ind = 1.35; % induced power correction factor 
a0 = 0; % coning angle [rad]

%% frame parameters

Ax = 0.023; % equivalent flat plate fdrag area ortogonal to i_b [m^2]
Ay = 0.023; % equivalent flat plate fdrag area ortogonal to j_b [m^2]
Az = 0.106; % equivalent flat plate fdrag area ortogonal to k_b [m^2]
STACP = 0; % station line of frame centre of pressure [m]
BLCP = 0; % butt line of frame centre of pressure [m]
WLCP = -0.08; % waterline line of frame centre of pressure [m]

%% powerplant parameters
% ESC + motor system efficiency model for DJI OPTO 30 ESC + DJI 2212 motor
% eta_e = p00 + p10*Omega + p01*Q + p20*Omega^2 + p02*Q^2 + p11*Omega*Q;
p00 = 7.145e-2; p10 = 1.259e-3; p01 = 0.4377; p20 = -7.513e-7; p02 = -10.13; p11 = 1.284e-3;

% ESC + motor model for DJI OPTO 30 ESC + DJI 2212 motor + 8x4.5[in] prop
% Omega = k_Omega*delta^n
n = 0.6359;
k_Omega = 14.92;
PWM_idle = 1100; %[microseconds] idle value of pwm signal
PWM_max = 2000; %[microseconds] max value of pwm signal

%% Px4 controller parameters

% manual setpoint
Vsp = [0 0 0]';

% velocity controller
g0 = 9.80665; %  assumed gravity acceleration value for integral saturation [m/s2]
MPC_XY_VEL_P_ACC = 1.7;
MPC_Z_VEL_P_ACC = 4;

MPC_XY_VEL_I_ACC = 0.8;
MPC_Z_VEL_I_ACC = 2;

MPC_XY_VEL_D_ACC = 0.6;
MPC_Z_VEL_D_ACC = 0;

% attitude controller
MC_ROLL_P = 2; 
MC_PITCH_P = 2;
MC_YAW_P = 0.55;

% angular rate controller
MC_ROLLRATE_P = 0.09*3; 
MC_ROLLRATE_I = 0.05; 
MC_ROLLRATE_D = 0; 

MC_PITCHRATE_P = 0.09*3;
MC_PITCHRATE_I = 0.05;
MC_PITCHRATE_D = 0;

MC_YAWRATE_P = 0.15*3;
MC_YAWRATE_I = 0.02;
MC_YAWRATE_D = 0.05;

% trim condition
PWM_hover = (PWM_idle+PWM_max)/2;

%speed mapping from GCS input
MC_VEL_FWD = 15; %[m/s]
MC_VEL_LAT = 15; %[m/s]
MC_VEL_VERT = 15; %[m/s]

%attitude mapping from GCS input
MC_ATT_ROLL = 45*pi/180; %[rad]
MC_ATT_PITCH = 45*pi/180; %[rad]
MC_ATT_YAW = 180*pi/180; %[rad]

%angular rate mapping from GCS input
MC_RATE_ROLL = 500*pi/180; %[rad/s]
MC_RATE_PITCH = 500*pi/180; %[rad/s]
MC_RATE_YAW = 300*pi/180; %[rad/s]
