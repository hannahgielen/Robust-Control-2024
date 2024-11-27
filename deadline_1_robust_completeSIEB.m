%% Question 1.2

% Create SISO system
sys_siso = FWT(1,1);
sys_siso = -sys_siso;
sys_siso_min = minreal(sys_siso);
sys_tf = tf(sys_siso_min);

% Convert state-space to transfer function
% A_siso = A;
% B_siso = B(:, 1);
% C_siso = C(1, :);
% D_siso = D(1,1);
% [numerator, denominator] = ss2tf(A_siso, B_siso, C_siso, D_siso, 1);
% sys_tf = tf(numerator, denominator);
% sys_tf = -sys_tf;

% Pole-zero map
figure('Name', 'Pole zero map of SISO system');
pzmap(sys_siso_min);
 
% Plot step response
figure('Name', 'Step response of SISO system');
step(sys_siso_min);

% Bode plot with margins
figure('Name', 'Bode plot of SISO system');
margin(sys_siso_min);

% Display stepinfo of uncontrolled SISO system
stinf_unc = stepinfo(sys_siso_min);
disp('Stepinfo of uncontrolled SISO system:')
disp(stinf_unc);

% Display DC-gain to find possible steady-state error
dc_gain = dcgain(sys_siso_min);
disp('DC-gain of SISO sytem:')
disp(dc_gain);

% Display poles and zeros of SISO system
siso_poles = pole(sys_siso_min);
disp('SISO poles:')
disp(siso_poles);
siso_zeros = tzero(sys_siso_min);
disp('SISO zeros:')
disp(siso_zeros);

%% Question 1.3

% Define variable 's'
s = tf('s');

% Define controller constants
Kp = 0.7;
Ki = 0.28;

% Construct PI controller for reference tracking
contr_ref = pid(Kp, Ki);

% Open-loop system
siso_ref_ol = contr_ref * sys_siso_min; % Corresponds to C*G

% Closed-loop system
siso_ref_cl = feedback(contr_ref * sys_siso_min, 1);

% Display stepinfo of reference tracking PI controller
stinf = stepinfo(siso_ref_cl);
disp('Stepinfo of closed-loop SISO system with reference tracking PI controller:')
disp(stinf);

% Display bandwidth of reference tracking PI controller
bandw = bandwidth(siso_ref_cl);
disp('Bandwidth of closed-loop SISO system with reference tracking PI controller:')
disp(bandw);

% Display DC-gain to find possible steady-state error
dc_gain = dcgain(siso_ref_cl);
disp('DC-gain of closed-loop SISO system with reference tracking PI controller, value of 1 indicates no steady-state error:')
disp(dc_gain);

% Plot closed-loop step response of controlled system
figure('Name', 'Step response of SISO system with reference tracking PI controller');
step(siso_ref_cl);

% Bode plot with margins of open-loop system
figure('Name', 'Bode plot of SISO system with reference tracking PI controller');
margin(siso_ref_ol);

%% Question 1.4

% Use the SISO controller designed for reference tracking for disturbance rejection
sisocontroller_dist = contr_ref;

% Construct a generalized plant for simulation reasons
warning off
systemnames = 'FWT';     % The full wind turbine model
inputvar = '[V; Beta; Tau]';    % Input generalized plant
input_to_FWT = '[Beta; Tau; V]';
outputvar = '[FWT; Beta; Tau; FWT]';    % Output generalized plant also includes the inputs
sysoutname = 'Gsim';
sysic;
warning on

% Construct SISO controller for disturbance rejection
CL_sisocontroller_dist = minreal(lft(Gsim(1:end-1,1:end-1), sisocontroller_dist));


% Plot step response of the SISO controller for disturbance rejection
figure('Name', 'Step response of SISO system controlled with reference tracking PI controller used for disturbance rejection')
step(CL_sisocontroller_dist);

%% Question 2.1

% Construct MIMO plant with two inputs and two outputs
% A_mimo = A;
% B_mimo = B(:, [1,2]);
% C_mimo = C;
% D_mimo = D(:,[1,2]);
% sys_mimo = ss(A_mimo, B_mimo, C_mimo, D_mimo);
sys_mimo = FWT(1:2,1:2);
sys_mimo = minreal(sys_mimo);

% Construct RGA for omega = 0
G_0 = evalfr(sys_mimo,0);
S_0 = pinv(G_0)';
RGA0 = G_0.*S_0;
disp('RGA for omega = 0');
disp(RGA0);

% Construct RGA for omega = 0.3 x 2pi
G_0_3 = evalfr(sys_mimo,0.3*2*pi);
S_0_3 = pinv(G_0_3)';
RGA0_3 = G_0_3.*S_0_3;
disp('RGA for omega = 0.3x2pi')
disp(RGA0_3);

%% Question 2.2
% Display poles of the MIMO system
mimo_poles = pole(sys_mimo);
disp('MIMO poles:')
disp(mimo_poles);

% Display zeros of the MIMO system
mimo_zeros = tzero(sys_mimo);
disp('MIMO zeros:')
disp(mimo_zeros);

% Display Pole-zero map
figure('Name', 'Pole-zero map of MIMO system');
pzmap(sys_mimo);

%% Question 2.3

% Define parameters as defined in the problem discription
wB1 = 0.3 * 2 * pi;
a = 1 / 10000;
M = 3;
s = tf('s');

% Define Wp matrix
Wp=ss([((s/M)+wB1)/(s+wB1*a),0; 0 , 0.2]); 

% Define Wu matrix
Wu=ss([0.01,0;0,(5*(10^(-3))*s^2+ 7*(10^(-4)*s)+5*(10^(-5)))/(s^2+14*(10^(-4))*s+(10^(-6)))]);

%% Question 2.5

% Construct G_mimo transfer function from state space of plant
G_mimo = tf(sys_mimo);

% Construct elements of generalized plant P
P11 = [Wp; zeros(2, 2)];
P12 = [Wp * G_mimo; Wu];
P21 = -eye(2);
P22 = -G_mimo;

% Construct minimal realization of generalized plant P
P = ss([P11, P12; P21, P22]);
P = minreal(P);

% Find and display number of states in the plant
states_plant = size(P.A, 1);
disp(['Number of states in plant: ' num2str(states_plant)]);

% Alternative way of defining the generalized plant P
% G_mimo = tf(sys_mimo);

% % Construct a generalized plant for simulation reasons 
% warning off
% systemnames = 'Wp Wu G_mimo'; %The full model
% inputvar = '[w{2}; u{2}]'; %Input generalized plant
% input_to_Wp = '[w - G_mimo]';
% input_to_Wu = '[u]';
% input_to_G_mimo = '[u]';
% outputvar = '[Wp; Wu; w - G_mimo]'; %Output generalized plant also includes the inputs
% sysoutname = 'P';
% cleanupsysic = 'yes';
% sysic;
% warning on
% 
% P = minreal(P);
% P = ss(P);
% 
% % Find and display number of states in the plant
% states_plant = size(P.A, 1);
% disp(['Number of states in plant: ' num2str(states_plant)]);
%% Question 2.7
[K, CL, GAM, INFO] = hinfsyn(P, 2, 2);
L = K * G_mimo;
d = eye(2) + L;
det = d(1, 1) * d(2, 2) - d(1, 2) * d(2, 1);
figure('Name', 'Generalized Nyquist');
nyquist(det);

% Find and display number of states in the controller
states_cont = size(K.A, 1);
disp(['Number of states in controller : ' num2str(states_cont)]);

% Find and display number of states in the plant
states_plant = size(P.A, 1);
disp(['Number of states in plant: ' num2str(states_plant)]);

% Find open-loop poles of controller
poles_ol = pole(L);
disp('Open-loop poles:')
disp(poles_ol)

states_contr = size(K.A, 1);
disp(['Number of states in controller: ' num2str(states_contr)]);
%% Question 2.8

% Reference tracking of mixed-Sensitivity generalized controller
mimo_RF = feedback(G_mimo * K, eye(size(K,2)));
figure('Name', 'Mixed-sensitivity generalized controller vs manually designed PI controller (reference tracking)');
step(mimo_RF(1, 1))
hold on 
step(siso_ref_cl);
legend('Mixed-sensitivity generalized controller', 'Manually designed PI controller')
hold off

figure;
step(mimo_RF)

disp('Stepinfo of mixed-sensitivity generalized controller:');
disp(stepinfo(mimo_RF(1,1)));

% Disturbance rejection of mixed-sensitivity generalized controller
mimocontroller_dist = K;
figure('Name', 'Time-domain simulations of the mixed-sensitivity generalized controller');
CL_mimocontroller_dist = minreal(lft(Gsim, -mimocontroller_dist));
step(CL_mimocontroller_dist);
hold on
step(CL_sisocontroller_dist);
legend('Mixed-sensitivity generalized controller', 'Manually designed PI controller')
hold off

% Compare disturbance rejection of mixed-sensitivity to manually designed PI controller
figure('Name', 'Mixed-sensitivity generalized controller vs manually designed PI controller (disturbance rejection');
step(CL_mimocontroller_dist(1,1));
hold on
step(CL_sisocontroller_dist(1,1));
legend('Mixed-sensitivity generalized controller', 'Manually designed PI controller')
hold off

%% Fixed Structure SISO controller

% Setting up plant to synthesize fixed structure controller
s = tf('s');
Kp = realp('Kp', 0); 
Ki = realp('Ki', 0); 
Wp_simple = 0.95 * (s + 0.02 * 2 * pi) / (0.0002 * 8 * pi + s);
C_struct = Kp + Ki / s;

% SISO hinfstruct
Wp_siso = Wp_simple;
Wp_siso.u = 'y_act';
Wp_siso.y = 'z1';
G_siso = sys_tf;
G_siso.u = 'u';
G_siso.y = 'y_plant';
C_struct.u = 'e';
C_struct.y = 'u';

Sum1 = sumblk('e = -y_act');
Sum2 = sumblk('y_act = y_plant + y_dist');
Siso_Con = connect(G_siso, Wp_siso, C_struct, Sum1, Sum2, {'y_dist'}, {'y_act', 'z1'});
opt = hinfstructOptions('Display', 'final', 'RandomStart', 5); 
N_siso = hinfstruct(Siso_Con, opt);

% Extract controller gains:
kp_opt = N_siso.Blocks.Kp.Value;
ki_opt = N_siso.Blocks.Ki.Value;
kfb_opt = kp_opt + ki_opt/s;
SYS = feedback(G_siso * kfb_opt, 1);

% Simulate reference tracking and comparing with manual PI controller
figure('Name', 'Fixed structure vs manually designed controller (reference tracking)');
step(SYS)
hold on
step(siso_ref_cl)
legend('Fixed-structure', 'Manually designed')

disp('Optimal value for Kp:')
disp(kp_opt);
disp('Optimal value for Ki:')
disp(ki_opt);

S = stepinfo(SYS);
disp('Step info of fixed-structure controller:')
disp(S)

disp('Step info of manually designed PI controller:')
disp(stinf)

%% Fixed Structure MIMO controller

% Look at bode plot of K to see which in-/output channels are of influence
figure('Name', 'Bode plot of controller K');
bode(K);

kp1=realp('kp1',1);
ki1=realp('ki1',1);
kd1=realp('kd1',1);
tf1=realp('tf1',1);

kp2=realp('kp2',1);
ki2=realp('ki2',1);
kd2=realp('kd2',1);
tf2=realp('tf2',1);

C1 = kp1 + ki1/s + kd1*s/(tf1*s + 1 );
C2 = kp2 + ki2/s + kd2*s/(tf2*s + 1 );
C_struct = [C1 0;C2 0];

Wp_simp=[((s/M)+wB1)/(s+wB1*a),0; 0 ,0.2];
Wu_simp=[0.01,0;0,(5*10^(-3)*s^2+ 7*10^(-4)*s+5*10^(-5))/(s^2+14*10^(-4)*s+10^(-6))];

% MIMO hinfstruct
Wp_mimo = Wp_simp;
Wp_mimo.u = {'y_act'};
Wp_mimo.y = {'z1'};

Wu_mimo = Wu_simp;
Wu_mimo.u = {'u'};
Wu_mimo.y = {'z2'};

G_mimo=-1*G_mimo;
G_mimo.u= {'u'};
G_mimo.y = {'y_plant'};

C_struct.u = {'e'};
C_struct.y = {'u'};

S1 = sumblk('e = -y_act',2);
S2 = sumblk('y_act = y_plant + y_dist',2);

Mimo_Con = connect(G_mimo, Wp_mimo, Wu_mimo, C_struct, S1, S2, {'y_dist'}, {'z1','z2'});
Opt = hinfstructOptions('Display','final','RandomStart',5) ;
N_mimo = hinfstruct(Mimo_Con, Opt);

% Extract controller gains:

kp1_opt = N_mimo.Blocks.kp1.Value;
ki1_opt = N_mimo.Blocks.ki1.Value;
kd1_opt = N_mimo.Blocks.kd1.Value;
tf1_opt = N_mimo.Blocks.tf1.value;
kfb1_opt = kp1_opt+ki1_opt/s+kd1_opt*s/((tf1_opt*s)+1);

kp2_opt = N_mimo.Blocks.kp2.Value;
ki2_opt = N_mimo.Blocks.ki2.Value;
kd2_opt = N_mimo.Blocks.kd2.Value;
tf2_opt = N_mimo.Blocks.tf2.value;
kfb2_opt = kp2_opt+ki2_opt/s+kd2_opt*s/((tf2_opt*s)+1);

K_mimo = [kfb1_opt 0;kfb2_opt 0];

% Reference tracking for fixed structure MIMO controller
sys_mimo_fixed = feedback(G_mimo*K_mimo,eye(size(K_mimo,2)));

% Compare H_inf controller to fixed structure controller for reference tracking
figure('Name', 'H_inf vs fixed structure controller for disturbance rejection')
step(sys_mimo_fixed(1:2, 1));
hold on
step(mimo_RF(1:2, 1));
legend('Fixed structure', 'H_inf')
hold off

% Display step info of fixed structure controller for reference tracking
disp('Step info of fixed structure controller for reference tracking:')
disp(stepinfo(sys_mimo_fixed(1,1)))


% Disturbance rejection for fixed structure MIMO controller
mimocontroller_dist_fixed = K_mimo;
CL_mimocontroller_dist_fixed = minreal(lft(Gsim, mimocontroller_dist_fixed));

% Compare H_inf controller to fixed structure controller for disturbance rejection
figure('Name', 'H_inf vs fixed structure controller for disturbance rejection')
step(CL_mimocontroller_dist_fixed);
hold on
step(CL_mimocontroller_dist);
legend('Fixed structure', 'H_inf')
hold off

% Display transfer functions of the two PID controllers
disp('Transfer function of controller kfb1 (from input 1 to output 1:')
disp(kfb1_opt)

disp('Transfer function of controller kfb2 (from input 1 to output 2:')
disp(kfb2_opt)

% Plot (complementary) sensitivity bode plots
S = (1+(G_mimo*K_mimo))^-1;
T = G_mimo*K_mimo/(1+(G_mimo*K));
S_inf = (1+(G_mimo*K))^-1;
T_inf = G_mimo*K/(1+(G_mimo*K));

figure('Name', 'Fixed-structure sensitivity');
bode(S)
figure('Name', 'H_inf sensitivity');
bode(S_inf)

figure('Name', 'Fixed-structure complementary sensitivity');
bode(T)
figure('Name', 'H_inf complementary sensitivity');
bode(T_inf)




