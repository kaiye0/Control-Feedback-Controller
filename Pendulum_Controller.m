%% Hwk10 ME 5281 Fall 2020
% Control of Inverted Pendulum in State Space, Simulink Intro
% be sure to download Simulink Files and place them in same folder 
% as this file (e.g. Hwk9_PendulumModel.slx)


%% Define model parameters:
% tau =  J * x_dd +  D * x_d  + mgL*sin(x) /2
% u = tau % input torque
% y = x   % output theta, pendulum output position from simulink
% X = [x_dd x_d x]' = states % Full state from simulink

% the following are all used by the simulink model and pendulum subsystem:
g = 9.81;           % gravitational acc constant:  F = mg
m = 0.5 ;           % mass [kg]
L = 1;              % length [m]
J = m*L/3;          % rotational Inertia
D_friction = 1;     % friction

theta_0 = 90 ...   % initial pendulum position [degrees] converted to [rad]
    * pi/180;       % NOT IMPLEMENTED!  you must edit simulink model use
                    % this parameter.  Simulink system has hard-coded 90
                    % deg IC. We want to set arbitrary IC, theta_0, here.

Ts = 0.01;           % sample period in sec
tend = 10;          % simulation end time in sec
t_in = 0:Ts:tend;   % time input vector for simulink (simulink spits out t)
input = ...         % desired input = [ time_samples; command_values ]
    [ t_in'   zeros(length(t_in),1) ]; 

% simulink gives:
% t -- actual time in simulation world
% torque -- actual torque applied to model in simulation world at times t
% y -- actual position outputs at time t
% states -- all internal states [ x_dd x_d x] at times t

% %% Simulate Plant in Simulink at nonzero initial conditions
% sim('./Hwk9_PendulumModel.slx')  % <-- has 90 deg IC hardcoded; 
%                                   %  change this to theta_0 & save model
% open('./Hwk9_PendulumModel.slx') % don't need this every time

% %% Plot response with nonzero initial conditions
% figure(1);clf
% plot(t,states(:,1), 'bx', t,states(:,2), 'ro' )
% legend({'$$\theta$$', '$$\dot{\theta}$$'},'interpreter','latex')
% xlabel('time [sec]')
% ylabel('states (rad/[   ])')
% title('\bfPendulum Response - Nonzero Initial Conditions')
% 
% figure(2);clf
% subplot(2,1,1)
% plot(t, command , 'k.', t, plantInput, 'r-');
% legend({ 'command', 'plantInput'})
% ylabel('Input')
% xlabel('time [sec]')
% title('\bfPendulum Response - Nonzero Initial Conditions')
% 
% subplot(2,1,2)
% plot(t,y*180/pi, 'bx')  % converting to degress!
% xlabel('time [sec]')
% ylabel('Output [degrees]')
% 
% return; % stops running the script. Comment this line out to run the whole script

%% Design Full State Feedback Controller, Simulate in Simulink
%open('./Hwk9_PendulumModel_wFullStateFeedbackController.slx') % don't need this every time

% set new IC near set point:
theta_setPoint = 180 *pi/180  ;      % desired position: set point linearizing about
theta_0 = 175 * [pi/180];            % initial pendulum position... Do not change this for Full State Feedback Section

% set desired command trajectory:
input = ...         % desired input = [ time_samples; command_values ]
    [ t_in'   ones(length(t_in),1) *  theta_setPoint]; 

% get target poles: 
OS = 10;    % % overshoot
Ts = 0.5;   % settling time

% FILL IN Target Characteristics
z = -log(OS/100) / sqrt( pi()^2 + log(OS/100)^2 ); % damping ratio
wn = 4 / (z*Ts);                                % natural freq.
den =[1 2*z*wn wn^2] ;                         % desired tf denominator
desiredPoles = roots(den);                      % want to place dominant poles here

% linearized system CREATE THESE MATRICES
A = [0 1; -m*g*L*cos(theta_setPoint)/2/J -D_friction/J];
B = [0 ; 1/J];
C = [1 0];
D = [0];
syms s;

% test observability 
Om = obsv(A,C);
if( det(Om) ~= 0 )
    disp('System is observable...');
else
    warning('System is NOT observable.');
end      

wn_new = 10*wn; % a natural frequency 10 times 
% place observer poles
% place observer poles 
denOb =[1 2*z*wn_new wn_new^2];  % desired tf denominator
desiredTwoObserverPoles = roots(denOb); % want to place dominant poles

desiredObserverPoles = desiredTwoObserverPoles;
LL = place( A', C', desiredObserverPoles)';  % don't forget transposes!

% The whole enchilada.  
% Because of separation principle, we can place poles of observer
% independently from poles of closed loop system 
Ai = [A 0*B; -C 0]; Bi = [B;0];  Ci = [C 0];
Kall = place(Ai, Bi, [desiredPoles' -100]); % added the extra pole a little
    % further from the one at -40.  Double poles don't work well with
    % place.  For that, use acker() instead.  It implements Ackerman's
    % formula for pole placement.

%  For full system, add an integrator to Fig 12.23 just like Fig 12.21
%  Now read off the new A matrix before and after adding the feedback gains
%  Note: the gains K that went to states x in FSF go to x_hat just like
%  they did in the observer.  The gain ki goes to the new state xi you add.
%  This will be combined into Kall = [K ki]
K = Kall(1:(end-1)); 
Ki = -Kall(end);  % <---- NOTE: this sign is flipped due to neg. feedback

% observer with integrator, closed loop response:
A_ = [A  -B*K B*Ki;  LL*C  A-LL*C-B*K B*Ki ;  -C  0*C  0];  
B_ = [ B*0; B*0; 1];
C_ = [C C*0 0]; % using C*0 to get block of 0's the same size as C, etc.
T_ = ss(A_, B_, C_, 0);

% you can confirm observer poles were placed correctly with eig(A-L*C)
% and that the full system closed loop poles are just the combo of desired 
% poles and observer poles via eig(A_OBS)                   
                
stepinfo( T_ )  % OR just do this for dynamic info
E = (1/s)*[1-C_*[(s*eye(length(A_))-A_)^-1]*B_];  % for ss error.
ssError = limit( s*E, s, 0);
double( ssError )  % display as a double precision float value (decimal)

% expected SS error in linear system
Ess = 1 + C_* A_^-1*B_; % Linear System Steady state error to step input

% see what a pure linear system would do if we control it with these gains:
controlledLinSystem = ss( A_, B_, C_, 0);
figure(3)
subplot(2,1,1)
uu = input(:,2);
tt = input(:,1);
[yy,t] = lsim(controlledLinSystem,uu,tt); % Simulate linear system response
plot(tt,uu,'k-','LineWidth',2); % Reference Input
hold on
plot(t,yy,'b-','LineWidth',2); % Error Coordinate Output
ylim([0 4]);
plot(t, yy+pi, 'gx','MarkerSize',1); % Output in Real World Coordinates
legend('Reference Input','Error Output','Pendulum Position');

hold on;
% Check STEP response to confirm desired response is achieved in linear
% system
subplot(2,1,2)
step(controlledLinSystem); % use this to confirm settling time & %OS are OK. 
title('Linearized System Step Response')

%% BUT! our system is not linear;  It is nonlinear.  Let's see what 
% REALLY happens
% now simulate the system WITH control; should start at a different IC
sim('./Hwk9_PendulumModel_wFullStateFeedbackController.slx')                                

% Plot response
figure(4);clf
subplot(2,1,1)
plot(t, command*180/pi , 'k-',t,y*180/pi, 'b-')
legend({ 'Desired Position [deg]', 'Pendulum Position [deg]'})
ylabel('Input & Closed Loop Output')
xlabel('time [sec]')
title('\bfControlled Pendulum - FSF')

subplot(2,1,2)
plot(t,states(:,1)*180/pi(), 'b-', t,states(:,2)*180/pi(), 'g-' , t, plantInput, 'r-' )
legend({ '$\theta $ [deg]','$\dot{\theta} $ [deg]', 'plantInput,u [Nm]'},'interpreter','latex')
xlabel('time [sec]')
ylabel('states & u')

%% Note: Settling time in ACTUAL system is > spec.; Target faster closed-loop pole locations
%% Note: Steady state error to step input is nonzero, add integrator
%% TO DO: copy the "Hwk9_PendulumModel_wFullStateFeedbackController.slx"
%         and save it as "Hwk9_PendulumModel_wFSFwInt.slx" to indicate
%         Full State Feedback plus an integrator.   Add this integrator
%         and repeat the plots above after placing all closed loop poles.