%% ME 5281 Fall 2020: Final Exam; Surgical Robot Grasping nonlinear tissue
% be sure to download Simulink Files and place them in same folder
% as this file (e.g. robotGrasper.slx)

%% Define model and parameters:
% The following are all used by the simulink model and robot subsystem:
syms stress strain strain_d strain_dd m d alpha beta gamma c1 c2 c3
stress = m*strain_dd + d*strain_d + c1*strain^4 + c2*strain^3 + c3*strain^2;
f = matlabFunction(stress);  % creates a callable matlab function f

%% might as well do linearization here...

%% DO NOT CHANGE ANY OF THESE. 
stress_MAX  = 150;     % kPa,  Damage occurs past this point
m           = .155;    % kg, lumped tissue mass/ and mechanism inertia
d           = 3.1;     % kg/s, lumped mechanism friction and tissue viscosity
c1          = 9982;    % nonlinear tissue stiffness (kPa)
c2          =-2871;    % ...
c3          = 419.3;   % ...
a           = 56.3e-6; % m^2, surface area of tool-tissue contact

i_MAX       = 10;      % A, maximum current achievable by motor driver
KtKm        = 1e-3*11.3; % kN/A, Force-out/current-in transfer function (gain)

strain_MAX  = .407;     % damage occurs past this point, never reach it!
                        % found via: vpa(f(c1,c2,c3,d,m,strain_MAX,strain_d*0,strain_dd*0))
stress_setPoint = stress_MAX*2/3;  % our 33% safety factor grasping stress
strain_setPoint = .371; % typical grasping stress target
                        % found via vpa(f(c1,c2,c3,d,m,strain_MAX*2/3,strain_d*0,strain_dd*0))
i_setPoint  = stress_setPoint / (KtKm/a);   % current predicted by the
                        % model that's required to keep grasper at setPoint.
currentInput = (c1*strain_setPoint^4+c2*strain_setPoint^3+c3*strain_setPoint^2)/(KtKm/a);
Ts          = 0.100;    % sec, target settling time 

%% OK TO CHANGE STUFF BELOW...
% You need to understand all the code from this point on, Change what you like ...
setPoint = strain_setPoint; % our targeted strain; linearize about this
Tsample = 0.001;         % sample period in sec
tend = 0.5;              % simulation end time in sec
t_in = 0:Tsample:tend;   % time input vector for simulink (simulink spits out t)
targetVal  = setPoint;   % commanded target (in you target other than setPoint)

% generate a step input for response
input = ...         % desired input = [ time_samples; command_values ]
    [ t_in'  ones(length(t_in),1)];
input(1:10, 2) = 0; % set 1st 10 samples zero to get step starting 10 samples in
                    % just to see how things look before the step
                    
% Simulink gives the following each time you run it via sim():
% t -- actual time in simulation world
% y -- actual strain output at time t
% states -- all internal states [ x_dd x_d x] at times t where x is strain
% plantInput -- the saturation-limited current actually fed into the plant

% Simulate Plant in Simulink
sim('./robotGrasper.slx')
open('./robotGrasper.slx') % don't need this command, just opens file

% Plot response with  initial conditions
figure(1);clf
subplot(2,1,1);plot(t,states(:,1), 'k-');hold on
line([0 tend], [1 1]*strain_MAX, 'color','r', 'linestyle', '--');
line([0 tend], [1 1]*strain_setPoint, 'color','g', 'linestyle', '--');
legend('Actual Strain', 'Max Strain (damage threshold)',...
    'Target Strain (setpoint)', 'location', 'southeast')
ylabel('x1 strain')
title('\bfThe Plant Natural Response to a Step Input')
subplot(2,1,2);plot(t,states(:,2), 'm-' );
xlabel('Time (sec)')
ylabel('x2 strain-dot')


figure(2);clf
subplot(3,1,1)
plot(t, plantInput , 'b-' );
xlabel(' ')
ylabel('i [A]')
title('\bf Actual Plant Input Current  '); % after the saturation block
subplot(3,1,2:3)
plot(t, stressApplied , 'b.-' );hold on
xlabel('Time (sec)')
ylabel('stress [kPa]')
title('\bf Stress Applied to Tissue  '); 
line([0 tend], [1 1]*stress_MAX, 'color','r', 'linestyle', '--');
line([0 tend], [1 1]*stress_setPoint, 'color', 'g', 'linestyle', '--');
legend('Actual Applied Stress', 'Max Stress (damage threshold)',...
    'Target Stress (setpoint)', 'location', 'southeast')

% return settling time and other info (useful!)
lsiminfo(y,t)



%%  TO DO: design your controller here:  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%  TO DO:  Simulate Plant in Simulink (copy robotGrasper.slx to
%           robotGrasperFSFwInt.slx and implement your controller there).

% create a new step input, in units of what we want (output strain)
input = ...         % desired input = [ time_samples; command_values ]
    [ t_in'  targetVal * ones(length(t_in),1)];
input(1:10, 2) = 0; % set 1st 10 samples zero to get step starting 10 samples in
                    % just to see how things look before the step

A = [0 1; -1/m*(4*c1*strain_setPoint^3+3*c2*strain_setPoint^2+2*c3*strain_setPoint) -d/m];
B = [0 ; KtKm/m/a];
C = [1 0];
D = [0];

% Test Controllability...
Cm = ctrb(A,B);
if( det(Cm) ~= 0 )
    disp('System is controllable.');
else
    warning('System is NOT controllable.');
end

% Test observability 
Om = obsv(A,C);
if( det(Om) ~= 0 )
    disp('System is observable.');
else
    warning('System is NOT observable.');
end 

[NG,DG] = ss2tf(A,B,C,D);
G = tf(NG, DG);

% get target poles: 
Ts = 0.1;   % settling time/sec

% FILL IN Target Characteristics
z = 1;                                          % critically damped with zero overshhot
wn = 4 / (z*Ts);                                % natural freq.
den = [1 2*z*wn wn^2];        % target characteristic equation when placing system poles
desiredPoles = roots(den);                      % want to place dominant poles here

% add an integrator based on feedback control system matrices ABCD
%Eq. 12.113a from Nise
Ai = [A B*0; -C 0];    
Bi = [B ; 0];          

% find desired gains K for full-state feedback with integrator
Ki = acker(Ai, Bi, [desiredPoles' -400]);  % same desired parameter zeta & wn thus same desired poles
Ke = -Ki(3);

K_FSFwInt = [Ki(1) Ki(2)];
%Eq. 12.115a from Nise
A_FSFwInt = [A-B*K_new, B*Ke; -C, 0];  
B_FSFwInt = [0*B; 1];
%Eq. 12.115b from Nise
C_FSFwInt = [C 0];          
D_FSFwInt = 0;

% Observers
wn_new = 10*wn; % a natural frequency 10 times  
denOb =[1 2*z*wn_new wn_new^2];  % target characteristic equation when placing obs. poles
desiredObsPoles = roots(denOb); % want to place dominant poles
L = acker( A', C', desiredObsPoles)';  % observer gains L

%closed loop system WITH observer (encode Gigure 12.23 into state space
A_FSFwIntwObs = [A  -B*K_FSFwInt B*Ke;  L*C  A-L*C-B*K_FSFwInt B*Ke ;  -C  0*C  0];  
B_FSFwIntwOb = [B*0; B*0; 1];
C_FSFwIntwOb = [C C*0 0]; 
T_FSFwIntwOb = ss(A_FSFwIntwOb, B_FSFwIntwOb, C_FSFwIntwOb, 0);


% Eq 3.73
syms s;
G_I2E = vpa(simplify(C*(s*eye(length(C))-A)^-1*B+D));
NG_ = 1423764555534154.75; DG_ = [1099511627776 21990232555520 8260768763833007];
G_ = tf(NG_/DG_(1), DG_/DG_(1));

% 
% %% uncoment this below to to run the simulation...
% sim('./robotGrasper_FSFwInt.slx')  % assumes you put controller in this file
% open('./robotGrasper_FSFwInt.slx')
% 
% % Plot response FULL STATE FEEDBACK
% figure(11);clf
% subplot(2,1,1);
% plot(t,states(:,1), 'k-');hold on
% line([0 tend], [1 1]*strain_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*strain_setPoint, 'color','g', 'linestyle', '--');
% legend('Actual Strain', 'Max Strain (damage threshold)',...
%     'Target Strain (setpoint)', 'location', 'southeast')
% ylabel('x1 strain')
% title('\bfFull State Feedback Control Results')
% subplot(2,1,2);plot(t,states(:,2), 'm-' );
% xlabel('Time (sec)')
% ylabel('x2 strain-dot')
% 
% figure(12);clf
% subplot(3,1,1)
% plot(t, plantInput , 'b-' );
% xlabel(' ')
% ylabel('i [A]')
% title('\bf Actual Plant Input Current (Full State Feedback)  '); % after the saturation block
% subplot(3,1,2:3)
% plot(t, stressApplied , 'b.-' );hold on
% xlabel('Time (sec)')
% ylabel('stress [kPa]')
% title('\bf Stress Applied to Tissue  (Full State Feedback) '); 
% line([0 tend], [1 1]*stress_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*stress_setPoint, 'color', 'g', 'linestyle', '--');
% legend('Actual Applied Stress', 'Max Stress (damage threshold)',...
%     'Target Stress (setpoint)', 'location', 'southeast')
% 
% % return settling time and other info
% disp('Results of full state feedback:')
% lsiminfo(y,t,targetVal)
% 
% 
% 
% %% TO DO:  Implement Observer + Integrator (copy robotGrasperFSFwInt.slx to
% %    robotGrasper_ObserverInt.slx to implement your observer w/ integrator).
% 
% %% uncoment this below to to run the simulation...
% sim('./robotGrasper_ObserverInt.slx')  % assumes you put controller in this file
% open('./robotGrasper_ObserverInt.slx')
% 
% 
% % Plot response OBSERVER
% figure(21);clf
% subplot(2,1,1);plot(t,states(:,1), 'k-');hold on
% line([0 tend], [1 1]*strain_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*strain_setPoint, 'color','g', 'linestyle', '--');
% legend('Actual Strain', 'Max Strain (damage threshold)',...
%     'Target Strain (setpoint)', 'location', 'southeast')
% ylabel('x1 strain')
% title('\bfObserver Control Results')
% subplot(2,1,2);plot(t,states(:,2), 'm-' );
% xlabel('Time (sec)')
% ylabel('x2 strain-dot')
% 
% figure(22);clf
% subplot(3,1,1)
% plot(t, plantInput , 'b-' );
% xlabel(' ')
% ylabel('i [A]')
% title('\bf Actual Plant Input Current (Observer)  '); % after the saturation block
% subplot(3,1,2:3)
% plot(t, stressApplied , 'b.-' );hold on
% xlabel('Time (sec)')
% ylabel('stress [kPa]')
% title('\bf Stress Applied to Tissue   '); 
% line([0 tend], [1 1]*stress_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*stress_setPoint, 'color', 'g', 'linestyle', '--');
% legend('Actual Applied Stress', 'Max Stress (damage threshold)',...
%     'Target Stress (setpoint)', 'location', 'southeast')
% 
% % return settling time and other info
% disp('Results of Observer:')
% lsiminfo(y,t,targetVal)
% 
% 
% %% TO DO: CLASSICAL CONTROL: Come up with Transfer Function &  Compensator
% %  Called PID colloquially....
% 
% % s = tf('s');
% % S = C*(s*eye(length(A)) - A)^-1 * B
% % design controller (eg in SisoTool)
% % sisotool(S)
% 
% % Note: you can export from sisotool to workspace (saves time),
% %   e.g. Gc  = C (from sisotool).
% 
% % replace this with YOUR controller from sisotool (hint export to Gc directly
% % but manually write it in below so your simulink simulation can load in 
% % Gc every time you run this script (no need to run sisotool every time):
% Gc = 10*(s+1)/(s+10) ;  
% 
% 
% 
% % Note: you can grab an ltiblock in simulink (under control system toolbox 
% % and call it Gc.  This will implement your compensator directly into Simulink.
% sim('./robotGrasper_PID.slx')
% open('./robotGrasper_PID.slx') % don't need this every time
% 
% 
% 
% % Plot response for CLASSICAL (FREQUENCY DOMAIN) CONTROL (PID)
% % Plot response 
% figure(31);clf
% subplot(2,1,1);plot(t,states(:,1), 'k-');hold on
% line([0 tend], [1 1]*strain_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*strain_setPoint, 'color','g', 'linestyle', '--');
% legend('Actual Strain', 'Max Strain (damage threshold)',...
%     'Target Strain (setpoint)', 'location', 'southeast')
% ylabel('x1 strain')
% title('\bfClassical Control Results')
% subplot(2,1,2);plot(t,states(:,2), 'm-' );
% xlabel('Time (sec)')
% ylabel('x2 strain-dot')
% 
% figure(32);clf
% subplot(3,1,1)
% plot(t, plantInput , 'b-' );
% xlabel(' ')
% ylabel('i [A]')
% title('\bf Actual Plant Input Current (Classical Control)  '); % after the saturation block
% subplot(3,1,2:3)
% plot(t, stressApplied , 'b.-' );hold on
% xlabel('Time (sec)')
% ylabel('stress [kPa]')
% title('\bf Stress Applied to Tissue   '); 
% line([0 tend], [1 1]*stress_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*stress_setPoint, 'color', 'g', 'linestyle', '--');
% legend('Actual Applied Stress', 'Max Stress (damage threshold)',...
%     'Target Stress (setpoint)', 'location', 'southeast')
% 
% 
% % return settling time and other info
% disp('Results for CLASSICAL CONTROL:')
% lsiminfo(y,t,targetVal)
% 
% 
% 
% 
% %% TO DO: Evaluate the effect of substantial, realistic disturbances
% % CHOOSE your controller below: (comment out what you DON'T want to use)
% %mySys = './robotGrasper_FSFwInt.slx';  sysName = 'Full State Feedback';
% %mySys = './robotGrasper_ObserverInt.slx';  sysName = 'Observer';
% %mySys = './robotGrasper_PID.slx';  sysName = 'PID ';
% 
% % preturb the system:
% aOriginal = a; % save it
% a = a*0.6;     % reduce by 40%
% sim(mySys);  % simulate 
% a = aOriginal; % restore value 
% 
% % Plot response for 40% Disturbance in robot plant ( decrease a by 40%)
% figure(51);clf
% subplot(2,1,1);plot(t,states(:,1), 'k-');hold on
% line([0 tend], [1 1]*strain_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*strain_setPoint, 'color','g', 'linestyle', '--');
% legend('Actual Strain', 'Max Strain (damage threshold)',...
%     'Target Strain (setpoint)', 'location', 'southeast')
% ylabel('x1 strain')
% title(['\bf 40% Robot Disurbance Response - ' sysName])
% subplot(2,1,2);plot(t,states(:,2), 'm-' );
% xlabel('Time (sec)')
% ylabel('x2 strain-dot')
% 
% figure(52);clf
% subplot(3,1,1)
% plot(t, plantInput , 'b-' );
% xlabel(' ')
% ylabel('i [A]')
% title(['\bf 40% Robot Disurbance Response - ' sysName]); % after the saturation block
% subplot(3,1,2:3)
% plot(t, stressApplied , 'b.-' );hold on
% xlabel('Time (sec)')
% ylabel('stress [kPa]')
% title('\bf Stress Applied to Tissue  '); 
% line([0 tend], [1 1]*stress_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*stress_setPoint, 'color', 'g', 'linestyle', '--');
% legend('Actual Applied Stress', 'Max Stress (damage threshold)',...
%     'Target Stress (setpoint)', 'location', 'southeast')
% 
% % return settling time and other info
% disp('Results for 40% robot plant disurbance:')
% lsiminfo(y,t,targetVal)
% 
% 
% 
% %% TO DO: Preturb the system (nonlinear tissue significantly different):
% c1          = 9982*1.5;    % nonlinear tissue stiffness (kPa)
% c2          =-2871*1.5;    % ...
% c3          = 419.3*1.5;   % ...
% sim(mySys);  % simulate 
% 
% 
% % Plot response for TISSUE disturbance ( decrease a by 40%)
% figure(61);clf
% subplot(2,1,1);plot(t,states(:,1), 'k-');hold on
% line([0 tend], [1 1]*strain_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*strain_setPoint, 'color','g', 'linestyle', '--');
% legend('Actual Strain', 'Max Strain (damage threshold)',...
%     'Target Strain (setpoint)', 'location', 'southeast')
% ylabel('x1 strain')
% title(['\bf Tissue Disurbance Response - ' sysName])
% subplot(2,1,2);plot(t,states(:,2), 'm-' );
% xlabel('Time (sec)')
% ylabel('x2 strain-dot')
% 
% figure(62);clf
% subplot(3,1,1)
% plot(t, plantInput , 'b-' );
% xlabel(' ')
% ylabel('i [A]')
% title(['\bf Tissue Disurbance Response - ' sysName]); % after the saturation block
% subplot(3,1,2:3)
% plot(t, stressApplied , 'b.-' );hold on
% xlabel('Time (sec)')
% ylabel('stress [kPa]')
% title('\bf Stress Applied to Tissue  (Full State Feedback) '); 
% line([0 tend], [1 1]*stress_MAX, 'color','r', 'linestyle', '--');
% line([0 tend], [1 1]*stress_setPoint, 'color', 'g', 'linestyle', '--');
% legend('Actual Applied Stress', 'Max Stress (damage threshold)',...
%     'Target Stress (setpoint)', 'location', 'southeast')
% 
% % return settling time and other info
% disp('Results for TISSUE disurbance:')
% lsiminfo(y,t,targetVal)
