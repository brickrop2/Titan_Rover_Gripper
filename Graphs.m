%%Gripper output



%Global Variables
radians=pi./180;
F_in=150; %input force




%Slider 

%Givens
a_s=0.904;
e_s=0.474;
h_s=0.787;
r_s=a_s;

%From open to closed
theta_min_max=[82.00 44.29];
theta_s=linspace(82,44.29);

%Modified Slider 
%a_s=1.3;
%e_s=.838;
%theta_min_max=[83.25 44];
%theta_s=linspace(83.25,44);

%Slider Equations
s=(2.*a_s.*cos(theta_s.*radians)+(4.*a_s.^2.*cos(theta_s.*radians).^2-4.*(-2.*a_s.*e_s.*sin(theta_s.*radians)-h_s.^2+a_s.^2+e_s.^2)).^(1./2))./2;
MA_s=(r_s).*s./(a_s.*s.*sin(theta_s.*radians)-a_s.*e_s.*cos(theta_s.*radians));
F_s=F_in./MA_s;




%Four Bar 

%Givens:
a_4=2.505;
b_4=2.067;
h_4=0.819;
cx_4=-0.591;
cy_4=0.750;
gamma=129.1; %geometry value
theta_4=theta_s+gamma; %ideal parallel grip: theta_4 = psi_4

%Modified Givens:
%a_4=2.375;     %with a 110 degree on the new joint

% Constraint Equation (psi solution)
Amin=2.*cx_4.*b_4-2.*b_4.*a_4.*cos(theta_4.*radians);
Bmin=2.*cy_4.*b_4-2.*b_4.*a_4.*sin(theta_4.*radians);
Cmin=cx_4.^2+cy_4.^2+b_4.^2+a_4.^2-h_4.^2-2.*cx_4.*a_4.*cos(theta_4.*radians)-2.*cy_4.*a_4.*sin(theta_4.*radians);
delta=atan2(Bmin,Amin);
psi_4=delta+acos((-Cmin)./(Amin.^2+Bmin.^2).^(1./2));
psi_4=psi_4./radians;
%guess=linspace(217.6,170);
%psi=guess;

% Constraint Equation (phi solution)
phi_4=atan2((cy_4+b_4.*sin(psi_4.*radians)-a_4.*sin(theta_4.*radians)),(cx_4+b_4.*cos(psi_4.*radians)-a_4.*cos(theta_4.*radians)));
phi_4=phi_4./radians; %ideal parallel grip: phi_4 = const

% Mechanical Advantage
MA_4=(b_4.*a_4.*sin(psi_4.*radians-theta_4.*radians)-b_4.*cx_4.*sin(psi_4.*radians)+b_4.*cy_4.*cos(psi_4.*radians))./(b_4.*a_4.*sin(-psi_4.*radians+theta_4.*radians)+a_4.*cx_4.*sin(theta_4.*radians)-a_4.*cy_4.*cos(theta_4.*radians));
MA_4=abs(MA_4);




%Grip piece 

%Givens
mp=.748; %distance from midpoint to second fixed pivot
d_x=1.173; %x-dim of grip center to coupler revolute
d_y=.394; %y-dim of grip center to coupler revolute
d_angle=100; %angle of coupler to grip piece

%Grip piece distance:
d_h=d_y.*cos((d_angle-phi_4).*radians)+d_x.*sin((d_angle-phi_4).*radians); 
d_gripper=(mp+b_4.*-sin(psi_4.*radians)-d_h)*2;
%d_guess=(.750-.433+b_4.*-sin((psi_4).*radians))*2
%check_parallel=d_angle-phi_4; %parallel grip when equal to zero



%Linear actuator distance:
%Original
d_actuator=s-min(s); %sets the initial to zero

%Modified actuator distance (slider):
%d_actuator=s-min(s); %sets the initial to zero



%Output Force
r=(d_x.*cos((100-phi_4).*radians)+b_4.*-cos((psi_4).*radians)); 
%r_guess=3.2
F_out=(F_in).*(a_s./r).*(MA_4./MA_s);
maxTorque_Fs=max(F_s.*a_s./2)
disp('unit: in-lbs')
maxForce_Fs=max(F_s./2)
disp('unit: lbs')
%maxForce_F4=max(F_s.*a_s./a_4./2)
%disp('unit: lbs')



%Graphs
figure
subplot(2,1,1)
plot(d_actuator,F_out,'.')
xlabel('Linear Actuator Distance, in')
ylabel('Gripper Force, lbs')
title('Output force vs Input distance')

subplot(2,1,2)

d_gripper=abs(d_gripper-max(d_gripper)); %sets the initial to zero with flipped axis
%d_gripper=fliplr(d_gripper); %Flip array of x values
plot(d_gripper,F_out,'.')
xlim([0 3])
%xlim([0 max(d_gripper)])
XLimits = get(gca,'XLim');  %# Get the x axis limits
XTicks = XLimits(2)-get(gca,'XTick');  %# Get the x axis tick values and
                                       %#   subtract them from the upper limit
set(gca,'XTickLabel',num2str(XTicks.'));  %'# Convert the tick values to strings
                                           %#   and update the x axis labels

xlabel('Gripper Distance, in')
ylabel('Gripper Force, lbs')
title('Output force vs Output distance')
%%