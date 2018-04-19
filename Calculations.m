%%Gripper output



%Global Variables
radians=pi/180;
r_closed=3.213;
r_open=2.779;
r_half=3.210;
F_in=150;
gamma=129.1;
M=linspace(82,44)



%Slider Givens
a_s=0.904;
e_s=0.474;
h_s=0.787;
r_s=a_s;
theta_min_max=[82.00 44.29];
theta_open_s=82.00;
theta_closed_s=44.29;
theta_s=82.00;

%Slider Equations
s_open=(2*a_s*cos(theta_open_s*radians)+(4*a_s^2*cos(theta_open_s*radians)^2-4*(-2*a_s*e_s*sin(theta_open_s*radians)-h_s^2+a_s^2+e_s^2))^(1/2))/2;
MA_open_s=r_s*s_open/(a_s*s_open*sin(theta_open_s*radians)-a_s*e_s*cos(theta_open_s*radians));
s_closed=(2*a_s*cos(theta_closed_s*radians)+(4*a_s^2*cos(theta_closed_s*radians)^2-4*(-2*a_s*e_s*sin(theta_closed_s*radians)-h_s^2+a_s^2+e_s^2))^(1/2))/2;
MA_closed_s=r_s*s_closed/(a_s*s_closed*sin(theta_closed_s*radians)-a_s*e_s*cos(theta_closed_s*radians));

F_s_open=F_in/MA_open_s;
F_s_closed=F_in/MA_closed_s;




%Four Bar Givens:
a_4=2.505;
b_4=2.067;
h_4=0.819;
cx_4=-0.591;
cy_4=0.750;
theta_4=theta_s+gamma;
theta_open_4=theta_open_s+gamma;
theta_closed_4=theta_closed_s+gamma;

% Constraint Equation (psi solution)
Amin=2*cx_4*b_4-2*b_4*a_4*cos(theta_open_4*radians);
Bmin=2*cy_4*b_4-2*b_4*a_4*sin(theta_open_4*radians);
Cmin=cx_4^2+cy_4^2+b_4^2+a_4^2-h_4^2-2*cx_4*a_4*cos(theta_open_4*radians)-2*cy_4*a_4*sin(theta_open_4*radians);
deltamin=atan2(Bmin,Amin);
psiopen=deltamin-acos((-Cmin)/(Amin^2+Bmin^2)^(1/2));
psiopen=psiopen/radians;

Amax=2*cx_4*b_4-2*b_4*a_4*cos(theta_closed_4*radians);
Bmax=2*cy_4*b_4-2*b_4*a_4*sin(theta_closed_4*radians);
Cmax=cx_4^2+cy_4^2+b_4^2+a_4^2-h_4^2-2*cx_4*a_4*cos(theta_closed_4*radians)-2*cy_4*a_4*sin(theta_closed_4*radians);
deltamax=atan2(Bmax,Amax);
psiclosed=deltamax-acos((-Cmax)/(Amax^2+Bmax^2)^(1/2));
psiclosed=psiclosed/radians;

% Mechanical Advantage
MA_open_4=(b_4*a_4*sin(psiopen*radians-theta_open_4*radians)-b_4*cx_4*sin(psiopen*radians)+b_4*cy_4*cos(psiopen*radians))/(b_4*a_4*sin(-psiopen*radians+theta_open_4*radians)+a_4*cx_4*sin(theta_open_4*radians)-a_4*cy_4*cos(theta_open_4*radians));
MA_closed_4=(b_4*a_4*sin(psiclosed*radians-theta_closed_4*radians)-b_4*cx_4*sin(psiclosed*radians)+b_4*cy_4*cos(psiclosed*radians))/(b_4*a_4*sin(-psiclosed*radians+theta_closed_4*radians)+a_4*cx_4*sin(theta_closed_4*radians)-a_4*cy_4*cos(theta_closed_4*radians));
MA_open_4=abs(MA_open_4)
MA_closed_4=abs(MA_closed_4)




%Output Force
F_out_open=(F_in)*(a_s/b_4)*(MA_open_4/MA_open_s)*(b_4/r_open)
F_out_closed=(F_in)*(a_s/b_4)*(MA_closed_4/MA_closed_s)*(b_4/r_closed)
F_s_open;
F_s_closed;
F4_open=F_out_open/MA_open_4;
F4_closed=F_out_closed/MA_closed_4;


%%