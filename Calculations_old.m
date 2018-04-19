%%Gripper output

%Min values are fully open gripper
%Max values are fully closed gripper

%Global Variables
radians=pi/180;

%Slider Givens
a_s=.8886;
e_s=1.310;
h_s=.7874;
r_s=a_s;
thetamin_s=[42.85 42.85];
thetamin_s=78.33;
thetamax_s=42.85;
F_in=150;

%Slider Equations
s_min=(2*a_s*cos(thetamin_s*radians)+(4*a_s^2*cos(thetamin_s*radians)^2-4*(-2*a_s*e_s*sin(thetamin_s*radians)-h_s^2+a_s^2+e_s^2))^(1/2))/2;
MA_min_s=r_s*s_min/(a_s*s_min*sin(thetamin_s*radians)-a_s*e_s*cos(thetamin_s*radians));
s_max=(2*a_s*cos(thetamax_s*radians)+(4*a_s^2*cos(thetamax_s*radians)^2-4*(-2*a_s*e_s*sin(thetamax_s*radians)-h_s^2+a_s^2+e_s^2))^(1/2))/2;
MA_max_s=r_s*s_max/(a_s*s_max*sin(thetamax_s*radians)-a_s*e_s*cos(thetamax_s*radians));

F_s_min=F_in/MA_min_s;
F_s_max=F_in/MA_max_s;


%Four Bar Givens:
a_4=1.260;
b_4=2.094;
h_4=2.820;
cx_4=-.5;
cy_4=1.012;
theta_4=[175 200];
thetamin_4 = 175;
thetamax_4 = 200;

% Constraint Equation (psi solution)
Amin=2*cx_4*b_4-2*b_4*a_4*cos(thetamin_4*radians);
Bmin=2*cy_4*b_4-2*b_4*a_4*sin(thetamin_4*radians);
Cmin=cx_4^2+cy_4^2+b_4^2+a_4^2-h_4^2-2*cx_4*a_4*cos(thetamin_4*radians)-2*cy_4*a_4*sin(thetamin_4*radians);
deltamin=atan2(Bmin,Amin);
psimin=deltamin-acos((-Cmin)/(Amin^2+Bmin^2)^(1/2));
psimin=psimin/radians;

Amax=2*cx_4*b_4-2*b_4*a_4*cos(thetamax_4*radians);
Bmax=2*cy_4*b_4-2*b_4*a_4*sin(thetamax_4*radians);
Cmax=cx_4^2+cy_4^2+b_4^2+a_4^2-h_4^2-2*cx_4*a_4*cos(thetamax_4*radians)-2*cy_4*a_4*sin(thetamax_4*radians);
deltamax=atan2(Bmax,Amax);
psimax=deltamax-acos((-Cmax)/(Amax^2+Bmax^2)^(1/2));
psimax=psimax/radians;

% Mechanical Advantage
MA_min_4=(b_4*a_4*sin(psimin*radians-thetamin_4*radians)-b_4*cx_4*sin(psimin*radians)+b_4*cy_4*cos(psimin*radians))/(b_4*a_4*sin(-psimin*radians+thetamin_4*radians)+a_4*cx_4*sin(thetamin_4*radians)-a_4*cy_4*cos(thetamin_4*radians));
MA_max_4=(b_4*a_4*sin(psimax*radians-thetamax_4*radians)-b_4*cx_4*sin(psimax*radians)+b_4*cy_4*cos(psimax*radians))/(b_4*a_4*sin(-psimax*radians+thetamax_4*radians)+a_4*cx_4*sin(thetamax_4*radians)-a_4*cy_4*cos(thetamax_4*radians));


%Output Force
F_out_min=(F_in/2)*(a_s/a_4)*(MA_min_4/MA_min_s)*(2/3.5)*(1/b_4)
F_out_max=(F_in/2)*(a_s/a_4)*(MA_max_4/MA_max_s)*(2/3.5)*(1/b_4)
F_s_min;
F_s_max;
F4_min=F_out_min/MA_min_4;
F4_max=F_out_max/MA_max_4;


%%