
%% Four Bar Linkage 

% Givens:
a=1.260;
b=2.094;
h=2.820;
cx=1;
cy=1.012;
theta=[141.12 103.6]
thetamin = 141.12;
thetamax = 103.6;
radians = pi/180;
thetadot=1;

% Constraint Equation
Amin=2*cx*b-2*b*a*cos(thetamin*radians);
Bmin=2*cy*b-2*b*a*sin(thetamin*radians);
Cmin=cx^2+cy^2+b^2+a^2-h^2-2*cx*a*cos(thetamin*radians)-2*cy*a*sin(thetamin*radians);
deltamin=atan2(Bmin,Amin);
psimin=deltamin-acos((-Cmin)/(Amin^2+Bmin^2)^(1/2));
psimin=psimin/radians

Amax=2*cx*b-2*b*a*cos(thetamax*radians);
Bmax=2*cy*b-2*b*a*sin(thetamax*radians);
Cmax=cx^2+cy^2+b^2+a^2-h^2-2*cx*a*cos(thetamax*radians)-2*cy*a*sin(thetamax*radians);
deltamax=atan2(Bmax,Amax);
psimax=deltamax-acos((-Cmax)/(Amax^2+Bmax^2)^(1/2));
psimax=psimax/radians

% Position Loop Equation
phimin=atan2(cy-a*sin(thetamin*radians)+b*sin(psimin*radians),cx-a*cos(thetamin*radians)+b*cos(psimin*radians));
phimin=phimin/radians
phimax=atan2(cy-a*sin(thetamax*radians)+b*sin(psimax*radians),cx-a*cos(thetamax*radians)+b*cos(psimax*radians));
phimax=phimax/radians

% Velocity Loop Equation
phidot_min=(a*thetadot*sin(-psimin*radians+thetamin*radians))/(h*sin(psimin*radians-phimin*radians));
phidot_min=phidot_min/radians
phidot_max=(a*thetadot*sin(-psimax*radians+thetamax*radians))/(h*sin(psimax*radians-phimax*radians));
phidot_max=phidot_max/radians

psidot_min=(a*thetadot*sin(-phimin*radians+thetamin*radians))/(b*sin(psimin*radians-phimin*radians));
psidot_min=psidot_min/radians
psidot_max=(a*thetadot*sin(-phimax*radians+thetamax*radians))/(b*sin(psimax*radians-phimax*radians));
psidot_max=psidot_max/radians

% Mechanical Advantage
MA_min=(b*a*sin(psimin*radians-thetamin*radians)-b*cx*sin(psimin*radians)+b*cy*cos(psimin*radians))/(b*a*sin(-psimin*radians+thetamin*radians)+a*cx*sin(thetamin*radians)-a*cy*cos(thetamin*radians))
MA_max=(b*a*sin(psimax*radians-thetamax*radians)-b*cx*sin(psimax*radians)+b*cy*cos(psimax*radians))/(b*a*sin(-psimax*radians+thetamax*radians)+a*cx*sin(thetamax*radians)-a*cy*cos(thetamax*radians))

%
%%