%%Slider Crank

%%Givens
a=.8886;
e=1.310;
h=.7874;
r=a;
theta=[78.33 42.85]
theta=78.33;
F_out=15
radians=pi/180;

%%Equations
s=(2*a*cos(theta*radians)+(4*a^2*cos(theta*radians)^2-4*(-2*a*e*sin(theta*radians)-h^2+a^2+e^2))^(1/2))/2
phi=atan(e-a*sin(theta*radians)/(s-a*cos(theta*radians)));
phi=phi/radians;
MA=r*s/(a*s*sin(theta*radians)-a*e*cos(theta*radians));
F_in=F_out/MA




%%
