function [Hz] = GHz2Hz(GHz)
% Convert frequency from gigahertz to hertz.
GHz=input('freq in Ghz=')
Hz = GHz*1e+9
Emax=0;
Hmax=0;
C=3*10^8;
E2=1; 
r=1; 
%Dimensions in mm
rhomm= input('enter rho in mm =');
a1mm=input('enter a1 in mm =');
bmm=input('enter bin mm =');
amm=input('enter a in mm =');
%Conversions in m
rho=rhomm/1000
a1=a1mm/1000
b=bmm/1000
a=amm/1000

lamda=C/Hz
k=2*pi/lamda
% E-plane
phi=pi/2;
kx=pi/a1;
kxp=-pi/a1;
f1=kx^2*rho/(2*k);
f2=kxp^2*rho/(2*k);
t1=sqrt(1/(pi*k*rho))*(-k*a1/2-kx*rho);
t1p=sqrt(1/(pi*k*rho))*(-k*a1/2-kxp*rho);
t2=sqrt(1/(pi*k*rho))*(k*a1/2-kx*rho);
t2p=sqrt(1/(pi*k*rho))*(k*a1/2-kxp*rho);
Ct1=mfun('FresnelC',t1);
Ct2=mfun('FresnelC',t2);
Ct1p=mfun('FresnelC',t1p);
Ct2p=mfun('FresnelC',t2p);
St1=mfun('FresnelS',t1);
St2=mfun('FresnelS',t2);
St1p=mfun('FresnelS',t1p);
St2p=mfun('FresnelS',t2p);
a0=((Ct2-Ct1)-1i*(St2-St1));
b0=((Ct2p-Ct1p)-1i*(St2p-St1p));
Ne=90;
i=1:Ne;
Dtheta=360/Ne;
thetas(i)= i*Dtheta;
theta = thetas*pi/180;
Y=(k*b/2)*sin(theta)*sin(phi);
if Y==0
Ys=1;
else
Ys=sin(Y)/Y;
end
Eth=1i*E2*b/8*(sqrt(k*rho/pi))*(exp(-1i*k*r))/r*((sin(phi))*(1+(cos(theta))).*Ys.*(exp(1i*f1).*a0+exp(1i*f2).*b0));
D = abs(Eth);
for i = 1:Ne
if(D(i) > Emax);
Emax = D(i);
end
end
% normalization
D=abs(Eth)/Emax;
Emin=-40;
for i= 1:Ne
if D(i) <= 10^(Emin/20)
Edb(i)=Emin;
else
Edb(i) = 20*log10(D(i));
end
end
x=theta*180/pi;
q1=Edb;
plot(x,q1,'r')
hold on

% H- plane
Nh=180;
m=1:Nh;
Dtheta=360/Nh;
thetas(m)=m*Dtheta;
theta = thetas*pi/180;
phi=0;
kx=k*sin(theta)+pi/a1;
kxp=k*sin(theta)-pi/a1;
f1=kx.^2*rho/(2*k);
f2=kxp.^2*rho/(2*k);
t1=sqrt(1/(pi*k*rho))*(-k*a1/2-kx*rho);
t1p=sqrt(1/(pi*k*rho))*(-k*a1/2-kxp*rho);
t2=sqrt(1/(pi*k*rho))*(k*a1/2-kx*rho);
t2p=sqrt(1/(pi*k*rho))*(k*a1/2-kxp*rho);
Ct1=mfun('FresnelC',t1);
Ct2=mfun('FresnelC',t2);
Ct1p=mfun('FresnelC',t1p);
Ct2p=mfun('FresnelC',t2p);
St1=mfun('FresnelS',t1);
St2=mfun('FresnelS',t2);
St1p=mfun('FresnelS',t1p);
St2p=mfun('FresnelS',t2p);
a0=((Ct2-Ct1)-1i*(St2-St1));
b0=((Ct2p-Ct1p)-1i*(St2p-St1p));

Y=(k*b/2)*sin(theta)*sin(phi);
if Y==0
Ys=1;
else
Ys=sin(Y)./Y;
end
Ephi=1i*E2*b/8*(sqrt(k*rho/pi))*(exp(-1i*k*r))/r*((cos(phi))*(1+(cos(theta))).*Ys.*(exp(1i*f1).*a0+exp(1i*f2).*b0));
D = abs(Ephi);
for m = 1:Nh
if(D(m) > Hmax);
Hmax = D(m);
end
end
% normalization
D=abs(Ephi)/Hmax;
Hmin=-40;
for m= 1:Nh
if D(m) <= 10^(Hmin/20)
Hdb(m)= Hmin;
else
Hdb(m) = 20*log10(D(m));
end
end
e=theta*180/pi;
q2=Hdb;
plot(e,q2,'b--')
ylabel('Field Pattern (dB)');
title('Horn Analysis');
legend('E-Plane','H-Plane');
grid on;
