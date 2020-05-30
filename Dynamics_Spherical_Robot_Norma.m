%	@Author: Saeed Moazami
%
%	Dynamics of Norma, a slider pendulum spherical robot

%	Please cite this paper:
%	https://arxiv.org/pdf/1908.02243
%
%   refer to this video for a full demonstration:
%   https://youtu.be/WO22QfuMlqs
%	Don't hesitate to contact:
%	moazami.iut@gmail.com
%
%	for any information

clear
clc

%Forces and torques (This should be the output of your controller,
%It's for the purpose of demonstration)
t1 = 0;
t2 = 0.2;             %Torque applied to the pendulum
t3 = 0;
t4 = 0;             %Force applied to the slider

render_interval=2;  %render interval
dt = 0.02;           %Time increment

m_Sphere = 3;       %Sphere's Mass
R = 0.2;            %Radius of the Sphere
L = 0.15;           %Lenght of the Pendulum
m_Slider = 1;       %Slider's Mass
m_Pendulum = 1;     %Pendulum's Mass
m_Shaft = 0.5;      %Shaft's Mass
D0 = 0.1;           %Slider's initial position
L0 = 0.1;           %Pendulum's Position

%Moments of Inertia:
ISh=(1/12)*m_Shaft*(2*R)^2;
ISp=(2/3)*m_Sphere*R^2;
g=9.80665;
ISx0=0.1;   ISy0=0.1;	ISz0=0.1;
ISx=ISx0;	ISy=ISy0;	ISz=ISz0;
Ipx=m_Pendulum*(L^2);

%Friction
C2=0;
C3=0;

%Initial Condition of the Sphere
x = 0;
y = 0;
z = 0;

al = 0;
theta = 0;
phi = 0;
dl = 0;
psi = 0;

ad = 0;
td = 0;
phd = 0;
dld = 0;

hold on
iteration = 0;



%inside this loop you should update inputs (Torques and Forces) based on
%your controller:

for t = 0:dt:10
    m11=(L^2*m_Pendulum*cos(al-theta)^2*(1+cos(phi)^4)-2*L*L0*m_Pendulum*cos(phi)*sin(phi)^3+2*L*m_Pendulum*cos(al-theta)*cos(phi)*(-R+L0*cos(phi)^2*sin(phi))+L^2*m_Pendulum*sin(phi)^2*(sin(al-theta)^2+sin(phi)^2)+cos(phi)^2*(ISp+(m_Pendulum+m_Slider+m_Sphere)*R^2+L^2*m_Pendulum*sin(al-theta)^2+(ISh+2*(L0^2*m_Pendulum+m_Slider*(D0+dl)^2))*sin(phi)^2));
    m12=-m_Pendulum*(L^2*cos(al-theta)^2*(1+cos(phi)^4)-2*L*L0*cos(phi)*sin(phi)^3+L*cos(al-theta)*cos(phi)*(-R+2*L0*cos(phi)^2*sin(phi))+L^2*sin(phi)^2*(sin(al-theta)^2+sin(phi)^2)+cos(phi)^2*(L^2*sin(al-theta)^2+2*L0^2*sin(phi)^2));
    m13=L*m_Pendulum*sin(al-theta)*(L0-R*sin(phi));
    m14=0;
    
    C11=cos(phi)*(L*m_Pendulum*(ad-td)*sin(al-theta)*(R-cos(phi)^2*(L*cos(al-theta)*cos(phi)+L0*sin(phi)))+m_Slider*(D0+dl)*dld*sin(phi)*sin(2*phi))+1/8*phd*(8*L*L0*m_Pendulum*cos((al-theta)/2)^2*cos(4*phi)+8*L*m_Pendulum*(-L0*cos(2*phi)*sin((al-theta)/2)^2+R*cos(al-theta)*sin(phi))-2*(2*ISp-L^2*m_Pendulum+2*(m_Pendulum+m_Slider+m_Sphere)*R^2+L^2*m_Pendulum*cos(2*(al-theta)))*sin(2*phi)+(2*ISh-3*L^2*m_Pendulum+4*L0^2*m_Pendulum+4*m_Slider*(D0+dl)^2-L^2*m_Pendulum*cos(2*(al-theta)))*sin(4*phi));
    C12=L*m_Pendulum*cos(phi)*(ad-td)*sin(al-theta)*(-R+L*cos(al-theta)*cos(phi)^3+L0*cos(phi)^2*sin(phi))+1/4*m_Pendulum*phd*(8*L^2*cos(al-theta)^2*cos(phi)^3*sin(phi)-2*L*cos(al-theta)*(L0*(cos(2*phi)+cos(4*phi))+R*sin(phi))+4*L*sin(phi)*(-L*cos(phi)+L0*sin(3*phi))+(L^2-2*L0^2)*sin(4*phi));
    C13=1/8*(8*L*L0*m_Pendulum*cos((al-theta)/2)^2*cos(4*phi)*(-ad+td)+8*L*L0*m_Pendulum*cos(2*phi)*(ad-td)*sin((al-theta)/2)^2-8*L^2*m_Pendulum*phd*sin(2*(al-theta))-4*L*m_Pendulum*R*cos(al-theta)*(ad-2*td)*sin(phi)+2*(-(2*ISp-L^2*m_Pendulum+2*(m_Pendulum+m_Slider+m_Sphere)*R^2+L^2*m_Pendulum*cos(2*(al-theta)))*td-2*L^2*m_Pendulum*ad*sin(al-theta)^2)*sin(2*phi)+(m_Pendulum*(3*L^2-4*L0^2+L^2*cos(2*(al-theta)))*ad+(2*ISh-3*L^2*m_Pendulum+4*L0^2*m_Pendulum+4*m_Slider*(D0+dl)^2-L^2*m_Pendulum*cos(2*(al-theta)))*td)*sin(4*phi));
    C14=2*m_Slider*(D0+dl)*cos(phi)^2*td*sin(phi)^2;
    
    g1=-g*L*m_Pendulum*cos(phi)*sin(al-theta);
    
    m21=m12;
    m22= m_Pendulum*(L^2*cos(al-theta)^2+L^2*cos(phi)^2*sin(al-theta)^2+L^2*sin(al-theta)^2*sin(phi)^2+sin(phi)^2*(L0*cos(phi)-L*sin(phi))^2+cos(phi)^2*(L*cos(al-theta)*cos(phi)+L0*sin(phi))^2);
    m23=-L*m_Pendulum*sin(al-theta)*(L0-R*sin(phi));
    m24=0;
    
    C21=L*m_Pendulum*cos(phi)^3*(ad-td)*sin(al-theta)*(L*cos(al-theta)*cos(phi)+L0*sin(phi))+1/4*m_Pendulum*phd*(8*L^2*cos(al-theta)^2*cos(phi)^3*sin(phi)-2*L*cos(al-theta)*(L0*(cos(2*phi)+cos(4*phi))+R*sin(phi))+4*L*sin(phi)*(-L*cos(phi)+L0*sin(3*phi))+(L^2-2*L0^2)*sin(4*phi));
    C22=-L*m_Pendulum*cos(phi)^3*(ad-td)*sin(al-theta)*(L*cos(al-theta)*cos(phi)+L0*sin(phi))-1/4*m_Pendulum*phd*(-4*L*cos(phi)*(L0*cos(al-theta)*cos(3*phi)+L*sin(phi)-2*L*cos(al-theta)^2*cos(phi)^2*sin(phi))+4*L*L0*sin(phi)*sin(3*phi)+(L^2-2*L0^2)*sin(4*phi));
    C23=1/8*m_Pendulum*(8*L*L0*cos((al-theta)/2)^2*cos(4*phi)*(ad-td)+8*L*L0*cos(2*phi)*(-ad+td)*sin((al-theta)/2)^2+8*L^2*phd*sin(2*(al-theta))-4*L*R*cos(al-theta)*td*sin(phi)+4*L^2*(ad-td)*sin(al-theta)^2*sin(2*phi)-(3*L^2-4*L0^2+L^2*cos(2*(al-theta)))*(ad-td)*sin(4*phi));
    C24=0;
    
    g2=g*L*m_Pendulum*cos(phi)*sin(al-theta);
    
    m31=m13;
    m32=m23;
    m33=(ISh+ISp+L^2*m_Pendulum+2*L0^2*m_Pendulum+2*D0^2*m_Slider+(m_Pendulum+m_Slider+m_Sphere)*R^2+4*D0*m_Slider*dl+2*m_Slider*dl^2+L*m_Pendulum*(L*cos(2*(al-theta))-2*R*cos(al-theta)*cos(phi))+2*R*(-L0*m_Pendulum+m_Slider*(D0+dl))*sin(phi));
    m34=-m_Slider*R*cos(phi);
    
    C31=L*m_Pendulum*(2*L*cos(al-theta)-R*cos(phi))*phd*sin(al-theta)+1/4*m_Pendulum*ad*(2*L*(-4*L*cos(al-theta)^2*cos(phi)^3*sin(phi)+cos(al-theta)*(L0*(2+cos(2*phi)+cos(4*phi))-R*sin(phi))+L*sin(2*phi)-2*L0*sin(phi)*sin(3*phi))-(L^2-2*L0^2)*sin(4*phi))+1/8*td*(-8*L*L0*m_Pendulum*cos(al-theta)-8*L*L0*m_Pendulum*cos((al-theta)/2)^2*cos(4*phi)+8*L*L0*m_Pendulum*cos(2*phi)*sin((al-theta)/2)^2+2*(2*ISp-L^2*m_Pendulum+2*(m_Pendulum+m_Slider+m_Sphere)*R^2+L^2*m_Pendulum*cos(2*(al-theta)))*sin(2*phi)-(2*ISh-3*L^2*m_Pendulum+4*L0^2*m_Pendulum+4*m_Slider*(D0+dl)^2-L^2*m_Pendulum*cos(2*(al-theta)))*sin(4*phi));
    C32=1/8*m_Pendulum*(8*L*L0*cos((al-theta)/2)^2*cos(4*phi)*(-ad+td)+8*L*L0*cos(2*phi)*(ad-td)*sin((al-theta)/2)^2+8*L*R*cos(phi)*phd*sin(al-theta)+8*L*cos(al-theta)*(-L0*ad+L0*td-2*L*phd*sin(al-theta))+4*L*R*cos(al-theta)*(2*ad-td)*sin(phi)+4*L^2*(-ad+td)*sin(al-theta)^2*sin(2*phi)+(3*L^2-4*L0^2+L^2*cos(2*(al-theta)))*(ad-td)*sin(4*phi));
    C33 = L*m_Pendulum*(-2*L*cos(al-theta)+R*cos(phi))*(ad-td)*sin(al-theta)+m_Slider*dld*(2*(D0+dl)+R*sin(phi))+R*phd*((-L0*m_Pendulum+m_Slider*(D0+dl))*cos(phi)+L*m_Pendulum*cos(al-theta)*sin(phi));
    C34=m_Slider*phd*(2*(D0+dl)+R*sin(phi));
    
    g3_balance = (-L0*m_Pendulum+m_Slider*(D0+dl));
    g3= g*(  g3_balance *cos(phi)+L*m_Pendulum*cos(al-theta)*sin(phi));
    
    m41=0;
    m42=0;
    m43=m34;
    m44=m_Slider;
    
    C41=-2*m_Slider*(D0+dl)*cos(phi)^2*td*sin(phi)^2;
    C42=0;
    C43=-2*m_Slider*(D0+dl)*phd;
    C44=0;
    
    g4=g*m_Slider*sin(phi);
    
    %Inertia Matrix
    M= [ m11, m12, m13, m14 ;
        m21, m22, m23, m24 ;
        m31, m32, m33, m34 ;
        m41, m42, m43, m44];
    
    %Coriolis Terms
    C= [ C11, C12, C13, C14 ;
        C21, C22, C23, C24 ;
        C31, C32, C33, C34 ;
        C41, C42, C43, C44];
    
    %Gravitational Terms
    G=[g1,g2,g3,g4]';
    
    %Velocities
    QD=[td,ad,phd,dld]';
    
    CQD=C*QD;
    
    VV=CQD+G;
    
    % t1 = 0;
    % t2=t2-(ad-td)*C2;
    % t3=0-C3*phd;
    
    %Forces and torques
    TT=[t1  ; t2 ; t3 ; t4];
    
    %Accerelatins
    Out = M\(TT-VV);
    
    t2d=Out(1);
    a2d=Out(2);
    ph2d=Out(3);
    dl2d=Out(4);
    
    %Integrators
    td = t2d * dt + td;
    ad = a2d * dt + ad;
    phd = ph2d * dt + phd;
    dld = dl2d * dt + dld;
    
    theta = td * dt + theta;
    al = ad * dt + al;
    phi = phd * dt + phi;
    dl = dld * dt + dl;
    
    %Termination Condition (Slider out of bound)
    %if (dl > 0.1 || dl <-0.1)
    %    break;
    %end
    
    %Kinematics
    dx   = R*(td*cos(phi)*cos(psi) + phd*sin(psi));
    dy   = R*(td*cos(phi)*sin(psi) - phd*cos(psi));
    psd  = -td*sin(phi);
    
    
    
    x = dx * dt + x;
    y = dy * dt + y;
    psi = psd * dt + psi;
    z = 0;
    
    %render
    if mod(iteration, render_interval)  == 0
        Render (x,y,theta, al, phi, dl, psi, t, L0, D0)
    end
    
    iteration = iteration+1;
    
end
