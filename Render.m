function Render (x,y,theta, alph, phi, delta, psi, t, L0, dl0)


% L0 = 0;
% dl0=0.1;

L=0.15;
r_slider = 0.02;
r_pmass = 0.02;
scale_factor=1;
rS=0.2;
z=0.2;
[zr,xr,yr] = sphere(16);

x_S = rS*xr; y_S = rS*yr; z_S = rS*zr;
x_Sl=r_slider*xr; y_Sl=r_slider*yr; z_Sl=r_slider*zr;
x_pm=r_pmass*xr; y_pm=r_pmass*yr; z_pm=r_pmass*zr;

clf
hold on

r1 = 0.2;

view([45,25]);
map=[0,0,1];
colormap(map);
axis equal; 
grid on;
xlabel('X','FontSize',18,'FontName','Times New Roman')
ylabel('Y','FontSize',18,'FontName','Times New Roman')

title(['t= ',num2str(t)],'FontSize',35,'FontName','Times New Roman')

sp_surf  = surf(x_S+x ,y_S+y ,z_S+z ); alpha(sp_surf,0.2);

theta=theta*180/pi;
th_dir = [0,1,0];
th_center = [x,y,z];

coder.extrinsic('rotate')
rotate(sp_surf , th_dir , theta , th_center)

theta=theta*pi/180;

Slider_surf      = surf(x_Sl+x,y_Sl+y+delta+dl0,z_Sl+z); alpha(Slider_surf,1);
Pendulum_endmass = surf(x_pm+x+L*sin(scale_factor*(alph-theta)),y_pm+y-L0,z_pm+z-L*cos(scale_factor*(alph-theta))); alpha(Pendulum_endmass,1);
line_shaft       = line([x,x],[y-rS,y+rS],[z,z],'LineWidth',2.5,'color','red');
line_pendulum    = line([x,x+L*sin(scale_factor*(alph-theta))],[y-L0,y-L0],[z,z-L*cos(scale_factor*(alph-theta))],'LineWidth',2.5,'color','green');
line_coordx      = line([-5.5,5.5],[0,0],[0,0],'LineWidth',1,'color','k');
line_coordy      = line([0,0],[-5.5,5.5],[0,0],'LineWidth',1,'color','k');

% center_line = line([0 xc] , [0 y] , [0 z]);

%%%%%%%%%%%%%%%%%%%%%%%%%rotation Phase:

%Theta Rotation

% Phi Rotation
phi=phi*180/pi;
phi_dir = [1,0,0];
phi_center = [x,y,z];
rotate(sp_surf,           phi_dir, phi , phi_center)
rotate(line_shaft,        phi_dir, phi , phi_center)
rotate(line_pendulum,     phi_dir, phi , phi_center)
rotate(Slider_surf,       phi_dir, phi , phi_center)
rotate(Pendulum_endmass,  phi_dir, phi , phi_center)

% Phi Rotation
psi=psi*180/pi;
psi_dir = [0,0,1];
psi_center = [x,y,z];
rotate(sp_surf,           psi_dir, psi , psi_center)
rotate(line_shaft,        psi_dir, psi , psi_center)
rotate(line_pendulum,     psi_dir, psi , psi_center)
rotate(Slider_surf,     psi_dir, psi , psi_center)
rotate(Pendulum_endmass,  psi_dir, psi , psi_center)
axis([x-0.7 x+0.7 y-0.7 y+0.7 z-0.2 z+0.2]);
% axis([-5,5,-4,4,-0,0.5]);

pause(0.01)
end

