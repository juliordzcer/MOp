clc
clear all
% close all

%Tiempo
T = 10;
np = 50;

% Localizacion de la estacion de trabajo
ex = 0;%mm
ey = 0;%mm
ez = 0;%mm

%Parametros de emplazamiento de tarea
tx = 804.5;%mm
ty = 499.9;%mm
tz =  663.1;%mm
phi = -9.9; %grados

% Parametros del robot
d2 = 150; %mm
d3 = 600; %mm
d4 = 200; %mm
r4 = 640; %mm

bR = 450;
eo = 100;

% Parametros del organo terminal
rho = -22;   %Grados
dhx = -49.8; %mm
dhy = 0;     %mm
dhz = 504;   %mm

% Vertices para graficar el cuerpo del robot.
p0r = [0 0 0 1]';
p1r = [0 0 bR 1]';
p2r = [0 d2 bR 1]';
p3r = [0 d2 bR+d3 1]';
p4r = [0 d2 bR+d3+d4 1]';
p5r = [0 d2+r4 bR+d3+d4 1]';
p6r = [0 d2+r4+eo bR+d3+d4 1]';

% Vertices para graficar el cuerpo del robot.
p0h = [0 0 0 1]';
p1h = [0 300 0 1]';
p2h = [0 404 -104*tan(deg2rad(22)) 1]';


% Limites articulares
th1min =-170; %deg
th1max = 170; %deg
th1mean=((th1max+th1min)/2);
delth1mx =abs(th1max-th1mean);

th2min =-90;  %deg
th2max = 160; %deg
th2mean=((th2max+th2min)/2);
delth2mx=abs(th2max-th2mean);

th3min =-180; %deg
th3max = 267; %deg
th3mean=((th3max+th3min)/2);
delth3mx=abs(th3max-th3mean);

th4min =-190; %deg
th4max = 190; %deg
th4mean=((th4max+th4min)/2);
delth4mx=abs(th4max-th4mean);

th5min =-270; %deg
th5max = 270; %deg
th5mean=((th5max+th5min)/2);
delth5mx=abs(th5max-th5mean);

th6min =-360; %deg
th6max = 360; %deg
th6mean=((th6max+th6min)/2);
delth6mx=abs(th6max-th6mean);

% Velocidades maximas
th1pmax = 230; %deg/s
th2pmax = 225; %deg/s
th3pmax = 230; %deg/s
th4pmax = 430; %deg/s
th5pmax = 430; %deg/s
th6pmax = 630; %deg/s

% Variables articulares en grados
th1g=zeros(1,np);
th2g=zeros(1,np);
th3g=zeros(1,np);
th4g=zeros(1,np);
th5g=zeros(1,np);
th6g=zeros(1,np);
time=zeros(1,np);

x0g=zeros(1,np);
y0g=zeros(1,np);
z0g=zeros(1,np);

x1g=zeros(1,np);
y1g=zeros(1,np);
z1g=zeros(1,np);

x2g=zeros(1,np);
y2g=zeros(1,np);
z2g=zeros(1,np);

x3g=zeros(1,np);
y3g=zeros(1,np);
z3g=zeros(1,np);

x4g=zeros(1,np);
y4g=zeros(1,np);
z4g=zeros(1,np);

x5g=zeros(1,np);
y5g=zeros(1,np);
z5g=zeros(1,np);

x6g=zeros(1,np);
y6g=zeros(1,np);
z6g=zeros(1,np);

xhg=zeros(1,np);
yhg=zeros(1,np);
zhg=zeros(1,np);

%Parametros del movimiento del robot:
%solucion del mip del robot:

%Epsilon
e1=1;
e2=1;
e4=-1;

% Matriz del marco 6 al marco de la herramienta

T6h=[cos(deg2rad(rho))   0   -sin(deg2rad(rho))    dhx;
        0       1       0        dhy;
     sin(deg2rad(rho))   0    cos(deg2rad(rho))    dhx;
        0       0       0         1 ];    
    
Th6=inv(T6h);


% Emplazamiento en el piso:

TE0=[1  0  0 ex ;
     0  1  0 ey   ;
     0  0  1 ez+bR ;
     0  0  0  1  ];

% Emplazamiento de la tarea:
TEt =[ 0  -1   0  tx ;
       0   0  -1  ty ;
       1   0   0  tz ;
       0   0   0  1  ];
   
T0E=inv(TE0);
T0t=T0E*TEt;

R0T=[T0t(1,1) T0t(1,2) T0t(1,3)
     T0t(2,1) T0t(2,2) T0t(2,3)
     T0t(3,1) T0t(3,2) T0t(3,3)];
 

% Especificacion de los parametros de la ruta deseada de la herramienta;
% xinih=300;
% delx=0;
% 
% yinih=10;
% dely=150;

zinih=100;
delz=10;

% Orientacion en angulos de Euler
alphaini=deg2rad(0);
delalpha=deg2rad(0);

betaini=deg2rad(0);
delbeta=deg2rad(0);

gammaini=deg2rad(0);
delgamma=deg2rad(0);

for i=0:np
    t=T*i/np;
    
    % Especificacion de las coordenadas operacionales de la herramienta con respecto al marco de la tarea como funcion cicloidal del tiempo
    funct = (t/T) - (1/(2*pi)) * (sin(2*pi*t/T));
    
    alpha=alphaini+delalpha*funct;
    beta=betaini+delbeta*funct;
    gamma=gammaini+delgamma*funct;
    
    Rthd11 = cos(alpha)*cos(beta);
    Rthd21 = sin(alpha)*cos(beta);
    Rthd31 =-sin(beta);
    Rthd12 = cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);
    Rthd22 = sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
    Rthd32 = cos(beta)*sin(gamma);
    Rthd13 = cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
    Rthd23 = sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
    Rthd33 =  cos(beta)*cos(gamma);
    
    Rthd=[Rthd11 Rthd12 Rthd13;
          Rthd21 Rthd22 Rthd23;
          Rthd31 Rthd32 Rthd33];

%     xph=xinih+delx*funt;
%     yph=yinih+dely*funt;
%     zph=zinih+delz*funct;
    
%     % Coordenadas del centro del círculo
    x_center = 100; % Coordenada x del centro del círculo
    y_center = 1000;  % Coordenada y del centro del círculo

    % Radio del círculo
    radius = 70;   % Radio del círculo

    % Cálculo de las coordenadas x e y en función del tiempo para seguir un círculo
    xph1 = x_center + radius * cos(2*pi*funct);
    yph1 = y_center + radius * sin(2*pi*funct);
    zph1 = zinih;

%            xph yph zph
%     TEt =[ 0  -1   0  tx ;    xph1
%            0   0  -1  ty ;    yph1
%            1   0   0  tz ;    zph1
%            0   0   0  1  ];
    
    xph = zph1;
    yph = xph1;
    zph = yph1;

    % Cálculo de las variaciones en las coordenadas x, y, z y en los ángulos de orientación alpha, beta y gamma
    delx = 0;%(2*pi*radius/T) * (-sin(2*pi*t/T));
    dely = 0;%(2*pi*radius/T) * (cos(2*pi*t/T));
    delz = 0; % No hay variación en la coordenada z
    delalpha = 0; % No hay variación en el ángulo alpha
    delbeta = 0;  % No hay variación en el ángulo beta
    delgamma = 0; % No hay variación en el ángulo gamma

    
    % Matriz homogenea de la pose deseada del marco h c.r al marco t;
    Tthd=[Rthd(1,1) Rthd(1,2) Rthd(1,3) xph;
          Rthd(2,1) Rthd(2,2) Rthd(2,3) yph;
          Rthd(3,1) Rthd(3,2) Rthd(3,3) zph;
           0        0          0         1];

    %Especificacion del estado de velocidad de la herramienta,
    %con respecto al marco t de la tarea.
    funtp=(1/T)*(1-cos(2*pi*t/T));
    
    xphpun=delx*funtp;
    yphpun=dely*funtp;
    zphpun=delz*funtp;
    
    alphahpun=delalpha*funtp;
    betahpun=delbeta*funtp;
    gammahpun=delgamma*funtp;
    
    wxh=gammahpun;
    wyh=betahpun;
    wzh=alphahpun; 
    
    % Matriz de la pose deseada del marco h c.r al marco 0 
    T0hd=T0t*Tthd;
    % Matriz de la pose deseada del marco 6 c.r al marco 0:
    T06d=T0hd*Th6; 

    %MATRIZ SNAP DEFINICION DE LOS ELEMENTOS DE LA MATRIZ SNAP PARA SU USO EN EL MIP
    SX=T06d(1,1);
    SY=T06d(2,1);
    SZ=T06d(3,1);

    NX=T06d(1,2);
    NY=T06d(2,2);
    NZ=T06d(3,2);

    AX=T06d(1,3);
    AY=T06d(2,3);
    AZ=T06d(3,3);

    PX=T06d(1,4);
    PY=T06d(2,4);
    PZ=T06d(3,4);

    Snap=[SX NX AX PX
          SY NY AY PY
          SZ NZ AZ PZ
          0  0  0  1];
      
    %Calculo del estado de velocidad deseado de la herramienta con respecto
    % al marco 0
    vph_t=[xphpun yphpun zphpun]';
    whh_t=[wxh wyh wzh]';

    vph_0=R0T*vph_t;
    whh_0=R0T*whh_t;

    vphy_0(i+1)=vph_0(2);

    %Calculo del estado de veocidad deseado  del marco n,
    %con respecto al marco 0:

    rnh_0=[AX*tx AY*ty AZ*tz];
    wnn_0=whh_0;

    difvnhx_0=wnn_0(2)*rnh_0(3)-wnn_0(3)*rnh_0(2);
    difvnhy_0=wnn_0(3)*rnh_0(1)-wnn_0(1)*rnh_0(3);
    difvnhz_0=wnn_0(1)*rnh_0(2)-wnn_0(2)*rnh_0(1);

    difvnh_0=[difvnhx_0 difvnhy_0 difvnhz_0];
    von_0= vph_0-difvnh_0;
    Spun_0=[von_0(1) von_0(2) von_0(3) wnn_0(1) wnn_0(2) wnn_0(3)]';

    % Calculo de las variables articulares:
    th1 = atan2(e1 * PY,e1 * PX);

    z1 = -d2 + PX * cos(th1) + PY * sin(th1);
    b1 = 2 * (-(d4 * PZ) - r4 * z1);
    b2 = 2 * (PZ *r4 - d4 * z1);
    b3 = d3^2 - d4^2 - PZ^2 - r4^2 - z1^2;

    SQ = (b1 * b3 + b2 * sqrt(b1^2 + b2^2 - b3^2)*e2)/(b1^2 + b2^2);
    CQ = (b2 * b3 - b1 * sqrt(b1^2 + b2^2 - b3^2)*e2)/(b1^2 + b2^2);

    th2 = atan2(-((-PZ-r4 * CQ + d4 * SQ)/(d3)),(z1 - d4 * CQ - r4 * SQ)/(d3));

    th3 = atan2(SQ,CQ)-th2;

    X = -(AY * cos(th1)) + AX * sin(th1);
    Y = -(AX * cos(th1) * cos(th2 + th3)) - AY * cos(th2 + th3) * sin(th1) - AZ * sin(th2 + th3);

    th4 = atan2(-X * e4,Y * e4);

    Y12 = -(cos(th4) * (AX * cos(th1) * cos(th2 + th3) + AY * cos(th2 + th3) * sin(th1) + AZ * sin(th2 + th3))) - (- (AY * cos(th1) + AX * sin(th1))*sin(th4));
    Y1 = - AZ*cos(th2+th3) - AX*cos(th1)*sin(th2+th3)-AY*sin(th1)*sin(th2 + th3);

    th5 = atan2(-Y12,-Y1);

    Y22 = - (cos(th4)*(-SY * cos(th1) + SX * sin(th1))) + (SX*cos(th1)*cos(th2+th3) + SY*cos(th2+th3)*sin(th1)+SZ*sin(th2+th3))*sin(th4);
    Y21 = - (cos(th4)*(-NY * cos(th1) + NX * sin(th1))) + (NX*cos(th1)*cos(th2+th3) + NY*cos(th2+th3)*sin(th1)+NZ*sin(th2+th3))*sin(th4);

    th6 = atan2(-Y22,-Y21);

    % Matriz jacobiana
    J11 = (-d2 - d3*cos(th2) - d4*cos(th2 + th3) - r4*sin(th2 + th3))*sin(th1);
    J21 = (d2 + d3*cos(th2) + d4*cos(th2 + th3) + r4*sin(th2 + th3))*cos(th1);
    J31 = 0;
    J41 = 0;
    J51 = 0;
    J61 = 1;
    J12 = (-d3*sin(th2) - d4*sin(th2 + th3) + r4*cos(th2 + th3))*cos(th1);
    J22 = (-d3*sin(th2) - d4*sin(th2 + th3) + r4*cos(th2 + th3))*sin(th1);
    J32 = d3*cos(th2) + d4*cos(th2 + th3) + r4*sin(th2 + th3);
    J42 = sin(th1);
    J52 = -cos(th1);
    J62 = 0;
    J13 = (-d4*sin(th2 + th3) + r4*cos(th2 + th3))*cos(th1);
    J23 = (-d4*sin(th2 + th3) + r4*cos(th2 + th3))*sin(th1);
    J33 = d4*cos(th2 + th3) + r4*sin(th2 + th3);
    J43 = sin(th1);
    J53 = -cos(th1);
    J63 = 0;
    J14 = 0;
    J24 = 0;
    J34 = 0;
    J44 = sin(th2 + th3)*cos(th1);
    J54 = sin(th1)*sin(th2 + th3);
    J64 = -cos(th2 + th3);
    J15 = 0;
    J25 = 0;
    J35 = 0;
    J45 = sin(th1)*cos(th4) - sin(th4)*cos(th1)*cos(th2 + th3);
    J55 = -sin(th1)*sin(th4)*cos(th2 + th3) - cos(th1)*cos(th4);
    J65 = -sin(th4)*sin(th2 + th3);
    J16 = 0;
    J26 = 0;
    J36 = 0;
    J46 = sin(th1)*sin(th4)*sin(th5) + sin(th5)*cos(th1)*cos(th4)*cos(th2 + th3) + sin(th2 + th3)*cos(th1)*cos(th5);
    J56 = sin(th1)*sin(th5)*cos(th4)*cos(th2 + th3) + sin(th1)*sin(th2 + th3)*cos(th5) - sin(th4)*sin(th5)*cos(th1);
    J66 = sin(th5)*sin(th2 + th3)*cos(th4) - cos(th5)*cos(th2 + th3);


    JTA=[J11 J12 J13 J14 J15 J16;
         J21 J22 J23 J24 J25 J26;
         J31 J32 J33 J34 J35 J36];

    JRW=[ J44 J45 J46;
          J54 J55 J56;
          J64 J65 J66];

    Mtrs1=sqrt(det(JTA*JTA'));
    Mrts1=sqrt(det(JRW+JRW'));

    J=  [J11 J12 J13 J14 J15 J16;
         J21 J22 J23 J24 J25 J26;
         J31 J32 J33 J34 J35 J36;
         J41 J42 J43 J44 J45 J46;
         J51 J52 J53 J54 J55 J56;
         J61 J62 J63 J64 J65 J66];

    %Calculo de las velocidades articulares:
    qp=inv(J)*Spun_0;
    th1p(i+1)=qp(1);
    th2p(i+1)=qp(2);
    th3p(i+1)=qp(3);
    th4p(i+1)=qp(4);
    th5p(i+1)=qp(5);
    th6p(i+1)=qp(6);
     

    %PREPARACION DE MATRICES PARA LA ANIMACION:;
    %MATRICES DE TRANSFORMACION HOMOGENEAS

%     Tramsformation matrix 0 T 6
    T0T611 = ((sin(th1)*sin(th4) + cos(th1)*cos(th4)*cos(th2 + th3))*cos(th5) - sin(th5)*sin(th2 + th3)*cos(th1))*cos(th6) + (sin(th1)*cos(th4) - sin(th4)*cos(th1)*cos(th2 + th3))*sin(th6);
    T0T621 = ((sin(th1)*cos(th4)*cos(th2 + th3) - sin(th4)*cos(th1))*cos(th5) - sin(th1)*sin(th5)*sin(th2 + th3))*cos(th6) + (-sin(th1)*sin(th4)*cos(th2 + th3) - cos(th1)*cos(th4))*sin(th6);
    T0T631 = (sin(th5)*cos(th2 + th3) + sin(th2 + th3)*cos(th4)*cos(th5))*cos(th6) - sin(th4)*sin(th6)*sin(th2 + th3);
    T0T612 = -((sin(th1)*sin(th4) + cos(th1)*cos(th4)*cos(th2 + th3))*cos(th5) - sin(th5)*sin(th2 + th3)*cos(th1))*sin(th6) + (sin(th1)*cos(th4) - sin(th4)*cos(th1)*cos(th2 + th3))*cos(th6);
    T0T622 = -((sin(th1)*cos(th4)*cos(th2 + th3) - sin(th4)*cos(th1))*cos(th5) - sin(th1)*sin(th5)*sin(th2 + th3))*sin(th6) + (-sin(th1)*sin(th4)*cos(th2 + th3) - cos(th1)*cos(th4))*cos(th6);
    T0T632 = -(sin(th5)*cos(th2 + th3) + sin(th2 + th3)*cos(th4)*cos(th5))*sin(th6) - sin(th4)*sin(th2 + th3)*cos(th6);
    T0T613 = (sin(th1)*sin(th4) + cos(th1)*cos(th4)*cos(th2 + th3))*sin(th5) + sin(th2 + th3)*cos(th1)*cos(th5);
    T0T623 = (sin(th1)*cos(th4)*cos(th2 + th3) - sin(th4)*cos(th1))*sin(th5) + sin(th1)*sin(th2 + th3)*cos(th5);
    T0T633 = sin(th5)*sin(th2 + th3)*cos(th4) - cos(th5)*cos(th2 + th3);
    T0T614 = d2*cos(th1) + d3*cos(th1)*cos(th2) + d4*cos(th1)*cos(th2 + th3) + r4*sin(th2 + th3)*cos(th1);
    T0T624 = d2*sin(th1) + d3*sin(th1)*cos(th2) + d4*sin(th1)*cos(th2 + th3) + r4*sin(th1)*sin(th2 + th3);
    T0T634 = d3*sin(th2) + d4*sin(th2 + th3) - r4*cos(th2 + th3);

%     Tramsformation matrix 0 T 5
    T0T511 = (sin(th1)*sin(th4) + cos(th1)*cos(th4)*cos(th2 + th3))*cos(th5) - sin(th5)*sin(th2 + th3)*cos(th1);
    T0T521 = (sin(th1)*cos(th4)*cos(th2 + th3) - sin(th4)*cos(th1))*cos(th5) - sin(th1)*sin(th5)*sin(th2 + th3);
    T0T531 = sin(th5)*cos(th2 + th3) + sin(th2 + th3)*cos(th4)*cos(th5);
    T0T512 = -(sin(th1)*sin(th4) + cos(th1)*cos(th4)*cos(th2 + th3))*sin(th5) - sin(th2 + th3)*cos(th1)*cos(th5);
    T0T522 = -(sin(th1)*cos(th4)*cos(th2 + th3) - sin(th4)*cos(th1))*sin(th5) - sin(th1)*sin(th2 + th3)*cos(th5);
    T0T532 = -sin(th5)*sin(th2 + th3)*cos(th4) + cos(th5)*cos(th2 + th3);
    T0T513 = sin(th1)*cos(th4) - sin(th4)*cos(th1)*cos(th2 + th3);
    T0T523 = -sin(th1)*sin(th4)*cos(th2 + th3) - cos(th1)*cos(th4);
    T0T533 = -sin(th4)*sin(th2 + th3);
    T0T514 = d2*cos(th1) + d3*cos(th1)*cos(th2) + d4*cos(th1)*cos(th2 + th3) + r4*sin(th2 + th3)*cos(th1);
    T0T524 = d2*sin(th1) + d3*sin(th1)*cos(th2) + d4*sin(th1)*cos(th2 + th3) + r4*sin(th1)*sin(th2 + th3);
    T0T534 = d3*sin(th2) + d4*sin(th2 + th3) - r4*cos(th2 + th3);

%     Tramsformation matrix 0 T 4
    T0T411 = sin(th1)*sin(th4) + cos(th1)*cos(th4)*cos(th2 + th3);
    T0T421 = sin(th1)*cos(th4)*cos(th2 + th3) - sin(th4)*cos(th1);
    T0T431 = sin(th2 + th3)*cos(th4);
    T0T412 = sin(th1)*cos(th4) - sin(th4)*cos(th1)*cos(th2 + th3);
    T0T422 = -sin(th1)*sin(th4)*cos(th2 + th3) - cos(th1)*cos(th4);
    T0T432 = -sin(th4)*sin(th2 + th3);
    T0T413 = sin(th2 + th3)*cos(th1);
    T0T423 = sin(th1)*sin(th2 + th3);
    T0T433 = -cos(th2 + th3);
    T0T414 = d2*cos(th1) + d3*cos(th1)*cos(th2) + d4*cos(th1)*cos(th2 + th3) + r4*sin(th2 + th3)*cos(th1);
    T0T424 = d2*sin(th1) + d3*sin(th1)*cos(th2) + d4*sin(th1)*cos(th2 + th3) + r4*sin(th1)*sin(th2 + th3);
    T0T434 = d3*sin(th2) + d4*sin(th2 + th3) - r4*cos(th2 + th3);

%     Tramsformation matrix 0 T 3
    T0T311 = cos(th1)*cos(th2 + th3);
    T0T321 = sin(th1)*cos(th2 + th3);
    T0T331 = sin(th2 + th3);
    T0T312 = -sin(th2 + th3)*cos(th1);
    T0T322 = -sin(th1)*sin(th2 + th3);
    T0T332 = cos(th2 + th3);
    T0T313 = sin(th1);
    T0T323 = -cos(th1);
    T0T333 = 0;
    T0T314 = d2*cos(th1) + d3*cos(th1)*cos(th2);
    T0T324 = d2*sin(th1) + d3*sin(th1)*cos(th2);
    T0T334 = d3*sin(th2);

%     Tramsformation matrix 0 T 2
    T0T211 = cos(th1)*cos(th2);
    T0T221 = sin(th1)*cos(th2);
    T0T231 = sin(th2);
    T0T212 = -sin(th2)*cos(th1);
    T0T222 = -sin(th1)*sin(th2);
    T0T232 = cos(th2);
    T0T213 = sin(th1);
    T0T223 = -cos(th1);
    T0T233 = 0;
    T0T214 = d2*cos(th1);
    T0T224 = d2*sin(th1);
    T0T234 = 0;

    % Tramsformation matrix 0 T 1
    T0T111 = cos(th1);
    T0T121 = sin(th1);
    T0T131 = 0;
    T0T112 = -sin(th1);
    T0T122 = cos(th1);
    T0T132 = 0;
    T0T113 = 0;
    T0T123 = 0;
    T0T133 = 1;
    T0T114 = 0;
    T0T124 = 0;
    T0T134 = 0;

    T01 = [T0T111 T0T112 T0T113 T0T114;
           T0T121 T0T122 T0T123 T0T124;
           T0T131 T0T132 T0T133 T0T134;
           0      0      0      1];

    T02 = [T0T211 T0T212 T0T213 T0T214;
           T0T221 T0T222 T0T223 T0T224;
           T0T231 T0T232 T0T233 T0T234;
           0      0      0      1];

    T03 = [T0T311 T0T312 T0T313 T0T314;
           T0T321 T0T322 T0T323 T0T324;
           T0T331 T0T332 T0T333 T0T334;
           0      0      0      1];

    T04 = [T0T411 T0T412 T0T413 T0T414;
           T0T421 T0T422 T0T423 T0T424;
           T0T431 T0T432 T0T433 T0T434;
           0      0      0      1];

    T05 = [T0T511 T0T512 T0T513 T0T514;
           T0T521 T0T522 T0T523 T0T524;
           T0T531 T0T532 T0T533 T0T534;
           0      0      0      1];

    T06 = [T0T611 T0T612 T0T613 T0T614;
           T0T621 T0T622 T0T623 T0T624;
           T0T631 T0T632 T0T633 T0T634;
           0      0      0      1];
    
    T0h=T06*T6h;

    % Poses de los marcos de la cadena cinematica
    TE1=TE0*T01;
    TE2=TE0*T02;
    TE3=TE0*T03;
    TE4=TE0*T04;
    TE5=TE0*T05;
    TE6=TE0*T06;
    TEh=TE0*T0h; 

    T01 = [cos(th1) -sin(th1) 0 0;sin(th1) cos(th1) 0 0;0 0 1 0;0 0 0 1];
    T12 = [cos(th2) -sin(th2) 0 d2; 0 0 -1 0;sin(th2) cos(th2) 0 0;0 0 0 1];
    T23 = [cos(th3) -sin(th3) 0 d3;sin(th3) cos(th3) 0 0;0 0 1 0;0 0 0 1];
    T34 = [cos(th4) -sin(th4) 0 d4; 0 0 -1 -r4;sin(th4) cos(th4) 0 0;0 0 0 1];
    T45 = [cos(th5) -sin(th5) 0 0; 0 0 1 0;-sin(th5) -cos(th5) 0 0;0 0 0 1];
    T56 = [cos(th6) -sin(th6) 0 0; 0 0 -1 0;sin(th6) cos(th6) 0 0;0 0 0 1];
    
    % COORDENASD DE LA FIGURA 1 (EMPLAZADO)
    x0=TE0(1,4);
    y0=TE0(2,4);
    z0=TE0(3,4);

    x1=TE1(1,4);
    y1=TE1(2,4);
    z1=TE1(3,4);

    x2=TE2(1,4);
    y2=TE2(2,4);
    z2=TE2(3,4);

    x3=TE3(1,4);
    y3=TE3(2,4);
    z3=TE3(3,4);

    x4=TE4(1,4);
    y4=TE4(2,4);
    z4=TE4(3,4);

    x5=TE5(1,4);
    y5=TE5(2,4);
    z5=TE5(3,4);

    x6=TE6(1,4);
    y6=TE6(2,4);
    z6=TE6(3,4);
    
    xh=TEh(1,4);
    yh=TEh(2,4);
    zh=TEh(3,4);
    
    th1g(i+1)=rad2deg(th1);
    th2g(i+1)=rad2deg(th2);
    th3g(i+1)=rad2deg(th3);
    th4g(i+1)=rad2deg(th4);
    th5g(i+1)=rad2deg(th5);
    th6g(i+1)=rad2deg(th6);
    time(i+1)=t;
        
    x0g(i+1)=x0;
    y0g(i+1)=y0;
    z0g(i+1)=z0;
    
    x1g(i+1)=x1;
    y1g(i+1)=y1;
    z1g(i+1)=z1;
    
    x2g(i+1)=x2;
    y2g(i+1)=y2;
    z2g(i+1)=z2;
    
    x3g(i+1)=x3;
    y3g(i+1)=y3;
    z3g(i+1)=z3;
    
    x4g(i+1)=x4;
    y4g(i+1)=y4;
    z4g(i+1)=z4;
    
    x5g(i+1)=x5;
    y5g(i+1)=y5;
    z5g(i+1)=z5;
    
    x6g(i+1)=x6;
    y6g(i+1)=y6;
    z6g(i+1)=z6;
    
    xhg(i+1)=xh;
    yhg(i+1)=yh;
    zhg(i+1)=zh;

    % Definicion de cadenas cinematica
    r1x=[x0 x1];
    r1y=[y0 y1];
    r1z=[z0 z1];

    r2x=[x1 x2];
    r2y=[y1 y2];
    r2z=[z1 z2];

    r3x=[x2 x3];
    r3y=[y2 y3];
    r3z=[z2 z3];
    
    r4x=[x3 x4];
    r4y=[y3 y4];
    r4z=[z3 z4];

    r5x=[x4 x5];
    r5y=[y4 y5];
    r5z=[z4 z5];

    r6x=[x5 x6];
    r6y=[y5 y6];
    r6z=[z5 z6];
    
    rhx=[x6 xh];
    rhy=[y6 yh];
    rhz=[z6 zh];

    rz=[0 0];
    
    % Definicion de vastago
    ss1=[0 0 -400 1]';
    xx1=TE0*ss1;
    s1x=[TE0(1,4) xx1(1)];
    s1y=[TE0(2,4) xx1(2)];
    s1z=[TE0(3,4) xx1(3)];

    % Definicion de la base
    ss2=[100 100 -400 1]';
    xx2=TE0*ss2;

    ss3=[-100 100 -400 1]';
    xx3=TE0*ss3;

    ss4=[100 -100 -400 1]';
    xx4=TE0*ss4;

    ss5=[-100 -100 -400 1]';
    xx5=TE0*ss5;

    b1x=[xx2(1) xx3(1)];
    b1y=[xx2(2) xx3(2)];
    b1z=[xx2(3) xx3(3)];

    b2x=[xx3(1) xx5(1)];
    b2y=[xx3(2) xx5(2)];
    b2z=[xx3(3) xx5(3)];

    b3x=[xx5(1) xx4(1)];
    b3y=[xx5(2) xx4(2)];
    b3z=[xx5(3) xx4(3)];

    b4x=[xx4(1) xx2(1)];
    b4y=[xx4(2) xx2(2)];
    b4z=[xx4(3) xx2(3)];
    
    %Graficar Robo
    RJ0 = TE0 * p0r;
    RJ1 = TE1 * p1r;
    RJ2 = TE2 * p2r;
    RJ3 = TE3 * p3r;
    RJ4 = TE4 * p4r;
    RJ5 = TE5 * p5r;
    RJ6 = TE6 * p6r;
    
    %Grafica de Herramienta
    HA0 = TE6 * p0h;
    HA1 = TE6 * p1h;
    HA2 = TE6 * p2h;
    
    RJ0x = [RJ0(1) RJ1(1)];
    RJ0y = [RJ0(2) RJ1(2)];
    RJ0z = [RJ0(3) RJ1(3)];
    
    RJ1x = [RJ1(1) RJ2(1)];
    RJ1y = [RJ1(2) RJ2(2)];
    RJ1z = [RJ1(3) RJ2(3)];
    
    RJ2x = [RJ2(1) RJ3(1)];
    RJ2y = [RJ2(2) RJ3(2)];
    RJ2z = [RJ2(3) RJ3(3)];
    
    RJ3x = [RJ3(1) RJ4(1)];
    RJ3y = [RJ3(2) RJ4(2)];
    RJ3z = [RJ3(3) RJ4(3)];
    
    RJ4x = [RJ4(1) RJ5(1)];
    RJ4y = [RJ4(2) RJ5(2)];
    RJ4z = [RJ4(3) RJ5(3)];
    
    RJ5x = [RJ5(1) RJ6(1)];
    RJ5y = [RJ5(2) RJ6(2)];
    RJ5z = [RJ5(3) RJ6(3)];
    
    RJ6x = [RJ6(1) x6];
    RJ6y = [RJ6(2) y6];
    RJ6z = [RJ6(3) z6];
    
    % Definicion del marco 0
    ss6=[300 0 0 1]';
    xx6=TE0*ss6;
    ss7=[0 300 0 1]';
    xx7=TE0*ss7;
    ss8=[0 0 300 1]';
    xx8=TE0*ss8;

    e1x=[TE0(1,4) xx6(1)];
    e1y=[TE0(2,4) xx6(2)];
    e1z=[TE0(3,4) xx6(3)];

    e2x=[TE0(1,4) xx7(1)];
    e2y=[TE0(2,4) xx7(2)];
    e2z=[TE0(3,4) xx7(3)];

    e3x=[TE0(1,4) xx8(1)];
    e3y=[TE0(2,4) xx8(2)];
    e3z=[TE0(3,4) xx8(3)];
    
    figure (1)
    clf
    hold on
    xlabel('Eje x')
    ylabel('Eje y')
    zlabel('Eje z')
    
    
    plot3(r1x,r1y,r1z,r2x,r2y,r2z,r3x,r3y,r3z,r4x,r4y,r4z,'linewidth',3)

    grid off
    % Define las trayectorias de los marcos del robot
%     plot3(r1x,r1y,r1z,'k-o',r2x,r2y,r2z,'k-o',r3x,r3y,r3z,'k-o',r4x,r4y,r4z,'k-o',r5x,r5y,r5z,'k-o',r6x,r6y,r6z,'k-o','linewidth',3)

% plot3(r1x,r1z,r1y,r2x,r2z,r2y,r3x,r3z,r3y,r4x_rectangulo,r4z_rectangulo,r4y_constante,'linewidth',3)


    % Define la base del robot
    plot3(b1x, b1y, b1z, 'k', 'LineWidth', 2);
    plot3(b2x, b2y, b2z, 'k', 'LineWidth', 2);
    plot3(b3x, b3y, b3z, 'k', 'LineWidth', 2);
    plot3(b4x, b4y, b4z, 'k', 'LineWidth', 2);
    plot3(s1x,s1y,s1z,'k','MarkerSize',5,'linewidth',3)


    % Define los marcos del robot
    plot3(e1x, e1y, e1z, 'k--', 'LineWidth', 1);
    plot3(e2x, e2y, e2z, 'k--', 'LineWidth', 1);
    plot3(e3x, e3y, e3z, 'k--', 'LineWidth', 1);


    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    axis equal;
%     view([.5,.5,.5]);
   view([0,1,0]);
    plot3(xhg, yhg, zhg, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 5);
end




% Gráfica de la animación del cuerpo del robot
plot3(x0g, y0g, z0g, 'r', 'LineWidth', 2); hold on;
plot3(x1g, y1g, z1g, 'g', 'LineWidth', 2);
plot3(x2g, y2g, z2g, 'b', 'LineWidth', 2);
plot3(x3g, y3g, z3g, 'm', 'LineWidth', 2);
plot3(x4g, y4g, z4g, 'c', 'LineWidth', 2);
plot3(x5g, y5g, z5g, 'y', 'LineWidth', 2);
plot3(x6g, y6g, z6g, 'k', 'LineWidth', 2);

% Trayectoria del órgano terminal
% plot3(xhg, yhg, zhg, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);

% Configuración de la gráfica
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
legend('Link 0', 'Link 1', 'Link 2', 'Link 3', 'Link 4', 'Link 5', 'Link 6', 'End-Effector Trajectory');
title('Robot Animation');



figure(2)

subplot(3,1,1);
hold on;
plot(time, x0g, 'LineWidth', 2);
plot(time, x1g, 'LineWidth', 2);
plot(time, x2g, 'LineWidth', 2);
plot(time, x3g, 'LineWidth', 2);
plot(time, x4g, 'LineWidth', 2);
plot(time, x5g, 'LineWidth', 2);
plot(time, x6g, 'LineWidth', 2);
plot(time, xhg, 'LineWidth', 2);
hold off;
legend('x0', 'x1', 'x2', 'x3', 'x4', 'x5', 'x6','xh');

subplot(3,1,2);
hold on;
plot(time, y0g, 'LineWidth', 2);
plot(time, y1g, 'LineWidth', 2);
plot(time, y2g, 'LineWidth', 2);
plot(time, y3g, 'LineWidth', 2);
plot(time, y4g, 'LineWidth', 2);
plot(time, y5g, 'LineWidth', 2);
plot(time, y6g, 'LineWidth', 2);
plot(time, yhg, 'LineWidth', 2);
hold off;
legend('y0', 'y1', 'y2', 'y3', 'y4', 'y5', 'y6','yh');

subplot(3,1,3);
hold on;
plot(time, z0g, 'LineWidth', 2);
plot(time, z1g, 'LineWidth', 2);
plot(time, z2g, 'LineWidth', 2);
plot(time, z3g, 'LineWidth', 2);
plot(time, z4g, 'LineWidth', 2);
plot(time, z5g, 'LineWidth', 2);
plot(time, z6g, 'LineWidth', 2);
plot(time, zhg, 'LineWidth', 2);
hold off;
legend('z0', 'z1', 'z2', 'z3', 'z4', 'z5', 'z6','zh');


figure(3)
subplot(231);
plot(time,th1g,'c','LineWidth',2)
hold on

plot(time,th1max,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerSize',2)
plot(time,th1min,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerSize',2)
xlabel('Time(seg)')
ylabel('Theta1(deg)')
grid on

subplot(232);
plot(time,th2g,'c','LineWidth',2)
hold on
plot(time,th2max,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2)
plot(time,th2min,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerSize',2)
  
xlabel('Time(seg)')
ylabel('Theta2(deg)')
grid on

subplot(233);
plot(time,th3g,'c','LineWidth',2)
hold on
plot(time,th3max,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2)
plot(time,th3min,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerSize',2)

xlabel('Time(seg)')
ylabel('Theta3(deg)')
grid on

subplot(234);
plot(time,th4g,'c','LineWidth',2)
hold on
plot(time,th4max,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor','r', 'MarkerSize',2)
plot(time,th4min,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerSize',2)

xlabel('Time(seg)')
ylabel('Theta4(deg)')
grid on

subplot(235);
plot(time,th5g,'c','LineWidth',2)
hold on
plot(time,th5max,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',2)
plot(time,th5min,'--s','LineWidth',2,'MarkerEdgeColor','r', 'MarkerSize',2)
 
xlabel('Time(seg)')
ylabel('Theta5(deg)')
grid on

subplot(236);
plot(time,th6g,'c','LineWidth',2)
hold on
plot(time,th6max,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor','r', 'MarkerSize',2)
plot(time,th6min,'--s','LineWidth',2,'MarkerEdgeColor','r','MarkerSize',2)

xlabel('Time(seg)')
ylabel('Theta6(deg)')
grid on
