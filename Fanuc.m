clc; clear all; close all

%Tiempo
T = 10;
np = 50;

% Localizacion de la estacion de trabajo
ex = 0; ey = 0; ez = 0;

%Parametros de emplazamiento de tarea
tx = 804.5; ty = 499.9; tz = 663.1;%mm

% Parametros del robot
d2 = 150; d3 = 600; d4 = 200; r4 = 640;

bR = 450;
eo = 100;

% Parametros del organo terminal
rho = -22; dhx = 49.8; dhy = 0; dhz = 404;

% Limites articulares
th1min =-170; th1max = 170;
th2min =-90;  th2max = 160; 
th3min =-180; th3max = 267;
th4min =-190; th4max = 190;
th5min =-270; th5max = 270;
th6min =-360; th6max = 360;

% Velocidades maximas
th1pmax = 230; th2pmax = 225; th3pmax = 230;
th4pmax = 430; th5pmax = 430; th6pmax = 630;

%Parametros del movimiento del robot:
%solucion del mip del robot:
%Epsilon
% e1=1;  e2=1;  e4=-1;

% Matriz del marco 6 al marco de la herramienta

T6h=[sind(rho)  0   -sind(rho)    dhx;
        0       1       0         dhy;
     cosd(rho)  0    cosd(rho)    dhz;
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
delz=0;

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
    xph = x_center + radius * cos(2*pi*funct);
    yph = y_center + radius * sin(2*pi*funct);
    zph = zinih;

    % Cálculo de las variaciones en las coordenadas x, y, z y en los ángulos de orientación alpha, beta y gamma
    delx = 0;%(2*pi*radius/T) * (-sin(2*pi*t/T));
    dely = 0;%(2*pi*radius/T) * (cos(2*pi*t/T));
    delz = 0; % No hay variación en la coordenada z
    delalpha = 0; 
    delbeta = 0;  
    delgamma = 0;

    
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

    % % Calculo de las variables articulares:
    % th1 = atan2(e1 * PY,e1 * PX);
    % 
    % z1 = -d2 + PX * cos(th1) + PY * sin(th1);
    % b1 = 2 * (-(d4 * PZ) - r4 * z1);
    % b2 = 2 * (PZ *r4 - d4 * z1);
    % b3 = d3^2 - d4^2 - PZ^2 - r4^2 - z1^2;
    % 
    % SQ = (b1 * b3 + b2 * sqrt(b1^2 + b2^2 - b3^2)*e2)/(b1^2 + b2^2);
    % CQ = (b2 * b3 - b1 * sqrt(b1^2 + b2^2 - b3^2)*e2)/(b1^2 + b2^2);
    % 
    % th2 = atan2(-((-PZ-r4 * CQ + d4 * SQ)/(d3)),(z1 - d4 * CQ - r4 * SQ)/(d3));
    % 
    % th3 = atan2(SQ,CQ)-th2;
    % 
    % X = -(AY * cos(th1)) + AX * sin(th1);
    % Y = -(AX * cos(th1) * cos(th2 + th3)) - AY * cos(th2 + th3) * sin(th1) - AZ * sin(th2 + th3);
    % 
    % th4 = atan2(-X * e4,Y * e4);
    % 
    % Y12 = -(cos(th4) * (AX * cos(th1) * cos(th2 + th3) + AY * cos(th2 + th3) * sin(th1) + AZ * sin(th2 + th3))) - (- (AY * cos(th1) + AX * sin(th1))*sin(th4));
    % Y1 = - AZ*cos(th2+th3) - AX*cos(th1)*sin(th2+th3)-AY*sin(th1)*sin(th2 + th3);
    % 
    % th5 = atan2(-Y12,-Y1);
    % 
    % Y22 = - (cos(th4)*(-SY * cos(th1) + SX * sin(th1))) + (SX*cos(th1)*cos(th2+th3) + SY*cos(th2+th3)*sin(th1)+SZ*sin(th2+th3))*sin(th4);
    % Y21 = - (cos(th4)*(-NY * cos(th1) + NX * sin(th1))) + (NX*cos(th1)*cos(th2+th3) + NY*cos(th2+th3)*sin(th1)+NZ*sin(th2+th3))*sin(th4);
    % 
    % th6 = atan2(-Y22,-Y21);


    % Inverse Geometric Model for frame 6
    
    % Parámetros geométricos
    % j       ant     sigma   mu      gamma   b       alpha   d       theta   r       
    % 1       0       0       1       0       0       0       0       th1     0       
    % 2       1       0       1       0       0       pi/2    d2      th2     0       
    % 3       2       0       1       0       0       0       d3      th3     0       
    % 4       3       0       1       0       0       pi/2    d4      th4     r4      
    % 5       4       0       1       0       0       -pi/2   0       th5     0       
    % 6       5       0       1       0       0       pi/2    0       th6     0       
    
    S1 = SX;
    S2 = SY;
    S3 = SZ;
    
    N1 = NX;
    N2 = NY;
    N3 = NZ;
    
    A1 = AX;
    A2 = AY;
    A3 = AZ;

    P1 = PX;
    P2 = PY;
    P3 = PZ;

    % Ecuaciones:
    % Solving type 2
    % X*sin(th1) + Y*cos(th1) = Z

    YPSth1 = 0 ;%[1, 0];

    th1 = pi*YPSth1 + atan2(-P2, -P1);
    
    % Solving type 6, 7
    % V*cos(th2) + W*sin(th2) = X*cos(th3) + Y*sin(th3) + Z1
    % eps*(V*sin(th2) - W*cos(th2)) = X*sin(th3) - Y*cos(th3) + Z2
    Vth3 = -P1*cos(th1) - P2*sin(th1) + d2;
    B1th3 = 2*d3*r4;
    B2th3 = 2*d3*d4;
    B3th3 = P3^2 + Vth3^2 - d3^2 - d4^2 - r4^2;
    Bth3 = B1th3^2 + B2th3^2;
    Dth3 = -B3th3^2 + Bth3;

    YPSth3 = 1; %[1, -1];

    Sth3 = (B1th3*B3th3 + B2th3*sqrt(Dth3)*YPSth3)/Bth3;
    Cth3 = (-B1th3*sqrt(Dth3)*YPSth3 + B2th3*B3th3)/Bth3;
    th3 = atan2(Sth3, Cth3);
    Zi1th2 = -d3 - d4*cos(th3) - r4*sin(th3);
    Zi2th2 = -d4*sin(th3) + r4*cos(th3);
    
    % X1*sin(th2) + Y1*cos(th2) = Z1
    % X2*sin(th2) + Y2*cos(th2) = Z2
    Dth2 = P3^2 + Vth3^2;
    Cth2 = (-P3*Zi2th2 + Vth3*Zi1th2)/Dth2;
    Sth2 = (-P3*Zi1th2 - Vth3*Zi2th2)/Dth2;
    th2 = atan2(Sth2, Cth2);
    
    % Solving type 2
    % X*sin(th5) + Y*cos(th5) = Z
    Zth5 = A1*sin(th2 + th3)*cos(th1) + A2*sin(th1)*sin(th2 + th3) - A3*cos(th2 + th3);


    YPSth5 = -1; %[1, -1];
    
    
    th5 = atan2(YPSth5*sqrt(-Zth5^2 + 1), Zth5);
    
    % Solving type 3
    % X1*sin(th4) + Y1*cos(th4) = Z1
    % X2*sin(th4) + Y2*cos(th4) = Z2
    Y1th4 = sin(th5);
    Z1th4 = A1*cos(th1)*cos(th2 + th3) + A2*sin(th1)*cos(th2 + th3) + A3*sin(th2 + th3);
    Z2th4 = A1*sin(th1) - A2*cos(th1);
    th4 = atan2((Z2th4/Y1th4), (Z1th4/Y1th4));


    % Solving type 3
    % X1*sin(th6) + Y1*cos(th6) = Z1
    % X2*sin(th6) + Y2*cos(th6) = Z2
    Z1th6 = S1*sin(th2 + th3)*cos(th1) + S2*sin(th1)*sin(th2 + th3) - S3*cos(th2 + th3);
    Z2th6 = N1*sin(th2 + th3)*cos(th1) + N2*sin(th1)*sin(th2 + th3) - N3*cos(th2 + th3);
    th6 = atan2(Z2th6/Y1th4, -Z1th6/Y1th4);




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
    ss1=[0 0 -bR 1]';
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
    
end

frames = length(x1g); % Suponiendo que x1g está definido
Li = 100;
% Calcular los límites de los ejes redondeando a la centena más cercana
x_min = min([x1g x2g x3g x4g x5g x6g xhg]) - Li;
x_max = max([x1g x2g x3g x4g x5g x6g xhg]) + Li;
y_min = min([y1g y2g y3g y4g y5g y6g yhg]) - Li;
y_max = max([y1g y2g y3g y4g y5g y6g yhg]) + Li;
z_min = min([z1g z2g z3g z4g z5g z6g zhg]) - 250;
z_max = max([z1g z2g z3g z4g z5g z6g zhg]) + Li;

x_min = floor(x_min / 100) * 100;
x_max = ceil(x_max / 100) * 100;
y_min = floor(y_min / 100) * 100;
y_max = ceil(y_max / 100) * 100;
z_min = floor(z_min / 100) * 100;
z_max = ceil(z_max / 100) * 100;

% Inicializar la figura y el eje 3D
figure(1);
ax = axes('XLim', [x_min, x_max], ...
          'YLim', [y_min, y_max], ...
          'ZLim', [z_min, z_max]);
view(3);
grid on;
hold on;

% Ajustar los ticks del eje para que vayan de 100 en 100
ax.XTick = x_min:100:x_max;
ax.YTick = y_min:100:y_max;
ax.ZTick = z_min:100:z_max;

% Color amarillo de FANUC en RGB
darkerFanucYellow = [0 0 0];%[255, 204, 0] / 255 * 0.9; % Ajusta el factor según la oscuridad deseada

% Grafica de la base del robot
rcil = 100; % Radio del base
lcil = -400; % Altura del base

[Xc, Yc, Zc] = cylinder(rcil, 50);
Zc = Zc * lcil; 

% Graficar el cilindro
plot3(Xc(1,:), Yc(1,:), Zc(1,:), 'k'); hold on; % Base inferior
plot3(Xc(2,:), Yc(2,:), Zc(2,:), 'k'); % Base superior

% Graficar los lados del cilindro
for i = 1:size(Xc, 2)
    plot3([Xc(1,i), Xc(2,i)], [Yc(1,i), Yc(2,i)], [Zc(1,i), Zc(2,i)], 'k');
end

% Definir los vértices del cuadrado superior
lado = 2 * rcil; % Longitud del lado del cuadrado
Xcuadrado = [-lado/2, lado/2, lado/2, -lado/2, -lado/2];
Ycuadrado = [-lado/2, -lado/2, lado/2, lado/2, -lado/2];
Zcuadrado = lcil * ones(size(Xcuadrado));

% Rellenar el cuadrado superior
fill3(Xcuadrado, Ycuadrado, Zcuadrado, 'k', 'FaceAlpha', 0.5);
fill3(Xcuadrado, Ycuadrado, ones(size(Xcuadrado)), 'k', 'FaceAlpha', 0.5);

% Grafica del primer eslabon de la base del robot
plot3(s1x, s1y, s1z, 'k', 'MarkerSize', 5, 'linewidth', 3, 'Color', darkerFanucYellow)

% Crear las líneas iniciales para los eslabones
lines = gobjects(7, 1);
for i = 1:7
    lines(i) = plot3(nan, nan, nan, 'LineWidth', 2, 'Color', darkerFanucYellow);
end

% Crear las líneas iniciales para la trayectoria de las articulaciones
trajLines = gobjects(7, 1);
AzulS = [0.4, 0.6, 0.8]; % Un azul más suave en formato RGB
for i = 1:7
    trajLines(i) = plot3(nan, nan, nan, 'LineWidth', 2, 'Color', AzulS);
end

% Crear los círculos iniciales para las articulaciones
jointCircles = gobjects(7, 1);
for i = 1:7
    jointCircles(i) = drawCircle(nan, nan, nan, 10);
end

% Crear la animación
for frame = 1:frames
    update(lines, frame, x1g, y1g, z1g, x2g, y2g, z2g, x3g, y3g, z3g, x4g, y4g, z4g, x5g, y5g, z5g, x6g, y6g, z6g, xhg, yhg, zhg);
    update_trajectory(trajLines, frame, x1g, y1g, z1g, x2g, y2g, z2g, x3g, y3g, z3g, x4g, y4g, z4g, x5g, y5g, z5g, x6g, y6g, z6g, xhg, yhg, zhg);

    % Actualizar las posiciones de los círculos en las articulaciones
    jointCoords = [x1g(frame), y1g(frame), z1g(frame);
                   x2g(frame), y2g(frame), z2g(frame);
                   x3g(frame), y3g(frame), z3g(frame);
                   x4g(frame), y4g(frame), z4g(frame);
                   x5g(frame), y5g(frame), z5g(frame);
                   x6g(frame), y6g(frame), z6g(frame);
                   xhg(frame), yhg(frame), zhg(frame)];
    for i = 1:7
        delete(jointCircles(i)); % Eliminar el círculo anterior
        jointCircles(i) = drawCircle(jointCoords(i, 1), jointCoords(i, 2), jointCoords(i, 3), 10);
    end

    drawnow;
    pause(0.05); % Ajustar el intervalo de tiempo si es necesario
end


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





% Función para actualizar las líneas de los eslabones
function update(lines, frame, x1g, y1g, z1g, x2g, y2g, z2g, x3g, y3g, z3g, x4g, y4g, z4g, x5g, y5g, z5g, x6g, y6g, z6g, xhg, yhg, zhg)
    coordinates = [
        x1g(frame), y1g(frame), z1g(frame);
        x2g(frame), y2g(frame), z2g(frame);
        x3g(frame), y3g(frame), z3g(frame);
        x4g(frame), y4g(frame), z4g(frame);
        x5g(frame), y5g(frame), z5g(frame);
        x6g(frame), y6g(frame), z6g(frame);
        xhg(frame), yhg(frame), zhg(frame)
    ];
    
    for i = 1:length(lines)-1
        set(lines(i), 'XData', [coordinates(i, 1), coordinates(i+1, 1)], ...
                      'YData', [coordinates(i, 2), coordinates(i+1, 2)], ...
                      'ZData', [coordinates(i, 3), coordinates(i+1, 3)]);
    end
end

% Función para actualizar la trayectoria de las articulaciones
function update_trajectory(trajLines, frame, x1g, y1g, z1g, x2g, y2g, z2g, x3g, y3g, z3g, x4g, y4g, z4g, x5g, y5g, z5g, x6g, y6g, z6g, xhg, yhg, zhg)
    set(trajLines(1), 'XData', x1g(1:frame), 'YData', y1g(1:frame), 'ZData', z1g(1:frame));
    set(trajLines(2), 'XData', x2g(1:frame), 'YData', y2g(1:frame), 'ZData', z2g(1:frame));
    set(trajLines(3), 'XData', x3g(1:frame), 'YData', y3g(1:frame), 'ZData', z3g(1:frame));
    set(trajLines(4), 'XData', x4g(1:frame), 'YData', y4g(1:frame), 'ZData', z4g(1:frame));
    set(trajLines(5), 'XData', x5g(1:frame), 'YData', y5g(1:frame), 'ZData', z5g(1:frame));
    set(trajLines(6), 'XData', x6g(1:frame), 'YData', y6g(1:frame), 'ZData', z6g(1:frame));
    set(trajLines(7), 'XData', xhg(1:frame), 'YData', yhg(1:frame), 'ZData', zhg(1:frame));
end


% Función para dibujar un círculo
function h = drawCircle(x, y, z, radius)
    theta = linspace(0, 2*pi, 50);
    X = radius * cos(theta) + x;
    Y = radius * sin(theta) + y;
    Z = z * ones(size(X));
    h = fill3(X, Y, Z, 'k', 'EdgeColor', 'none');
end