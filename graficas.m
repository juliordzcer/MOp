clc
close all
clear all

% Definir los parámetros del cilindro
rcil = 1; % Radio del cilindro
lcil = 5; % Altura del cilindro

[Xc, Yc, Zc] = cylinder(rcil, 50);
Zc = Zc * lcil; % Corregir la altura del cilindro

% Graficar el cilindro
figure;
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

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Cilindro con cuadrado en la parte superior (con relleno)');
axis equal;
grid on;
