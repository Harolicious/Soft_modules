clear all
close all

a=7;
b=8;


Datos_opt = csvread('yz_plat.csv');
Desp_X = -Datos_opt(:,a);
Desp_Y = -Datos_opt(:,b);

ZerX = Desp_X(1);
ZerY = Desp_Y(1);
Desp_X = Desp_X - ZerX;
Desp_Y = -(Desp_Y - ZerY);

%% Prueba1

Desp_x_1= -Datos_opt(1:382,a);
Desp_y_1= -Datos_opt(1:382,b);

Desp_y_1 = Desp_y_1 * 1000;
Desp_x_1 = Desp_x_1 * 1000;

Zero_x = Desp_x_1(1);
Zero_y = Desp_y_1(1);

Desp_x_1 = - Zero_x + Desp_x_1;
Desp_y_1 = Zero_y - Desp_y_1;

%% Prueba 2 

Desp_x_2= -Datos_opt(383:764,a);
Desp_y_2= -Datos_opt(383:764,b);

Desp_y_2 = Desp_y_2 * 1000;
Desp_x_2 = Desp_x_2 * 1000;


Desp_x_2 = - Zero_x + Desp_x_2;
Desp_y_2 = Zero_y - Desp_y_2;

%% Prueba 3 

Desp_x_3= -Datos_opt(765:1146,a);
Desp_y_3= -Datos_opt(765:1146,b);

Desp_y_3 = Desp_y_3 * 1000;
Desp_x_3 = Desp_x_3 * 1000;

Desp_x_3 = - Zero_x + Desp_x_3;
Desp_y_3 = Zero_y - Desp_y_3;

%% Prueba 4 

Desp_x_4= -Datos_opt(1147:1528,a);
Desp_y_4= -Datos_opt(1147:1528,b);

Desp_y_4 = Desp_y_4 * 1000;
Desp_x_4 = Desp_x_4 * 1000;

Desp_x_4 = - Zero_x + Desp_x_4;
Desp_y_4 = Zero_y - Desp_y_4;

%% Prueba 5 

Desp_x_5= -Datos_opt(1529:1910,a);
Desp_y_5= -Datos_opt(1529:1910,b);

Desp_y_5 = Desp_y_5 * 1000;
Desp_x_5 = Desp_x_5 * 1000;

Desp_x_5 = - Zero_x + Desp_x_5;
Desp_y_5 = Zero_y - Desp_y_5;

%% Prueba 6 

Desp_x_6= -Datos_opt(1911:2292,a);
Desp_y_6= -Datos_opt(1911:2292,b);

Desp_y_6 = Desp_y_6 * 1000;
Desp_x_6 = Desp_x_6 * 1000;

Desp_x_6 = - Zero_x + Desp_x_6;
Desp_y_6 = Zero_y - Desp_y_6;

%% Prueba 7 

Desp_x_7= -Datos_opt(2293:2674,a);
Desp_y_7= -Datos_opt(2293:2674,b);

Desp_y_7 = Desp_y_7 * 1000;
Desp_x_7 = Desp_x_7 * 1000;

Desp_x_7 = - Zero_x + Desp_x_7;
Desp_y_7 = Zero_y - Desp_y_7;

%% Prueba 8 

Desp_x_8= -Datos_opt(2675:3056,a);
Desp_y_8= -Datos_opt(2675:3056,b);

Desp_y_8 = Desp_y_8 * 1000;
Desp_x_8 = Desp_x_8 * 1000;

Desp_x_8 = - Zero_x + Desp_x_8;
Desp_y_8 = Zero_y - Desp_y_8;

%% Prueba 9

Desp_x_9= -Datos_opt(3057:3438,a);
Desp_y_9= -Datos_opt(3057:3438,b);

Desp_y_9 = Desp_y_9 * 1000;
Desp_x_9 = Desp_x_9 * 1000;

Desp_x_9 = - Zero_x + Desp_x_9;
Desp_y_9 = Zero_y - Desp_y_9;

%% Prueba 10

Desp_x_10= -Datos_opt(3439:3820,a);
Desp_y_10= -Datos_opt(3439:3820,b);

Desp_y_10 = Desp_y_10 * 1000;
Desp_x_10 = Desp_x_10 * 1000;

Desp_x_10 = - Zero_x + Desp_x_10;
Desp_y_10 = Zero_y - Desp_y_10;

%% Matriz

Matriz_datos_desp_x = [Desp_x_2 Desp_x_3 Desp_x_4 Desp_x_5 Desp_x_6 Desp_x_7 Desp_x_8 Desp_x_9 Desp_x_10];
Matriz_datos_desp_y = [Desp_y_2 Desp_y_3 Desp_y_4 Desp_y_5 Desp_y_6 Desp_y_7 Desp_y_8 Desp_y_9 Desp_y_10];

%% Promedio
Promedio_datos_desp_x = mean(Matriz_datos_desp_x,2);
Promedio_datos_desp_y = mean(Matriz_datos_desp_y,2);

%% Desviacion 
Desviacion_std_error_desp_x = std(Matriz_datos_desp_x,0,2)/sqrt(9);
Desviacion_std_error_desp_y = std(Matriz_datos_desp_y,0,2)/sqrt(9);

%% Cuadrado2D
zeros=linspace(0,0,10);
r=linspace(0,3.15,10);
e=linspace(0,2,10);
w=zeros+2;
q=zeros+3.15;

%% Error intervalos

 J_x = 0.5; % Valor inicial del intervalo
 u_x = 0.1; % Paso entre intervalos
 l_x = 1; 
 
 J_y = 2; % Valor inicial del intervalo
 u_y = 0.1; % Paso entre intervalos
 l_y = 1; 


% X constante
 for i=96:1:191
 if  (Promedio_datos_desp_y(i) >= J_y)
     subindice_Y(l_y) = i;
     J_y = J_y + u_y;
     l_y = l_y + 1;
 end 
 end
 
 % Y constante
 for i=1:1:96
 if (Promedio_datos_desp_x(i) >= J_x)
     subindice_X(l_x) = i;
     J_x = J_x + u_x;
     l_x = l_x + 1;
 end
 end
  
 % X constante
 for i=288:1:382
 if  (Promedio_datos_desp_y(i) <= J_y)
     subindice_Y(l_y) = i;
     J_y = J_y - u_y;
     l_y = l_y + 1;
 end 
 end
 
  % Y constante
 
 for i=192:1:287
 if (Promedio_datos_desp_x(i) <= J_x)
     subindice_X(l_x) = i;
     J_x = J_x - u_x;
     l_x = l_x + 1;
 end
 end
 
  for i = 1:1:length(subindice_X)
      j = subindice_X(i);
        Prom_desp_x_X(i) = Promedio_datos_desp_x(j);
        Prom_desp_y_X(i) = Promedio_datos_desp_y(j);
        Prom_err_X(i) = Desviacion_std_error_desp_x(j);
  end

  for i = 1:1:length(subindice_Y)
      j = subindice_Y(i);
        Prom_desp_x_Y(i) = Promedio_datos_desp_x(j);
        Prom_desp_y_Y(i) = Promedio_datos_desp_y(j);
        Prom_err_Y(i) = Desviacion_std_error_desp_y(j);
  end
  


%% Figure

figure(1)
hold on;
grid on;
axis equal
plot(Desp_x_2,Desp_y_2);
plot(Desp_x_3,Desp_y_3);
plot(Desp_x_4,Desp_y_4);
plot(Desp_x_5,Desp_y_5);
plot(Desp_x_6,Desp_y_6);
plot(Desp_x_7,Desp_y_7);
plot(Desp_x_8,Desp_y_8);
plot(Desp_x_9,Desp_y_9);

newline = char(10);
leg  = ['Average Trajectory', newline, 'Obtained'];
leg2 = ['Average Trajectory', newline, 'Measure'];

figure(2)
hold on;
grid on;
axis equal
plot(Promedio_datos_desp_x,Promedio_datos_desp_y,'b');
errorbar(Prom_desp_x_X,Prom_desp_y_X,Prom_err_X,'+r','LineStyle', 'none');
% plot(zeros,r,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
% plot(e,zeros,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
% plot(w,r,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
% plot(e,q,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
h=herrorbar(Prom_desp_x_Y,Prom_desp_y_Y,Prom_err_Y);
set(h(2), 'linestyle', 'none');
set(h(1), 'color','r');
% legend({leg,'Standard Error',leg2'},'Location','northeast','Interpreter','latex','FontSize',14)
str = {'For 10 times'};
%text(3,0.25,str,'Interpreter','latex','FontSize',14)
title('Displacement X vs Z Platform','Interpreter','latex','FontSize',16)
xlabel('Displacement X (mm)','Interpreter','latex','FontSize',16')
ylabel('Displacement Z (mm)','Interpreter','latex','FontSize',16)
print('-depsc','platform_square_xz_v2.eps')
