clear all
close all

Datos_opt = csvread('plat_smooth_3.csv');
Desp_X= Datos_opt(:,6);
Desp_Y= Datos_opt(:,7);
ZerX = Desp_X(1);
ZerY = Desp_Y(1);
Desp_X = Desp_X - ZerX;
Desp_Y = -(Desp_Y - ZerY);

%% Prueba1

Desp_x_1= Datos_opt(300:7000,6);
Desp_y_1= Datos_opt(300:7000,7);

Desp_y_1 = Desp_y_1 * 1000;
Desp_x_1 = Desp_x_1 * 1000;

Zero_x = Desp_x_1(1);
Zero_y = Desp_y_1(1);

Desp_x_1 = - Zero_x + Desp_x_1;
Desp_y_1 = Zero_y - Desp_y_1;

%% Prueba 2 

Desp_x_2= Datos_opt(6895:13595,6);
Desp_y_2= Datos_opt(6895:13595,7);

Desp_y_2 = Desp_y_2 * 1000;
Desp_x_2 = Desp_x_2 * 1000;


Desp_x_2 = - Zero_x + Desp_x_2;
Desp_y_2 = Zero_y - Desp_y_2;

%% Prueba 3 

Desp_x_3= Datos_opt(13480:20180,6);
Desp_y_3= Datos_opt(13480:20180,7);

Desp_y_3 = Desp_y_3 * 1000;
Desp_x_3 = Desp_x_3 * 1000;

Desp_x_3 = - Zero_x + Desp_x_3;
Desp_y_3 = Zero_y - Desp_y_3;

%% Prueba 4 

Desp_x_4= Datos_opt(20070:26770,6);
Desp_y_4= Datos_opt(20070:26770,7);

Desp_y_4 = Desp_y_4 * 1000;
Desp_x_4 = Desp_x_4 * 1000;

Desp_x_4 = - Zero_x + Desp_x_4;
Desp_y_4 = Zero_y - Desp_y_4;

%% Prueba 5 

Desp_x_5= Datos_opt(26640:33340,6);
Desp_y_5= Datos_opt(26640:33340,7);

Desp_y_5 = Desp_y_5 * 1000;
Desp_x_5 = Desp_x_5 * 1000;

Desp_x_5 = - Zero_x + Desp_x_5;
Desp_y_5 = Zero_y - Desp_y_5;

%% Prueba 6 

Desp_x_6= Datos_opt(33240:39940,6);
Desp_y_6= Datos_opt(33240:39940,7);

Desp_y_6 = Desp_y_6 * 1000;
Desp_x_6 = Desp_x_6 * 1000;

Desp_x_6 = - Zero_x + Desp_x_6;
Desp_y_6 = Zero_y - Desp_y_6;

%% Prueba 7 

Desp_x_7= Datos_opt(39830:46530,6);
Desp_y_7= Datos_opt(39830:46530,7);

Desp_y_7 = Desp_y_7 * 1000;
Desp_x_7 = Desp_x_7 * 1000;

Desp_x_7 = - Zero_x + Desp_x_7;
Desp_y_7 = Zero_y - Desp_y_7;

%% Prueba 8 

Desp_x_8= Datos_opt(46420:53120,6);
Desp_y_8= Datos_opt(46420:53120,7);

Desp_y_8 = Desp_y_8 * 1000;
Desp_x_8 = Desp_x_8 * 1000;

Desp_x_8 = - Zero_x + Desp_x_8;
Desp_y_8 = Zero_y - Desp_y_8;

%% Prueba 9

Desp_x_9= Datos_opt(52960:59660,6);
Desp_y_9= Datos_opt(52960:59660,7);

Desp_y_9 = Desp_y_9 * 1000;
Desp_x_9 = Desp_x_9 * 1000;

Desp_x_9 = - Zero_x + Desp_x_9;
Desp_y_9 = Zero_y - Desp_y_9;

%% Prueba 10

Desp_x_10= Datos_opt(59540:66240,6);
Desp_y_10= Datos_opt(59540:66240,7);

Desp_y_10 = Desp_y_10 * 1000;
Desp_x_10 = Desp_x_10 * 1000;

Desp_x_10 = - Zero_x + Desp_x_10;
Desp_y_10 = Zero_y - Desp_y_10;

%% Matriz

Matriz_datos_desp_x = [Desp_x_1 Desp_x_2 Desp_x_3 Desp_x_4 Desp_x_5 Desp_x_6 Desp_x_7 Desp_x_8 Desp_x_9 Desp_x_10];
Matriz_datos_desp_y = [Desp_y_1 Desp_y_2 Desp_y_3 Desp_y_4 Desp_y_5 Desp_y_6 Desp_y_7 Desp_y_8 Desp_y_9 Desp_y_10];

%% Promedio
Promedio_datos_desp_x = mean(Matriz_datos_desp_x,2);
Promedio_datos_desp_y = mean(Matriz_datos_desp_y,2);

%% Desviacion 
Desviacion_std_error_desp_x = std(Matriz_datos_desp_x,0,2)/sqrt(10);
Desviacion_std_error_desp_y = std(Matriz_datos_desp_y,0,2)/sqrt(10);

%% Cuadrado2D

zeros=linspace(0,0,10);
r=linspace(0,1.7,10);
e=linspace(0,1.7,10);
w=zeros+1.7;
q=zeros+1.7;

%% Error intervalos

 J_x = 0; % Valor inicial del intervalo
 u_x = 0.1; % Paso entre intervalos
 l_x = 1; 
 
 J_y = 0; % Valor inicial del intervalo
 u_y = 0.1; % Paso entre intervalos
 l_y = 1; 


% X constante
 for i=1:1:600
 if  (Promedio_datos_desp_y(i) >= J_y)
     subindice_Y(l_y) = i;
     J_y = J_y + u_y;
     l_y = l_y + 1;
 end 
 end
 
 % Y constante
 for i=800:1:1340
 if (Promedio_datos_desp_x(i) >= J_x)
     subindice_X(l_x) = i;
     J_x = J_x + u_x;
     l_x = l_x + 1;
 end
 end
  
 % X constante
 for i=1400:1:2800
 if  (Promedio_datos_desp_y(i) <= J_y)
     subindice_Y(l_y) = i;
     J_y = J_y - u_y;
     l_y = l_y + 1;
 end 
 end
 
  % Y constante
 for i=2800:1:5800
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
plot(Desp_x_1,Desp_y_1);
plot(Desp_x_2,Desp_y_2);
plot(Desp_x_3,Desp_y_3);
plot(Desp_x_4,Desp_y_4);
plot(Desp_x_5,Desp_y_5);
plot(Desp_x_6,Desp_y_6);
plot(Desp_x_7,Desp_y_7);
plot(Desp_x_8,Desp_y_8);
plot(Desp_x_9,Desp_y_9);
plot(Desp_x_10,Desp_y_10);



figure(2)
hold on;
grid on;
axis equal
plot(Promedio_datos_desp_x,Promedio_datos_desp_y,'b','LineWidth',1);
errorbar(Prom_desp_x_X,Prom_desp_y_X,Prom_err_X,'+r','LineStyle', 'none');
plot(zeros,r,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
plot(e,zeros,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
plot(w,r,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
plot(e,q,'-s','Color',[0.4660, 0.6740, 0.1880],'LineWidth',1);
h=herrorbar(Prom_desp_x_Y,Prom_desp_y_Y,Prom_err_Y);
set(h(2), 'linestyle', 'none');
set(h(1), 'color','r');newline = char(10);
leg  = ['Average Trajectory' newline 'Obtained'];
leg2 = ['Average Trajectory' newline 'Measure'];
legend({leg,'Standard Error',leg2'},'Position',[0.4 0.42 0.1 0.2],'Interpreter','latex','FontSize',12);
str = {'For 10 times'};
text(4.5,1.5,str,'Interpreter','latex','FontSize',14)
title('Displacement X vs Y Platform','Interpreter','latex','FontSize',16)
xlabel('Displacement X (mm)','Interpreter','latex','FontSize',16')
ylabel('Displacement Y (mm)','Interpreter','latex','FontSize',16)
print('-depsc','platform_square.eps')


% figure(3)
% hold on;
% grid on;
% axis equal
% plot(Promedio_datos_desp_x,Promedio_datos_desp_y);
% errorbar(Promedio_datos_desp_x,Promedio_datos_desp_y,Desviacion_std_error_desp_x,'r','LineStyle', 'none');
% h=herrorbar(Promedio_datos_desp_x,Promedio_datos_desp_y,Desviacion_std_error_desp_y);
% set(h(2), 'linestyle', 'none');
% set(h(1), 'color','r');

% figure(4)
% hold on;
% grid on;
% plot(Desp_y_1);
% plot(Desp_y_2);
% plot(Desp_y_3);
% plot(Desp_y_4);
% plot(Desp_y_5);
% plot(Desp_y_6);
% plot(Desp_y_7);
% plot(Desp_y_8);
% plot(Desp_y_9);
% plot(Desp_y_10);
% legend ('1','2','3','4','5','6','7','8','9','10');

% figure(5)
% hold on;
% grid on;
% plot(Desp_Y);


