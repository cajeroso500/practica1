% Parámetros del sistema
m1 = 0.2;   % kg
l1 = 0.3;   % m
m2 = 0.1;   % kg
l2 = 0.25;  % m
g = 9.81;   % m/s^2

% Condiciones iniciales
theta1_0 = 0;       % rad
theta2_0 = deg2rad(35); % convertir grados a radianes
theta1_dot_0 = 0;   % rad/s
theta2_dot_0 = 0;   % rad/s

% Crear la función de transferencia del sistema
numerator1 = [(m1 + m2) * l1, m2 * l2, 0, 0];
denominator1 = [(m1 + m2), 0, -m2 * l2 * g, 0];
numerator2 = [l2, l1, 0, 0];
denominator2 = [0, 0, -l1 * g, 0];

sys1 = tf(numerator1, denominator1);
sys2 = tf(numerator2, denominator2);

% Crear el modelo en espacio de estados
A = [0, 0, 1, 0; 0, 0, 0, 1; -m2 * l2 * g / (m1 + m2), m2 * g / (m1 + m2), 0, 0; l1 * g / (m1 + m2), -l1 * g / (m1 + m2), 0, 0];
B = [0; 0; 1 / (m1 + m2); -1 / (m1 + m2)];
C = eye(4);
D = zeros(4, 1);

sys_ss = ss(A, B, C, D);

% Condiciones iniciales
x0 = [theta1_0; theta2_0; theta1_dot_0; theta2_dot_0];

% Simulación
tspan = 0:0.01:10; % tiempo de simulación de 0 a 10 segundos
[time, state] = ode45(@(t, x) A*x + B*0, tspan, x0);

% Plotear resultados
figure;

subplot(2, 1, 1);
plot(time, rad2deg(state(:, 1)), 'r', time, rad2deg(state(:, 2)), 'b');
title('Ángulos vs Tiempo');
xlabel('Tiempo (s)');
ylabel('Ángulo (grados)');
legend('\theta_1', '\theta_2');

subplot(2, 1, 2);
plot(time, rad2deg(state(:, 3)), 'r', time, rad2deg(state(:, 4)), 'b');
title('Velocidades Angulares vs Tiempo');
xlabel('Tiempo (s)');
ylabel('Velocidad Angular (grados/s)');
legend('\omega_1', '\omega_2');
