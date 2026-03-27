%% D2 final plots
clear; close all; clc;

% Joint trajectories

DOFs = 5;

q0  = [0.0,    2.89,   -2.89,   0.0,     0.0];
qtg = [0.3530, 1.7217, -2.6405, 0.9188,  1.5708];
qtr = [1.5458, 1.1833, -0.9807, -1.7734, 3.1166];
qtv = [0.7484, 2.0075, -1.3783, -0.6292, 1.5708];

Ts = 0.03; % s, tick efectivo = 10 ms * EXTRA_PRESCALER_XMS(3)
names = {'q1','q2','q3','q4','q5'};

%% Part 1a
T1a = 2.0; % s
N1a = max(2, floor(T1a/Ts));
k = (0:N1a-1)';

beta = (qtg - q0)/(N1a^3);

q_1a = zeros(N1a, DOFs);
for i = 1:DOFs
    q_1a(:,i) = q0(i) + beta(i).*((3*N1a - 2*k).*(k.^2));
end
t_1a = k*Ts;

figure;
plot(t_1a, q_1a, 'LineWidth', 1.5);
grid on;
xlabel('t (s)');
ylabel('q (rad)');
title('Part 1a - CUBIC1 with real execution timing');
legend(names, 'Location', 'best');

%% Part 1b
T1 = 1.5;
T2 = 1.5;
N1 = max(2, floor(T1/Ts));
N2 = max(2, floor(T2/Ts));
N = N1 + N2;

A2 = zeros(1,DOFs); A3 = zeros(1,DOFs);
B2 = zeros(1,DOFs); B3 = zeros(1,DOFs);

for i = 1:DOFs
    fBeta1 = 2*(qtv(i)-qtg(i))/N1 + 2*(qtv(i)-qtr(i))/N2;
    fBeta2 = (qtv(i)-qtg(i))/(2*N1^2) - (qtv(i)-qtr(i))/(2*N2^2);

    A2(i) = 1.5*(fBeta1 + 2*fBeta2*N2)/N;
    B2(i) = 1.5*(fBeta1 - 2*fBeta2*N1)/N;

    A3(i) = -(fBeta1 + fBeta2*N2)/(N*N1);
    B3(i) = -(fBeta1 - fBeta2*N1)/(N*N2);
end

q_1b = zeros(N, DOFs);
for i = 1:DOFs
    for kk = 1:N
        if kk <= N1
            q_1b(kk,i) = qtg(i) + A2(i)*(kk^2) + A3(i)*(kk^3);
        else
            q_1b(kk,i) = qtr(i) + B2(i)*((N-kk)^2) + B3(i)*((N-kk)^3);
        end
    end
end
t_1b = (0:N-1)'*Ts;

figure;
plot(t_1b, q_1b, 'LineWidth', 1.5);
grid on;
xlabel('t (s)');
ylabel('q (rad)');
title('Part 1b - CUBIC2 with real execution timing');
legend(names, 'Location', 'best');
xline(N1*Ts, '--', 'Waypoint t_v', 'LabelVerticalAlignment', 'bottom');

fprintf('\nWaypoint check at k=N1:\n');
disp(table(names', qtv(:), q_1b(N1,:)', abs(qtv(:)-q_1b(N1,:)'), ...
    'VariableNames', {'Joint','qtv_target','q_at_tick_N1','abs_error'}));