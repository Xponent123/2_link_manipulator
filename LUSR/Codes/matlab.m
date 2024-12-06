clearvars;
close all;

t_span = [0 20];
q0 = [0.1; 0.1];
qd0 = [0; 0];
x0 = [0; 0];
qd = [0; 0];
ode_func = @(t, s) dyn(t, s, qd);
[t, s] = ode45(ode_func, t_span, [q0; qd0; x0]);
q1 = s(:, 1);
q2 = s(:, 2);
q1d = s(:, 3);
q2d = s(:, 4);
x1 = s(:, 5);
x2 = s(:, 6);

figure; 
subplot(2, 1, 1);
plot(t,q1);
title("q1");
xlabel("t");
ylabel("q1(t)");

subplot(2, 1, 2);
plot(t,q2);
title("q2");
xlabel("t");
ylabel("q2(t)");


function func = dyn(t, s, qd)
    q1 = s(1);
    q2 = s(2);
    q1d = s(3);
    q2d = s(4);
    x1 = s(5);
    x2 = s(6);
    
    Kp1 = 50; Kd1 = 0; Ki1 = 5;
    Kp2 = 50; Kd2 = 0; Ki2 = 5;

    e1 = qd(1) - q1;
    e2 = qd(2) - q2;

    f1 = Kp1 * e1 + Ki1 * x1 - Kd1 * q1d;
    f2 = Kp2 * e2 + Ki2 * x2 - Kd2 * q2d;
    F = [f1; f2];

    m1 = 10; m2 = 5;
    l1 = 0.2; l2 = 0.1;

    M11 = (m1 + m2) * (l1^2) + m2 * l2 * (l2 + 2 * l1 * cos(q2));
    M12 = m2 * l2 * (l2 + l1 * cos(q2));
    M22 = m2 * (l2^2);
    M = [M11, M12; M12, M22];

    tau = M * F;

    func = zeros(6, 1);
    func(1) = q1d;
    func(2) = q2d;
    func(3) = - (5 * (tau(1) - (981 * cos(q1 + q2)) / 200 - (2943 * cos(q1)) / 100 + (q1d * q2d * sin(q2)) / 10 + (q2d * sin(q2) * (q1d + q2d)) / 10)) / (cos(q2)^2 - 3) ...
                  - (5 * (2 * cos(q2) + 1) * ((sin(q2) * q2d^2) / 10 - tau(2) + (981 * cos(q1 + q2)) / 200)) / (cos(q2)^2 - 3);
    func(4) = (5 * (2 * cos(q2) + 1) * (tau(1) - (981 * cos(q1 + q2)) / 200 - (2943 * cos(q1)) / 100 + (q1d * q2d * sin(q2)) / 10 + (q2d * sin(q2) * (q1d + q2d)) / 10)) / (cos(q2)^2 - 3) ...
               + (5 * (4 * cos(q2) + 13) * ((sin(q2) * q2d^2) / 10 - tau(2) + (981 * cos(q1 + q2)) / 200)) / (cos(q2)^2 - 3);
    func(5) = e1;
    func(6) = e2;
end
