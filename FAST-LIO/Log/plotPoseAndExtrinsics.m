d = readmatrix('scan_states.txt');
p = readmatrix('mat_pre.txt');
d(:, 1) = d(:, 1) - d(1, 1);

close all;
figure;
plot3(d(:, 2), d(:, 3), d(:, 4), 'g');

plot3(p(:, 5), p(:, 6), p(:, 7), 'r');
axis equal; grid on;
zlabel('z');
xlabel('x');
ylabel('y');

figure
i = 1+3+4+3+6+3;
plot(d(:, 1), d(:, i + 1) * 100); hold on;
plot(d(:, 1), d(:, i + 2) * 100);
plot(d(:, 1), d(:, i + 3) * 100);
ylabel('cm');
legend('x', 'y', 'z');
title('T_LI');

figure
s = 180 / pi;
plot(d(:, 1), d(:, i+4) * s); hold on;
plot(d(:, 1), d(:, i+5) * s);
plot(d(:, 1), d(:, i+6) * s);
ylabel('deg');
title('R_LI');
legend('qx', 'qy', 'qz');

