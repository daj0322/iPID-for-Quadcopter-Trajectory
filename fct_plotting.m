function fct_plotting(TT,XX,xd_vec,yd_vec,zd_vec)
figure(1)
subplot(4,4,1)
plot(TT,XX(:,1))
title('Load X Position')
xlabel('Time(s)')
ylabel('x (m)')

subplot(4,4,2)
plot(TT,XX(:,2))
title('Load Y Position')
xlabel('Time(s)')
ylabel('y (m)')

subplot(4,4,3)
plot(TT,XX(:,3))
title('Load Z Position')
xlabel('Time(s)')
ylabel('z (m)')

subplot(4,4,4)
plot(TT,XX(:,4))
title('\phi')
xlabel('Time(s)')
ylabel('\phi (rad)')

subplot(4,4,5)
plot(TT,XX(:,5))
title('\theta')
xlabel('Time(s)')
ylabel('\theta (rad)')

subplot(4,4,6)
plot(TT,XX(:,6))
title('\psi')
xlabel('Time(s)')
ylabel('\psi (rad)')

subplot(4,4,7)
plot(TT,XX(:,7))
title('\alpha')
xlabel('Time(s)')
ylabel('\alpha (rad)')

subplot(4,4,8)
plot(TT,XX(:,8))
title('\beta')
xlabel('Time(s)')
ylabel('\beta (rad)')
ylim([-0.3 0.3])

subplot(4,4,9)
plot(TT,XX(:,9))
title('Load X Velocity')
xlabel('Time(s)')
ylabel('vx (m/s)')

subplot(4,4,10)
plot(TT,XX(:,10))
title('Load Y Velocity')
xlabel('Time(s)')
ylabel('vy (m/s)')

subplot(4,4,11)
plot(TT,XX(:,11))
title('Load Z Velocity')
xlabel('Time(s)')
ylabel('vz (m/s)')

subplot(4,4,12)
plot(TT,XX(:,12))
title('Angular velocity \phi_{dot}')
xlabel('Time(s)')
ylabel('\phi_{dot} (rad/s)')

subplot(4,4,13)
plot(TT,XX(:,13))
title('Angular velocity \theta_{dot}')
xlabel('Time(s)')
ylabel('\theta_{dot} (rad/s)')

subplot(4,4,14)
plot(TT,XX(:,14))
title('Angular velocity \psi_{dot}')
xlabel('Time(s)')
ylabel('\psi_{dot} (rad/s)')

subplot(4,4,15)
plot(TT,XX(:,15))
title('Load angular velocity \alpha_{dot}')
xlabel('Time(s)')
ylabel('\alpha_{dot} (rad/s)')

subplot(4,4,16)
plot(TT,XX(:,16))
title('Load angular velocity \beta_{dot}')
xlabel('Time(s)')
ylabel('\beta_{dot} (rad/s)')




figure(10)
subplot(221)
plot(TT,XX(:,7)*180/pi)
title('\alpha')
xlabel('Time(s)')
ylabel('\alpha (deg)')
ylim([-30 30])
grid on;

subplot(223)
plot(TT,XX(:,15)*180/pi)
title('Load angular velocity \alpha_{dot}')
xlabel('Time(s)')
ylabel('\alpha_{dot} (deg/s)')
ylim([-60 60])
grid on;

subplot(222)
plot(TT,XX(:,8)*180/pi)
title('\beta')
xlabel('Time(s)')
ylabel('\beta (deg)')
ylim([-30 30])
grid on;

subplot(224)
plot(TT,XX(:,16)*180/pi)
title('Load angular velocity \beta_{dot}')
xlabel('Time(s)')
ylabel('\beta_{dot} (deg/s)')
ylim([-60 60])
grid on;

figure(11); hold on;
plot3(XX(:,1),XX(:,2),XX(:,3))
grid on;
axis on;
axis([-1 11 -1 6 -1 11])
view([100 40])
scatter3(XX(1,1), XX(1,2), XX(1,3), 'or','MarkerFaceColor', 'r');
scatter3(xd_vec(end), yd_vec(end), zd_vec(end), 'or','MarkerFaceColor', 'r');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

plot3(xd_vec,yd_vec,zd_vec, 'r--')

end
