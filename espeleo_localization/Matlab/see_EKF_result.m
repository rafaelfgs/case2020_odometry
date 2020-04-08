







%--------------------------------------------------------------
%Results vizualization for the espeleo_localization ROS package 
%Author: Adriano M. C. Rezende
%Date: February 2020
%--------------------------------------------------------------


%% Config

%Load some configuration parameters
config


%Logs folder
% log_dir = '../log_files';
log_dir = '../../corredor/log_files';

%World size
% ws = [-14 26 -2 34]; %
ws = [-2 30 -1 3]; % corredor


%% Read files

%Read the log file
file_name_1 = sprintf('%s/state_log.txt',log_dir);
file_name_2 = sprintf('%s/gps_position.txt',log_dir);
file_name_3 = sprintf('%s/imu_log.txt',log_dir);

% Load the files
try
    M1 = dlmread(file_name_1);
catch
    error('Error when reading log file file_name_1')
end
try
    M2 = dlmread(file_name_2);
catch
    error('Error when reading log file file_name_2')
end
try
    M3 = dlmread(file_name_3);
catch
    error('Error when reading log file file_name_3')
end


%% Extract data

%Get filter states (x, y, psi, v)
states = M1(:,1:4)';
time_states = M1(:,5)';

%Get GPS data
pos_gps = M2(:,1:2)';
time_gps = M2(:,3)';

%Get IMU data
imu_acc = M3(:,1:3)';
imu_gyro = M3(:,4:6)';
imu_quat = M3(:,7:10)';
imu_euler = [];
for k = 1:1:length(imu_quat(1,:))
    imu_euler(:,k) = flipud(quat2eul(imu_quat(:,k)')');
end
time_imu = M3(:,11)';



k = 1;
while k <= length(time_states)
    if inpolygon(states(1,k),states(2,k),ws([2 1 1 2]),ws([4 4 3 3]));
        k = k+1;
    else
        states(:,k) = [];
        time_states(k) = [];
    end
end



%% Plot the x y position
figure(1)
plot(pos_gps(1,:),pos_gps(2,:),'k--','LineWidth',LINE_WID/2)
axis equal
axis(ws)
grid on
title('XY Position','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$x \ (\mathrm{m})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$y \ (\mathrm{m})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
hold on
plot(states(1,:),states(2,:),'b-','LineWidth',LINE_WID)
hold off
legend('GPS','Estimation')



%% Plot all of the states
figure(2)
subplot(4,1,1)
plot(time_states,states(1,:),'k-','LineWidth',LINE_WID)
xlim(time_states([1 end]))
grid on
title('X Position','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$x \ (\mathrm{m})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
%
subplot(4,1,2)
plot(time_states,states(2,:),'k-','LineWidth',LINE_WID)
xlim(time_states([1 end]))
grid on
title('Y Position','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$y \ (\mathrm{m})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
%
subplot(4,1,3)
plot(time_states,states(3,:),'k-','LineWidth',LINE_WID)
xlim(time_states([1 end]))
grid on
title('$\psi$ angle','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$\psi \ (\mathrm{rad})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
%
subplot(4,1,4)
plot(time_states,states(4,:),'k-','LineWidth',LINE_WID)
xlim(time_states([1 end]))
grid on
title('Forward velocity','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$v \ (\mathrm{m/s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)




%% IMU data
figure(3)
subplot(3,1,1)
plot(time_imu,imu_acc(1,:),'r-','LineWidth',LINE_WID)
xlim(time_imu([1 end]))
grid on
title('Accelerometer','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$(\mathrm{m/s^2})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
hold on
plot(time_imu,imu_acc(2,:),'g-','LineWidth',LINE_WID)
plot(time_imu,imu_acc(3,:),'b-','LineWidth',LINE_WID)
hold off
l = legend('$a_x^b$','$a_y^b$','$a_z^b$'); l.Interpreter = 'latex'; l.FontSize = LEGEND_FONT_SIZE;
%
subplot(3,1,2)
plot(time_imu,imu_gyro(1,:),'r-','LineWidth',LINE_WID)
xlim(time_imu([1 end]))
grid on
title('Gyro','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$(\mathrm{rad/s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
hold on
plot(time_imu,imu_gyro(2,:),'g-','LineWidth',LINE_WID)
plot(time_imu,imu_gyro(3,:),'b-','LineWidth',LINE_WID)
hold off
l = legend('$\omega_x$','$\omega_y$','$\omega_z$'); l.Interpreter = 'latex'; l.FontSize = LEGEND_FONT_SIZE;
%
subplot(3,1,3)
plot(time_imu,imu_euler(1,:),'r-','LineWidth',LINE_WID)
xlim(time_imu([1 end]))
grid on
title('Euler angles','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
xlabel('$t \ (\mathrm{s})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
ylabel('$(\mathrm{rad})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
hold on
plot(time_imu,imu_euler(2,:),'g-','LineWidth',LINE_WID)
plot(time_imu,imu_euler(3,:),'b-','LineWidth',LINE_WID)
hold off
l = legend('$\phi$','$\theta$','$\psi$'); l.Interpreter = 'latex'; l.FontSize = LEGEND_FONT_SIZE;
%









%% Animation

if (ANIMATION == 1)
    
    figure(100)
    h1 = plot(pos_gps(1,:),pos_gps(2,:),'k-','LineWidth',LINE_WID);
    axis equal
    axis(ws)
    grid on
    title('XY Position','Interpreter','latex','FontSize',TITLE_FONT_SIZE)
    xlabel('$x \ (\mathrm{m})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
    ylabel('$y \ (\mathrm{m})$','Interpreter','latex','FontSize',LABEL_FONT_SIZE)
    hold on
%     h2 = quiver(states(1,1),states(2,1),cos(states(3,1)),sin(states(3,1)),'b','AutoScale','off','LineWidth',3,'MaxHeadSize',0.7);
    H = eye(4);
    H(1:2,4) = states(1:2,1);
    H(1:3,1:3) = Rot('z',states(3,1));
    h2 = plot_frame(H,1,3);
    hold off
    legend('GPS','Estimation')
    
    
    for k = 1:10:length(time_states)
        
        
        
        
%         vec = [cos(states(3,k)); sin(states(3,k))];
%         pos = [states(1,k); states(2,k)];
%         set(h2,'XData',states(1,k),'YData',states(2,k),'UData',cos(states(3,k)),'VData',sin(states(3,k)));
        
        
        H(1:2,4) = states(1:2,k);
%         H(1:3,1:3) = Rot('z',states(3,k));
        H(1:3,1:3) = quat2rotm(imu_quat(:,k)');
        set_frame(H,h2,1);
        
        
        drawnow
        
    end
    
    
    
end

