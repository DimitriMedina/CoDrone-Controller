close all

% Specify log file location
log_file = 'C:\Users\Dimit\Desktop\UaV\Codrone\FinalProject\Part4.txt';

% Load data
data = readtable(log_file);

% Plot data
% Plot accelerometer data
figure;
subplot(3,1,1);hold on;grid on;title('Accelerometer Measurements')
    plot(data.time,data.accelX,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Accel X (m/s/s)');set(gca,'fontweight','bold')
subplot(3,1,2);hold on;grid on;
    plot(data.time,data.accelY,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Accel Y (m/s/s)');set(gca,'fontweight','bold')
subplot(3,1,3);hold on;grid on;
    plot(data.time,data.accelZ,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Accel Z (m/s/s)');set(gca,'fontweight','bold')

% Plot gyro data
figure;
subplot(3,1,1);hold on;grid on;title('Gyroscope Mesurements')
    plot(data.time,data.gyroX,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Gyro X (deg/s)');set(gca,'fontweight','bold')
subplot(3,1,2);hold on;grid on;
    plot(data.time,data.gyroY,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Gyro Y (deg/s)');set(gca,'fontweight','bold')
subplot(3,1,3);hold on;grid on;
    plot(data.time,data.gyroZ,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Gyro Z (deg/s)');set(gca,'fontweight','bold')

% Plot Position inertial Estimate
figure;
subplot(3,1,1);hold on;grid on;title('Inertial Position Estimates')
    plot(data.time,data.pIX_est,'linewidth',1.5)
    plot(data.time,data.pIX_cmd,'r--','linewidth',1.5)
    plot(data.time,data.pIX_est + [-3 3].*data.pIX_sigma,'k--')
    xlabel('Time (s)');ylabel ('Pos I X (m)');set(gca,'fontweight','bold')
    legend('Estimate','Command','3 \sigma')
subplot(3,1,2);hold on;grid on;
    plot(data.time,data.pIY_est,'linewidth',1.5)
    plot(data.time,data.pIY_cmd,'r--','linewidth',1.5)
    plot(data.time,data.pIY_est + [-3 3].*data.pIY_sigma,'k--')
    xlabel('Time (s)');ylabel ('Pos I Y (m)');set(gca,'fontweight','bold')
subplot(3,1,3);hold on;grid on;
    plot(data.time,data.pIZ_est,'linewidth',1.5)
    plot(data.time,data.altitude_cmd,'r--','linewidth',1.5)
    plot(data.time,data.alt_meas,'g-','linewidth',1.5)
    plot(data.time,data.pIZ_est + [-3 3].*data.pIZ_sigma,'k--')
    xlabel('Time (s)');ylabel ('Pos I Z (m)');set(gca,'fontweight','bold')
    
% Plot Velocity Inertial Estimate
figure;
subplot(3,1,1);hold on;grid on;title('Inertial Velocity Estimates')
    plot(data.time,data.vIX_est,'linewidth',1.5)
    plot(data.time,data.vIX_est + [-3 3].*data.vIX_sigma,'k--')
    xlabel('Time (s)');ylabel ('Vel I X (m/s)');set(gca,'fontweight','bold')
    legend('Estimate','Command','3 \sigma')
subplot(3,1,2);hold on;grid on;
    plot(data.time,data.vIY_est,'linewidth',1.5)
    plot(data.time,data.vIY_est + [-3 3].*data.vIY_sigma,'k--')
    xlabel('Time (s)');ylabel ('Vel I Y (m/s)');set(gca,'fontweight','bold')
subplot(3,1,3);hold on;grid on;
    plot(data.time,data.vIZ_est,'linewidth',1.5)
    plot(data.time,data.vIZ_est + [-3 3].*data.vIZ_sigma,'k--')
    xlabel('Time (s)');ylabel ('Vel I Z (m/s)');set(gca,'fontweight','bold')
   
% Plot Attitude Estimate
figure;
subplot(3,1,1);hold on;grid on;title('Attitude Estimates')
    plot(data.time,data.phi_est*180/pi,'linewidth',1.5)
    plot(data.time,data.phi,'linewidth',1.5)
    plot(data.time,data.phi_est*180/pi + [-3 3].*data.phi_sigma*180/pi,'k--')
    legend('EKF','CoDrone','3 \sigma')
    xlabel('Time (s)');ylabel ('Roll (deg)');set(gca,'fontweight','bold')
subplot(3,1,2);hold on;grid on;
    plot(data.time,data.theta_est*180/pi,'linewidth',1.5)
    plot(data.time,data.theta,'linewidth',1.5)
    plot(data.time,data.theta_est*180/pi + [-3 3].*data.theta_sigma*180/pi,'k--')
    xlabel('Time (s)');ylabel ('Pitch (deg)');set(gca,'fontweight','bold')
subplot(3,1,3);hold on;grid on;
    plot(data.time,data.psi_est*180/pi,'linewidth',1.5)
    plot(data.time,data.psi,'linewidth',1.5)
    plot(data.time,data.psi_est*180/pi + [-3 3].*data.psi_sigma*180/pi,'k--')
    xlabel('Time (s)');ylabel ('Yaw (deg)');set(gca,'fontweight','bold')

% Plot Measured Velocity
figure;
subplot(3,1,1);hold on;grid on;title('Measured Velocity & ChiSq Value')
    plot(data.time,data.vx_meas,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Vel B X (m/s)');set(gca,'fontweight','bold')
subplot(3,1,2);hold on;grid on;
    plot(data.time,data.vy_meas,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Vel B Y (m/s)');set(gca,'fontweight','bold')
subplot(3,1,3);hold on;grid on;
    plot(data.time,data.v_chiSq,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('V Chi Sq');set(gca,'fontweight','bold')

% Plot pitch/roll power commands
figure;
subplot(2,1,1);hold on;grid on;title('Roll & Pitch Power Commands')
    plot(data.time,data.roll_power,'linewidth',1.5)
    legend('Command Power')
    xlabel('Time (s)');ylabel ('Roll Power [-100 to 100]');set(gca,'fontweight','bold')
subplot(2,1,2);hold on;grid on;
    plot(data.time,data.pitch_power,'linewidth',1.5)
    xlabel('Time (s)');ylabel ('Pitch Power [-100 to 100]');set(gca,'fontweight','bold')

