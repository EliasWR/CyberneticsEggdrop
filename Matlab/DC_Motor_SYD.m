%% System identification of DC motor with measurements from Arduino
% By Group #1, Cybernetics, IIR, NTNU, 2023
% Filip Arthur Blaafjell, Elias Woie Refsdal, Martin Simengård

clear; clc; close all;

%% Import dataset
filename = "serial_data.csv";
dataset = importdata(filename, ',').data;

%% Manipulate data

Timestamp_raw = dataset(:, 1); % ms after startup [ms]
Target_raw = dataset(:, 2); % Target position
PWM_raw = dataset(:, 3); % PWM-signal to motor 
Actual_raw = dataset(:, 4); % Actual position (encoder value)


% Skalere input fra encoder
enc_Zero = 0;
enc_Limit = 12144;
cm_Zero = 0;
cm_Limit = 20;
Target_scaled = simpleScale(Target_raw, enc_Zero, enc_Limit, cm_Zero, cm_Limit);
Actual_scaled = (Actual_raw - enc_Zero) / (enc_Limit - enc_Zero) * (cm_Limit-cm_Zero) + cm_Zero; 

Input_voltage = PWM_raw; % PWM converted to voltage [V]

%% Sampling time from timestamps

timestamp = zeros(length(Timestamp_raw),1);
Ts_vect = zeros(length(Timestamp_raw)-1,1);

for i = 2:length(Timestamp_raw)
     Ts = Timestamp_raw(i) - Timestamp_raw(i-1);
     Ts_vect(i-1) = Ts; 
     
     timestamp(i) = (Timestamp_raw(i) - Timestamp_raw(1))/1000; 
end

Ts_max = max(Ts_vect);
disp('Maximum sample time [ms]: ')
disp(Ts_max)

Ts_min = min(Ts_vect);
disp('Minimum sample time [ms]: ') 
disp(Ts_min)

Ts_average = mean(Ts_vect);
disp('The average sample time is [ms]')
disp(Ts_average)

disp('Our sampling time is, Ts [sec]')
Ts = round(Ts_average)*10^-3;
disp(Ts)

%% Structure as iddata for SystemIdentification
% Format: (Output, Input, Sample time)

DC_Motor_DATA = iddata(Actual_scaled, Input_voltage, Ts);

%% Visualizing the data

figure(1)

subplot(2,1,1)
plot(timestamp, Target_scaled)
grid on
xlabel('time [s]')
ylabel('Target position')

subplot(2,1,2)
plot(timestamp, Actual_scaled)
grid on
xlabel('time [s]')
ylabel('Encoder position')

%% Identifacting the system
Gp = tfest(DC_Motor_DATA, 2, 0) 

%% Digital twin (Open loop)

H = [1];   % Feedback (No controller)

figure(2)
M = feedback(Gp, H);    
step(M)
hold on
legend('Uncontrolled system')

%% Controller parameters

Gc = tunablePID('Controller','PID')
