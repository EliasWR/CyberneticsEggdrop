%% System identification of DC motor with measurements from Arduino
% By Group #1, Cybernetics, IIR, NTNU, 2023
% Filip Arthur Blaafjell, Elias Woie Refsdal, Martin Simengård

clear; clc; close all;

%% Import dataset
filename = "serial_data1.csv";
dataset = importdata(filename, ',').data;

%% Manipulate data

Timestamp_raw = dataset(:, 1); % ms after startup [ms]
Target_raw = dataset(:, 2); % Target position
PWM_raw = dataset(:, 3); % PWM-signal to motor 
Actual_raw = dataset(:, 4); % Actual position (encoder value)


% Skalere input fra encoder
enc_Zero = 0;
enc_Limit = 12144/4;
cm_Zero = 0;
cm_Limit = 20;
Target_scaled = simpleScale(Target_raw, enc_Zero, enc_Limit, cm_Zero, cm_Limit);
Actual_scaled = simpleScale(Actual_raw, enc_Zero, enc_Limit, cm_Zero, cm_Limit);

Input_voltage = PWM_raw; % PWM converted to voltage [V]

%% Dividing into training and validation set
Dataset_length = length(Actual_scaled);
Training_size = 0.7;
Dividing_index = ceil(Training_size * Dataset_length);

Target_raw_training = Target_raw(1:Dividing_index);
Target_raw_validation = Target_raw(Dividing_index+1:end);

Actual_raw_training = Actual_raw(1:Dividing_index);
Actual_raw_validation = Actual_raw(Dividing_index+1:end);

PWM_raw_training = PWM_raw(1:Dividing_index);
PWM_raw_validation = PWM_raw(Dividing_index+1:end);


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
DC_Motor_DATA_RAW = iddata(Actual_raw, PWM_raw, Ts);

%% Visualizing the data

figure(1)

subplot(3,1,1)
plot(timestamp, Target_scaled)
grid on
xlabel('time [s]')
ylabel('Target position')

subplot(3,1,2)
plot(timestamp, Actual_scaled)
grid on
xlabel('time [s]')
ylabel('Encoder position')

subplot(3,1,3)
plot(timestamp, PWM_raw)
grid on
xlabel('time [s]')
ylabel('Motor output')

%% Identifacting the system
Gp = tfest(DC_Motor_DATA, 2, 1)

%% Digital twin (Open loop)

H = [1];   % Feedback (No controller)

figure(2)
M = feedback(Gp, H);
step(M)
hold on
legend('Uncontrolled system')

%% Controller parameters

Gc = pidtune(Gp,'PI')

Mc = feedback(Gc*Gp, H)

step(Mc)
grid on
hold off
legend('uncontrolled system' ,'controlled system')

