%% System identification of DC motor with measurements from Arduino
% By Group #1, Cybernetics, IIR, NTNU, 2023
% Filip Arthur Blaafjell, Elias Woie Refsdal, Martin Simengård

clear; clc; close all;

%% Import dataset
filename = "DC-Motor-data.csv";
dataset = importdata(filename);

%% Manipulate data

Timestamp_raw = dataset(:, 1); % ms after startup [ms]
PWM_raw = dataset(:, 2); % PWM-signal to motor 
Target_raw = dataset(:, 3); % Target position
Actual_raw = dataset(:, 4); % Actual position (encoder value)

Input_voltage = PWM_raw * 5/255; % PWM converted to voltage [V]

%% Sampling time from timestamps

timestamp = zeros(length(time_raw),1);
Ts_vect = zeros(length(time_raw),1);

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
% Format: (Input, Output, Sample time)

DC_Motor_DATA = iddata(Input_voltage, Actual_raw, Ts);

%% Visualizing the data

figure(1)

subplot(2,1,1)
plot(timestamp, Input_voltage)
grid on
xlabel('time [s]')
ylabel('Voltage [V]')

subplot(2,1,2)
plot(time, Actual_raw)
grid on
xlabel('time [s]')
ylabel('Encoder position')