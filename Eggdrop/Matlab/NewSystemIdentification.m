% System identification of DC motor with measurements from Arduino
% By Group #1, Cybernetics, IIR, NTNU, 2023
% Filip Arthur Blaafjell, Elias Woie Refsdal, Martin Simengård

clear; clc; close all;

% Import dataset
filename = "serial_data copy.csv";
dataset = importdata(filename, ',').data;

% Extract and scale data
Timestamp_raw = dataset(:, 1); % Time in ms
Target_raw = dataset(:, 2); % Target position
PWM_raw = dataset(:, 3); % PWM-signal to motor 
Actual_raw = dataset(:, 4); % Actual position (encoder value)

% Scaling parameters
enc_Zero = 0;
enc_Limit = 12144/4;
cm_Zero = 0;
cm_Limit = 20;

% Scaling the data
Target_scaled = simpleScale(Target_raw, enc_Zero, enc_Limit, cm_Zero, cm_Limit);
Actual_scaled = simpleScale(Actual_raw, enc_Zero, enc_Limit, cm_Zero, cm_Limit);

% PWM converted to voltage [V]
Input_voltage = PWM_raw; 

% Dividing into training and validation set
Dataset_length = length(Actual_scaled);
Training_size = 0.7;
Dividing_index = ceil(Training_size * Dataset_length);

% Training data
Target_raw_training = Target_raw(1:Dividing_index);
Actual_raw_training = Actual_raw(1:Dividing_index);
PWM_raw_training = PWM_raw(1:Dividing_index);

% Validation data
Target_raw_validation = Target_raw(Dividing_index+1:end);
Actual_raw_validation = Actual_raw(Dividing_index+1:end);
PWM_raw_validation = PWM_raw(Dividing_index+1:end);

% Calculate sampling time from timestamps
timestamp = (Timestamp_raw - Timestamp_raw(1)) / 1000; % Convert to seconds
Ts_vect = diff(Timestamp_raw) / 1000; % Sampling intervals in seconds
Ts = mean(Ts_vect) % Average sampling time in seconds

% Prepare data for System Identification
TrainingData = iddata(Actual_raw_training, PWM_raw_training, Ts);
ValidationData = iddata(Actual_raw_validation, PWM_raw_validation, Ts);

% Visualization of the data
figure(1);
subplot(3,1,1)
plot(timestamp, Target_scaled);
title('Target Position');
xlabel('Time [s]'); ylabel('Position [cm]');
grid on;

subplot(3,1,2)
plot(timestamp, Actual_scaled);
title('Encoder Position');
xlabel('Time [s]'); ylabel('Position [cm]');
grid on;

subplot(3,1,3)
plot(timestamp, PWM_raw);
title('Motor Output');
xlabel('Time [s]'); ylabel('PWM Signal');
grid on;

% System Identification using TF model
tf_model = tfest(TrainingData, 2, 1)
tf_model2 = tfest(TrainingData, 2, 2)
tf_model3 = tfest(TrainingData, 3, 1)
tf_model4 = tfest(TrainingData, 3, 2)
tf_model5 = tfest(TrainingData, 3, 3)

% System Identification using State-Space Model
ss_model = ssest(TrainingData, 2)

% System Identification using Polynomial Model
poly_model = pem(TrainingData)

%Validation
figure;
compare(ValidationData, tf_model, tf_model2, tf_model3, tf_model4, tf_model5, tf_model6);
%compare(ValidationData, tf_model3, ss_model, poly_model);

legend(['Validation Data (Actual Output)'], ...
       '2 poles , 1 zero', ...
       '2 poles , 2 zero', ...
       '3 poles , 1 zero', ...
       '3 poles , 2 zero', ...
       '3 poles , 3 zero',...
       '2 poles , 0 zero ');
title('TF estimation paramater comparison');
grid on;

%% Digital twin (Open loop)

H = [1];   % Feedback (No controller)

figure(2)
M = feedback(tf_model3, H);
step(M)
hold on
legend('Uncontrolled system')

%% Controller parameters
H = [1];
Gc = pidtune(tf_model3,'PI')

Kp = 0.12;
Ki = 0.06;
Kd = 0;
Gx = pid(Kp, Ki, Kd)

Mc = feedback(Gx*tf_model3, H)
y = step(Mc)

grid on
hold off
legend('controlled system')