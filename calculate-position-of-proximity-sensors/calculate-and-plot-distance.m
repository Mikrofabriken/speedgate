% GNU Octave - Motor Speed Profile Parameter & Acceleration Script

clc;
clear;
close all;

%% User defined variables
% USER SETTINGS
motor_low_speed_hz      = 5.0; % Low operational speed [Hz]
motor_high_speed_hz     = 20.0; % High operational speed [Hz]
travel_distance_angle   = 150; % Degrees [°]
travel_pre_angle        = 18; % Degrees [°]

% Motor parameters
%motor_nominal_speed_rpm = 925.0; % Nominal motor speed [Rpm]
motor_nominal_speed_rpm = 1100.0; % Nominal motor speed [Rpm] (measured rpm @ 5 Hz = 110 rpm) TODO: Investigate!!
motor_nominal_speed_hz  = 50.0; % Nominal motor frequency [Hz]
gear_ratio              = 925/5.2; %

% Ramp times (seconds required to accelerate/decelerate between 0 and nominal speed)
motor_acceleration_s  = 5.0;  % Time to accelerate from 0 to nominal speed [s]
motor_deceleration_s  = 5.0;  % Time to decelerate from nominal speed to 0 [s]


%% Derived calculations
% Calculate linear acceleration and deceleration rates in Hz/second
acc_rate_hz_s = motor_nominal_speed_hz / motor_acceleration_s;
dec_rate_hz_s = motor_nominal_speed_hz / motor_deceleration_s;

% Calculate specific transition times between setpoints
time_0_to_low     = motor_low_speed_hz / acc_rate_hz_s;
time_low_to_high  = (motor_high_speed_hz - motor_low_speed_hz) / acc_rate_hz_s;
time_0_to_high    = motor_high_speed_hz / acc_rate_hz_s;

% Calculate amount of motor rotations on full travel angle
travel_motor_rotations  = (travel_distance_angle / 360) * gear_ratio;

% Calculate useful stuff
motor_nominal_speed_rps = motor_nominal_speed_rpm / 60;
motor_nominal_speed_rps_per_hertz = motor_nominal_speed_rps / 50;
travel_angle_per_motor_rev = (1 / gear_ratio) * 360;
travel_motor_rev_per_angle = gear_ratio / 360;


%% Output summary to console
fprintf('==================================================\n');
fprintf('  MOTOR PARAMETERS\n');
fprintf('==================================================\n');
fprintf('Nominal speed : %6.2f Rpm\n', motor_nominal_speed_rpm);
fprintf('Nominal speed : %6.2f Hz\n', motor_nominal_speed_hz);
fprintf('Gear ratio    : %6.2f \n', gear_ratio);
fprintf('\n');
fprintf('==================================================\n');
fprintf('  VFD PARAMETERS\n');
fprintf('==================================================\n');
fprintf('Low speed         : %6.2f Hz\n', motor_low_speed_hz);
fprintf('High speed        : %6.2f Hz\n', motor_high_speed_hz);
fprintf('Acceleration rate : %6.2f Hz/s (0 -> Nom in %.2f s)\n', acc_rate_hz_s, motor_acceleration_s);
fprintf('Deceleration rate : %6.2f Hz/s (Nom -> 0 in %.2f s)\n', dec_rate_hz_s, motor_deceleration_s);
fprintf('\n');
fprintf('==================================================\n');
fprintf('  TRAVEL DISTANCE\n');
fprintf('==================================================\n');
fprintf('Total travel angle : %6.2f °\n', travel_distance_angle);
fprintf('Motor rotations    : %5.1f revs\n', travel_motor_rotations);
fprintf('\n');
fprintf('==================================================\n');
fprintf('  TRANSITION RAMP TIMES\n');
fprintf('==================================================\n');
fprintf('0 Hz       -> Low speed  (%2.0f Hz) : %6.2f s\n', motor_low_speed_hz, time_0_to_low);
fprintf('Low speed  -> High speed (%2.0f Hz) : %6.2f s\n', motor_high_speed_hz, time_low_to_high);
fprintf('0 Hz       -> High speed (%2.0f Hz) : %6.2f s\n', motor_high_speed_hz, time_0_to_high);
fprintf('\n');


%% Calculate travel times

% Stage 1, ramp up
st1_revs = (motor_high_speed_hz / 2) * time_0_to_high * motor_nominal_speed_rps_per_hertz;
st1_travel_time = time_0_to_high;
st1_start_time = 0;
st1_angle = st1_revs * travel_angle_per_motor_rev;

% Stage 3, ramp down
st3_revs = (((motor_high_speed_hz - motor_low_speed_hz) / 2) + motor_low_speed_hz) * time_low_to_high * motor_nominal_speed_rps_per_hertz;
travel_ramp_down_angle = st3_revs * travel_angle_per_motor_rev;
st3_travel_time = time_low_to_high;
st3_angle = st3_revs * travel_angle_per_motor_rev;

% Stage 4, remaining pre angle
st4_angle = travel_pre_angle - travel_ramp_down_angle;
if (st4_angle < 0)
  st4_angle = 0;
endif
st4_revs = st4_angle * travel_motor_rev_per_angle;
st4_travel_time = st4_revs / (motor_nominal_speed_rps_per_hertz * motor_low_speed_hz);

% Stage 2, high speed
st2_revs = travel_motor_rotations - st1_revs - st3_revs - st4_revs;
st2_travel_time = st2_revs / (motor_nominal_speed_rps_per_hertz * motor_high_speed_hz);
st2_angle = st2_revs * travel_angle_per_motor_rev;

% Stage 5, ramp down to stop
st5_revs = (motor_low_speed_hz / 2) * time_0_to_low * motor_nominal_speed_rps_per_hertz;
st5_travel_time = time_0_to_low;
st5_angle = st5_revs * travel_angle_per_motor_rev;

st2_start_time = st1_start_time + st1_travel_time;
st3_start_time = st2_start_time + st2_travel_time;
st4_start_time = st3_start_time + st3_travel_time;
st5_start_time = st4_start_time + st4_travel_time;


%% Print
fprintf('==================================================\n');
fprintf('  TRAVEL SUMMARY AND PLOT\n');
fprintf('==================================================\n');
fprintf('Stage 1: Travel from 0 Hz -> high speed\n');
fprintf('Travel time      : %6.2f s\n', st1_travel_time)
fprintf('Motor rotations  : %6.2f revs\n', st1_revs);
fprintf('Angle traveled   : %6.2f °\n', st1_angle);
fprintf('\n');
fprintf('--------------------------------------------------\n');

fprintf('Stage 2: Travel remaining angle at high speed\n');
fprintf('Travel time      : %6.2f s\n', st2_travel_time);
fprintf('Motor Rotations  : %6.2f revs\n', st2_revs);
fprintf('Angle traveled   : %6.2f °\n', st2_angle);
fprintf('\n');
fprintf('--------------------------------------------------\n');

fprintf('Stage 3: Travel from high speed -> low speed\n');
fprintf('Travel time      : %6.2f s\n', st3_travel_time);
fprintf('Motor rotations  : %6.2f revs\n', st3_revs);
fprintf('Angle traveled   : %6.2f °\n', st3_angle);
fprintf('\n');
fprintf('--------------------------------------------------\n');

fprintf('Stage 4: Travel remaining pre angle (if any)\n');
fprintf('Travel time      : %6.2f s\n', st4_travel_time);
fprintf('Motor rotations  : %6.2f revs\n', st4_revs);
fprintf('Angle traveled   : %6.2f °\n', st4_angle);
fprintf('Total pre angle  : %6.2f °\n', travel_pre_angle);
fprintf('\n');
fprintf('--------------------------------------------------\n');

fprintf('Stage 5: Ramp down to stop\n');
fprintf('Travel time      : %6.2f s\n', st5_travel_time);
fprintf('Motor rotations  : %6.2f revs\n', st5_revs);
fprintf('Angle traveled   : %6.2f °\n', st5_angle);
fprintf('\n');
fprintf('--------------------------------------------------\n');

fprintf('Total travel time : %5.1f s\n', st1_travel_time + st2_travel_time + st3_travel_time + st4_travel_time + st5_travel_time);
fprintf('Total travel angle : %5.1f °\n', st1_angle + st2_angle + st3_angle + st4_angle + st5_angle);


%% Plot vectors
time_values  = [st1_start_time, st2_start_time, st3_start_time, st4_start_time, st5_start_time, st5_start_time + st5_travel_time];
speed_values = [0, motor_high_speed_hz, motor_high_speed_hz, motor_low_speed_hz, motor_low_speed_hz, 0];
angle_values = cumsum([0, st1_angle, st2_angle, st3_angle, st4_angle, st5_angle]);

figure;

% Dual y-axis plot compatible with GNU Octave
[ax, h1, h2] = plotyy(time_values, speed_values, time_values, angle_values);

% Configure left y-axis (Speed)
ylabel(ax(1), "VFD motor frequency [Hz]");
ylim(ax(1), [0 50]);
set(h1, "LineWidth", 1.5);

% Configure right y-axis (Angle)
ylabel(ax(2), "Angle [degrees]");
set(h2, "LineWidth", 1.5);
angle_axis_max = ceil(max(angle_values) / 10) * 10;
ylim(ax(2), [0 angle_axis_max]);
set(ax(2), "YTick", 0:10:angle_axis_max);
grid(ax(2), "on");

% Configure shared axis settings
xlabel("Time [seconds]");
xticks(0:0.5:50);
grid on;

waitfor(h1);
