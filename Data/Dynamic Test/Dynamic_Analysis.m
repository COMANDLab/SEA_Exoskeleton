%Dynamic Test Gryoscope Processing - Higher Frequency
%Author: Hailey Levan
%Last Updated: 7/24/24

%% Read data from excel document
filename = '1_18V.xlsx';
trialname = '1_18V Test';
volt = '(18V)';

raw_data = readmatrix(filename); % First 7 columns are from IMUs, last 2 are motor

%% Extract IMU time, Z angular velocity forearm, motor time, and motor position
IMU_time = raw_data(:,1);
angveloZ_forearm = raw_data(:,4);

%% Report max angular velocity of forearm
abs_angvelo = abs(angveloZ_forearm);
[~,velo_idx] = max(abs_angvelo);
max_angvelo = angveloZ_forearm(velo_idx);

%% Graph angular velocity data sets v. time
figure(1);
plot(IMU_time,angveloZ_forearm,'m');
title(['Z Direction Angular Velocity in Forearm Bracket ' volt]);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
saveas(gcf,[trialname,'_Raw.png']);
close(1);

%% Truncate velocity and IMU time data to start at onset of motor movement
velo_start_index = find_start_velo_index(angveloZ_forearm);

angveloZ_forearm = angveloZ_forearm(velo_start_index:end);
IMU_time = IMU_time(velo_start_index:end);

%% Truncate velocity and IMU time data to stop when arm has been at zero for a long time
velo_end_index = find_end_velo_index(angveloZ_forearm);

angveloZ_forearm = angveloZ_forearm(1:velo_end_index+3);
IMU_time = IMU_time(1:velo_end_index+3) - IMU_time(1);

%% Graph zoomed in angular velocity data sets v. time
figure(2);
plot(IMU_time,angveloZ_forearm,'m', 'LineWidth',2);
title(['Z Direction Angular Velocity in Forearm Bracket - Zoomed In ' volt]);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
saveas(gcf,[trialname,'_Zoomed.png']);
close(2);

%% Integrate
% Integrate angular velocity to get angular position
angular_position = cumtrapz(IMU_time, angveloZ_forearm);

%% Convert angular position data to degrees & report max
angular_position_deg = angular_position * (180/pi);

%% Plot angular position data
figure(3);
plot(IMU_time, angular_position_deg, 'k', 'DisplayName', 'Angular Position in Degrees','LineWidth',2);
xlabel('Time (s)');
ylabel('Angular Position (deg)');
title([volt ' Integrated Angular Position vs. Time']);
saveas(gcf,[trialname,'_Integrated.png']);
close(3);

%% Determine overshoot, undershoot, and final angles
% Create a vector that is the absolute value of all the data in the original vector
abs_angles = abs(angular_position_deg);

% Find the index of the greatest angle in the original vector & report peak
% angle and angular velo at that point
[~, peak_index] = max(abs_angles);
peak_angle = angular_position_deg(peak_index);
peak_velo = angveloZ_forearm(peak_index);
peak_check = 0;

% Check to make sure its grabbing the right peak
while peak_index/length(angular_position_deg) >= 0.95
    wrong_peak_idx = peak_index;
    new_abs_angles = abs_angles(1:peak_index-1);
    [~,peak_index] = max(new_abs_angles);
    peak_angle = angular_position_deg(peak_index);
    peak_velo = angveloZ_forearm(peak_index);
    peak_check = 1;
end

% Determine the first repeated value after the maximum index and treat this
% as the final angle of rest

% Variable to hold the repeated value
repeated_value = [];

% Initialize a map to keep track of occurrences of each value
occurrences = containers.Map('KeyType', 'double', 'ValueType', 'int32');

% Iterate through the vector starting from the specified index
for i = peak_index:length(angular_position_deg)
    value = angular_position_deg(i);
    
    % Update the count of the current value in the map
    if isKey(occurrences, value)
        occurrences(value) = occurrences(value) + 1;
    else
        occurrences(value) = 1;
    end
    
    % Check if the current value has been repeated 3 or more times
    if occurrences(value) >= 4  % always reset to 4 if changed
        repeated_value = value;
        break;
    end
end

final_angle = repeated_value;

% Find the index of the first occurrence of the most repeated value after the maximum index
first_occurrence_index = find(angular_position_deg == final_angle, 1, 'first');

% Extract the sub-vector between the max angle index and the first occurrence of the most repeated value
if first_occurrence_index > peak_index + 1
    sub_vector = angular_position_deg(peak_index+1:first_occurrence_index-1);
else
    sub_vector = [];  % Empty sub-vector if no elements are in between
end

% Find the smallest angle in the sub-vector
if ~isempty(sub_vector)
    abs_sub_vector = abs(sub_vector);
    [~,min_idx] = min(abs_sub_vector);
    min_angle = sub_vector(min_idx);
else
    min_angle = [];  % No minimum angle if sub-vector is empty
end

if abs(min_angle) < abs(final_angle)
    trough_angle = min_angle;
else
    trough_angle = final_angle;
end

%% Calculate the overshoot (how far forearm went past the final angle), oscillation amplitude (peak to trough), and percent overshoot
overshoot = peak_angle - final_angle;
amplitude = peak_angle - trough_angle;
percent_overshoot = (overshoot/final_angle) * 100;

%% Calculate settling time (moment motor stops to instant where arm stops oscillating at final angle) & and peak time
if overshoot ~= 0
    % Determine start time... estimate the start time by interpolating the
    % data to find when time at which the angle equaled the final angle on
    % the rising edge of the peak


    x1 = IMU_time(peak_index-6); 
    y1 = angular_position_deg(peak_index-6);

    x2 = IMU_time(peak_index-4);
    y2 = angular_position_deg(peak_index-4);

    y = final_angle;

    x = x1 + (y - y1)*((x2 - x1)/(y2 - y1));

    start_time = x;

    % Determine end time... estimate as the instant the forearm reaches its
    % final angle
    end_time = IMU_time(first_occurrence_index);

    settling_time = end_time - start_time;
    peak_time = IMU_time(peak_index) - start_time;
else
    settling_time = [];
    peak_time = [];
end

fprintf('Full Trial Max Angular Velo (rad/s)'); disp(max_angvelo);
fprintf('Angular Velo at Peak Angle (rad/s)'); disp(peak_velo);
fprintf('Overshoot (deg)'); disp(overshoot);
fprintf('Oscillation Amplitude (deg)'); disp(amplitude);
fprintf('Final Angle'); disp(final_angle);
fprintf('Percent Overshoot'); disp(percent_overshoot);
fprintf('Peak Time (s)'); disp(peak_time);
fprintf('Settling Time (s)'); disp(settling_time);

time_subvec = IMU_time(peak_index-10:first_occurrence_index+4);
position_subvec = angular_position_deg(peak_index-10:first_occurrence_index+4);

figure(4);
plot(time_subvec,position_subvec,'k','LineWidth',2);
xlabel('Time (s)');
ylabel('Angular position (deg)');
title(['Zoomed In to View Peak and Steady State Values ', volt]);
hold on;

% Highlight the specific data point
plot(start_time, final_angle, 'ro', 'MarkerSize', 6, 'LineWidth', 4);

saveas(gcf,[trialname,'_Integrated_Zoomed.png']);

%% Functions
function index = find_start_velo_index(angular_velocity)
    % Initialize the index to -1 (or any invalid index)
    index = -1;
    
    % Get the length of the array
    n = length(angular_velocity);
    
    % Loop through the array
    for i = 1:n-20
        % Check if the current element is zero and the next 10 elements are not zero
        if angular_velocity(i) == 0 && all(angular_velocity(i+1:i+20) ~= 0)
            % Assign the index and break the loop
            index = i;
            break;
        end
    end
end

function lastIndex = find_end_velo_index(angular_velocity)
    % Get the length of the array
    n = length(angular_velocity);

% Loop through the array to find the first index with a non-zero followed by at least 4 zeros
    for i = 1:n-4 %reset to n-4
        % Check if the current element is not zero
        if angular_velocity(i) ~= 0
            % Check if at least the next 4 elements are zero
            if all(angular_velocity(i+1:i+4) == 0) %reset to (i+1:i+4)
                lastIndex = i;
                break;
            end
        end
    end
end