%Author: Hailey Levan
%Last Updated: 7/11/24

%% Code Description:
% This code reads the xyz angular velocity data from two gyroscopes. It
% plots raw z angular velocity v. time for both IMUs (one is static at 0 rad/s,
% the other dynamic) before truncating the raw data to include just the
% period of movement. The truncated data is then integrated to obtain
% angular position data. This data is the angle of the exoskeleton elbow.
% The code reports the maximum angle (also the final angle for these
% trials), as this is the angle indicative of either the gear contributed
% compliance or total system compliance.

%% Read data from excel document
% Change filename according to what file you want read and analyzed
filename = 'Static1_Gear_7.11.xlsx';

raw_data = readmatrix(filename);

%% Extract time, Z angular velocity forearm, and Z angular velocity upperarm 
time = raw_data(:,1);
angveloZ_forearm = raw_data(:,4);
angveloZ_upperarm = raw_data(:,7);

%% Graph both angular velocity data sets v. time
figure(1);
plot(time,angveloZ_forearm,'m',time, angveloZ_upperarm,'b');
title('Z Direction Angular Velocity in Forearm and Upperarm Brackets');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('Forearm','Upperarm');

%% Integrate
% Find indices where angular velocity of forearm does NOT equal zero
nonzero_indices = find(angveloZ_forearm ~= 0);

% Loop through the indices to find start and end times
start_idx = [];
end_idx = [];

for i = 1:length(nonzero_indices)-1
    % Check if the next index is 1 greater than the current index
    if nonzero_indices(i+1) == nonzero_indices(i) + 1
        % If we are at the beginning or coming from a non-sequential part, mark start time
        if isempty(start_idx) || (i > 1 && nonzero_indices(i) ~= nonzero_indices(i-1) + 1)
            start_idx = [start_idx,nonzero_indices(i)];
        end
    else
        % If the sequence breaks, mark end time for the previous start time
        if ~isempty(start_idx) && (isempty(end_idx) || (length(end_idx) < length(start_idx)))
            end_idx = [end_idx,nonzero_indices(i)];
        end
    end
end

% % Handle the case where the last segment is continuous until the end of the vector
 if ~isempty(start_idx) && (isempty(end_idx) || (length(end_idx) < length(start_idx)))
     end_idx = nonzero_indices(end);
 end

% Extract the portion of the data to be integrated
t_portion = time(start_idx(1):end_idx(1));
angvelo_forearm_portion = angveloZ_forearm(start_idx(1):end_idx(1));

% Integrate angular velocity to get angular position
angular_position = cumtrapz(t_portion, angvelo_forearm_portion);

% Plot the results
figure(2);

%% Convert angular position data to degrees & report max
angular_position_deg = angular_position * (180/pi);

plot(t_portion, angular_position_deg, 'm');
xlabel('Time (s)');
ylabel('Angular Position (deg)');
legend;
title('Angular Position Calculated by Integrating Non-Zero Portion of Angular Velocity Data');

abs_ang_pos = abs(angular_position_deg);
max_abs = max(abs_ang_pos);
max_abs_idx = find(abs_ang_pos == max_abs);
max_angle = angular_position_deg(max_abs_idx);

fprintf('Max Angle (deg): ');
disp(max_angle);