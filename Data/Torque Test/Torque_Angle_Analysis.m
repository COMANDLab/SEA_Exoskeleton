%Torque Test Gryoscope Processing
%Author: Hailey Levan
%Last Updated: 7/26/24

%% Read data from excel document
filename = '18V.txt';
trialname = '18V Test';
volt = '(18V)';

data_table = readtable(filename, Delimiter=",");
raw_data = table2array(data_table);

%% Extract time, Z angular velocity forearm, and Z angular velocity upperarm 
time = raw_data(:,1);
angveloZ_forearm = raw_data(:,4);

%% Report max angular velocity of forearm
abs_angvelo = abs(angveloZ_forearm);
[~,velo_idx] = max(abs_angvelo);
max_angvelo = angveloZ_forearm(velo_idx);

%% Graph both angular velocity data sets v. time
figure(1);
plot(time,angveloZ_forearm,'m');
title(['Z Direction Angular Velocity in Forearm Bracket ' volt]);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
saveas(gcf,[trialname,'_Raw_Torque_Test.png']);
%close(1);

%% Integrate
% Integrate angular velocity to get angular position
angular_position = cumtrapz(time, angveloZ_forearm);

%% Convert angular position data to degrees & report max
angular_position_deg = angular_position * (180/pi);

figure(2);
plot(time, angular_position_deg, 'k', 'DisplayName', 'Angular Position in Degrees');
xlabel('Time (s)');
ylabel('Angular Position (deg)');
title(['Integrated Angular Position vs. Time ' volt]);
saveas(gcf,[trialname,'_Integrated_Torque_Test.png']);
%close(2);

%% Truncate integrated data to get rid of flatline and positive values at beginning
% This will make the first angle in the vector the moment of impact with
% the scale
abs_angle = abs(angular_position_deg);
idx_to_remove = find(abs_angle < 0.01);

angular_position_deg = angular_position_deg(idx_to_remove(end)+1:end);

angular_position_deg(angular_position_deg > 0) = [];

%% Calculate slopes and find where there is a large change from one data point to another
threshold = 0.01; % Define what is considered a large change
[index, slopes] = detect_large_slope_change(angular_position_deg, threshold);

function [index, slopes] = detect_large_slope_change(data, threshold)
    % Function to detect large slope change in a data series
    % data: input data series
    % threshold: the change in slope that is considered large
    % index: the index at which the large change in slope is detected
    % slopes: the calculated slopes up to the detection point
    
    n = length(data);
    slopes = zeros(n-1,1);
    index = NaN; % Initialize index to NaN

    % Calculate slopes between consecutive points
    for i = 1:n-1
        slopes(i) = data(i+1) - data(i);
        
        % Check if the change in slope exceeds the threshold
        if i > 1 && abs(slopes(i) - slopes(i-1)) > threshold
            index = i;
            break;
        end
    end
    
    % If no large change in slope is detected, set index to empty
    if isnan(index)
        index = 'No significant slope change detected';
    end
end