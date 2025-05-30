classdef PathFollowingRobot < handle
    properties
        SerialPort
        Waypoints
        CurrentWaypointIndex
        FuzzySystem
        Running
        MapFigure
        PathPlot
        RobotPlot
        WaypointPlot
        LastPositionTime
        PositionHistory
        LinearSpeed = 0.2 % meters per second
        PositionThreshold = 2 % meters
        HeadingThreshold = 15 % degrees
        LogFile
        LogData
        LogStartTime
        PerformanceFigure
        LastValidPosition = [0, 0] % Store last valid position
    end
    
    methods
        function obj = PathFollowingRobot(portName)
            % Initialize serial port
            obj.SerialPort = serialport(portName, 115200);
            configureTerminator(obj.SerialPort, "LF");
            flush(obj.SerialPort);
            
            % Load waypoints (example coordinates)
            obj.Waypoints = [
                7.214721, 124.248377;
                7.214782, 124.248421;
                % Add more waypoints as needed
            ];
            
            obj.CurrentWaypointIndex = 1;
            obj.Running = false;
            obj.PositionHistory = [];
            obj.LastPositionTime = datetime('now');
            
            % Initialize logging
            obj.initializeLogging();
            
            % Initialize fuzzy logic system
            obj.initializeFuzzySystem();
            
            % Initialize map visualization
            obj.initializeMap();
            
            disp('Robot initialized and ready to run');
        end
        
        function initializeLogging(obj)
            timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
            obj.LogFile = sprintf('robot_performance_%s.csv', timestamp);
            
            obj.LogData = table('Size', [0, 13], ...
                'VariableTypes', {'datetime', 'double', 'double', 'double', 'double', ...
                                'double', 'double', 'double', 'double', 'double', 'double', ...
                                'double', 'double'}, ...
                'VariableNames', {'Timestamp', 'Latitude', 'Longitude', 'Speed', 'Heading', ...
                                 'TargetLat', 'TargetLon', 'CrossTrackError', 'HeadingError', ...
                                 'DistanceToTarget', 'LeftPWM', 'RightPWM', 'PathBearing'});
            
            % Write header to CSV file
            fid = fopen(obj.LogFile, 'w');
            fprintf(fid, '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n', ...
                'Timestamp', 'Latitude', 'Longitude', 'Speed', 'Heading', ...
                'TargetLat', 'TargetLon', 'CrossTrackError', 'HeadingError', ...
                'DistanceToTarget', 'LeftPWM', 'RightPWM', 'PathBearing');
            fclose(fid);
            
            obj.LogStartTime = datetime('now');
        end
        
        function logData(obj, timestamp, lat, lon, speed, heading, ...
                        targetLat, targetLon, crossTrackError, headingError, ...
                        distance, leftPWM, rightPWM, pathBearing)
            newRow = {timestamp, lat, lon, speed, heading, ...
                     targetLat, targetLon, crossTrackError, headingError, ...
                     distance, leftPWM, rightPWM, pathBearing};
            obj.LogData = [obj.LogData; newRow];
            
            fid = fopen(obj.LogFile, 'a');
            fprintf(fid, '%s,%.8f,%.8f,%.2f,%.2f,%.8f,%.8f,%.2f,%.2f,%.2f,%d,%d,%.2f\n', ...
                datestr(timestamp, 'yyyy-mm-dd HH:MM:SS.FFF'), ...
                lat, lon, speed, heading, ...
                targetLat, targetLon, crossTrackError, headingError, ...
                distance, leftPWM, rightPWM, pathBearing);
            fclose(fid);
        end
        
        function initializeFuzzySystem(obj)
            fis = mamfis('Name', 'path_following');
            
            % Input 1: Cross-track error (meters)
            fis = addInput(fis, [-10 10], 'Name', 'cross_track_error');
            fis = addMF(fis, 'cross_track_error', 'trimf', [-10 -10 -5], 'Name', 'Left');
            fis = addMF(fis, 'cross_track_error', 'trimf', [-5 0 5], 'Name', 'Center');
            fis = addMF(fis, 'cross_track_error', 'trimf', [5 10 10], 'Name', 'Right');
            
            % Input 2: Heading error (degrees)
            fis = addInput(fis, [-180 180], 'Name', 'heading_error');
            fis = addMF(fis, 'heading_error', 'trimf', [-180 -180 -45], 'Name', 'Left');
            fis = addMF(fis, 'heading_error', 'trimf', [-45 0 45], 'Name', 'Straight');
            fis = addMF(fis, 'heading_error', 'trimf', [45 180 180], 'Name', 'Right');
            
            % Output 1: Left motor PWM
            fis = addOutput(fis, [-255 255], 'Name', 'pwm_left');
            fis = addMF(fis, 'pwm_left', 'trimf', [-255 -255 -127], 'Name', 'Reverse');
            fis = addMF(fis, 'pwm_left', 'trimf', [-191 0 191], 'Name', 'Slow');
            fis = addMF(fis, 'pwm_left', 'trimf', [127 255 255], 'Name', 'Forward');
            
            % Output 2: Right motor PWM
            fis = addOutput(fis, [-255 255], 'Name', 'pwm_right');
            fis = addMF(fis, 'pwm_right', 'trimf', [-255 -255 -127], 'Name', 'Reverse');
            fis = addMF(fis, 'pwm_right', 'trimf', [-191 0 191], 'Name', 'Slow');
            fis = addMF(fis, 'pwm_right', 'trimf', [127 255 255], 'Name', 'Forward');
            
            % Rule Base
            ruleList = [
                1 1 3 1 1 1;   % Far left, heading left -> strong right turn
                1 2 3 2 1 1;   % Far left, straight -> medium right turn
                1 3 3 3 1 1;   % Far left, heading right -> gentle right turn
                
                2 1 2 1 1 1;   % Slightly left, heading left -> gentle right turn
                2 2 3 3 1 1;   % On path, straight -> go straight
                2 3 1 2 1 1;   % Slightly left, heading right -> gentle left turn
                
                3 1 1 1 1 1;   % Far right, heading left -> gentle left turn
                3 2 2 3 1 1;   % Far right, straight -> medium left turn
                3 3 1 3 1 1;   % Far right, heading right -> strong left turn
            ];
            
            fis = addRule(fis, ruleList);
            obj.FuzzySystem = fis;
        end
        
        function initializeMap(obj)
            obj.MapFigure = figure('Name', 'Robot Path Monitoring', 'NumberTitle', 'off');
            
            [lat, lon] = obj.convertToNumeric(obj.Waypoints);
            geoplot(lat, lon, 'g-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
            hold on;
            geoscatter(lat, lon, 'filled', 'MarkerFaceColor', 'red', 'DisplayName', 'Waypoints');
            
            obj.PathPlot = geoplot(lat(1), lon(1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Path');
            obj.RobotPlot = geoscatter(lat(1), lon(1), 100, 'filled', 'MarkerFaceColor', 'blue', 'DisplayName', 'Robot');
            
            geobasemap('satellite');
            title('Robot Path Monitoring');
            legend('Location', 'best');
            drawnow;
        end
        
        function updateMap(obj, currentLat, currentLon)
            xData = obj.PathPlot.LatitudeData;
            yData = obj.PathPlot.LongitudeData;
            
            obj.PathPlot.LatitudeData = [xData, currentLat];
            obj.PathPlot.LongitudeData = [yData, currentLon];
            obj.PositionHistory = [obj.PositionHistory; currentLat, currentLon];
            
            obj.RobotPlot.LatitudeData = currentLat;
            obj.RobotPlot.LongitudeData = currentLon;
            
            targetLat = obj.Waypoints(obj.CurrentWaypointIndex, 1);
            targetLon = obj.Waypoints(obj.CurrentWaypointIndex, 2);
            geoscatter(targetLat, targetLon, 100, 'filled', 'MarkerFaceColor', 'yellow', 'DisplayName', 'Current Target');
            
            latLim = [min([currentLat; obj.Waypoints(:,1)])-0.0001, max([currentLat; obj.Waypoints(:,1)])+0.0001];
            lonLim = [min([currentLon; obj.Waypoints(:,2)])-0.0001, max([currentLon; obj.Waypoints(:,2)])+0.0001];
            geolimits(latLim, lonLim);
            
            legend('Planned Path', 'Waypoints', 'Robot Path', 'Robot', 'Current Target');
            
            drawnow;
        end
        
        function [crossTrackError, headingError, pathBearing] = calculateErrors(obj, currentLat, currentLon, currentHeading)
            currentTarget = obj.Waypoints(obj.CurrentWaypointIndex, :);
            
            if obj.CurrentWaypointIndex < size(obj.Waypoints, 1)
                nextTarget = obj.Waypoints(obj.CurrentWaypointIndex+1, :);
                pathBearing = obj.calculateBearing(currentTarget(1), currentTarget(2), nextTarget(1), nextTarget(2));
            else
                pathBearing = currentHeading;
            end
            
            bearingToTarget = obj.calculateBearing(currentLat, currentLon, currentTarget(1), currentTarget(2));
            distanceToTarget = obj.calculateDistance(currentLat, currentLon, currentTarget(1), currentTarget(2));
            angleDiff = obj.wrapTo180(bearingToTarget - pathBearing);
            crossTrackError = distanceToTarget * sind(angleDiff);
            
            headingError = obj.wrapTo180(pathBearing - currentHeading);
        end
        
        function start(obj)
            obj.Running = true;
            obj.run();
        end
        
        function stop(obj)
            obj.Running = false;
            writeline(obj.SerialPort, "CMD:0,0");
            obj.plotPerformance();
            saveas(obj.MapFigure, strrep(obj.LogFile, '.csv', '_map.png'));
        end
        
        function run(obj)
            while obj.Running
                if obj.SerialPort.NumBytesAvailable > 0
                    data = readline(obj.SerialPort);
                    
                    if startsWith(data, "GPS:")
                        try
                            if strcmp(data, "GPS:INVALID") || strcmp(data, "GPS:LOST")
                                fprintf('Invalid GPS data received\n');
                                continue;
                            end
                            
                            gpsData = sscanf(data, "GPS:%f,%f,%f,%f");
                            if length(gpsData) == 4
                                currentLat = gpsData(1);
                                currentLon = gpsData(2);
                                speed = gpsData(3);
                                heading = gpsData(4);
                                timestamp = datetime('now');
                                
                                if currentLat == 0 && currentLon == 0
                                    fprintf('Invalid GPS position (0,0)\n');
                                    continue;
                                end
                                
                                obj.LastValidPosition = [currentLat, currentLon];
                                obj.updateMap(currentLat, currentLon);
                                
                                target = obj.Waypoints(obj.CurrentWaypointIndex, :);
                                targetLat = target(1);
                                targetLon = target(2);
                                distance = obj.calculateDistance(currentLat, currentLon, targetLat, targetLon);
                                
                                [crossTrackError, headingError, pathBearing] = ...
                                    obj.calculateErrors(currentLat, currentLon, heading);
                                
                                boundedCrossTrack = max(-10, min(10, crossTrackError));
                                inputs = [boundedCrossTrack, headingError];
                                outputs = evalfis(obj.FuzzySystem, inputs);
                                
                                distanceFactor = min(1, distance / obj.PositionThreshold);
                                leftPWM = round(outputs(1) * distanceFactor);
                                rightPWM = round(outputs(2) * distanceFactor);
                                
                                leftPWM = max(-255, min(255, leftPWM));
                                rightPWM = max(-255, min(255, rightPWM));
                                
                                obj.logData(timestamp, currentLat, currentLon, speed, heading, ...
                                           targetLat, targetLon, crossTrackError, headingError, ...
                                           distance, leftPWM, rightPWM, pathBearing);
                                
                                if distance < obj.PositionThreshold
                                    if obj.CurrentWaypointIndex < size(obj.Waypoints, 1)
                                        obj.CurrentWaypointIndex = obj.CurrentWaypointIndex + 1;
                                        fprintf('Reached waypoint %d, moving to waypoint %d\n', ...
                                            obj.CurrentWaypointIndex-1, obj.CurrentWaypointIndex);
                                    else
                                        obj.stop();
                                        disp('Final waypoint reached!');
                                        return;
                                    end
                                end
                                
                                cmdStr = sprintf("CMD:%d,%d", leftPWM, rightPWM);
                                writeline(obj.SerialPort, cmdStr);
                                
                                fprintf('Position: (%.6f, %.6f) | Target: (%.6f, %.6f)\n', ...
                                    currentLat, currentLon, targetLat, targetLon);
                                fprintf('Cross-track: %.2fm | Heading: %.1f° (Error: %.1f°) | Path Bearing: %.1f°\n', ...
                                    crossTrackError, heading, headingError, pathBearing);
                                fprintf('Outputs: L=%d, R=%d | Distance: %.2fm\n\n', ...
                                    leftPWM, rightPWM, distance);
                            end
                        catch ME
                            fprintf('Error processing GPS data: %s\n', ME.message);
                        end
                    end
                end
                pause(0.1);
            end
        end
        
        function distance = calculateDistance(~, lat1, lon1, lat2, lon2)
            R = 6371000; % Earth radius in meters
            dLat = deg2rad(lat2-lat1);
            dLon = deg2rad(lon2-lon1);
            a = sin(dLat/2) * sin(dLat/2) + ...
                cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * ...
                sin(dLon/2) * sin(dLon/2);
            c = 2 * atan2(sqrt(a), sqrt(1-a));
            distance = R * c;
        end
        
        function bearing = calculateBearing(~, lat1, lon1, lat2, lon2)
            lat1 = deg2rad(lat1);
            lon1 = deg2rad(lon1);
            lat2 = deg2rad(lat2);
            lon2 = deg2rad(lon2);
            
            y = sin(lon2-lon1) * cos(lat2);
            x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
            bearing = rad2deg(atan2(y, x));
        end
        
        function angle = wrapTo180(~, angle)
            angle = mod(angle + 180, 360) - 180;
        end
        
        function [lat, lon] = convertToNumeric(~, waypoints)
            lat = waypoints(:,1);
            lon = waypoints(:,2);
        end
        
     function plotPerformance(obj)
    if height(obj.LogData) < 2
        disp('Not enough data to plot performance');
        return;
    end
    
    if isempty(obj.PerformanceFigure) || ~isvalid(obj.PerformanceFigure)
        obj.PerformanceFigure = figure('Name', 'Performance Analysis', 'NumberTitle', 'off');
    else
        figure(obj.PerformanceFigure);
        clf;
    end
    
    elapsedTime = seconds(obj.LogData.Timestamp - obj.LogData.Timestamp(1));
    
    % Plot 1: Cross-track and Heading errors
    subplot(4,2,[1,3]);
    yyaxis left;
    plot(elapsedTime, obj.LogData.CrossTrackError, 'r-', 'DisplayName', 'Cross-track Error (m)');
    ylabel('Cross-track Error (m)');
    
    yyaxis right;
    plot(elapsedTime, obj.LogData.HeadingError, 'b-', 'DisplayName', 'Heading Error (deg)');
    ylabel('Heading Error (deg)');
    title('Navigation Errors');
    legend;
    grid on;
    
    % Plot 2: Distance to Target
    subplot(4,2,2);
    plot(elapsedTime, obj.LogData.DistanceToTarget, 'm-');
    ylabel('Distance (m)');
    title('Distance to Target');
    grid on;
    
    % Plot 3: Motor Commands
    subplot(4,2,4);
    plot(elapsedTime, obj.LogData.LeftPWM, 'b-', 'DisplayName', 'Left PWM');
    hold on;
    plot(elapsedTime, obj.LogData.RightPWM, 'r-', 'DisplayName', 'Right PWM');
    ylabel('PWM Value');
    title('Motor Commands');
    legend;
    grid on;
    
    % Plot 4: Heading vs Path Bearing
    subplot(4,2,[5,7]);
    plot(elapsedTime, obj.LogData.Heading, 'b-', 'DisplayName', 'Robot Heading');
    hold on;
    plot(elapsedTime, obj.LogData.PathBearing, 'r-', 'DisplayName', 'Path Bearing');
    ylabel('Degrees');
    xlabel('Elapsed Time (s)');
    title('Heading vs Path Bearing');
    legend;
    grid on;
    
    % Calculate metrics
    totalTime = elapsedTime(end);
    avgSpeed = mean(obj.LogData.Speed);
    maxSpeed = max(obj.LogData.Speed);
    
    latDiff = diff(obj.LogData.Latitude) * 111320;
    lonDiff = diff(obj.LogData.Longitude) * 111320 * cosd(mean(obj.LogData.Latitude));
    distanceTraveled = sum(sqrt(latDiff.^2 + lonDiff.^2));
    
    avgCrossTrackError = mean(abs(obj.LogData.CrossTrackError));
    maxCrossTrackError = max(abs(obj.LogData.CrossTrackError));
    avgHeadingError = mean(abs(obj.LogData.HeadingError));
    
    % Create a subplot for the metrics text
    subplot(4,2,[6,8]);
    axis off;
    text(0, 0.5, ...
        sprintf(['Total Time: %.1f s\n', ...
                'Distance Traveled: %.1f m\n', ...
                'Avg Speed: %.2f m/s\n', ...
                'Max Speed: %.2f m/s\n', ...
                'Avg Cross-track Error: %.2f m\n', ...
                'Max Cross-track Error: %.2f m\n', ...
                'Avg Heading Error: %.1f°'], ...
        totalTime, distanceTraveled, avgSpeed, maxSpeed, ...
        avgCrossTrackError, maxCrossTrackError, avgHeadingError), ...
        'FontSize', 10);
    
    saveas(obj.PerformanceFigure, strrep(obj.LogFile, '.csv', '_performance.png'));
end
        
        function delete(obj)
            if obj.Running
                obj.stop();
            end
            clear obj.SerialPort;
        end
    end
end
% Check available ports
% serialportlist

% Create robot object (replace 'COM7' with your actual port)
% robot = PathFollowingRobot('COM7');

% Start the robot
% robot.start();

% When done, stop the robot
% robot.stop();

% Clean up
% clear robot;
% instrreset;
