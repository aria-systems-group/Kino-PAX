close all
clc
clear all

% Parameters
numFiles = 1;
radius = 5;
N = 8;
n = 4;
sampleSize = 10;
stateSize = 6;
controlSize = 3;
xGoal = [80, 95, 90];
alphaValue = 0.7;
STEP_SIZE = .1;

% Obstacle file path
obstacleFilePath = '/home/nicolas/dev/research/KPAX/include/config/obstacles/quadNarrowPassage/obstacles.csv';
obstacles = readmatrix(obstacleFilePath);

for i = 1:numFiles

    outputFilePath = "/home/nicolas/dev/research/KPAX/build/solutions/Output.csv";
    samples = readmatrix(outputFilePath);

    figure;
    hold on;
    axis equal;
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    plot3(samples(1,1), samples(1,2), samples(1,3), 'ko', 'MarkerFaceColor', 'k');

    % goal sphere plot
    [X, Y, Z] = sphere(20);
    surf(radius * X + xGoal(1), radius * Y + xGoal(2), radius * Z + xGoal(3), ...
         'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    % Obstacle Plot
    for j = 1:size(obstacles, 1)
        x_min = obstacles(j, 1);
        y_min = obstacles(j, 2);
        z_min = obstacles(j, 3);
        x_max = obstacles(j, 4);
        y_max = obstacles(j, 5);
        z_max = obstacles(j, 6);
        vertices = [
            x_min, y_min, z_min;
            x_max, y_min, z_min;
            x_max, y_max, z_min;
            x_min, y_max, z_min;
            x_min, y_min, z_max;
            x_max, y_min, z_max;
            x_max, y_max, z_max;
            x_min, y_max, z_max];
        faces = [
            1, 2, 6, 5;
            2, 3, 7, 6;
            3, 4, 8, 7;
            4, 1, 5, 8;
            1, 2, 3, 4;
            5, 6, 7, 8];
        patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', 0.2);
    end

    % Add light source
    camlight('headlight'); 
    lighting gouraud;

    % Plot paths
    segmentX = [];
    segmentY = [];
    segmentZ = [];
    for j = 2:size(samples, 1)
        x0 = samples(j-1, 1:stateSize);
        segmentX = [x0(1)];
        segmentY = [x0(2)];
        segmentZ = [x0(3)];
        u = samples(j, stateSize+1:sampleSize-1);
        duration = samples(j, sampleSize);
        numDisc = duration/STEP_SIZE;
        % x = x0(1);
        % y = x0(2);
        % z = x0(3);
        % vx = x0(4);
        % vy = x0(5);
        % vz = x0(6);
        % ax = u(1);
        % ay = u(2);
        % az = u(3);
        x = x0(1);
        y = x0(2);
        z = x0(3);
        yaw = x0(4);
        pitch = x0(5);
        v = x0(6);
        yawRate = u(1);
        pitchRate = u(2);
        % a = u(3);
        % for k = 1:(numDisc)
        %     % x = x + (vx + (vx + 2 * (vx + ax * STEP_SIZE / 2) + (vx + ax * STEP_SIZE))) * STEP_SIZE / 6;
        %     % y = y + (vy + (vy + 2 * (vy + ay * STEP_SIZE / 2) + (vy + ay * STEP_SIZE))) * STEP_SIZE / 6;
        %     % z = z + (vz + (vz + 2 * (vz + az * STEP_SIZE / 2) + (vz + az * STEP_SIZE))) * STEP_SIZE / 6;
        %     % vx = vx + (ax + 2 * ax + 2 * ax + ax) * STEP_SIZE / 6;
        %     % vy = vy + (ay + 2 * ay + 2 * ay + ay) * STEP_SIZE / 6;
        %     % vz = vz + (az + 2 * az + 2 * az + az) * STEP_SIZE / 6;
        %     x = x + (STEP_SIZE / 6.0) * ...
        %         (v * cos(pitch) * cos(yaw) + ...
        %          2.0 * ((v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * cos(yaw + 0.5 * STEP_SIZE * yawRate) + ...
        %                 (v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * cos(yaw + 0.5 * STEP_SIZE * yawRate)) + ...
        %          (v + STEP_SIZE * a) * cos(pitch + STEP_SIZE * pitchRate) * cos(yaw + STEP_SIZE * yawRate));
        % 
        %     y = y + (STEP_SIZE / 6.0) * ...
        %         (v * cos(pitch) * sin(yaw) + ...
        %          2.0 * ((v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * sin(yaw + 0.5 * STEP_SIZE * yawRate) + ...
        %                 (v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * sin(yaw + 0.5 * STEP_SIZE * yawRate)) + ...
        %          (v + STEP_SIZE * a) * cos(pitch + STEP_SIZE * pitchRate) * sin(yaw + STEP_SIZE * yawRate));
        % 
        %     z = z + (STEP_SIZE / 6.0) * ...
        %         (v * sin(pitch) + ...
        %          2.0 * ((v + 0.5 * STEP_SIZE * a) * sin(pitch + 0.5 * STEP_SIZE * pitchRate) + ...
        %                 (v + 0.5 * STEP_SIZE * a) * sin(pitch + 0.5 * STEP_SIZE * pitchRate)) + ...
        %          (v + STEP_SIZE * a) * sin(pitch + STEP_SIZE * pitchRate));
        % 
        %     yaw = yaw + STEP_SIZE * yawRate;
        %     pitch = pitch + STEP_SIZE * pitchRate;
        %     v = v + (STEP_SIZE / 6.0) * (a + 2.0 * (a + a) + a);
        %     segmentX = [segmentX, x];
        %     segmentY = [segmentY, y];
        %     segmentZ = [segmentZ, z];
        % end
        % segmentX = [segmentX, samples(j, 1)];
        % segmentY = [segmentY, samples(j, 2)];
        % segmentZ = [segmentZ, samples(j, 3)];
        % plot3(segmentX, segmentY, segmentZ, '-.', 'Color', 'k', 'LineWidth', 1);
        plot3(x, y, z, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 2);
    end

    view(3);
    drawnow;
end
