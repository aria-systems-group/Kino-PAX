close all
clc
clear all

% Parameters
numFiles = 9;
radius = .05;
N = 8;
n = 4;
sampleSize = 10;
stateSize = 6;
controlSize = 2;
xGoal = [.70, .95, .90];
alpha = .7;
STEP_SIZE = .1;
model = 2;

% Obstacle file path
obstacleFilePath = '/home/nicolas/dev/research/KPAX/include/config/obstacles/pillars/obstacles.csv';
obstacles = gpuArray(readmatrix(obstacleFilePath));

treeSizePath = "/home/nicolas/dev/research/KPAX/build/Data/TreeSize/TreeSize0/treeSize.csv";
treeSizes = gpuArray(readmatrix(treeSizePath));
treeSizes = [0; treeSizes];

colors = gpuArray([0 0 1;  % Blue
                   0 .9 .2;  % Green
                   1 0 1;  % Pink
                   .7 .7 0;  % Yellow
                   0 .7 .7; % Turquoise
                   1 .5 0]); % Orange

fig = figure('Position', [100, 100, 1000, 1000]); 
hold on;
axis equal;
axis off;
title('Iteration 0');

sampleFilePath = "/home/nicolas/dev/research/KPAX/build/Data/Samples/Samples0/samples1.csv";
samples = gpuArray(readmatrix(sampleFilePath));

controlPath = '/home/nicolas/dev/research/KPAX/build/Data/ControlPathToGoal/ControlPathToGoal0/controlPathToGoal.csv';
controls = gpuArray(flipud(readmatrix(controlPath)));
% controls = [samples(1,1), samples(1,2), samples(1,3), samples(1,4), samples(1,5), samples(1,6), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; controls];

plot3(gather(samples(1,1)), gather(samples(1,2)), gather(samples(1,3)), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 10);

[X, Y, Z] = sphere(20);
surf(radius * gather(X) + xGoal(1), radius * gather(Y) + xGoal(2), radius * gather(Z) + xGoal(3), ...
     'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

for j = 1:size(obstacles, 1)
    x_min = obstacles(j, 1);
    y_min = obstacles(j, 2);
    z_min = obstacles(j, 3);
    x_max = obstacles(j, 4);
    y_max = obstacles(j, 5);
    z_max = obstacles(j, 6);
    vertices = gpuArray([
        x_min, y_min, z_min;
        x_max, y_min, z_min;
        x_max, y_max, z_min;
        x_min, y_max, z_min;
        x_min, y_min, z_max;
        x_max, y_min, z_max;
        x_max, y_max, z_max;
        x_min, y_max, z_max]);
    faces = gpuArray([
        1, 2, 6, 5;
        2, 3, 7, 6;
        3, 4, 8, 7;
        4, 1, 5, 8;
        1, 2, 3, 4;
        5, 6, 7, 8]);
    patch('Vertices', gather(vertices), 'Faces', gather(faces), 'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', alpha);
end

camlight('headlight'); 
camlight('right');
lighting phong;

% view(3);
% drawnow;
% saveas(gcf, 'figs/KGMT_Iteration_0.jpg');
% print('figs/KGMT_Iteration_0.jpg', '-djpeg', '-r300');
% 
view(2);
drawnow;
saveas(gcf, 'figs/top_KGMT_Iteration_0.jpg');
print('figs/top_KGMT_Iteration_0.jpg', '-djpeg', '-r300');
% 
% midY = 0.5 * xGoal(2); 
% midZ = 0.5 * xGoal(3); 
% campos([0, midY, xGoal(3) + 1]); 
% camtarget([0, midY, midZ]); 
% view([-.4, -.2, 0.5]);
% drawnow;
% saveas(gcf, 'figs/xAxis_KGMT_Iteration_0.jpg');
% print('figs/xAxis_KGMT_Iteration_0.jpg', '-djpeg', '-r300'); 

close(gcf);
iteration = 1;

for i = 1:numFiles
    sampleFilePath = "/home/nicolas/dev/research/KPAX/build/Data/Samples/Samples0/samples" + i + ".csv";
    parentFilePath = "/home/nicolas/dev/research/KPAX/build/Data/Parents/Parents0/parents" + i + ".csv";

    samples = gpuArray(readmatrix(sampleFilePath));
    parentRelations = gpuArray(readmatrix(parentFilePath));

    fig = figure('Position', [100, 100, 1000, 1000]); 
    hold on;
    axis equal;
    axis off;
    % title(sprintf('Iteration %d', i));

    plot3(gather(samples(1,1)), gather(samples(1,2)), gather(samples(1,3)), 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 10);

    [X, Y, Z] = sphere(20);
    surf(radius * gather(X) + xGoal(1), radius * gather(Y) + xGoal(2), radius * gather(Z) + xGoal(3), ...
         'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    for j = 1:size(obstacles, 1)
        x_min = obstacles(j, 1);
        y_min = obstacles(j, 2);
        z_min = obstacles(j, 3);
        x_max = obstacles(j, 4);
        y_max = obstacles(j, 5);
        z_max = obstacles(j, 6);
        vertices = gpuArray([
            x_min, y_min, z_min;
            x_max, y_min, z_min;
            x_max, y_max, z_min;
            x_min, y_max, z_min;
            x_min, y_min, z_max;
            x_max, y_min, z_max;
            x_max, y_max, z_max;
            x_min, y_max, z_max]);
        faces = gpuArray([
            1, 2, 6, 5;
            2, 3, 7, 6;
            3, 4, 8, 7;
            4, 1, 5, 8;
            1, 2, 3, 4;
            5, 6, 7, 8]);
        patch('Vertices', gather(vertices), 'Faces', gather(faces), 'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', alpha);
    end

    camlight('headlight'); 
    camlight('right');
    lighting phong;

    colorIndex = 1;
    for j = 2:size(parentRelations, 1)
        if j > treeSizes(iteration)
            colorIndex = 3;
        else
            colorIndex = 1;
        end
        if parentRelations(j) == -1
            iteration = iteration + 1;
            break;
        end
        x0 = samples((parentRelations(j) + 1), 1:stateSize);
        sample = samples(j, :);
        if model == 1
            [segmentX, segmentY, segmentZ] = propDoubleIntegrator(x0, sample, STEP_SIZE, stateSize, sampleSize);
        elseif model == 2
            [segmentX, segmentY, segmentZ] = propDubinsAirplane(x0, sample, STEP_SIZE, stateSize, sampleSize);
        elseif model == 3
            [segmentX, segmentY, segmentZ] = propQuad(x0, sample, STEP_SIZE, stateSize, sampleSize);
        end
        plot3(gather(segmentX), gather(segmentY), gather(segmentZ), '-.', 'Color', 'k', 'LineWidth', 0.01);
        plot3(gather(samples(j, 1)), gather(samples(j, 2)), gather(samples(j, 3)), 'o', 'Color', gather(colors(colorIndex, :)), 'MarkerFaceColor', gather(colors(colorIndex, :)), 'MarkerSize', 2);
    end

    % Define the filename for the GIF
    % gifFilename = sprintf('figs/KGMT_Iteration_%d.gif', i);


    if i == numFiles
        for j = 2:size(controls, 1)
            x0 = controls(j-1, 1:stateSize);
            sample = controls(j,:);
            if model == 1
                [segmentX, segmentY, segmentZ] = propDoubleIntegrator(x0, sample, STEP_SIZE, stateSize, sampleSize);
            elseif model == 2
                [segmentX, segmentY, segmentZ] = propDubinsAirplane(x0, sample, STEP_SIZE, stateSize, sampleSize);
            elseif model == 3
                [segmentX, segmentY, segmentZ] = propQuad(x0, sample, STEP_SIZE, stateSize, sampleSize);
            end
            
            plot3(gather(segmentX), gather(segmentY), gather(segmentZ), 'Color', 'g', 'LineWidth', 1);
            plot3(gather(controls(j, 1)), gather(controls(j, 2)), gather(controls(j, 3)), 'o', 'Color', gather(colors(colorIndex, :)), 'MarkerFaceColor', gather(colors(colorIndex, :)), 'MarkerSize', 2);
        end
    end

    % Set up the initial view
    % view(3);
    % axis vis3d; % Maintain aspect ratio during rotation
    % 
    % % Rotate and capture frames for 360-degree view
    % for angle = 0:1:360  % Adjust step size for smoother or faster rotation
    %     view(angle, 30);
    %     % Capture the plot as a frame
    %     frame = getframe(gcf);
    %     im = frame2im(frame);
    %     [imind, cm] = rgb2ind(im, 256);
    % 
    %     % Write to the GIF file
    %     if angle == 0
    %         % Create the GIF file on the first iteration
    %         imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    %     else
    %         % Append to the GIF file on subsequent iterations
    %         imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    %     end
    % end

    % view(3);
    % drawnow;
    % saveas(gcf, sprintf('figs/KGMT_Iteration_%d.jpg', i));
    % print(sprintf('figs/KGMT_Iteration_%d.jpg', i), '-djpeg', '-r300');
    % 
    view(2);
    drawnow;
    saveas(gcf, sprintf('figs/top_KGMT_Iteration_%d.jpg', i));
    print(sprintf('figs/top_KGMT_Iteration_%d.jpg', i), '-djpeg', '-r300');
    % 
    % midY = (min(gather(samples(:,2))) + max(gather(samples(:,2)))) / 2;
    % midZ = (min(gather(samples(:,3))) + max(gather(samples(:,3)))) / 2;
    % campos([0, midY, max(gather(samples(:,3))) + 1]);
    % camtarget([0, midY, midZ]);
    % view([-.4, -.2, 0.5]);
    % drawnow;
    % 
    % saveas(gcf, sprintf('figs/xAxis_KGMT_Iteration_%d.jpg', i));
    % print(sprintf('figs/xAxis_KGMT_Iteration_%d.jpg', i), '-djpeg', '-r300');

    close(gcf);
end

function [segmentX, segmentY, segmentZ] = propDoubleIntegrator(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = gpuArray(x0(1));
    segmentY = gpuArray(x0(2));
    segmentZ = gpuArray(x0(3));
    u = gpuArray(sample(stateSize+1:sampleSize-1));
    duration = gpuArray(sample(sampleSize));
    numDisc = gpuArray(duration / STEP_SIZE);
    x = gpuArray(x0(1));
    y = gpuArray(x0(2));
    z = gpuArray(x0(3));
    vx = gpuArray(x0(4));
    vy = gpuArray(x0(5));
    vz = gpuArray(x0(6));
    ax = u(1);
    ay = u(2);
    az = u(3);
    for k = 1:numDisc
        x = x + (vx + (vx + 2 * (vx + ax * STEP_SIZE / 2) + (vx + ax * STEP_SIZE))) * STEP_SIZE / 6;
        y = y + (vy + (vy + 2 * (vy + ay * STEP_SIZE / 2) + (vy + ay * STEP_SIZE))) * STEP_SIZE / 6;
        z = z + (vz + (vz + 2 * (vz + az * STEP_SIZE / 2) + (vz + az * STEP_SIZE))) * STEP_SIZE / 6;
        vx = vx + (ax + 2 * ax + 2 * ax + ax) * STEP_SIZE / 6;
        vy = vy + (ay + 2 * ay + 2 * ay + ay) * STEP_SIZE / 6;
        vz = vz + (az + 2 * az + 2 * az + az) * STEP_SIZE / 6;
        segmentX = [segmentX, x];
        segmentY = [segmentY, y];
        segmentZ = [segmentZ, z];
    end
    % segmentX = [segmentX, gpuArray(sample(1))];
    % segmentY = [segmentY, gpuArray(sample(2))];
    % segmentZ = [segmentZ, gpuArray(sample(3))];
end

function [segmentX, segmentY, segmentZ] = propDubinsAirplane(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = gpuArray(x0(1));
    segmentY = gpuArray(x0(2));
    segmentZ = gpuArray(x0(3));
    u = gpuArray(sample(stateSize+1:sampleSize-1));
    duration = gpuArray(sample(sampleSize));
    numDisc = gpuArray(duration / STEP_SIZE);
    x = gpuArray(x0(1));
    y = gpuArray(x0(2));
    z = gpuArray(x0(3));
    yaw = gpuArray(x0(4));
    pitch = gpuArray(x0(5));
    v = gpuArray(x0(6));
    yawRate = u(1);
    pitchRate = u(2);
    a = u(3);
    for k = 1:numDisc
        x = x + (STEP_SIZE / 6.0) * ...
            (v * cos(pitch) * cos(yaw) + ...
             2.0 * ((v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * cos(yaw + 0.5 * STEP_SIZE * yawRate) + ...
                    (v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * cos(yaw + 0.5 * STEP_SIZE * yawRate)) + ...
             (v + STEP_SIZE * a) * cos(pitch + STEP_SIZE * pitchRate) * cos(yaw + STEP_SIZE * yawRate));
        
        y = y + (STEP_SIZE / 6.0) * ...
            (v * cos(pitch) * sin(yaw) + ...
             2.0 * ((v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * sin(yaw + 0.5 * STEP_SIZE * yawRate) + ...
                    (v + 0.5 * STEP_SIZE * a) * cos(pitch + 0.5 * STEP_SIZE * pitchRate) * sin(yaw + 0.5 * STEP_SIZE * yawRate)) + ...
             (v + STEP_SIZE * a) * cos(pitch + STEP_SIZE * pitchRate) * sin(yaw + STEP_SIZE * yawRate));
        
        z = z + (STEP_SIZE / 6.0) * ...
            (v * sin(pitch) + ...
             2.0 * ((v + 0.5 * STEP_SIZE * a) * sin(pitch + 0.5 * STEP_SIZE * pitchRate) + ...
                    (v + 0.5 * STEP_SIZE * a) * sin(pitch + 0.5 * STEP_SIZE * pitchRate)) + ...
             (v + STEP_SIZE * a) * sin(pitch + STEP_SIZE * pitchRate));
        
        yaw = yaw + STEP_SIZE * yawRate;
        pitch = pitch + STEP_SIZE * pitchRate;
        v = v + (STEP_SIZE / 6.0) * (a + 2.0 * (a + a) + a);
        segmentX = [segmentX, x];
        segmentY = [segmentY, y];
        segmentZ = [segmentZ, z];
    end
    segmentX = [segmentX, gpuArray(sample(1))];
    segmentY = [segmentY, gpuArray(sample(2))];
    segmentZ = [segmentZ, gpuArray(sample(3))];
end

function [segmentX, segmentY, segmentZ] = propQuad(x0, sample, STEP_SIZE, stateSize, sampleSize)
    segmentX = gpuArray(x0(1));
    segmentY = gpuArray(x0(2));
    segmentZ = gpuArray(x0(3));
    u = gpuArray(sample(stateSize+1:sampleSize-1));
    duration = gpuArray(sample(sampleSize));
    numDisc = gpuArray(duration / STEP_SIZE);
    Zc = u(1);
    Lc = u(2);
    Mc = u(3);
    Nc = u(4);
    h0 = x0(1:12);
    x = x0(1);
    y = x0(2);
    z = x0(3);

    for k = 1:numDisc
        h1 = ode(h0, Zc, Lc, Mc, Nc);
        h2 = ode(h0 + 0.5 * STEP_SIZE * h1, Zc, Lc, Mc, Nc);
        h3 = ode(h0 + 0.5 * STEP_SIZE * h2, Zc, Lc, Mc, Nc);
        h4 = ode(h0 + STEP_SIZE * h3, Zc, Lc, Mc, Nc);
        
        h0 = h0 + (STEP_SIZE / 6) * (h1 + 2 * h2 + 2 * h3 + h4);
        
        x = h0(1);
        y = h0(2);
        z = h0(3);
        
        segmentX = [segmentX, x];
        segmentY = [segmentY, y];
        segmentZ = [segmentZ, z];
    end
    
    segmentX = [segmentX, gpuArray(sample(1))];
    segmentY = [segmentY, gpuArray(sample(2))];
    segmentZ = [segmentZ, gpuArray(sample(3))];
end

function x0dot = ode(x0, Zc, Lc, Mc, Nc)
        
    NU = 10e-3;
    MU = 2e-6;
    IX = 1.0;
    IY = 1.0;
    IZ = 2.0;
    GRAVITY = -9.81;
    MASS = 1.0;
    MASS_INV = 1.0 / MASS;

    
    phi   = x0(4);
    theta = x0(5);
    psi   = x0(6);
    u     = x0(7);
    v     = x0(8);
    w     = x0(9);
    p     = x0(10);
    q     = x0(11);
    r     = x0(12);

    x0dot = zeros(1, 12);

    x0dot(1) = cos(theta) * cos(psi) * u + (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) * v + ...
               (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;

    x0dot(2) = cos(theta) * sin(psi) * u + (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) * v + ...
               (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;

    x0dot(3) = -sin(theta) * u + sin(phi) * cos(theta) * v + cos(phi) * cos(theta) * w;

    x0dot(4) = p + (q * sin(phi) + r * cos(phi)) * tan(theta);

    x0dot(5) = q * cos(phi) - r * sin(phi);

    x0dot(6) = (q * sin(phi) + r * cos(phi)) / cos(theta);

    XYZ = -NU * sqrt(u^2 + v^2 + w^2);
    X   = XYZ * u;
    x0dot(7)  = (r * v - q * w) - GRAVITY * sin(theta) + MASS_INV * X;

    Y  = XYZ * v;
    x0dot(8) = (p * w - r * u) + GRAVITY * cos(theta) * sin(phi) + MASS_INV * Y;

    Z  = XYZ * w;
    x0dot(9) = (q * u - p * v) + GRAVITY * cos(theta) * cos(phi) + MASS_INV * Z + MASS_INV * Zc;

    LMN = -MU * sqrt(p^2 + q^2 + r^2);
    L   = LMN * p;
    x0dot(10) = (IY - IZ) / IX * q * r + (1 / IX) * L + (1 / IX) * Lc;

    M   = LMN * q;
    x0dot(11) = (IZ - IX) / IY * p * r + (1 / IY) * M + (1 / IY) * Mc;

    N   = LMN * r;
    x0dot(12) = (IX - IY) / IZ * p * q + (1 / IZ) * N + (1 / IZ) * Nc;
end