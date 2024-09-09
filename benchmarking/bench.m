clc;
clear all;
close all;

%% consts:
titleSize = 10;
algorithmSize = 9;
figureSize = [100, 100, 1000, 1000];

%% 12DQuad:

% -- File Paths --
kgmtExecutionTimePath = '/home/nicolas/dev/research/KPAX/benchmarking/kgmtJetson/FlyingUni/house/executionTime.csv';
kgmtExpandedNodesPath = '/home/nicolas/dev/research/KPAX/benchmarking/kpax/DubinsAirplane/Data/ExpandedNodes/';
kgmtTreeSizePath = '/home/nicolas/dev/research/KPAX/benchmarking/kpax/DubinsAirplane/Data/TreeSize/';

kgmtStateGridExecutionTimePath = '/home/nicolas/dev/research/KPAX/benchmarking/kgmtStateGrid/FlyingUni/house/executionTime.csv';
kgmtStateGridExpandedNodesPath = '/home/nicolas/dev/research/KPAX/benchmarking/kgmtStateGrid/FlyingUni/Data/ExpandedNodes/';
kgmtStateGridTreeSizePath = '/home/nicolas/dev/research/KPAX/benchmarking/kgmtStateGrid/FlyingUni/Data/TreeSize/';

rrtParallelExecutionTimePath = '/home/nicolas/dev/research/KPAX/benchmarking/rrtParallel/FlyingUni/house/Data/ExecutionTime/executionTime.csv';
rrtParallelTreeSize = '/home/nicolas/dev/research/KPAX/benchmarking/rrtParallel/FlyingUni/Data/Vertices/vertices.csv';
rrtParallelExpandedNodesPath = '/home/nicolas/dev/research/KPAX/benchmarking/rrtParallel/FlyingUni/Data/Iterations/iterations.csv';

estParallelExecutionTimePath = '/home/nicolas/dev/research/KPAX/benchmarking/estParallel/FlyingUni/house/Data/ExecutionTime/executionTime.csv';
estParallelExpandedNodesPath = '/home/nicolas/dev/research/KPAX/benchmarking/estParallel/FlyingUni/Data/Iterations/iterations.csv';
estParallelTreeSize = '/home/nicolas/dev/research/KPAX/benchmarking/estParallel/FlyingUni/Data/Vertices/vertices.csv';

pdstParallelExecutionTimePath = '/home/nicolas/dev/research/KPAX/benchmarking/pdstParallel/FlyingUni/house/Data/ExecutionTime/executionTime.csv';
pdstParallelExpandedNodesPath = '/home/nicolas/dev/research/KPAX/benchmarking/pdstParallel/FlyingUni/Data/Iterations/iterations.csv';
pdstParallelTreeSize = '/home/nicolas/dev/research/KPAX/benchmarking/pdstParallel/FlyingUni/Data/Vertices/vertices.csv';

% -- Execution Time Data --
kgmtExecutionTime = readmatrix(kgmtExecutionTimePath) * 1000;
kgmtStateGridExecutionTime = readmatrix(kgmtStateGridExecutionTimePath) * 1000;
rrtParallelExecutionTime = readmatrix(rrtParallelExecutionTimePath) * 1000;
estParallelExecutionTime = readmatrix(estParallelExecutionTimePath) * 1000;
pdstParallelExecutionTime = readmatrix(pdstParallelExecutionTimePath) * 1000;

% -- Node Expansion / Tree Size Data --
N = length(dir(kgmtExpandedNodesPath))-2;
kgmtExpandedNodes = zeros(N, 1);
kgmtTreeSize = zeros(N, 1);
kgmtFrontierSize = zeros(N, 1);

% kgmtStateGridExpandedNodes = zeros(N, 1);
% kgmtStateGridTreeSize = zeros(N, 1);
% kgmtStateGridFrontierSize = zeros(N, 1);
% for i = 1:N
%     expandedNodesPath = append(kgmtExpandedNodesPath, 'ExpandedNodes', num2str(i-1), '/expandedNodes.csv');
%     treeSizePath = append(kgmtTreeSizePath, 'TreeSize', num2str(i-1), '/treeSize.csv');
%     kgmtExpandedNodes(i) = sum(readmatrix(expandedNodesPath));
%     treeSize = readmatrix(treeSizePath);
%     kgmtTreeSize(i) = treeSize(end);
% 
%     expandedNodesStateGridPath = append(kgmtStateGridExpandedNodesPath, 'ExpandedNodes', num2str(i-1), '/expandedNodes.csv');
%     treeSizeStateGridPath = append(kgmtStateGridTreeSizePath, 'TreeSize', num2str(i-1), '/treeSize.csv');
%     kgmtStateGridExpandedNodes(i) = sum(readmatrix(expandedNodesStateGridPath));
%     treeSizeStateGrid = readmatrix(treeSizeStateGridPath);
%     kgmtStateGridTreeSize(i) = treeSizeStateGrid(end);
% end

% rrtParallelExpandedNodes = readmatrix(rrtParallelExpandedNodesPath);
% rrtParallelTreeSize = readmatrix(rrtParallelTreeSize);
% 
% estParallelExpandedNodes = readmatrix(estParallelExpandedNodesPath);
% estParallelTreeSize = readmatrix(estParallelTreeSize);
% 
% pdstParallelExpandedNodes = readmatrix(pdstParallelExpandedNodesPath);
% pdstParallelTreeSize = readmatrix(pdstParallelTreeSize);

plotBenchmarkResultsDA(kgmtExecutionTime, kgmtStateGridExecutionTime, rrtParallelExecutionTime, estParallelExecutionTime, pdstParallelExecutionTime);

function plotBenchmarkResultsDA(kgmtExecutionTime, kgmtStateGridExecutionTime, rrtParallelExecutionTime, estParallelExecutionTime, pdstParallelExecutionTime)
    
    %% Execution Time
    
    kgmtStateGrid_mean = mean(kgmtStateGridExecutionTime);
    kgmtStateGrid_std = std(kgmtStateGridExecutionTime);
    kgmtStateGrid_min = min(kgmtStateGridExecutionTime);
    kgmtStateGrid_max = max(kgmtStateGridExecutionTime);
    kgmtStateGrid_success = sum(kgmtStateGridExecutionTime < 60000) / length(kgmtStateGridExecutionTime) * 100;

    kgmt_mean = mean(kgmtExecutionTime);
    kgmt_std = std(kgmtExecutionTime);
    kgmt_min = min(kgmtExecutionTime);
    kgmt_max = max(kgmtExecutionTime);
    kgmt_success = sum(kgmtExecutionTime < 60000) / length(kgmtExecutionTime) * 100;
    kgmtJetson_ratio = kgmt_mean / kgmtStateGrid_mean;
    
    rrtParallel_mean = mean(rrtParallelExecutionTime);
    rrtParallel_std = std(rrtParallelExecutionTime);
    rrtParallel_min = min(rrtParallelExecutionTime);
    rrtParallel_max = max(rrtParallelExecutionTime);
    rrtParallel_success = sum(rrtParallelExecutionTime < 60000) / length(rrtParallelExecutionTime) * 100;
    rrtParallel_ratio = rrtParallel_mean / kgmtStateGrid_mean;
    
    estParallel_mean = mean(estParallelExecutionTime);
    estParallel_std = std(estParallelExecutionTime);
    estParallel_min = min(estParallelExecutionTime);
    estParallel_max = max(estParallelExecutionTime);
    estParallel_success = sum(estParallelExecutionTime < 60000) / length(estParallelExecutionTime) * 100;
    estParallel_ratio = estParallel_mean / kgmtStateGrid_mean;
    
    pdstParallel_mean = mean(pdstParallelExecutionTime);
    pdstParallel_std = std(pdstParallelExecutionTime);
    pdstParallel_min = min(pdstParallelExecutionTime);
    pdstParallel_max = max(pdstParallelExecutionTime);
    pdstParallel_success = sum(pdstParallelExecutionTime < 60000) / length(pdstParallelExecutionTime) * 100;
    pdstParallel_ratio = pdstParallel_mean / kgmtStateGrid_mean;


    
    %% Print Execution Times, Success Percentages, and Ratios
    fprintf('/* KPAX Jetson Execution Time */\n');
    fprintf('Mean: %.2f ms\n', kgmt_mean);
    fprintf('Standard Deviation: %.2f ms\n', kgmt_std);
    fprintf('Minimum: %.2f ms\n', kgmt_min);
    fprintf('Maximum: %.2f ms\n', kgmt_max);
    fprintf('Success Percentage: %.2f%%\n', kgmt_success);
    fprintf('Ratio (Mean Time / KPAX State Grid Mean Time): %.2f\n', kgmtJetson_ratio);
    fprintf('/***************************/\n\n');
    
    fprintf('/* KPAX State Grid Execution Time */\n');
    fprintf('Mean: %.2f ms\n', kgmtStateGrid_mean);
    fprintf('Standard Deviation: %.2f ms\n', kgmtStateGrid_std);
    fprintf('Minimum: %.2f ms\n', kgmtStateGrid_min);
    fprintf('Maximum: %.2f ms\n', kgmtStateGrid_max);
    fprintf('Success Percentage: %.2f%%\n', kgmtStateGrid_success);
    fprintf('/***************************/\n\n');
    
    fprintf('/* RRT-Parallel Execution Time */\n');
    fprintf('Mean: %.2f ms\n', rrtParallel_mean);
    fprintf('Standard Deviation: %.2f ms\n', rrtParallel_std);
    fprintf('Minimum: %.2f ms\n', rrtParallel_min);
    fprintf('Maximum: %.2f ms\n', rrtParallel_max);
    fprintf('Success Percentage: %.2f%%\n', rrtParallel_success);
    fprintf('Ratio (Mean Time / KPAX State Grid Mean Time): %.2f\n', rrtParallel_ratio);
    fprintf('/***************************/\n\n');
    
    fprintf('/* EST-Parallel Execution Time */\n');
    fprintf('Mean: %.2f ms\n', estParallel_mean);
    fprintf('Standard Deviation: %.2f ms\n', estParallel_std);
    fprintf('Minimum: %.2f ms\n', estParallel_min);
    fprintf('Maximum: %.2f ms\n', estParallel_max);
    fprintf('Success Percentage: %.2f%%\n', estParallel_success);
    fprintf('Ratio (Mean Time / KPAX State Grid Mean Time): %.2f\n', estParallel_ratio);
    fprintf('/***************************/\n\n');
    
    fprintf('/* PDST-Parallel Execution Time */\n');
    fprintf('Mean: %.2f ms\n', pdstParallel_mean);
    fprintf('Standard Deviation: %.2f ms\n', pdstParallel_std);
    fprintf('Minimum: %.2f ms\n', pdstParallel_min);
    fprintf('Maximum: %.2f ms\n', pdstParallel_max);
    fprintf('Success Percentage: %.2f%%\n', pdstParallel_success);
    fprintf('Ratio (Mean Time / KPAX State Grid Mean Time): %.2f\n', pdstParallel_ratio);
    fprintf('/***************************/\n\n');




    % %% Nodes Expanded
    % data = [kgmtExpandedNodes; kgmtStateGridExpandedNodes];
    % group = [ones(length(kgmtExpandedNodes), 1); 2 * ones(length(kgmtStateGridExpandedNodes), 1)];
    % 
    % figure;
    % b = boxchart(group, data);
    % b.JitterOutliers = 'on';
    % b.MarkerStyle = '.';
    % 
    % title(sprintf('Number Of Nodes Expanded (KPAX vs KPAX State Grid) - %s', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    % ylabel('Number Of Expanded Nodes', 'FontSize', 14, 'FontWeight', 'Bold');
    % xticks([1, 2]);
    % xticklabels({'KPAX', 'KPAX State Grid'});
    % set(gca, 'FontSize', algorithmSize);
    % set(gcf, 'Position', figureSize);
    % 
    % saveas(gcf, fullfile(output_dir, 'NodesExpanded_KGMT_vs_KGMTStateGrid.jpg'));
    % print(fullfile(output_dir, 'NodesExpanded_KGMT_vs_KGMTStateGrid.jpg'), '-djpeg', '-r300');
    % 
    % %% Tree Size
    % data = [kgmtTreeSize; kgmtStateGridTreeSize];
    % group = [ones(length(kgmtTreeSize), 1); 2 * ones(length(kgmtStateGridTreeSize), 1)];
    % 
    % figure;
    % b = boxchart(group, data);
    % b.JitterOutliers = 'on';
    % b.MarkerStyle = '.';
    % 
    % title(sprintf('Tree Size (KPAX vs KPAX State Grid) - %s', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    % ylabel('Number Of Nodes In Tree', 'FontSize', 14, 'FontWeight', 'Bold');
    % xticks([1, 2]);
    % xticklabels({'KPAX', 'KPAX State Grid'});
    % set(gca, 'FontSize', algorithmSize);
    % set(gcf, 'Position', figureSize);
    % 
    % saveas(gcf, fullfile(output_dir, 'TreeSize_KGMT_vs_KGMTStateGrid.jpg'));
    % print(fullfile(output_dir, 'TreeSize_KGMT_vs_KGMTStateGrid.jpg'), '-djpeg', '-r300');

    %% Full Comparison (Original Code)
    % data = [kgmtExecutionTime; kgmtStateGridExecutionTime; rrtParallelExecutionTime; estParallelExecutionTime; pdstParallelExecutionTime];
    % group = [ones(length(kgmtExecutionTime), 1); 2 * ones(length(kgmtStateGridExecutionTime), 1); 3 * ones(length(rrtParallelExecutionTime), 1); 4 * ones(length(estParallelExecutionTime), 1); 5 * ones(length(pdstParallelExecutionTime), 1)];
    % 
    % figure;
    % b = boxchart(group, data);
    % b.JitterOutliers = 'on';
    % b.MarkerStyle = '.';
    % 
    % title(sprintf('Execution Time (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    % ylabel('Execution Time (ms)', 'FontSize', 14, 'FontWeight', 'Bold');
    % xticks([1, 2, 3, 4, 5]);
    % xticklabels({'KPAX', 'KPAX State Grid', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    % set(gca, 'FontSize', algorithmSize);
    % set(gcf, 'Position', figureSize);
    % 
    % saveas(gcf, fullfile(output_dir, 'ExecutionTime_Comparison.jpg'));
    % print(fullfile(output_dir, 'ExecutionTime_Comparison.jpg'), '-djpeg', '-r300');

    % %% Nodes Expanded (Original Code)
    % data = [kgmtExpandedNodes; kgmtStateGridExpandedNodes; rrtParallelExpandedNodes; estParallelExpandedNodes; pdstParallelExpandedNodes];
    % group = [ones(length(kgmtExpandedNodes), 1); 2 * ones(length(kgmtStateGridExpandedNodes), 1); 3 * ones(length(rrtParallelExpandedNodes), 1); 4 * ones(length(estParallelExpandedNodes), 1); 5 * ones(length(pdstParallelExpandedNodes), 1)];
    % 
    % figure;
    % b = boxchart(group, data);
    % b.JitterOutliers = 'on';
    % b.MarkerStyle = '.';
    % 
    % title(sprintf('Number Of Nodes Expanded (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    % ylabel('Number Of Expanded Nodes', 'FontSize', 14, 'FontWeight', 'Bold');
    % xticks([1, 2, 3, 4, 5]);
    % xticklabels({'KPAX', 'KPAX State Grid', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    % set(gca, 'FontSize', algorithmSize);
    % set(gcf, 'Position', figureSize);
    % 
    % saveas(gcf, fullfile(output_dir, 'NodesExpanded_Comparison.jpg'));
    % print(fullfile(output_dir, 'NodesExpanded_Comparison.jpg'), '-djpeg', '-r300');
    % 
    % %% Tree Size (Original Code)
    % data = [kgmtTreeSize; kgmtStateGridTreeSize; rrtParallelTreeSize; estParallelTreeSize; pdstParallelTreeSize];
    % group = [ones(length(kgmtTreeSize), 1); 2 * ones(length(kgmtStateGridTreeSize), 1); 3 * ones(length(rrtParallelTreeSize), 1); 4 * ones(length(estParallelTreeSize), 1); 5 * ones(length(pdstParallelTreeSize), 1)];
    % 
    % figure;
    % b = boxchart(group, data);
    % b.JitterOutliers = 'on';
    % b.MarkerStyle = '.';
    % 
    % title(sprintf('Tree Size (%s)', dynamics), 'FontSize', titleSize, 'FontWeight', 'Bold');
    % ylabel('Number Of Nodes In Tree', 'FontSize', 14, 'FontWeight', 'Bold');
    % xticks([1, 2, 3, 4, 5]);
    % xticklabels({'KPAX', 'KPAX State Grid', 'RRT (CPU Parallelization)', 'EST (CPU Parallelization)', 'PDST (CPU Parallelization)'});
    % set(gca, 'FontSize', algorithmSize);
    % set(gcf, 'Position', figureSize);
    % 
    % saveas(gcf, fullfile(output_dir, 'TreeSize_Comparison.jpg'));
    % print(fullfile(output_dir, 'TreeSize_Comparison.jpg'), '-djpeg', '-r300');
end

