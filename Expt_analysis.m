%% SWARM ROBOTICS ANALYSIS TOOL
% This script loads robot log data, computes nematic order and variance,
% and produces a high-performance, aesthetically pleasing animation.
%
% Instructions:
% 1. Ensure your data folders follow the structure: baseDir/n_select=X/log...csv
% 2. Update the 'CONFIGURATION' section below.
% 3. Run the script.

clear; clc; close all;

%% ========================== CONFIGURATION ==========================
n_select = 5;                        % Subdirectory index to analyze
baseDir  = fullfile('.', 'DATA_N=12'); % Main data directory
home_tolerance  = 5.0;               % Distance to consider "at home"
minRobotsAtHome = 13;                % Required count for homing trigger
animate_post_home_only = true;       % Animation toggle

% Automatic File Selection (Picks the most recent log in the folder)
subDir      = sprintf('n_select=%d', n_select);
filePattern = fullfile(baseDir, subDir, '*.csv');
files       = dir(filePattern);

if isempty(files)
    error('No log CSV files found in: %s', fullfile(baseDir, subDir));
end

[~, idxLatest] = max([files.datenum]);
filename = fullfile(files(idxLatest).folder, files(idxLatest).name);

fprintf('Analyzing file: %s\n', files(idxLatest).name);

%% ========================== RUN ANALYSIS ==========================
analyse_robot_log_mod1(filename, minRobotsAtHome, home_tolerance, animate_post_home_only);

%% ========================================================================
%  PRIMARY VISUALIZATION FUNCTION
%  ========================================================================
function analyse_robot_log_mod1(filename, minRobotsAtHome, home_tolerance, animate_post_home_only)
    % LOAD & PREPROCESS
    T = readtable(filename);
    T = sortrows(T, {'timestamp','tag_id'});
    T.timestamp = T.timestamp - T.timestamp(1);
    
    GLOBAL_CENTER = [334.0, 322.0];
    T.xc = T.x - GLOBAL_CENTER(1);
    T.yc = T.y - GLOBAL_CENTER(2);
    times = unique(T.timestamp);

    % COMPUTE OBSERVABLES
    [~, ~, S_rob]        = computeNematicOrder(T);
    [t_var, theta_var]   = computeNematicThetaVariance(T);
    [t_pos, ~, R_pos]    = computeCircularMeanPosition(T);
    [t_cmean, circ_mean] = computeCircMeanVelocity(T);

    % FIGURE & STYLING
    fig = figure('Color', 'w', 'Name', 'Swarm Analysis', 'Position', [100, 100, 1100, 750]);
    tlo = tiledlayout(2, 2, 'TileSpacing', 'loose', 'Padding', 'compact');
    
    colors = [0.15 0.45 0.65; 0.85 0.35 0.35; 0.40 0.65 0.40; 0.55 0.35 0.65];
    mainFont = 'Helvetica';

    % Panel 1: Swarm Animation
    axSwarm = nexttile([2 1]);
    hold(axSwarm, 'on'); axis(axSwarm, 'equal'); box(axSwarm, 'off');
    set(axSwarm, 'FontName', mainFont, 'FontSize', 10, 'GridAlpha', 0.1);
    grid on;
    
    pad = 30;
    xlim(axSwarm, [min(T.x)-pad, max(T.x)+pad]);
    ylim(axSwarm, [min(T.y)-pad, max(T.y)+pad]);
    xlabel('X Position'); ylabel('Y Position');

    tag_ids = unique(T.tag_id);
    numRobots = numel(tag_ids);
    cmap = lines(numRobots);
    
    hTrails = cell(numRobots, 1);
    hRobots = cell(numRobots, 1);
    hQuivers = cell(numRobots, 1);
    
    for k = 1:numRobots
        hTrails{k}  = plot(axSwarm, NaN, NaN, '-', 'Color', [cmap(k,:) 0.3], 'LineWidth', 1.2);
        hRobots{k}  = plot(axSwarm, NaN, NaN, 'o', 'MarkerFaceColor', cmap(k,:), 'MarkerEdgeColor', 'w', 'MarkerSize', 8);
        hQuivers{k} = quiver(axSwarm, NaN, NaN, NaN, NaN, 0, 'Color', cmap(k,:), 'LineWidth', 1.5);
    end
    title(axSwarm, 'Swarm Dynamic State', 'FontSize', 12, 'FontWeight', 'bold');

    % Panel 2: Nematic Order
    axNem = nexttile(2); hold on; box off; grid on;
    set(axNem, 'FontName', mainFont, 'GridAlpha', 0.1);
    plot(axNem, times, S_rob, 'Color', [0.9 0.9 0.9], 'LineWidth', 0.5, 'HandleVisibility', 'off'); 
    plot(axNem, t_cmean, circ_mean, 'Color', [0.9 0.9 0.9], 'LineWidth', 0.5, 'HandleVisibility', 'off'); 
    hSrob = plot(axNem, NaN, NaN, 'Color', colors(1,:), 'LineWidth', 2, 'DisplayName', 'S_{rob}');
    hCmean = plot(axNem, NaN, NaN, 'Color', colors(2,:), 'LineWidth', 2, 'DisplayName', 'Circ Mean');
    nemPt1 = plot(axNem, NaN, NaN, 'o', 'MarkerFaceColor', colors(1,:), 'MarkerEdgeColor', 'w');
    nemPt2 = plot(axNem, NaN, NaN, 'o', 'MarkerFaceColor', colors(2,:), 'MarkerEdgeColor', 'w');
    ylabel('Order Parameter'); ylim([0 1.1]);
    legend('Location', 'northeastoutside', 'FontSize', 8);

    % Panel 3: Nematic Variance
    axVar = nexttile(4); hold on; box off; grid on;
    set(axVar, 'FontName', mainFont, 'GridAlpha', 0.1);
    plot(axVar, t_var, theta_var, 'Color', [0.9 0.9 0.9], 'LineWidth', 0.5, 'HandleVisibility', 'off');
    hVarLine = plot(axVar, NaN, NaN, 'Color', colors(4,:), 'LineWidth', 2);
    varPt = plot(axVar, NaN, NaN, 's', 'MarkerFaceColor', colors(4,:), 'MarkerEdgeColor', 'w');
    ylabel('Var(\theta)'); xlabel('Time (s)');
    title('Angular Variance', 'FontWeight', 'bold');

    % ANIMATION LOOP
    trailN = 50; 
    scale = 1; % Velocity arrow scaling
    for it = 1:numel(times)
        tt = times(it);
        for k = 1:numRobots
            idx_full = find(T.tag_id == tag_ids(k) & T.timestamp <= tt);
            if isempty(idx_full), continue; end
            idx_trail = idx_full(max(1, end-trailN):end);
            set(hTrails{k}, 'XData', T.x(idx_trail), 'YData', T.y(idx_trail));
            curr_idx = idx_full(end);
            if T.timestamp(curr_idx) == tt
                set(hRobots{k}, 'XData', T.x(curr_idx), 'YData', T.y(curr_idx));
                set(hQuivers{k}, 'XData', T.x(curr_idx), 'YData', T.y(curr_idx), ...
                                 'UData', T.vx(curr_idx)*scale, 'VData', T.vy(curr_idx)*scale); 
            end
        end
        set(hSrob, 'XData', times(1:it), 'YData', S_rob(1:it));
        set(nemPt1, 'XData', tt, 'YData', S_rob(it));
        [~, ic] = min(abs(t_cmean - tt));
        set(hCmean, 'XData', t_cmean(1:ic), 'YData', circ_mean(1:ic));
        set(nemPt2, 'XData', t_cmean(ic), 'YData', circ_mean(ic));
        [~, iv] = min(abs(t_var - tt));
        set(hVarLine, 'XData', t_var(1:iv), 'YData', theta_var(1:iv));
        set(varPt, 'XData', t_var(iv), 'YData', theta_var(iv));
        title(axSwarm, sprintf('Swarm Dynamics | Time: %.2f s', tt));
        drawnow limitrate; 
    end
end

%% ========================================================================
%  MATHEMATICAL HELPER FUNCTIONS
%  ========================================================================

function [times, S_orig, S_rob] = computeNematicOrder(T)
    T.theta = atan2(T.vy, T.vx);
    times = unique(T.timestamp);
    S_orig = nan(size(times));
    S_rob  = nan(size(times));
    for i = 1:numel(times)
        th = T.theta(T.timestamp==times(i));
        if numel(th)<2, continue; end
        C2 = mean(cos(2*th)); S2 = mean(sin(2*th));
        S_orig(i) = hypot(C2,S2);
        if numel(th)<3, S_rob(i)=S_orig(i); continue; end
        phi = 0.5*atan2(S2,C2);
        dphi = angle(exp(1i*(2*(th-phi))));
        [~,w]=max(abs(dphi));
        th(w)=[];
        S_rob(i)=hypot(mean(cos(2*th)),mean(sin(2*th)));
    end
end

function [times, theta_var] = computeNematicThetaVariance(T)
    times = unique(T.timestamp);
    theta_var = nan(size(times));
    for i=1:numel(times)
        th = atan2(T.vy(T.timestamp==times(i)), T.vx(T.timestamp==times(i)));
        if numel(th)<2, continue; end
        th = foldNematic(th);
        nbins = max(5,round(sqrt(numel(th))));
        theta_var(i)=nematicVarianceHistogram(th,nbins);
    end
end

function th = foldNematic(th)
    th(th<0)=th(th<0)+pi;
    th(th>pi/2)=th(th>pi/2)-pi;
end

function v = nematicVarianceHistogram(theta,nbins)
    [pdf,edges]=histcounts(theta,nbins,'Normalization','pdf');
    centers=0.5*(edges(1:end-1)+edges(2:end));
    d=edges(2)-edges(1);
    m=sum(centers.*pdf)*d;
    v=sum((centers-m).^2.*pdf)*d;
end

function [times, phi_mean, R] = computeCircularMeanPosition(T)
    times = unique(T.timestamp);
    phi_mean = nan(size(times));
    R = nan(size(times));
    for i=1:numel(times)
        sub=T(T.timestamp==times(i),:);
        if height(sub)<2, continue; end
        phi=atan2(sub.yc,sub.xc);
        C=mean(cos(phi)); S=mean(sin(phi));
        phi_mean(i)=atan2(S,C);
        R(i)=hypot(C,S);
    end
end

function [times, circ_mean] = computeCircMeanVelocity(T)
    times = unique(T.timestamp);
    circ_mean = nan(size(times));
    for it = 1:numel(times)
        idx = T.timestamp == times(it);
        if nnz(idx) < 2, continue; end
        theta = atan2(T.vy(idx), T.vx(idx)); 
        C = mean(cos(2*theta)); S = mean(sin(2*theta));
        circ_mean(it) = hypot(C, S);
    end
end