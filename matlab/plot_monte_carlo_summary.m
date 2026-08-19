clear;
clc;
close all;

paths = project_paths();

summaryTable = readtable( ...
    fullfile(paths.Tables,"monte_carlo_summary.csv"));
controllerNames = string(summaryTable.Controller);
controllerColors = [
    0.10 0.55 0.95
    1.00 0.45 0.05
    0.95 0.80 0.10
];

figure( ...
    "Name","Monte Carlo Controller Comparison", ...
    "Position",[100 100 1400 500], ...
    "Color","white");

tiledlayout(1,3);

nexttile;
barHandle = bar(summaryTable.SuccessRatePct,"FaceColor","flat");
barHandle.CData = controllerColors;
ylim([0 105]);
xticks(1:3);
xticklabels(controllerNames);
ylabel("Success rate [%]");
title(sprintf("Success Rate (%d Trials)",summaryTable.Trials(1)));
grid on;
style_axes(gca);

nexttile;
barHandle = bar( ...
    summaryTable.MeanAngleRMSESuccessfulDeg, ...
    "FaceColor","flat");
barHandle.CData = controllerColors;
xticks(1:3);
xticklabels(controllerNames);
ylabel("Angle RMSE [deg]");
title("Precision on Successful Trials");
grid on;
style_axes(gca);

nexttile;
barHandle = bar( ...
    summaryTable.MeanControlEnergySuccessfulN2s, ...
    "FaceColor","flat");
barHandle.CData = controllerColors;
xticks(1:3);
xticklabels(controllerNames);
ylabel("Control energy [N^2 s]");
title("Energy on Successful Trials");
grid on;
style_axes(gca);

sgtitle( ...
    "PID vs MPC vs PPO: Shared Random Initial Conditions", ...
    "Color","black");

outputPath = fullfile( ...
    paths.Images, ...
    "monte_carlo_controller_comparison.png");

exportgraphics(gcf,outputPath,"Resolution",180);
fprintf("Saved: %s\n",outputPath);

function style_axes(axisHandle)
    axisHandle.Color = "white";
    axisHandle.XColor = "black";
    axisHandle.YColor = "black";
    axisHandle.GridColor = [0.75 0.75 0.75];
    axisHandle.Title.Color = "black";
    axisHandle.XLabel.Color = "black";
    axisHandle.YLabel.Color = "black";
end
