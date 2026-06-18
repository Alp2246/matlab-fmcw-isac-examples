function export_all_results()
%EXPORT_ALL_RESULTS FMCW/ISAC demolarini calistirir, figurleri results/ altina kaydeder.

root = fileparts(mfilename('fullpath'));
jobs = {
    'fmcw_radar_range_doppler_ornek',  'fmcw_range_doppler'
    'isac_ofdm_sensing_ornek',         'isac_ofdm_sensing'
    'fmcw_radar_mimo_animasyon_ornek', 'fmcw_mimo_animasyon'
    'fmcw_radar_kalman_tracker_ornek', 'fmcw_kalman_tracker'
};

for k = 1:size(jobs, 1)
    try
        run_demo_and_save(root, jobs{k, 1}, jobs{k, 2});
    catch ME
        fprintf('ATLANDI %s: %s\n', jobs{k, 1}, ME.message);
    end
end

fprintf('\nFMCW/ISAC export bitti: %s\n', fullfile(root, 'results'));
end

function run_demo_and_save(root, scriptName, tag)
cd(root);
addpath(root);
close all;
run_demo_isolated(fullfile(root, [scriptName '.m']));
save_open_figures(root, tag);
end

function run_demo_isolated(scriptPath)
run(scriptPath);
end

function save_open_figures(root, tag)
figs = findall(0, 'Type', 'figure');
figs = flipud(figs);
for fi = 1:numel(figs)
    if numel(figs) == 1
        name = tag;
    else
        name = sprintf('%s_fig%d', tag, fi);
    end
    save_github_figure(figs(fi), name);
end
end
