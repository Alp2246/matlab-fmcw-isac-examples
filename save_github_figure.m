function pathOut = save_github_figure(figHandle, fileBaseName, resolution)
%SAVE_GITHUB_FIGURE Grafikleri results/ altına PNG kaydeder (GitHub commit).
%
%   save_github_figure(gcf, 'fmcw_range_doppler');
%
% Betik repoda çalıştırıldığında .git kökü otomatik bulunur.

if nargin < 1 || isempty(figHandle)
    figHandle = gcf;
end
if nargin < 2
    error('save_github_figure: fileBaseName gerekli (ornek: ''fmcw_range_doppler'').');
end
if nargin < 3
    resolution = 150;
end

repoRoot = find_git_repo_root(fileparts(mfilename('fullpath')));
outDir = fullfile(repoRoot, 'results');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

pathOut = fullfile(outDir, [fileBaseName '.png']);

if exist('exportgraphics', 'file') == 2
    exportgraphics(figHandle, pathOut, 'Resolution', resolution);
else
    saveas(figHandle, pathOut);
end

fprintf('Kaydedildi: %s\n', pathOut);
fprintf('GitHub: push_to_github.ps1 -Message "Sonuc grafigi: %s"\n', fileBaseName);
end

function root = find_git_repo_root(startDir)
d = startDir;
for k = 1:12
    if exist(fullfile(d, '.git'), 'dir') == 7
        root = d;
        return;
    end
    parent = fileparts(d);
    if isequal(parent, d)
        break;
    end
    d = parent;
end
root = startDir;
end
