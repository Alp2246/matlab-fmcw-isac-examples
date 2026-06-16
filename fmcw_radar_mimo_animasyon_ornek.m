%% fmcw_radar_mimo_animasyon_ornek.m
% FMCW radar — animasyonlu, anten dizisi (SIMO) ile AÇI kestirimli,
% CFAR + kümeleme ile tek hedef başına (R, V, θ) etiketli mini demo.
%
% Bu betik 3 şeyi birleştirir:
%   (1) CFAR sonrası KÜMELEME -> her hedef için tek nokta + (R,V,θ) etiketi
%   (2) Hareketli hedeflerin ANİMASYONU -> her çerçevede yeniden işleme
%   (3) ULA (Uniform Linear Array) ile AÇI KESTİRİMİ
%       (3 boyutlu işleme küpü: Range × Doppler × Angle)
%
% Sadece "Signal Processing Toolbox" kullanır (fft, hann, asind, ...).
%
% Nasıl çalışır?
%   - 1 verici, 8 alıcı antenden oluşan ULA, λ/2 aralıklı.
%   - Her çerçeve = 1 CPI (Coherent Processing Interval) = Nchirps*Tchirp
%   - Çerçeveler arası "ölü zaman" frame_dt boyunca hedefler hareket eder.
%   - Her çerçevede: 3 kademeli FFT (range, doppler, angle) -> 3B güç küpü
%   - Range-Doppler düzleminde 2B CA-CFAR -> aday hücreler
%   - Her aday için açı boyutundaki tepe -> (R, V, θ) üçlüsü
%   - Izgara-tabanlı kümeleme -> hedef başına tek nokta
%   - 3 panelli görselleştirme: Range-Doppler / Range-Angle / Kuş bakışı

clear; clc; close all;
rng(7);

%% ================== Radar parametreleri ==================
fc          = 77e9;
c           = 3e8;
lambda      = c/fc;

rangeMax    = 100;                             % m
rangeRes    = 1;                               % m

B           = c/(2*rangeRes);                  % 150 MHz
T_round_max = 2*rangeMax/c;
Tchirp      = 5.5*T_round_max;
slope       = B/Tchirp;
fbeat_max   = 2*slope*rangeMax/c;
fs          = 2*fbeat_max;

Nchirps     = 64;                              % yavaş zaman (Doppler FFT)
Nsamples    = 2^nextpow2(round(Tchirp*fs));    % hızlı zaman (Range FFT)
fs          = Nsamples/Tchirp;                 % örnekleme freq.'i hizala
PRF         = 1/Tchirp;

%% ================== Anten dizisi (SIMO ULA) ==================
Nrx         = 8;
d_rx        = lambda/2;
NangFFT     = 64;                              % açı FFT için zero-pad

%% ================== Animasyon zamanı ==================
Nframes     = 60;                              % çerçeve sayısı
frameRate   = 20;                              % Hz (görsel hız)
frame_dt    = 1/frameRate;                     % çerçeveler arası süre [s]

%% ================== Hedefler [R0(m), V(m/s), θ(°), kazanç] ==================
% V işareti:  + = uzaklaşıyor,  − = yaklaşıyor
% θ işareti:  + = boresight'ın sağında, − = solunda
targets0 = [
     25,  -8,  -25, 1.0;     % yakın, sol, yaklaşan
     55,   5,  +10, 0.8;     % orta, hafif sağ, uzaklaşan
     80, -15,  +30, 0.6;     % uzak, sağ, hızlı yaklaşan
     40,   0,  -55, 0.7;     % sabit, çok solda
];
Ntarg = size(targets0,1);

%% ================== Sabit eksenler ve pencereler ==================
t_fast      = (0:Nsamples-1).' / fs;
freq_range  = (0:Nsamples/2-1).' * (fs/Nsamples);
range_axis  = freq_range * c / (2*slope);
dop_axis    = (-Nchirps/2:Nchirps/2-1) * (PRF/Nchirps);
vel_axis    = dop_axis * lambda/2;
ang_axis    = asind(2*(-NangFFT/2:NangFFT/2-1)/NangFFT);

win_r       = hann(Nsamples);
win_d       = hann(Nchirps).';
win_a       = reshape(hann(Nrx), 1, 1, Nrx);

%% ================== Şekil: 3 panel ==================
fig = figure('Name','MIMO Radar — Animasyon', 'Color','w', ...
             'Position',[60 60 1320 760]);
tl  = tiledlayout(fig, 2, 2, 'Padding','compact', 'TileSpacing','compact');
ax_rd = nexttile(tl, 1);
ax_ra = nexttile(tl, 2);
ax_be = nexttile(tl, 3, [1 2]);

%% ================== Animasyon döngüsü ==================
targets = targets0;
trail   = nan(Nframes, Ntarg, 2);              % gerçek (X,Y) izleri

for fr = 1:Nframes

    % ---------- (a) Hedefleri zamanla yürüt ----------
    targets(:,1) = targets(:,1) + targets(:,2)*frame_dt;
    % Sahnenin dışına çıkanları geri sar (animasyon devam etsin diye)
    targets(targets(:,1) < 5,           1) = rangeMax - 5;
    targets(targets(:,1) > rangeMax-2,  1) = 8;

    Rt = targets(:,1);  th = targets(:,3);
    Xt = Rt .* sind(th);
    Yt = Rt .* cosd(th);
    trail(fr,:,1) = Xt;
    trail(fr,:,2) = Yt;

    % ---------- (b) Beat sinyali küpü: Nsamples × Nchirps × Nrx ----------
    cube = zeros(Nsamples, Nchirps, Nrx);
    for k = 1:Nchirps
        t_slow = (k-1)*Tchirp;
        tx     = exp(1j*2*pi*(0.5*slope*t_fast.^2));

        chirp_cube = zeros(Nsamples, Nrx);
        for i = 1:Ntarg
            R0i = targets(i,1);  vi = targets(i,2);
            thi = targets(i,3);  ai = targets(i,4);

            R    = R0i + vi*t_slow;
            tau  = 2*R/c;

            % rx_n = base * exp(j*ψ_n),   ψ_n = 2π·(d/λ)·sin(θ)·n
            % beat_n = tx · conj(rx_n) = (tx · conj(base)) · exp(−j·ψ_n)
            phi_arr = 2*pi*d_rx/lambda * sind(thi) * (0:Nrx-1);    % 1×Nrx
            base    = ai * exp(1j*2*pi*( 0.5*slope*(t_fast - tau).^2 ...
                                         - fc*tau ));
            beat_n  = (tx .* conj(base)) * exp(-1j*phi_arr);       % Nsamples×Nrx
            chirp_cube = chirp_cube + beat_n;
        end
        cube(:,k,:) = chirp_cube;
    end

    % ---------- (c) AWGN ekle ----------
    SNR_dB   = 14;
    sigPow   = mean(abs(cube(:)).^2);
    noisePow = sigPow / 10^(SNR_dB/10);
    cube     = cube + sqrt(noisePow/2) * ...
               ( randn(size(cube)) + 1j*randn(size(cube)) );

    % ---------- (d) 3 kademeli FFT ----------
    R3  = fft(cube .* win_r, Nsamples, 1);
    R3  = R3(1:Nsamples/2, :, :);
    RD3 = fftshift(fft(R3  .* win_d, Nchirps, 2), 2);
    % FFT 3. eksen otomatik olarak NangFFT'ye zero-pad'ler
    RDA = fftshift(fft(RD3 .* win_a, NangFFT, 3), 3);
    % Beat tarafındaki (−ψ) konvansiyonunu telafi: açı eksenini ters çevir
    RDA = flip(RDA, 3);

    PowRDA = abs(RDA).^2;

    % ---------- (e) Range-Doppler + Range-Angle haritaları ----------
    PowRD = sum(PowRDA, 3);                    % açı üzerinden topla
    RDdB  = 10*log10(PowRD + eps);  RDdB = RDdB - max(RDdB(:));
    PowRA = squeeze(sum(PowRDA, 2));           % Doppler üzerinden topla
    RAdB  = 10*log10(PowRA + eps);  RAdB = RAdB - max(RAdB(:));

    % ---------- (f) 2B CA-CFAR (R-D düzleminde) ----------
    detMap = ca_cfar_2d(PowRD, [2 2], [4 4], 1e-4);
    [rIdx, dIdx] = find(detMap);

    % ---------- (g) Her algı için açıyı bul -> (R,V,θ) listesi ----------
    detList = zeros(numel(rIdx), 3);
    for q = 1:numel(rIdx)
        ang_slice    = squeeze(PowRDA(rIdx(q), dIdx(q), :));
        [~, am]      = max(ang_slice);
        detList(q,:) = [range_axis(rIdx(q)), vel_axis(dIdx(q)), ang_axis(am)];
    end

    % ---------- (h) Izgara tabanlı kümeleme ----------
    % R'yi 3 m, V'yi 3 m/s, θ'yi 4° kovalara böl, aynı kovaya düşenleri birleştir
    if ~isempty(detList)
        keys = [round(detList(:,1)/3), round(detList(:,2)/3), round(detList(:,3)/4)];
        [~, ~, ic] = unique(keys, 'rows');
        nC = max(ic);
        clusters = zeros(nC, 3);
        for q = 1:nC
            clusters(q,:) = mean(detList(ic == q, :), 1);
        end
    else
        clusters = zeros(0, 3);
    end

    % ================== Görselleştirme ==================
    % -- (1) Range-Doppler --
    cla(ax_rd);
    imagesc(ax_rd, vel_axis, range_axis, RDdB, [-35 0]);
    axis(ax_rd, 'xy'); ylim(ax_rd, [0 rangeMax]);
    colormap(ax_rd, turbo);  colorbar(ax_rd);
    xlabel(ax_rd, 'Hız (m/s)');  ylabel(ax_rd, 'Menzil (m)');
    title(ax_rd, sprintf('Range-Doppler  |  çerçeve %d/%d', fr, Nframes));
    if ~isempty(clusters)
        hold(ax_rd, 'on');
        plot(ax_rd, clusters(:,2), clusters(:,1), 'wo', ...
             'MarkerSize', 9, 'LineWidth', 1.4);
        text(ax_rd, clusters(:,2)+1, clusters(:,1), ...
             compose('%.0fm/%.0fm/s', clusters(:,1), clusters(:,2)), ...
             'Color','w', 'FontSize', 8);
    end

    % -- (2) Range-Angle --
    cla(ax_ra);
    imagesc(ax_ra, ang_axis, range_axis, RAdB, [-35 0]);
    axis(ax_ra, 'xy'); ylim(ax_ra, [0 rangeMax]);
    colormap(ax_ra, turbo);  colorbar(ax_ra);
    xlabel(ax_ra, 'Açı (°)');  ylabel(ax_ra, 'Menzil (m)');
    title(ax_ra, 'Range-Angle');
    if ~isempty(clusters)
        hold(ax_ra, 'on');
        plot(ax_ra, clusters(:,3), clusters(:,1), 'wo', ...
             'MarkerSize', 9, 'LineWidth', 1.4);
    end

    % -- (3) Kuş bakışı (X = sol/sağ, Y = ileri) --
    cla(ax_be);
    hold(ax_be, 'on');  grid(ax_be, 'on');
    % Geçmiş izler (gri)
    for i = 1:Ntarg
        plot(ax_be, squeeze(trail(1:fr,i,1)), squeeze(trail(1:fr,i,2)), ...
             '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
    end
    % Gerçek hedefler (yeşil)
    plot(ax_be, Xt, Yt, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(ax_be, Xt+1.5, Yt, compose(' T%d', (1:Ntarg).'), ...
         'Color', [0 0.5 0]);
    % Algılanan kümeler (kırmızı +)
    if ~isempty(clusters)
        Xd = clusters(:,1) .* sind(clusters(:,3));
        Yd = clusters(:,1) .* cosd(clusters(:,3));
        plot(ax_be, Xd, Yd, 'r+', 'MarkerSize', 14, 'LineWidth', 1.8);
    end
    % Radar konumu (siyah üçgen)
    plot(ax_be, 0, 0, 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    axis(ax_be, 'equal');
    xlim(ax_be, [-rangeMax rangeMax]);  ylim(ax_be, [-5 rangeMax]);
    xlabel(ax_be, 'X (m)  — sol/sağ');
    ylabel(ax_be, 'Y (m)  — radardan ileri');
    title(ax_be, ...
        sprintf('Kuş bakışı — yeşil:gerçek, kırmızı:algılanan  (kümeleme)'));

    title(tl, sprintf( ...
        'FMCW MIMO Radar  —  Nrx=%d, Ntarg=%d, fc=%.0f GHz, B=%.0f MHz', ...
        Nrx, Ntarg, fc/1e9, B/1e6), 'FontWeight','bold');

    drawnow limitrate;
end

fprintf('\nAnimasyon tamamlandı: %d çerçeve × %d hedef.\n', Nframes, Ntarg);
fprintf('Açı çözünürlüğü ~ %.1f° (Nrx=%d, %dλ/2 açıklık)\n', ...
        rad2deg(2/Nrx), Nrx, Nrx);

%% ================== Yardımcı: 2B Cell-Averaging CFAR ==================
function det = ca_cfar_2d(P, guard, train, Pfa)
    [Nr, Nd] = size(P);
    det      = false(Nr, Nd);
    NTrain   = (2*(guard(1)+train(1))+1)*(2*(guard(2)+train(2))+1) ...
              - (2*guard(1)+1)*(2*guard(2)+1);
    alpha    = NTrain * (Pfa^(-1/NTrain) - 1);
    for r = (guard(1)+train(1)+1) : (Nr-guard(1)-train(1))
        for d = (guard(2)+train(2)+1) : (Nd-guard(2)-train(2))
            win = P(r-guard(1)-train(1):r+guard(1)+train(1), ...
                    d-guard(2)-train(2):d+guard(2)+train(2));
            win(train(1)+1:end-train(1), train(2)+1:end-train(2)) = NaN;
            Pn = mean(win(:), 'omitnan');
            if P(r,d) > alpha*Pn
                det(r,d) = true;
            end
        end
    end
end
