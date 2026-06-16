%% fmcw_radar_range_doppler_ornek.m
% FMCW (Frequency Modulated Continuous Wave) radar benzetimi
% Sadece "Signal Processing Toolbox" gerektirir; Phased Array gerekmez.
%
% Bu betik şunları gösterir:
%   1) LFM (linear FM) chirp dalga şeklinin üretimi
%   2) Birden çok hedeften (farklı menzil + hız) yansıma + Doppler simülasyonu
%   3) Geri dönen sinyali iletilenle "dechirp" (mikser) ile karıştırma
%   4) Hızlı-zaman boyunca FFT     -> menzil profili
%   5) Yavaş-zaman boyunca FFT     -> Doppler profili
%   6) 2B Range-Doppler haritası   -> hedeflerin görselleştirilmesi
%   7) 2B hücre ortalama CFAR (CA-CFAR) ile basit hedef algılama
%
% Çalıştırmak için F5'e bas. Üst kısımdaki "Radar parametreleri" ve
% "targets" matrisi ile rahatça oynayabilirsin.

clear; clc; close all;
rng(42);                               % tekrarlanabilir gürültü

%% --------- 1) Radar parametreleri (otomotiv 77 GHz benzeri) ---------
fc          = 77e9;                    % Taşıyıcı frekans [Hz]
c           = 3e8;
lambda      = c/fc;

rangeMax    = 200;                     % Maks. tasarım menzili [m]
rangeRes    = 1;                       % Menzil çözünürlüğü   [m]

% Menzil çözünürlüğü ile bant genişliği:  ΔR = c/(2B)  ->  B = c/(2ΔR)
B           = c/(2*rangeRes);          % ~150 MHz

% Chirp süresi: en uzak hedeften dönüş süresinin >5x katı önerilir
T_round_max = 2*rangeMax/c;
Tchirp      = 5.5*T_round_max;         % chirp süresi [s]
slope       = B/Tchirp;                % LFM eğimi [Hz/s]

% Beat frekansı maksimumu  ->  yeterli örnekleme frekansı seç
fbeat_max   = 2*slope*rangeMax/c;
fs          = 2*fbeat_max;             % Nyquist payı

% Çerçeve boyutları
Nchirps     = 128;                     % yavaş-zaman örnek sayısı (Doppler FFT)
Nsamples    = 2^nextpow2(round(Tchirp*fs));   % hızlı-zaman örnek sayısı
fs          = Nsamples/Tchirp;         % örnekleme freq.'i tam hizala

%% --------- 2) Hedef listesi  [menzil(m), hız(m/s), kazanç] ---------
% Hız işareti: + = uzaklaşıyor,  - = yaklaşıyor
targets = [
     35,  -20,  1.0;     % yakın, yaklaşan araç
     90,  +12,  0.8;     % orta menzil, uzaklaşan araç
    140,  -30,  0.5;     % uzak, hızlı yaklaşan
];

%% --------- 3) İletim & alım sinyalini sentezle ----------------------
t_fast   = (0:Nsamples-1).' / fs;      % bir chirp içindeki zaman [s]
RD_cube  = zeros(Nsamples, Nchirps);   % beat sinyali (range x doppler)

for k = 1:Nchirps
    t_slow = (k-1)*Tchirp;             % chirp başlangıç anı

    % Temel banttaki TX chirp: phi(t) = 2π·(½·slope·t²)
    tx = exp(1j*2*pi*(0.5*slope*t_fast.^2));

    rx = zeros(size(tx));
    for i = 1:size(targets,1)
        R0 = targets(i,1);
        v  = targets(i,2);
        a  = targets(i,3);

        % Anlık menzil (chirp süresince ~sabit) ve gecikme
        R   = R0 + v*t_slow;
        tau = 2*R/c;

        % Gecikmeli RX (taşıyıcı fazı: -2π·fc·τ)
        rx_i = a * exp(1j*2*pi*( 0.5*slope*(t_fast - tau).^2 - fc*tau ));
        rx   = rx + rx_i;
    end

    % Mikser (dechirp): beat = tx * conj(rx)
    %   beat_freq ≈ slope * τ  -> menzil
    %   chirp'ten chirp'e faz farkı ≈ 2π·(2v/λ)·Tchirp -> Doppler
    RD_cube(:,k) = tx .* conj(rx);
end

%% --------- 4) Gürültü ekle (elle AWGN — toolbox gerekmez) -----------
SNR_dB    = 12;
sigPow    = mean(abs(RD_cube(:)).^2);                 % ölçülen sinyal gücü
noisePow  = sigPow / 10^(SNR_dB/10);
noise     = sqrt(noisePow/2) * ( randn(size(RD_cube)) + ...
                                 1j*randn(size(RD_cube)) );
RD_cube   = RD_cube + noise;

%% --------- 5) Range FFT (hızlı-zaman) -------------------------------
win_r       = hann(Nsamples);
RD_range    = fft(RD_cube .* win_r, Nsamples, 1);
RD_range    = RD_range(1:Nsamples/2, :);         % tek-yan
freq_range  = (0:Nsamples/2-1).' * (fs/Nsamples);
range_axis  = freq_range * c / (2*slope);        % m

%% --------- 6) Doppler FFT (yavaş-zaman) -----------------------------
win_d   = hann(Nchirps).';
RD_map  = fftshift(fft(RD_range .* win_d, Nchirps, 2), 2);

PRF       = 1/Tchirp;
dop_axis  = (-Nchirps/2:Nchirps/2-1) * (PRF/Nchirps);
vel_axis  = dop_axis * lambda/2;                  % m/s

% Genlik (dB), tepe normalize
RDdB = 20*log10(abs(RD_map) + eps);
RDdB = RDdB - max(RDdB(:));

%% --------- 7) Basit 2B CA-CFAR algılayıcı ---------------------------
guard = [2 2];     % [range, doppler] guard hücreleri
train = [4 4];     % training hücreleri
Pfa   = 1e-4;

[Nr, Nd] = size(RDdB);
detMap   = false(Nr, Nd);
PowMap   = abs(RD_map).^2;

NTrain = (2*(guard(1)+train(1))+1)*(2*(guard(2)+train(2))+1) ...
       - (2*guard(1)+1)*(2*guard(2)+1);
alpha  = NTrain * (Pfa^(-1/NTrain) - 1);

for r = (guard(1)+train(1)+1):(Nr-guard(1)-train(1))
    for d = (guard(2)+train(2)+1):(Nd-guard(2)-train(2))
        win = PowMap(r-guard(1)-train(1):r+guard(1)+train(1), ...
                     d-guard(2)-train(2):d+guard(2)+train(2));
        % guard + CUT bölgesini maskele
        win(train(1)+1:end-train(1), train(2)+1:end-train(2)) = NaN;
        Pn = mean(win(:), 'omitnan');
        if PowMap(r,d) > alpha*Pn
            detMap(r,d) = true;
        end
    end
end

%% --------- 8) Görselleştirme (4 panel) ------------------------------
figure('Name','FMCW Radar — Range-Doppler', 'Color','w', ...
       'Position',[80 80 1180 760]);
tl = tiledlayout(2,2, 'Padding','compact', 'TileSpacing','compact');

% (1) Tek chirp'in spektrogramı  -> linear ramp 0..B
nexttile;
spectrogram(exp(1j*2*pi*(0.5*slope*t_fast.^2)), 64, 56, 256, fs, 'yaxis');
title('TX chirp — spektrogram (1 chirp)');

% (2) İlk chirp'in menzil profili
nexttile;
plot(range_axis, 20*log10(abs(RD_range(:,1)) + eps), 'LineWidth', 1.2);
xlim([0 rangeMax]); grid on;
xlabel('Menzil (m)'); ylabel('Genlik (dB)');
title('Tek chirp menzil profili');

% (3) Range-Doppler haritası + gerçek hedef konumları
nexttile;
imagesc(vel_axis, range_axis, RDdB, [-40 0]);
axis xy; ylim([0 rangeMax]); colormap(gca, turbo); colorbar;
xlabel('Hız (m/s)'); ylabel('Menzil (m)');
title('Range-Doppler haritası (dB)');
hold on;
plot(targets(:,2), targets(:,1), 'wo', 'MarkerSize', 12, 'LineWidth', 1.6);
text(targets(:,2)+1.5, targets(:,1), ...
     compose('  T%d', (1:size(targets,1)).'), 'Color', 'w');

% (4) Aynı harita + CFAR algılamaları
nexttile;
imagesc(vel_axis, range_axis, RDdB, [-40 0]);
axis xy; ylim([0 rangeMax]); colormap(gca, turbo); colorbar;
hold on;
[rIdx, dIdx] = find(detMap);
plot(vel_axis(dIdx), range_axis(rIdx), 'w+', ...
     'MarkerSize', 9, 'LineWidth', 1.3);
xlabel('Hız (m/s)'); ylabel('Menzil (m)');
title(sprintf('CA-CFAR algılamaları (P_{fa} = %.0e)', Pfa));

title(tl, sprintf(['FMCW Radar Benzetimi  —  fc = %.0f GHz, ' ...
                   'B = %.0f MHz, T_{c} = %.2f \\mus, SNR = %d dB'], ...
                  fc/1e9, B/1e6, Tchirp*1e6, SNR_dB), 'FontWeight','bold');

%% --------- 9) Konsol özeti ------------------------------------------
fprintf('\n=== FMCW radar parametreleri ===\n');
fprintf('  fc       = %.1f GHz\n',  fc/1e9);
fprintf('  B        = %.1f MHz   ->  ΔR = %.2f m\n', B/1e6, c/(2*B));
fprintf('  Tchirp   = %.2f us    ->  PRF = %.1f kHz\n', Tchirp*1e6, PRF/1e3);
fprintf('  fs       = %.2f MHz\n', fs/1e6);
fprintf('  Nsamples = %d (fast time)\n', Nsamples);
fprintf('  Nchirps  = %d (slow time)\n', Nchirps);
fprintf('  ΔV (Dop) = %.2f m/s   |Vmax| = %.1f m/s\n', ...
        lambda/(2*Nchirps*Tchirp), lambda*PRF/4);

fprintf('\nGerçek hedefler:\n');
disp(array2table(targets, 'VariableNames', {'Range_m','Vel_mps','Gain'}));

fprintf('CFAR hücre sayısı (algılanan)  = %d\n', nnz(detMap));
fprintf('Beklenen ortalama yanlış alarm = %.2f hücre\n\n', Pfa*Nr*Nd);
