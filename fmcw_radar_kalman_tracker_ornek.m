%% fmcw_radar_kalman_tracker_ornek.m
% FMCW MIMO radar  +  Kalman filtre tabanlı çoklu hedef takibi (MOT)
%
% Önceki dosya (fmcw_radar_mimo_animasyon_ornek.m) her çerçevede bağımsız
% (R, V, θ) ölçümleri üretiyordu. Bu dosya o ölçümlerin üstüne profesyonel
% bir takipçi koyar:
%
%   • Sabit-hız modeli, 4D durum:  x = [X; Vx; Y; Vy]
%   • Kalman PREDICT + UPDATE
%   • Veri ilişkilendirme: greedy nearest-neighbor + Öklid "gate"
%   • Track yönetimi: tentative -> confirmed (M/N kuralı), confirmed -> lost
%   • Her track'e kalıcı ID, kendi rengi, geçmiş izi, 2σ belirsizlik elipsi
%   • Yeni hedef "doğum" senaryosu (5. hedef 25. çerçevede sahneye girer)
%
% Sadece "Signal Processing Toolbox" kullanır.

clear; clc; close all;
rng(11);

%% ================== Radar ve dizi parametreleri ==================
fc = 77e9; c = 3e8; lambda = c/fc;
rangeMax = 100; rangeRes = 1;
B           = c/(2*rangeRes);
T_round_max = 2*rangeMax/c;
Tchirp      = 5.5*T_round_max;
slope       = B/Tchirp;
fbeat_max   = 2*slope*rangeMax/c;
fs          = 2*fbeat_max;

Nchirps  = 64;
Nsamples = 2^nextpow2(round(Tchirp*fs));
fs       = Nsamples/Tchirp;
PRF      = 1/Tchirp;

Nrx     = 8;
d_rx    = lambda/2;
NangFFT = 64;

%% ================== Animasyon ==================
Nframes   = 80;
frameRate = 20;
frame_dt  = 1/frameRate;

%% ================== Hedefler (Kartezyen, +X = sağ, +Y = ileri) ==================
% [X(m), Y(m), Vx(m/s), Vy(m/s), kazanç]
targets0 = [
   -10,  25,   2,  -8,   1.0;     % soldan, yaklaşıyor
    10,  55,   5,   3,   0.8;     % sağa kayıyor, hafif uzaklaşıyor
    40,  70, -10, -10,   0.6;     % çapraz kesiyor
   -35,  20,   0,   0,   0.7;     % sabit
   -20,  85,   0, -20,   0.5;     % uzaktan hızlı yaklaşıyor (geç katılan)
];
Ntarg   = size(targets0,1);
spawn_at = [1 1 1 1 25];           % son hedef 25. çerçevede sahneye girer

%% ================== Kalman filtre parametreleri ==================
% Constant-velocity (CV) modeli:  x_{k+1} = F·x_k + w
F = @(dt) [1 dt 0  0;
           0  1 0  0;
           0  0 1 dt;
           0  0 0  1];
H = [1 0 0 0;
     0 0 1 0];                     % konum ölçeriz (X, Y)
sigma_a = 4;                       % proses gürültüsü ~ ivme std (m/s²)
Qf = @(dt) sigma_a^2 * ...
      [dt^4/4 dt^3/2 0       0;
       dt^3/2 dt^2   0       0;
       0      0      dt^4/4  dt^3/2;
       0      0      dt^3/2  dt^2 ];
sigma_pos = 1.0;                   % ölçüm gürültüsü (m)
R_meas    = sigma_pos^2 * eye(2);

% Track yaşam kuralları
M_confirm = 3;                     % son N içinde M kez vurulursa confirm
N_window  = 5;
miss_conf = 5;                     % confirmed track bu kadar üst üste kaçırılırsa sil
miss_tent = 2;                     % tentative track çabuk silinsin
gate_xy   = 6;                     % ilişkilendirme eşiği (m)

%% ================== Sabit eksenler ve pencereler ==================
t_fast      = (0:Nsamples-1).' / fs;
freq_range  = (0:Nsamples/2-1).' * (fs/Nsamples);
range_axis  = freq_range * c / (2*slope);
dop_axis    = (-Nchirps/2:Nchirps/2-1) * (PRF/Nchirps);
vel_axis    = dop_axis * lambda/2;
ang_axis    = asind(2*(-NangFFT/2:NangFFT/2-1)/NangFFT);

win_r = hann(Nsamples);
win_d = hann(Nchirps).';
win_a = reshape(hann(Nrx), 1, 1, Nrx);

%% ================== Şekil ==================
fig = figure('Name','MIMO Radar + Kalman Tracker', 'Color','w', ...
             'Position',[40 40 1400 800]);
tl = tiledlayout(fig, 2, 3, 'Padding','compact', 'TileSpacing','compact');
ax_rd   = nexttile(tl, 1);
ax_ra   = nexttile(tl, 2);
ax_info = nexttile(tl, 3);
ax_be   = nexttile(tl, 4, [1 3]);

% Renk paleti (track ID'sine göre)
colors = lines(20);

%% ================== Tracker durumu ==================
tracks  = struct([]);
next_id = 1;

%% ================== Ana döngü ==================
targets     = targets0;
trail_truth = nan(Nframes, Ntarg, 2);

for fr = 1:Nframes

    % -------- (a) Hedefleri yürüt --------
    active = (fr >= spawn_at(:));
    targets(active,1) = targets(active,1) + targets(active,3)*frame_dt;
    targets(active,2) = targets(active,2) + targets(active,4)*frame_dt;

    % Sahnenin dışına çıkanları geri sar (animasyon devam etsin)
    R_all = sqrt(targets(:,1).^2 + targets(:,2).^2);
    out = active & ( R_all > rangeMax-2 | targets(:,2) < 5 );
    for ii = find(out).'
        targets(ii,1) = (rand-0.5) * 0.6 * rangeMax;
        targets(ii,2) = 50 + 30*rand;
    end

    trail_truth(fr,:,1) = targets(:,1);
    trail_truth(fr,:,2) = targets(:,2);

    % Sadece aktif olanlar radar simülasyonuna girer
    targ_active = targets(active,:);
    Na = size(targ_active,1);

    % Polar hale getir (radar üreteci için)
    Rt = sqrt(targ_active(:,1).^2 + targ_active(:,2).^2);
    th = atan2d(targ_active(:,1), targ_active(:,2));
    Vr = (targ_active(:,1).*targ_active(:,3) + ...
          targ_active(:,2).*targ_active(:,4)) ./ Rt;     % radyal hız

    % -------- (b) Beat küpü --------
    cube = zeros(Nsamples, Nchirps, Nrx);
    for k = 1:Nchirps
        t_slow = (k-1)*Tchirp;
        tx     = exp(1j*2*pi*(0.5*slope*t_fast.^2));
        chirp_cube = zeros(Nsamples, Nrx);
        for i = 1:Na
            R0i = Rt(i); vi = Vr(i); thi = th(i); ai = targ_active(i,5);
            R   = R0i + vi*t_slow;
            tau = 2*R/c;
            phi_arr = 2*pi*d_rx/lambda * sind(thi) * (0:Nrx-1);
            base    = ai * exp(1j*2*pi*( 0.5*slope*(t_fast - tau).^2 ...
                                         - fc*tau ));
            beat_n  = (tx .* conj(base)) * exp(-1j*phi_arr);
            chirp_cube = chirp_cube + beat_n;
        end
        cube(:,k,:) = chirp_cube;
    end

    % -------- (c) AWGN --------
    SNR_dB   = 14;
    sigPow   = mean(abs(cube(:)).^2);
    noisePow = sigPow / 10^(SNR_dB/10);
    cube     = cube + sqrt(noisePow/2) * ...
               ( randn(size(cube)) + 1j*randn(size(cube)) );

    % -------- (d) 3D FFT --------
    R3  = fft(cube .* win_r, Nsamples, 1);
    R3  = R3(1:Nsamples/2,:,:);
    RD3 = fftshift(fft(R3  .* win_d, Nchirps, 2), 2);
    RDA = fftshift(fft(RD3 .* win_a, NangFFT, 3), 3);
    RDA = flip(RDA, 3);                               % işaret konvansiyonu
    PowRDA = abs(RDA).^2;

    PowRD = sum(PowRDA, 3);
    RDdB  = 10*log10(PowRD + eps);  RDdB = RDdB - max(RDdB(:));
    PowRA = squeeze(sum(PowRDA, 2));
    RAdB  = 10*log10(PowRA + eps);  RAdB = RAdB - max(RAdB(:));

    % -------- (e) CFAR + (R,V,θ) kümeleme --------
    detMap = ca_cfar_2d(PowRD, [2 2], [4 4], 1e-4);
    [rIdx, dIdx] = find(detMap);

    detList = zeros(numel(rIdx), 3);
    for q = 1:numel(rIdx)
        sl = squeeze(PowRDA(rIdx(q), dIdx(q), :));
        [~, am] = max(sl);
        detList(q,:) = [range_axis(rIdx(q)), vel_axis(dIdx(q)), ang_axis(am)];
    end

    if ~isempty(detList)
        keys = [round(detList(:,1)/3), round(detList(:,2)/3), ...
                round(detList(:,3)/4)];
        [~, ~, ic] = unique(keys, 'rows');
        nC = max(ic);
        clusters = zeros(nC, 3);
        for q = 1:nC
            clusters(q,:) = mean(detList(ic==q, :), 1);
        end
    else
        clusters = zeros(0, 3);
    end

    % Polar -> Kartezyen ölçüm vektörü
    if ~isempty(clusters)
        meas_xy = [clusters(:,1).*sind(clusters(:,3)), ...
                   clusters(:,1).*cosd(clusters(:,3))];
        meas_v  = clusters(:,2);
        meas_th = clusters(:,3);
    else
        meas_xy = zeros(0,2); meas_v = []; meas_th = [];
    end

    %% ============== KALMAN MULTI-OBJECT TRACKER ==============

    % --- (1) PREDICT: tüm track'ler ---
    for ti = 1:numel(tracks)
        tracks(ti).x = F(frame_dt) * tracks(ti).x;
        tracks(ti).P = F(frame_dt) * tracks(ti).P * F(frame_dt).' + Qf(frame_dt);
    end

    % --- (2) ASSOCIATE: greedy en yakın komşu, gate = gate_xy ---
    Nm = size(meas_xy, 1);
    Nt = numel(tracks);
    assigned_meas  = false(Nm, 1);
    assigned_track = false(Nt, 1);
    pairs = zeros(0, 2);

    if Nm > 0 && Nt > 0
        track_pos = zeros(Nt, 2);
        for ti = 1:Nt
            track_pos(ti, :) = [tracks(ti).x(1), tracks(ti).x(3)];
        end
        D = pdist2_simple(track_pos, meas_xy);
        while true
            [mn, idx] = min(D(:));
            if isempty(mn) || mn > gate_xy, break; end
            [ti, mi] = ind2sub(size(D), idx);
            pairs(end+1, :) = [ti, mi];           %#ok<AGROW>
            assigned_track(ti) = true;
            assigned_meas(mi)  = true;
            D(ti, :) = inf;
            D(:, mi) = inf;
        end
    end

    % --- (3) UPDATE: eşleşen track'ler için Kalman update ---
    for p = 1:size(pairs, 1)
        ti = pairs(p,1);  mi = pairs(p,2);
        z      = meas_xy(mi,:).';
        P_pred = tracks(ti).P;
        x_pred = tracks(ti).x;
        S = H * P_pred * H.' + R_meas;
        K = P_pred * H.' / S;
        tracks(ti).x = x_pred + K * (z - H * x_pred);
        tracks(ti).P = (eye(4) - K*H) * P_pred;
        tracks(ti).hits   = tracks(ti).hits + 1;
        tracks(ti).misses = 0;
    end

    % --- (4) Kaçırılan track'lerde miss sayacı + tarih güncelle ---
    for ti = 1:numel(tracks)
        if ~assigned_track(ti)
            tracks(ti).misses = tracks(ti).misses + 1;
            tracks(ti).hit_history(end+1) = 0;          %#ok<AGROW>
        else
            tracks(ti).hit_history(end+1) = 1;          %#ok<AGROW>
        end
        if numel(tracks(ti).hit_history) > N_window
            tracks(ti).hit_history = tracks(ti).hit_history(end-N_window+1:end);
        end
        tracks(ti).age = tracks(ti).age + 1;
        tracks(ti).history(end+1, :) = [tracks(ti).x(1), tracks(ti).x(3)];

        if ~tracks(ti).confirmed && sum(tracks(ti).hit_history) >= M_confirm
            tracks(ti).confirmed = true;
        end
    end

    % --- (5) Eşleşmemiş ölçümlerden yeni track başlat ---
    for mi = find(~assigned_meas).'
        z    = meas_xy(mi, :);
        v_xy = [meas_v(mi)*sind(meas_th(mi)), meas_v(mi)*cosd(meas_th(mi))];
        new_track = struct( ...
            'id',          next_id, ...
            'x',           [z(1); v_xy(1); z(2); v_xy(2)], ...
            'P',           diag([1 25 1 25]), ...
            'hits',        1, ...
            'misses',      0, ...
            'age',         1, ...
            'confirmed',   false, ...
            'hit_history', 1, ...
            'history',     z );
        next_id = next_id + 1;
        if isempty(tracks)
            tracks = new_track;
        else
            tracks(end+1) = new_track;                  %#ok<AGROW>
        end
    end

    % --- (6) Çok fazla kaçırılanı sil (tentative çabuk, confirmed dayanıklı) ---
    keep = false(1, numel(tracks));
    for ti = 1:numel(tracks)
        if tracks(ti).confirmed
            keep(ti) = tracks(ti).misses < miss_conf;
        else
            keep(ti) = tracks(ti).misses < miss_tent;
        end
    end
    tracks = tracks(keep);

    %% ====================== Görselleştirme ======================

    % --- Range-Doppler ---
    cla(ax_rd);
    imagesc(ax_rd, vel_axis, range_axis, RDdB, [-35 0]);
    axis(ax_rd, 'xy'); ylim(ax_rd, [0 rangeMax]);
    colormap(ax_rd, turbo);  colorbar(ax_rd);
    xlabel(ax_rd, 'Hız (m/s)');  ylabel(ax_rd, 'Menzil (m)');
    title(ax_rd, sprintf('Range-Doppler  |  çerçeve %d/%d', fr, Nframes));
    if ~isempty(clusters)
        hold(ax_rd, 'on');
        plot(ax_rd, clusters(:,2), clusters(:,1), 'wo', ...
             'MarkerSize', 8, 'LineWidth', 1.2);
    end

    % --- Range-Angle ---
    cla(ax_ra);
    imagesc(ax_ra, ang_axis, range_axis, RAdB, [-35 0]);
    axis(ax_ra, 'xy'); ylim(ax_ra, [0 rangeMax]);
    colormap(ax_ra, turbo);  colorbar(ax_ra);
    xlabel(ax_ra, 'Açı (°)');  ylabel(ax_ra, 'Menzil (m)');
    title(ax_ra, 'Range-Angle');

    % --- Bilgi paneli ---
    cla(ax_info);  axis(ax_info, 'off');
    nConf = sum(arrayfun(@(t) t.confirmed, tracks));
    nTent = numel(tracks) - nConf;
    info_lines = {
        sprintf('Çerçeve     : %d / %d', fr, Nframes);
        sprintf('Gerçek hedef: %d', sum(active));
        sprintf('Algı (küme) : %d', size(clusters,1));
        sprintf('Track (toplam) : %d', numel(tracks));
        sprintf('  - confirmed  : %d', nConf);
        sprintf('  - tentative  : %d', nTent);
        sprintf('Sonraki track ID: %d', next_id);
        '';
        '— Renk = track ID —';
        '— Yeşil daire = gerçek konum —';
        '— Beyaz daire = ham algı —';
        };
    text(ax_info, 0.02, 0.97, info_lines, ...
         'VerticalAlignment','top', 'FontName','Consolas', 'FontSize', 10);
    title(ax_info, 'Tracker durumu');

    % --- Kuş bakışı ---
    cla(ax_be);  hold(ax_be, 'on');  grid(ax_be, 'on');

    % Menzil halkaları (referans)
    th_ring = linspace(-pi/2, pi/2, 80);
    for rr = [25 50 75 100]
        plot(ax_be, rr*sin(th_ring), rr*cos(th_ring), ...
             ':', 'Color', [0.85 0.85 0.85]);
    end

    % Gerçek hedeflerin geçmiş izleri (gri) + şimdiki konum (yeşil)
    for i = 1:Ntarg
        if fr >= spawn_at(i)
            plot(ax_be, squeeze(trail_truth(spawn_at(i):fr, i, 1)), ...
                       squeeze(trail_truth(spawn_at(i):fr, i, 2)), ...
                 '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
            plot(ax_be, targets(i,1), targets(i,2), 'go', ...
                 'MarkerSize', 11, 'MarkerFaceColor', [0.4 0.9 0.4]);
        end
    end

    % Ham algılar (zayıf, beyaz halka)
    if ~isempty(meas_xy)
        plot(ax_be, meas_xy(:,1), meas_xy(:,2), 'wo', ...
             'MarkerSize', 7, 'LineWidth', 1, ...
             'MarkerEdgeColor', [0.3 0.3 0.3]);
    end

    % Track'ler
    for ti = 1:numel(tracks)
        col = colors(mod(tracks(ti).id-1, size(colors,1)) + 1, :);
        % Geçmiş Kalman izi
        plot(ax_be, tracks(ti).history(:,1), tracks(ti).history(:,2), ...
             '-', 'Color', col, 'LineWidth', 1.6);
        % Şimdiki konum
        if tracks(ti).confirmed
            plot(ax_be, tracks(ti).x(1), tracks(ti).x(3), 's', ...
                 'MarkerSize', 12, 'MarkerFaceColor', col, ...
                 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
            % 2σ konum belirsizlik elipsi
            [ex, ey] = ellipse_xy([tracks(ti).x(1) tracks(ti).x(3)], ...
                                  tracks(ti).P([1 3],[1 3]), 2);
            plot(ax_be, ex, ey, '-', 'Color', col, 'LineWidth', 0.8);
            tag = sprintf('  ID %d', tracks(ti).id);
        else
            plot(ax_be, tracks(ti).x(1), tracks(ti).x(3), 'd', ...
                 'MarkerSize', 8, 'MarkerEdgeColor', col, 'LineWidth', 1);
            tag = sprintf('  ?%d', tracks(ti).id);
        end
        text(ax_be, tracks(ti).x(1)+1.2, tracks(ti).x(3), tag, ...
             'Color', col, 'FontWeight', 'bold');
    end

    % Radar
    plot(ax_be, 0, 0, 'k^', 'MarkerSize', 14, 'MarkerFaceColor', 'k');

    axis(ax_be, 'equal');
    xlim(ax_be, [-rangeMax rangeMax]);  ylim(ax_be, [-5 rangeMax+5]);
    xlabel(ax_be, 'X (m)');  ylabel(ax_be, 'Y (m)');
    title(ax_be, ['Kuş bakışı — Kalman tracker  ' ...
                  '(kare = confirmed, elmas = tentative, halka = 2σ belirsizlik)']);

    title(tl, sprintf( ...
        'FMCW MIMO Radar + Kalman MOT  —  Nrx=%d, fr=%d/%d, conf=%d, tent=%d', ...
        Nrx, fr, Nframes, nConf, nTent), 'FontWeight','bold');

    drawnow limitrate;
end

fprintf('\nTakip tamamlandı: %d çerçeve, son durumda %d track (%d confirmed).\n', ...
        Nframes, numel(tracks), sum(arrayfun(@(t) t.confirmed, tracks)));

%% ============================ Yardımcılar ============================
function det = ca_cfar_2d(P, guard, train, Pfa)
    [Nr, Nd] = size(P);
    det = false(Nr, Nd);
    NTrain = (2*(guard(1)+train(1))+1)*(2*(guard(2)+train(2))+1) ...
           - (2*guard(1)+1)*(2*guard(2)+1);
    alpha  = NTrain * (Pfa^(-1/NTrain) - 1);
    for r = (guard(1)+train(1)+1):(Nr-guard(1)-train(1))
        for d = (guard(2)+train(2)+1):(Nd-guard(2)-train(2))
            win = P(r-guard(1)-train(1):r+guard(1)+train(1), ...
                    d-guard(2)-train(2):d+guard(2)+train(2));
            win(train(1)+1:end-train(1), train(2)+1:end-train(2)) = NaN;
            Pn = mean(win(:), 'omitnan');
            if P(r,d) > alpha*Pn, det(r,d) = true; end
        end
    end
end

function D = pdist2_simple(A, B)
    % İki nokta kümesi arası Öklid mesafe matrisi (Stats Toolbox gerekmez)
    NA = size(A,1);  NB = size(B,1);
    D = zeros(NA, NB);
    for i = 1:NA
        D(i,:) = sqrt(sum( (B - A(i,:)).^2, 2 )).';
    end
end

function [x, y] = ellipse_xy(mu, P, sigma_scale)
    % 2D Gauss kovaryans elipsi (sigma_scale-σ konturu)
    th = linspace(0, 2*pi, 60);
    [V, D] = eig(P);
    pts = V * sqrt(max(D,0)) * [cos(th); sin(th)] * sigma_scale;
    x = mu(1) + pts(1,:);
    y = mu(2) + pts(2,:);
end
