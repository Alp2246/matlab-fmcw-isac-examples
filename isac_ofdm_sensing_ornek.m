%% isac_ofdm_sensing_ornek.m
% ISAC / JCAS demo:  TEK bir OFDM dalga şekliyle aynı anda
%
%   (1) İLETİŞİM   :  random bits  ->  QPSK  ->  OFDM  ->  AWGN  ->  BER
%   (2) ALGILAMA   :  TX echo  ->  element-wise divide  ->  2B FFT  ->
%                     menzil-Doppler haritası
%
% Bu, Sturm & Wiesbeck (2011) "Waveform Design and Signal Processing
% Aspects for Fusion of Wireless Communications and Radar Sensing"
% makalesindeki ISAC OFDM modelinin sade, monostatik halidir.
%
% 5G NR mmWave numerolojiye yakın parametreler:
%   fc = 28 GHz, df = 120 kHz (μ=3), Nsc = 512, Nsym = 64.
%
% Sadece temel MATLAB + Signal Processing Toolbox kullanır
% (Communications Toolbox / 5G Toolbox şart değil).

clear; clc; close all;
rng(42);

%% =================== Parametreler ===================
fc       = 28e9;             % taşıyıcı [Hz]
c        = 3e8;
lambda   = c/fc;

Nsc      = 512;              % alt-taşıyıcı sayısı (FFT noktası)
Nsym     = 64;               % OFDM sembol sayısı (1 sensing frame)
df       = 120e3;            % alt-taşıyıcı aralığı (5G NR μ=3)
BW       = Nsc*df;           % toplam bant genişliği
Tsym0    = 1/df;             % CP'siz sembol süresi
CPratio  = 1/4;
Tcp      = CPratio*Tsym0;
Tsym     = Tsym0 + Tcp;      % CP dahil sembol süresi (radar için "PRI")
PRF      = 1/Tsym;

% Türev (sensing) sınırları
range_res = c/(2*BW);
range_max = c/(2*df);        % CP'nin desteklediği teorik maks
vel_res   = lambda/(2*Nsym*Tsym);
vel_max   = lambda/(4*Tsym);

%% =================== Hedefler [R (m), V (m/s), kazanç] ===================
targets = [
     30,  -25,   1.0;     % yakın yaklaşan
     80,  +15,   0.7;     % uzaklaşan
    150,    0,   0.5;     % sabit (köprü ayağı, bina vb.)
    115,  -45,   0.8;     % hızlı yaklaşan
];
Ntarg = size(targets,1);

%% =================== TX:  bitler -> QPSK -> grid X ===================
M            = 4;
bits_per_sym = log2(M);
Nbits        = Nsc*Nsym*bits_per_sym;
bitsTX       = randi([0 1], Nbits, 1);

% Gray QPSK (sabit modül 1)
b1 = bitsTX(1:2:end);
b2 = bitsTX(2:2:end);
qpsk = ( (1 - 2*b1) + 1j*(1 - 2*b2) ) / sqrt(2);   % |qpsk| = 1
X    = reshape(qpsk, Nsc, Nsym);                   % freq-domain grid

%% =================== (1) İLETİŞİM RX  (uzak bir UE) ===================
% Basitleştirme: comm tarafı düz AWGN üstünden alır (kanal yok)
SNR_comm_dB = 8;
sigP_c      = mean(abs(X(:)).^2);
nP_c        = sigP_c / 10^(SNR_comm_dB/10);
Yc          = X + sqrt(nP_c/2) * ( randn(size(X)) + 1j*randn(size(X)) );

% Hard-decision QPSK demap
b1_hat   = real(Yc) < 0;
b2_hat   = imag(Yc) < 0;
bitsRX   = reshape([b1_hat(:).' ; b2_hat(:).'], [], 1);
ber_comm = mean(bitsRX ~= bitsTX);

%% =================== (2) SENSING:  echo modeli  ===================
% Frekans-zaman düzleminde ÇARPAN olarak hedefler (ICI ihmal):
%   Y[k,m] = X[k,m] · Σ_i  α_i · exp(-j·2π·k·df·τ_i) · exp(j·2π·fd_i·m·Tsym)
%   τ_i = 2R_i / c       fd_i = 2v_i / λ
k_idx = (0:Nsc-1).';
m_idx = (0:Nsym-1);
Hch   = zeros(Nsc, Nsym);
for i = 1:Ntarg
    R = targets(i,1);  v = targets(i,2);  a = targets(i,3);
    tau = 2*R/c;
    fd  = 2*v/lambda;
    Hch = Hch + a * exp(-1j*2*pi*k_idx*df*tau) * exp(1j*2*pi*fd*m_idx*Tsym);
end
Y = X .* Hch;

% Sensing AWGN (monostatik, TX gücü bilinir -> tipik olarak SNR yüksek)
SNR_sens_dB = 20;
sigP_s = mean(abs(Y(:)).^2);
nP_s   = sigP_s / 10^(SNR_sens_dB/10);
Y      = Y + sqrt(nP_s/2)*( randn(size(Y)) + 1j*randn(size(Y)) );

%% =================== (2.b) ISAC sensing prosesi ===================
% --- ANA FİKİR ---
% QPSK sabit modüllü ( |X[k,m]| = 1 ) olduğu için Y'yi X'e böldüğümüzde
% TX modülasyonu silinir, geriye SADECE kanal kalır:
%       H_est[k,m] = Y[k,m] / X[k,m]  =  Σ_i α_i · exp(...)
% Bu, k'da ve m'de lineer faz rampaları olan iki boyutlu kompleks sinüstür.
% IFFT_k -> menzil ekseni,   FFT_m -> Doppler ekseni.

H_est = Y ./ X;

% (i) IFFT alt-taşıyıcı boyunca -> her sembol için menzil profili
range_profile_mat = ifft(H_est, Nsc, 1) * Nsc;     % korunum için *Nsc

% (ii) FFT sembol boyunca -> her menzil hücresi için Doppler
RD = fftshift(fft(range_profile_mat, Nsym, 2), 2);

% Eksenler
range_axis = (0:Nsc-1) * range_res;
dop_axis   = (-Nsym/2:Nsym/2-1) * (PRF/Nsym);
vel_axis   = dop_axis * lambda/2;

% Genlik (dB)
PowRD = abs(RD).^2;
RDdB  = 10*log10(PowRD + eps);
RDdB  = RDdB - max(RDdB(:));

%% =================== Görselleştirme ===================
fig = figure('Name','ISAC OFDM — Comm + Sensing', 'Color','w', ...
             'Position',[60 60 1320 780]);
tl = tiledlayout(fig, 2, 2, 'Padding','compact', 'TileSpacing','compact');

% (1) TX QPSK takımyıldızı
nexttile(tl, 1);
plot(real(qpsk(1:2000)), imag(qpsk(1:2000)), '.', 'MarkerSize', 10);
axis equal; grid on; xlim([-1.7 1.7]); ylim([-1.7 1.7]);
xlabel('I'); ylabel('Q');
title('Comm TX  —  QPSK (gönderilen)');

% (2) Comm RX takımyıldızı + BER
nexttile(tl, 2);
sample = Yc(:);
sample = sample(1:min(numel(sample), 5000));
plot(real(sample), imag(sample), '.', 'MarkerSize', 4);
axis equal; grid on; xlim([-1.7 1.7]); ylim([-1.7 1.7]);
xlabel('I'); ylabel('Q');
title(sprintf('Comm RX  —  SNR = %d dB,  BER = %.2e', SNR_comm_dB, ber_comm));

% (3) Tek sembolün menzil profili
nexttile(tl, 3);
prof_dB = 20*log10(abs(range_profile_mat(:,1)) + eps);
prof_dB = prof_dB - max(prof_dB);
plot(range_axis, prof_dB, 'LineWidth', 1.2);
xlim([0 220]); grid on;
xlabel('Menzil (m)'); ylabel('Genlik (dB)');
title('Sensing  —  tek OFDM sembolünün menzil profili');
hold on;
xline(targets(:,1), '--', compose('T%d', (1:Ntarg).'), ...
      'LabelHorizontalAlignment','center', 'Color',[0.4 0.4 0.4]);

% (4) Range-Doppler haritası
nexttile(tl, 4);
imagesc(vel_axis, range_axis, RDdB, [-40 0]);
axis xy; ylim([0 220]); colormap(turbo); colorbar;
xlabel('Hız (m/s)'); ylabel('Menzil (m)');
title('Sensing  —  Range-Doppler haritası (dB)');
hold on;
plot(targets(:,2), targets(:,1), 'wo', 'MarkerSize', 12, 'LineWidth', 1.6);
text(targets(:,2)+2, targets(:,1), compose('  T%d', (1:Ntarg).'), 'Color','w');

title(tl, sprintf( ...
    ['ISAC / JCAS — fc=%.0f GHz, BW=%.1f MHz, \\Deltaf=%.0f kHz, ' ...
     'N_{sc}=%d, N_{sym}=%d   |   tek dalga şekli, iki iş'], ...
    fc/1e9, BW/1e6, df/1e3, Nsc, Nsym), 'FontWeight','bold');

%% =================== Konsol özeti ===================
fprintf('\n=== ISAC OFDM parametreleri ===\n');
fprintf('  fc        = %.0f GHz   lambda = %.2f mm\n', fc/1e9, lambda*1e3);
fprintf('  N_sc=%d   N_sym=%d   \\Delta f=%.0f kHz   BW=%.2f MHz\n', ...
        Nsc, Nsym, df/1e3, BW/1e6);
fprintf('  T_sym = %.2f us  (CP=%.2f us, CP/T=%.0f%%)\n', ...
        Tsym*1e6, Tcp*1e6, CPratio*100);
fprintf('  Range:    \\Delta r = %.2f m   R_max = %.0f m (CP)\n', ...
        range_res, range_max);
fprintf('  Doppler:  \\Delta v = %.2f m/s   |V_max| = %.0f m/s\n', ...
        vel_res, vel_max);

fprintf('\n=== (1) Comm tarafı ===\n');
fprintf('  SNR = %d dB   ->   BER = %.3e   (%d bit gönderildi)\n', ...
        SNR_comm_dB, ber_comm, Nbits);

fprintf('\n=== (2) Sensing tarafı — gerçek hedefler ===\n');
disp(array2table(targets, 'VariableNames', {'R_m','V_mps','Gain'}));
fprintf('Aynı OFDM dalga şekli her iki işi de aynı anda yaptı.\n\n');
