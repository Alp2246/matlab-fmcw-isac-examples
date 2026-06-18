# MATLAB FMCW radar ve ISAC örnekleri

FMCW (77 GHz otomotiv benzeri) radar simülasyonları ve 28 GHz OFDM tabanlı ISAC (Integrated Sensing and Communication) demosu. Tüm betikler **Signal Processing Toolbox** ile çalışır (Phased Array gerekmez).

**GitHub:** https://github.com/Alp2246/matlab-fmcw-isac-examples

## Dosyalar

| Betik | Konu |
|-------|------|
| `fmcw_radar_range_doppler_ornek.m` | LFM chirp, dechirp, 2B Range-Doppler, CA-CFAR |
| `fmcw_radar_mimo_animasyon_ornek.m` | ULA açı kestirimi, animasyonlu çok hedef, kümeleme |
| `fmcw_radar_kalman_tracker_ornek.m` | Kalman MOT: track ID, gate, M/N doğrulama |
| `isac_ofdm_sensing_ornek.m` | Tek OFDM dalga şekli: QPSK iletişim + menzil-Doppler algılama |

Her betik bağımsızdır; dosyayı açıp **F5** ile çalıştırın.

## Önerilen sıra

1. `fmcw_radar_range_doppler_ornek.m` — temel menzil-Doppler
2. `fmcw_radar_mimo_animasyon_ornek.m` — açı + animasyon
3. `fmcw_radar_kalman_tracker_ornek.m` — takip katmanı
4. `isac_ofdm_sensing_ornek.m` — iletişim + algılama birleşimi

## GitHub'a yükleme (MATLAB → sonuç → push)

| Adım | Ne yapılır |
|------|------------|
| 1 | Betiği MATLAB'te çalıştır |
| 2 | `save_github_figure(gcf, 'fmcw_range_doppler')` |
| 3 | `.\push_to_github.ps1 -Message "Range-Doppler guncellendi"` |

**Repoya giren:** `.m`, `results/*.png`, README  
**Repoya girmeyen:** `.mat`, büyük veri (`.gitignore`)

## İlişkili repolar

- [gnss-spoofing-research](https://github.com/Alp2246/gnss-spoofing-research) — GNSS spoofing tespiti
- [matlab-wireless-comm-examples](https://github.com/Alp2246/matlab-wireless-comm-examples) — 5G / WLAN / BPSK-QPSK örnekleri

## Lisans

Eğitim/araştırma amaçlı örnek betikler. ISAC modeli Sturm & Wiesbeck (2011) makalesine dayalı sadeleştirilmiş simülasyondur.
