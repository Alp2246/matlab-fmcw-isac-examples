# FMCW / ISAC — simülasyon çıktıları

| Dosya | Betik | Açıklama |
|-------|-------|----------|
| [fmcw_range_doppler.png](fmcw_range_doppler.png) | `fmcw_radar_range_doppler_ornek.m` | 77 GHz FMCW, Range-Doppler, CA-CFAR |
| [isac_ofdm_sensing.png](isac_ofdm_sensing.png) | `isac_ofdm_sensing_ornek.m` | 28 GHz OFDM ISAC, BER + menzil-Doppler |
| [fmcw_mimo_animasyon.png](fmcw_mimo_animasyon.png) | `fmcw_radar_mimo_animasyon_ornek.m` | ULA açı, animasyon son kare |
| [fmcw_kalman_tracker.png](fmcw_kalman_tracker.png) | `fmcw_radar_kalman_tracker_ornek.m` | Kalman MOT, 80 çerçeve son kare |

## Yeniden üretme

```matlab
export_all_results
```

```powershell
.\push_to_github.ps1 -Message "Guncel sonuclar"
```
