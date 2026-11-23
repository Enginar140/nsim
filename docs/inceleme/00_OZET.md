# Gazebo İncelemesi — Özet (23.11.2026)

Referans kaynaklar: `D:\rust-gazebo-ref\` (sığ klon). Gazebo: gz-sim 11.0.0-pre3, gz-physics 10. Fizik: Rapier 0.36.1, MuJoCo 4.0.2, Jolt 5.7.0, JSBSim 1.3.4dev, PhysX 5.13.0 (02 dosyasının 6. bölümü).

| Dosya | İçerik |
|---|---|
| [01_cekirdek_mimari.md](01_cekirdek_mimari.md) | ECS, sistem fazları, döngü, eklenti yükleme, SDF, transport, log |
| [02_fizik_motorlari.md](02_fizik_motorlari.md) | gz-physics soyutlaması, Rapier/MuJoCo/Jolt/JSBSim, önerilen trait |
| [03_sensorler_render_gui.md](03_sensorler_render_gui.md) | Sensör + gürültü modelleri, GPU sensörler, render, GUI eklentileri |
| [04_alan_modelleri.md](04_alan_modelleri.md) | Uçak/dron/su altı/batarya/kara denklemleri, ArduPilot JSON protokolü |

## En önemli 10 bulgu
1. **Gazebo'da gerçek hatalar var:**
   - NavSat gürültüsü derece cinsinden eklendiği için "1 m" sapma yaklaşık 111 km oluyor.
   - AirSpeed gürültüsü 100 kat büyük.
   - Manyetometre ve GPS bias kayması hiç çalışmıyor.
   - OGRE2'de lens distorsiyonu yok.
   - ArduPilot eklentisinde EFFORT kanalı null pointer hatası veriyor.
2. **Determinizm zayıf:**
   - Global RNG kullanılıyor ve v11'de iterasyon sırası belirsiz.
   - PostUpdate asenkron, varlıklar arka planda yükleniyor.
3. **Eklenti ABI'si sahte:** Gerçekte "aynı derleyici ve aynı STL" şartı var.
4. **Fizik soyutlaması eksik özellikleri sessizce atlıyor**, her arka uç SDF'yi kendisi ayrıştırıyor. Akışkan kuvvetleri zaten motor dışında hesaplanıyor.
5. **Gazebo da Zenoh'a ve MuJoCo'ya geçiyor.** Doğru yöne bakıyoruz.
6. **Sabit kanat modeli zayıf:**
   - Temel LiftDrag'da CD0 ve indüklenmiş sürükleme yok.
   - Zephyr'in itkisi pervane palalarının LiftDrag'ından geliyor.
7. **Multikopterde itki hava hızından bağımsız (`kω²`).** VRS, yer etkisi ve ESC modeli yok. ArduPilot iris'i motor modelini hiç kullanmıyor.
8. **Batarya voltajı itkiyi etkilemiyor.** Rüzgarda Dryden türbülansı yok; rüzgar kuvveti kütleyle orantılı.
9. **Su altında dalga yok, mesh için kısmi batma yok.** Eklenmiş kütle sonlu farkla hesaplanıyor, bu kararsızlık riski taşıyor.
10. **Render fizikle kilitli adımda ve GPU okuma senkron.** GPU lidar 6 yüzlü cubemap kullandığı için pahalı.

## Rust için ana yön (taslak, karar değil)
- **ECS:** Tiplerle zorlanmış Pre/Update/PostUpdate fazları ve okuma/yazma kümesi bildiren zamanlayıcı. Değişim takibi otomatik; ucuz anlık görüntü ile reset, seek ve kayıt tek mekanizma.
- **Fizik:** Varsayılan motor Rapier f64 (`enhanced-determinism` ile). Opsiyonel motorlar: MuJoCo (robot kolu ve RL), PhysX (CPU artikülasyon ve GPU toplu RL), Jolt (araçlar). JSBSim LGPL olduğu için dinamik bağlanacak.
- **Kuvvet modelleri:** Aero, rotor, güç aktarımı ve hidro, motordan bağımsız `ForceModel` eklentileri olacak.
- **Altyapı:**
  - Transport: Zenoh.
  - Dosya formatı: SDF/URDF/MJCF yalnızca içe aktarma için, kendi tipli şemamız olacak.
  - ArduPilot JSON köprüsü birinci sınıf özellik olacak.
- **Görüntü:**
  - wgpu ve WGSL; GPU okuma boru hattına alınacak, segmentasyon ve ID ayrı hedeflere yazılacak.
  - Gerçekçi sensör gürültüsü yoğunluk cinsinden (Allan varyansı) tanımlanacak.
