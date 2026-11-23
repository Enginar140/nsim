# 03 — Sensörler, Görüntüleme, Arayüz (gz-sensors, gz-rendering, gz-gui)

> Kaynak: inceleme ajanı raporu, 23.11.2026. Yollar `D:\rust-gazebo-ref\gazebo\` altına göredir.

## 1. gz-sensors
- `Sensor` taban sınıfı: `Load/Init/Update(now)` (`gz-sensors/include/gz/sensors/Sensor.hh:66-132`). Zamanlayıcı (`src/Sensor.cc:489-552`): `now < nextUpdateTime` ise atla; geride kalırsa **kaçan örnekler atılır**, enterpolasyon yok. Rate 0 = her çağrı.
- `Manager` sadece `map<SensorId, unique_ptr<Sensor>>` döngüsü.
- **Veri akışı:** her sensör tipi bir gz-sim sistemi; PostUpdate'te ECM'den poz/hız/ivme kopyalayıp `sensor->Update(simTime)` çağırır (örn. IMU `gz-sim/src/systems/imu/Imu.cc:130-337`).
- Manyetometre kaba 13×37 tablo; barometre US-1976; navsat küresel koordinat; logical camera CPU frustum testi; DVL akustik huzme (GpuRays).

**Gürültü** (`src/GaussianNoiseModel.cc`): ilk bias `N(mean,σ)` + %50 işaret çevirme; **Gauss-Markov dinamik bias** `σ_d = sqrt(σ_b²·τ/2·(1−e^{−2dt/τ}))`, `b ← e^{−dt/τ}·b + N(0,σ_d)` (yalnızca dt>0); çıktı `in + b + N(mean,σ)`, opsiyonel kuantalama. **σ örnek başına, yoğunluk değil** → frekans değişince random walk sessizce değişir.

### Gazebo'da bulunan gerçek hatalar
| Hata | Yer |
|---|---|
| NavSat yatay gürültüsü **derece** cinsinden enlem/boylama ekleniyor → "1 m" σ ≈ 111 km; enlem/boylam aynı gürültü nesnesini paylaşıyor; `dt` verilmiyor → bias hiç evrilmiyor | `NavSatSensor.cc:179-181` |
| Manyetometre `dt`'siz → bias kayması yok | `MagnetometerSensor.cc:175` |
| AirSpeed: "PASCALS" gürültüsü hPa değere ekleniyor sonra ×100 → **gürültü 100 kat** | `AirSpeedSensor.cc:173-188` |
| IMU `dt` float'a kırpılıyor | `ImuSensor.cc:219` |
| **Brown lens distorsiyonu OGRE2'de yok** (sadece OGRE1'de pas var), katsayılar yine de CameraInfo'ya yazılıyor | `ImageBrownDistortionModel.cc:100-110` |

### Gerçek donanıma göre eksikler
- **IMU:** titreşim/rotor harmoniği, Allan varyansı (ARW/VRW yoğunluk, bias kararsızlığı, rate random walk), ölçek faktörü, eksen kayıklığı, g-hassasiyeti, doyma, sıcaklık kayması, anti-alias filtre, coning/sculling.
- **GNSS:** gecikme, fix tipi/HDOP/uydu sayısı, multipath, kopma, RTK, metre cinsinden korelasyonlu random walk.
- **Manyetometre:** motor akımı girişimi, hard/soft-iron, yerel anomali.
- **Barometre:** pervane rüzgarı / yer etkisi, sıcaklık gecikmesi.
- **Kamera:** rolling shutter, hareket bulanıklığı, pozlama/kazanç, Poisson shot gürültüsü, vinyet, renk sapması, JPEG artefaktı, Bayer/ISP modeli.
- **Zamanlama:** hiçbir sensörde gecikme/jitter yok.

## 2. GPU sensör hattı
- Ayrı `renderThread` (`gz-sim/src/systems/sensors/Sensors.cc:409-565`); PostUpdate **önceki render bitene kadar bloklanıyor** (`:1023-1026`) → fizik render'a kilitli: deterministik ama yavaş.
- Kamera: abone yoksa çizmez; `CopyTo` doğrudan protobuf'a, sonra serileştirme + kopya. **GPU okuma senkron**, N+1 karesi boru hattına alınmıyor (`gz-rendering/ogre2/src/Ogre2GpuReadbackTicket.hh`).

| Sensör | Yöntem | Maliyet |
|---|---|---|
| Derinlik | çok paslı compositor + shader gürültü | orta |
| RGBD | derinlik + renk + CPU nokta bulutu | CPU |
| **GPU lidar** | **6 yüzlü cubemap** + yeniden örnekleme | en pahalı (taramada 6 sahne render) |
| Geniş açı | cubemap + fisheye remap | 6 render |
| Segmentasyon | malzeme değiştirici → 24-bit renk | +1 pas |
| Sınırlayıcı kutu | ID pası + CPU'da 2D/3D kutu | CPU ağır |
| Termal | nesne başına sıcaklık/ısı dokusu, 10 mK | radyometri/emisivite/atmosfer yok |

## 3. gz-rendering
- OGRE1/OGRE2 eklentileri; **OptiX kaldırılmış**. Soyut arayüz → `Base*` şablon → `Ogre2*`.
- PBR metal/roughness (normal, emissive, env map). GI = voxel cone (VCT/CIVCT, opsiyonel).
- **Fotogerçekçiliği sınırlayanlar:** tone mapping, pozlama, bloom, SSAO, motion blur yok; sadece shadow map; ışın izleme yok; basit gökyüzü.

## 4. Arayüz (Qt/QML)
- `gz::gui::Plugin : QObject`, QML kartları; gz-sim `GuiSystem::Update(UpdateInfo, ECM)` ekler; `GuiRunner` ayna ECM'i `/world/<w>/state`'ten besler. `MinimalScene` ayrı render thread + Qt RHI (GL/Vulkan/Metal).
- **gz-gui eklentileri:** 3D sahne, orbit/pan, kamera takip, grid, marker, nokta bulutu, dünya kontrol (oynat/durdur/adım), istatistik (RTF), ekran görüntüsü, cetvel, topic görüntüleyici/echo, grafik, görüntü paneli, navsat harita, yayıncı, teleop, klavye.
- **gz-sim eklentileri:** varlık ağacı, bileşen denetçisi (+ düzenleyici), seçim, sağ tık menüsü, transform gizmo, hizalama, kopyala/yapıştır, şekil/ışık/spawn, kaynak (Fuel) gezgini, fare ile sürükleme, kuvvet/tork uygulama, eklem kontrolü, görselleştirme (çarpışma, atalet, COM, eklem, tel kafes), temas/lidar/frustum gösterimi, 3D iz grafiği, ortam verisi, GI, video kaydı, log oynatma kaydırıcısı.

**MVP için şart:** 3D görünüm + orbit, dünya kontrol + istatistik, varlık ağacı, bileşen denetçisi (önce salt okuma), seçim + transform gizmo, görselleştirme anahtarları (çarpışma, çerçeve, COM, lidar ışını, frustum), sensör görüntü paneli, grafik, teleop.

## Rust tasarımı için dersler
```rust
trait Sensor: Send {
    fn spec(&self) -> &SensorSpec;   // rate, frame, latency, jitter, trigger
    fn needs(&self) -> StateMask;    // pose|twist|accel|wrench|contacts|env
    fn sample(&mut self, t: SimTime, s: &SensorInput, rng: &mut SensorRng) -> Option<Measurement>;
}
trait GpuSensor { fn encode(&mut self, ctx: &mut RenderCtx, t: SimTime) -> ReadbackHandle; }
```
- Zamanlayıcı ayrı; sensör zamanı **tamsayı ns**; fizik alt adımları örnek zamanına denk gelecek şekilde bölünür (örnek atma yok).
- Sensör başına tohumlu RNG (`ChaCha(world_seed, sensor_id)`).
- Gecikme/jitter modeli olan teslim kuyruğu (t'de alınan ölçüm t+gecikmede yayınlanır).
- **Gürültü yoğunluk cinsinden** (σ/√Hz → σ/√dt), bias kararsızlığı + rate random walk → doğrudan Allan varyans datasheet'inden; `dt` her zaman verilir.
- IMU: ölçek/eksen matrisi, g-hassasiyeti, doyma, kuantalama, anti-alias LPF, sıcaklık kayması, **rotor RPM'den titreşim**; alt adımlarda entegrasyon.
- GNSS: yerel NED/ENU'da metre → doğru lat/lon dönüşümü, Gauss-Markov, fix/HDOP durum makinesi, 100–200 ms gecikme.
- Manyetometre: WMM, hard/soft-iron, motor akımı girişimi.
- Kamera: rolling shutter, hız kaynaklı bulanıklık, pozlama + shot/read gürültüsü, Bayer/ISP.

**wgpu GPU sensör planı:** fizik pozlarını yansıtan `RenderWorld`, WGSL PBR; bu tikte vadesi gelen tüm sensörler tek `CommandEncoder`'da; **2–3'lü `MAP_READ` halka tampon + `map_async`** (fizik asla bloklanmaz, 1 kare gecikme yakalama zamanıyla damgalanır; testler için katı kilitli adım modu); post-process compute shader'larda (distorsiyon UV LUT, PCG gürültü, Bayer, rolling shutter); **segmentasyon/instance ID ayrı `R32Uint` hedefe** aynı pasta yazılır; lidar GPU ray-query veya compute BVH (6× cubemap yok); kutular ID tamponundan atomik min/max ile GPU'da; geniş açı yalnızca FOV>150°'de; `Arc<[u8]>` sıfır kopya yayın.

**Minimum GUI** (egui veya Bevy + bevy_egui): 3D görünüm + orbit/takip, oynat/durdur/adım/RTF, varlık ağacı + denetçi, tıkla seç + gizmo (`transform-gizmo-egui`), görselleştirme katmanları, sensör görüntü panelleri, `egui_plot` grafikleri, klavye/gamepad teleop, ekran görüntüsü/video, kayıt + kaydırıcılı oynatma.
