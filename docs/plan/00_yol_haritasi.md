# Yol Haritası ve Mimari Taslağı — v0 (23.11.2026)

> Durum: **TASLAK, tartışmaya açık.** Dayanak: `docs/inceleme/` altındaki 5 inceleme notu.
> Kararlar (kullanıcı, 23.11.2026): ilk hedef **sabit kanat + multikopter birlikte**; arayüz **Bevy (yalnızca render+UI)**; repo **baştan açık kaynak**; konum **D:\rust-gazebo**.

## 1. Vizyon
Windows ve Linux'ta tek komutla derlenen, **deterministik**, İHA fiziğinde Gazebo'dan belirgin şekilde daha gerçekçi, ArduPilot/PX4 ile kilitli adımda çalışan; zamanla su altı, kara robotu ve robot kolunu da kapsayan açık kaynak bir robotik simülatörü.

## 2. Tasarım ilkeleri
1. **Çekirdek başsız (headless) ve Bevy'den bağımsız.** Simülasyon bir kütüphanedir; görüntüleyici, Python, RL ve CLI bunun istemcisidir.
2. **Determinizm baştan gelir:**
   - Zaman tamsayı nanosaniye (`i64`).
   - Dünya ve sensör başına tohumlu RNG.
   - Kararlı iterasyon sırası.
   - Dış komutlar yalnızca adım sınırında uygulanır.
   - Rapier `enhanced-determinism` ile çalışır.
3. **Fizik motoru değiştirilebilir, kuvvet modelleri motordan bağımsızdır.** Aero, rotor, güç aktarımı ve hidro, `ForceModel` olarak wrench üretir.
4. **Sessiz yok sayma yasak.** Desteklenmeyen özellik açık hata verir (Gazebo'nun en büyük tuzağı).
5. **Fiziksel doğruluk testle kanıtlanır.** Her model analitik çözüm, JSBSim veya gerçek uçuş loguyla karşılaştırma testi taşır.
6. **Tipli konfigürasyon.** SDF/URDF/MJCF yalnızca içe aktarılır; kendi şemamız serde ile tanımlanır.
7. **f64 her yerde.** Uzun uçuşlar ve büyük alanlar için gerekli.

## 3. Crate düzeni (Cargo workspace)

```
crates/
  core/            zaman (SimTime ns), ID'ler, nalgebra f64 matematik, çerçeveler (NED/ENU/FRD/FLU, WGS84), RNG akışları
  world/           varlık/bileşen deposu, fazlı zamanlayıcı (PreUpdate/Update/PostUpdate), anlık görüntü + geri yükleme
  scene/           VDF (`nvdf` crate) ile çözülmüş modeli simülasyon varlıklarına çevirir
  physics/         PhysicsBackend trait'i + yetenek bayrakları
  physics-rapier/  varsayılan arka uç (rapier3d-f64)
  atmosphere/      ISA, rüzgar alanı, Dryden/von Kármán türbülansı, gust, shear
  aero/            sabit kanat: CL/CD/Cm(α, β, δ) tabloları, CD0 + indüklenmiş, α̇/dinamik stall, downwash, yer etkisi
  rotor/           pervane/rotor: CT(J), CQ(J), H-kuvveti/flapping, VRS, yer etkisi — multikopter + itici pervane ortak
  powertrain/      ESC (PWM→duty, slew) → BLDC (Kv, R, back-EMF, atalet) → batarya (OCV(SoC) + R0 + RC)
  sensors/         IMU (Allan varyansı, titreşim), GNSS, baro, mag, pitot, mesafe ölçer; gecikme/jitter kuyruğu
  bridge-ardupilot/ JSON SITL köprüsü (servo paketi ↔ JSON durum, kilitli adım)
  runner/          dünya döngüsü, RTF, başsız step(n) API, kayıt (anahtar kare + diff)
  cli/             `xx run world.ron`, `xx replay log`
  viewer/          Bevy uygulaması: 3D sahne, kontrol, varlık ağacı, grafikler
models/            örnek araçlar (quad, sabit kanat; sonra Bertaraf)
docs/              inceleme/, plan/, adr/ (mimari karar kayıtları)
```
İleride eklenecekler: `transport-zenoh`, `python` (PyO3 + Gymnasium), `render-sensors` (wgpu kamera/lidar), `hydro`, `ground`, `physics-mujoco`, `physics-physx`, `physics-jolt`, `fdm-jsbsim`, `bridge-px4`.

## 4. MVP: ilk dikey dilim
**Hedef:** Windows ve Linux'ta aşağıdakilerin çalışması:
1. Bir **quad** ArduCopter SITL ile kalkar, misyon uçar, iner.
2. Bir **sabit kanat** ArduPlane SITL ile kalkar (el veya fırlatma), misyon uçar, iner.
3. Aynı tohumla tekrar çalıştırıldığında **birebir aynı** sonucu verir.
4. Bevy görüntüleyicisinde izlenir, grafikler çizilir.

**Kabul kriterleri (taslak):**
- `cargo test` ve CI yeşil (Windows ve Linux), çekirdek crate'lerde %80+ kapsam.
- Determinizm testi: Aynı tohumla 2 koşu sonunda durum hash'leri eşit.
- Rotor modeli: Statik itki tablosuyla örtüşür; J ile itki düşümü JSBSim pervane tablosuyla tutarlıdır.
- Aero: Trim edilmiş seyir hücum açısı ve sürükleme, aynı katsayılarla JSBSim trim sonucuna yakındır (tolerans belirlenecek).
- Batarya: Sabit akımda voltaj düşüşü ve itki kaybı gözlenir.
- SITL: Arm, kalkış, AUTO misyon ve iniş otomatik testle doğrulanır (CI'da Linux).

## 5. Aşamalar
| # | Aşama | İçerik | Çıktı |
|---|---|---|---|
| 0 | İskelet | workspace, CI (GitHub Actions win + linux), rustfmt/clippy, MIT/Apache-2.0, CONTRIBUTING, ADR şablonu | boş ama yeşil repo |
| 1 | Çekirdek | core, world, runner, physics trait + Rapier, anlık görüntü, determinizm testi | başsız düşen kutu, tekrarlanabilir |
| 2 | Uçuş fiziği | atmosphere, rotor, powertrain, aero + doğrulama testleri | Python/JSBSim karşılaştırma grafikleri |
| 3 | Sensör + SITL | CPU sensörleri, ArduPilot JSON köprüsü, kilitli adım; quad ve uçak modelleri | **ArduCopter + ArduPlane uçuşu** |
| 4 | Görüntüleyici | Bevy: 3D, takip kamerası, oynat/durdur/adım, varlık ağacı, egui_plot | görsel MVP |
| 5 | Sahne + arazi | RON şema, SDF/URDF içe aktarma, heightmap/DEM, WGS84 | Gazebo modellerini açabilme |
| 6 | Ekosistem | Zenoh transport, ROS 2 (`rmw_zenoh`), Python API + Gymnasium, çoklu paralel dünya | RL ve ROS kullanımı |
| 7 | Görüntü sensörleri | wgpu kamera/derinlik/segmentasyon/lidar, lens ve gürültü, YOLO veri dışa aktarımı | Savaşan İHA kilitlenme verisi |
| 8 | Alan genişletme | su altı (Siren/ArduSub: Fossen, dalga, itici), kara robotu, robot kolu (MuJoCo/PhysX) | çok alanlı sim |
| 9 | İleri | PX4, çoklu ajan Savaşan senaryosu, hata enjeksiyonu, gerçek log karşılaştırma/sistem tanılama | yarışma ve araştırma araçları |

Süre tahmini bilinçli olarak yazılmadı. 0–1 tamamlandıktan sonra gerçek hızla kalibre edilecek.

## 6. Kilit arayüzler (taslak)
```rust
// physics
trait PhysicsBackend: Send {
    fn caps(&self) -> Capabilities;
    fn build(&mut self, scene: &SceneDesc) -> Result<IdMap>;
    fn step(&mut self, dt: SimDuration, inputs: &StepInputs, out: &mut StepOutputs);
    fn save_state(&self) -> StateBlob; fn load_state(&mut self, s: &StateBlob) -> Result<()>;
    fn as_queries(&self) -> Option<&dyn Queries> { None }   // lidar/mesafe ölçer
    fn as_articulation(&mut self) -> Option<&mut dyn Articulation> { None }
}
// motordan bağımsız kuvvet modeli (aero, rotor, hidro…)
trait ForceModel: Send {
    fn apply(&mut self, ctx: &ForceCtx /* gövde durumu, atmosfer, aktüatörler, t */) -> Wrench /* CoM'da, dünya çerçevesinde */;
}
// sensör
trait Sensor: Send {
    fn spec(&self) -> &SensorSpec;        // rate, latency, jitter
    fn sample(&mut self, t: SimTime, s: &SensorInput, rng: &mut SensorRng) -> Option<Measurement>;
}
```

## 7. Kararlar ve açık konular
**Verilen kararlar (kullanıcı, 23.11.2026):**
- **Ad:** NSIM. "OpenSim" Stanford'un biyomekanik simülatörü ve OpenSimulator ile çakıştığı için elendi. crates.io'da `nsim` boş; PyPI'da küçük, ilgisiz bir `nsim` paketi var, Python paketine farklı ad verilecek.
- **ECS:** `hecs` + kendi fazlı zamanlayıcımız.
- **Tanım formatı:** **VDF (Vehicle Description Format)**, ayrı açık kaynak repo `D:\nvdf`. Crate/PyPI adı `nvdf`, uzantı `.nvdf` (YAML). Tasarım taslağı `D:\nvdf\docs\tasarim\00_vdf_tasarimi.md`. NSIM'in `scene` crate'i VDF'yi okur; SDF/URDF dönüşümü VDF tarafında yapılır.
- **Sıralama:** VDF v0.1 ile NSIM MVP paralel ilerler.
- **İlk sabit kanat modeli:** Önce Rascal 110 (JSBSim'den, doğrulama referansı), sonra Bertaraf (AeroSandbox).

**Açık konular:**
1. Görüntüleyici–çekirdek bağlantısı: MVP'de aynı süreç içinde kanal, aşama 6'da Zenoh (öneri).
2. MSRV / edition: Rust 2024, MSRV 1.86 (öneri).
3. Lisans: MIT + Apache-2.0 (öneri). JSBSim yalnızca dinamik bağlanır.
4. VDF açık kararları: saklama çerçevesi, açı birimi, katalog reposu, ifade dili (VDF tasarım belgesinin 10. bölümü).
