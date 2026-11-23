# 02 — Fizik Motorları (gz-physics, Rapier, MuJoCo, Jolt, JSBSim)

> Kaynak: inceleme ajanı raporu, 23.11.2026. Yollar `D:\rust-gazebo-ref\` altına göredir.

## 1. gz-physics (v10.0.0)
- **"Feature" API:** `Feature` içinde Engine/World/Model/Link/Joint/Shape şablon sınıfları (`gazebo/gz-physics/include/gz/physics/Feature.hh:35-127`); `FeatureList<...>` derleme zamanı liste; `FeaturePolicy<Scalar,Dim>` f32/f64 + 2D/3D. Arka uç `GZ_PHYSICS_ADD_PLUGIN` ile kaydolur, gz-plugin ile yüklenir.
- `ForwardStep` tipli girdi/çıktı torbaları: girdi `ApplyExternalForceTorques`, çıktı `WorldPoses`, `ChangedWorldPoses`, `Contacts`, `JointPositions`.
- gz-sim zorunlu `MinimumFeatureList` ister (`gz-sim/src/systems/physics/Physics.cc:180-191`), ~30 opsiyonel listeyi varlık başına `EntityCast` ile yoklar. **Eksik özellik → bir kez log, sessizce atla** (örn. heightmap `:1916-1925`). Varsayılan DART (`:1041`).

| Arka uç | Durum |
|---|---|
| **dartsim** | En eksiksiz: eklem limit/sürtünme/sönüm/yay, heightmap + mesh, ışın sorgusu (CPU lidar buna bağlı), **AddedMass yalnızca burada** |
| bullet (maksimal koordinat) | Minimal: döner/sabit eklem, hız komutu |
| bullet-featherstone | İndirgenmiş koordinat, limit/mimic/temas; **heightmap yok** |
| tpe | Sadece AABB, dinamik yok (kinematik dünya) |
| **mujoco** (yeni, 2025) | Kısmi: limit/efor/sönüm/yay/sürtünme **yorum satırında**; heightmap yok. Her yapısal düzenlemede (poz atamak dahil) **`mj_recompile`** (`gz-physics/mujoco/src/Base.cc:171-210`). `mjcb_contactfilter` **süreç genelinde global**. |

**Soyutlamanın acıları:** sessizce yok sayılan özellikler (motorlar arası fark), her arka ucun SDF'yi kendisi ayrıştırması (1100–1670 satırlık `SDFFeatures.cc`'ler), derlenmiş model motorlarına (MuJoCo) uymayan parça parça kurulum, aynı isimde farklı semantik (hız komutu, temas yumuşaklığı), ağır şablon makinesi. **Akışkan etkileri (lift/drag, buoyancy, hydro) motor dışında** gz-sim sistemleri olarak dış kuvvet uyguluyor.

## 2. Rapier (v0.36.1, Apache-2.0, Rust 1.86 / edition 2024)
- Crate'ler: `rapier3d`, `rapier3d-f64`, **`rapier3d-urdf`, `rapier3d-mjcf`, `mjcf-rs`, `rapier3d-meshloader`**; 0.36'da C bağlamaları da geldi.
- **Multibody = indirgenmiş koordinat** (`src/dynamics/joint/multibody_joint/multibody.rs:93`): Jacobian, ileri kinematik, IK, eklem kuplajı, motor + armature. Ayrıca impulse eklemler (sabit/döner/prizmatik/küresel/ip/yay/pin-slot).
- Motorlar: `AccelerationBased` (varsayılan) / `ForceBased`; limit, hız/poz motoru, max kuvvet.
- CCD var (`rigid_body.rs:596`, `max_ccd_substeps`).
- **Determinizm:** `enhanced-determinism` → **platformlar arası deterministik** (`simd8` ile olmaz, `parallel` ile olur, thread sayısından bağımsız; aynı ekleme sırası şart).
- SIMD 4 şerit varsayılan, `simd8` AVX, `parallel` rayon.
- Sorgular: `cast_ray`, `cast_ray_and_get_normal`, `cast_shape`, `QueryFilter` → CPU lidar için yeterli (rayon ile toplu).
- Şekiller: heightfield, trimesh, konveks ayrıştırma, konveks zarf, voxel. 0.36'da yumuşak cisim/FEM deneysel.
- Zayıflık: PGS/sıralı impulse çözücü → sert robot kolları için Featherstone/Newton kalitesinde değil.

## 3. MuJoCo (4.0.2, Apache-2.0)
- **Temiz C API** (bindgen'e uygun): `mj_loadXML`, `mj_compile`, `mj_step/step1/step2`, `mj_forward`, `mj_inverse`, `mj_contactForce`, **`mj_ray`/`mj_multiRay`** (`include/mujoco/mujoco.h`).
- **Yeni toplu API `mjbatch.h`**: çok simülasyonu thread havuzunda koşturur → RL için doğrudan.
- Varsayılan double; HFIELD + SDF geometri; implicitfast integratör, Newton çözücü.
- MJX (JAX) ve MuJoCo Warp GPU — Python tarafı.
- Windows: hazır `.zip`, **AVX şart**. Determinizm yalnızca aynı sürüm + aynı mimari.
- Geri çağrılar **süreç genelinde global** (çok dünya için sorun). Rust bağlaması: **MuJoCo-rs** (olgunluğu doğrulanmadı).
- Güçlü yanlar: yumuşak konveks temas + tam sürtünme konisi, tendon, aktüatör, ters dinamik, RL ekosistemi.

## 4. JoltPhysics (5.7.0, MIT)
- **Sadece C++**; C API yok → JoltC / jolt-rust gibi dış bağlamalar (olgunluğu doğrulanmadı).
- `CROSS_PLATFORM_DETERMINISTIC` (~%8 maliyet), MSVC/clang/gcc, x86/ARM/WASM testli. Broadphase sorgu ve callback sırası deterministik değil.
- **En iyi araç yığını:** tekerlekli, paletli, motosiklet + motor/şanzıman/diferansiyel/anti-roll. Yumuşak cisim, double hassasiyet, **yerleşik kaldırma kuvveti** (`Body::ApplyBuoyancyImpulse`).
- İndirgenmiş koordinatlı multibody yok.

## 5. JSBSim (1.3.4.dev1, **LGPL-2.1**)
- **Aero XML:** eksen başına (`DRAG/SIDE/LIFT/...`) `qbar × S × tablo(alpha, beta, mach, flap, stall histerezisi)` fonksiyonları (`aircraft/c172p/c172p.xml:474-660`); kontrol yüzeyleri FCS bileşenleriyle (aktüatör, PID, filtre).
- **İtki:** `FGElectric`, **`FGBrushLessDCMotor`** (Kv, sargı direnci, boşta akım), **`FGPropeller` C_T/C_P(J) tabloları** (`engine/prop_generic2f.xml`). F450 quad modeli de var.
- **Atmosfer/rüzgar:** standart atmosfer, MSIS; türbülans **Milspec / Tustin (Dryden)**, şiddet 0–7, gust, downburst (`src/models/atmosphere/FGWinds.h:57-270`).
- Yer teması: yay + sönüm + sürtünme.
- **Gömme API** `FGFDMExec`: `LoadModel`, `Setdt`, `RunIC`, `Run`, `DoTrim`, `Get/SetPropertyValue`; arazi için `FGGroundCallback`. **C API yok** → küçük `extern "C"` köprü (veya `cxx` crate).
- ArduPilot ve PX4 SITL JSBSim'i doğrudan çalıştırabiliyor.
- **LGPL:** **dinamik bağlama** (DLL/.so) ya da ayrı süreç + soket. Kendi kodumuz istediğimiz lisansta kalır.

## Öneri tablosu

| Motor | Rolü | Rust'tan entegrasyon | Lisans | Determinizm |
|---|---|---|---|---|
| **Rapier (f64)** | **Varsayılan katı cisim**: kara robotları, multikopter, su altı (kendi aero/hidro kuvvetlerimizle), lidar ışınları, arazi | Yerli crate, en ucuz | Apache-2.0 | `enhanced-determinism` ile platformlar arası |
| **MuJoCo** | Robot kolları, manipülasyon, **RL** (toplu rollout, MJCF) | Orta: bindgen / MuJoCo-rs, hazır DLL, AVX; çalışırken düzenleme pahalı | Apache-2.0 | Aynı sürüm + mimari |
| **Jolt** | Kara araçları (tekerlek/palet), büyük dünya, su üstü, yumuşak cisim | Yüksek: C++ köprüsü gerekli | MIT | Bayrakla platformlar arası (~%8) |
| **JSBSim** | **Sabit kanat FDM** — sadece kuvvet hesaplar, gövde ana motorda | Orta: C köprüsü, property I/O | **LGPL → dinamik bağla** | Tek thread sabit adım, aynı derlemede tekrarlanabilir |

## Önerilen Rust trait şekli
gz-physics'ten dersler: şablon yerine **çalışma zamanında sorgulanan trait'ler**, her arka ucun SDF ayrıştırması yerine **ortak sahne ara temsili (IR)**, toplu (batch) işlemler, **açık "desteklenmiyor" hatası** (sessiz yok sayma yok).

```rust
trait PhysicsBackend: Send {
    fn caps(&self) -> Capabilities;                       // bit bayrakları; asla sessizce yok sayma
    fn build(&mut self, scene: &SceneDesc) -> Result<IdMap>; // motordan bağımsız IR, çerçeveler çözülmüş, f64
    fn step(&mut self, dt: f64, inputs: &StepInputs, out: &mut StepOutputs);
    fn set_body_pose(&mut self, b: BodyId, p: Iso3) -> Result<()>;
    fn remove(&mut self, e: EntityId) -> Result<()>;
    fn save_state(&self) -> StateBlob; fn load_state(&mut self, s: &StateBlob); // geri sarma / RL / determinizm testi
}
// Opsiyonel yetenekler: fn as_joints(&mut self) -> Option<&mut dyn Articulation> ...
trait Articulation { /* eklem durumu, Effort|Vel|Pos{kp,kd} komut, limit, iletilen wrench */ }
trait DynamicEdit  { /* add_body/collider/joint, attach/detach — toplu + recompile serbest */ }
trait Shapes       { fn supports(&self, k: ShapeKind) -> bool; } // Heightfield, TriMesh, ConvexDecomp, Sdf
trait Queries      { fn cast_rays(&self, rays: &[Ray], f: &Filter, hits: &mut [Hit]); } // lidar/sonar
trait Contacts     { /* temaslar, malzeme, filtre */ }
trait FluidHooks   { fn set_added_mass(&mut self, b: BodyId, m: Mat6); } // yoksa dış kuvvetle emüle
trait Vehicles     { /* Jolt araçları */ }
```
**Aero/itki/hidro (JSBSim, Fossen, motor modelleri) ayrı `ForceModel` eklentileri** olsun: gövde durumunu okur, `StepInputs`'a wrench yazar → her arka uçla çalışır.

---

## 6. PhysX (SDK 5.13.0, Apache-2.0) — ek inceleme
> Yollar `D:\rust-gazebo-ref\physics\PhysX\` altına göre. LFS ikilileri indirilmedi, sadece kaynak.

**Klasörler:**
- `physx/`: çekirdek SDK. Tam CUDA kaynağı dahil (61 `.cu` dosyası).
- `blast/`: kırılma/parçalanma, içinde V-HACD var.
- `flow/`: GPU duman ve ateş efekti (görsel amaçlı).
- `ovphysx/` 0.8: USD tabanlı, C API'li başsız katman (Isaac Lab tarzı). Ancak Carbonite ikilileri Omniverse lisanslı ve henüz ön sürüm.
- `schemas/`: PhysX USD şemaları.
- Lisans: Kökteki `LICENSE.md` BSD diyor ama eski görünüyor. SDK Apache-2.0 (`physx/LICENSE.md`, CHANGELOG v5.11).

**Artikülasyon** (`PxArticulationReducedCoordinate`):
- İndirgenmiş koordinat. Eklem tipleri: sabit, prizmatik, döner, küresel; 6 eksen kilitli, limitli veya serbest.
- Örtük PD sürücü (kuvvet veya ivme modu), "performans zarfı" (max efor ve hız, hıza bağlı direnç), armature, statik/dinamik/viskoz sürtünme.
- Mimic eklem, sabit ve uzaysal tendon.
- Hesaplamalar: ileri/ters dinamik, Jacobian, kütle matrisi, Coriolis/yerçekimi, centroidal momentum. GPU'da toplu sürümleri de var.
- RL kolu ve bacaklı robot için MuJoCo düzeyinde. Kapalı zincir, genel eşitlik kısıtı ve yumuşak temas tarafında MuJoCo önde.

**Araç (vehicle2):**
- Modüler bileşen hattı; motor, debriyaj, şanzıman, diferansiyel; tank sürüşü.
- Tank sürüşü gerçek palet değil, tekerlek hızlarını eşitleyerek yapılıyor.
- **Lastik modeli Pacejka değil.** Yüke bağlı doyan yanal sertlik ve 3 noktalı sürtünme–kayma eğrisi kullanılıyor.
- Yalnızca CPU'da çalışıyor. Jolt'la denk, Jolt'un yerine geçmeye değmez.

**Determinizm:**
- `eENABLE_ENHANCED_DETERMINISM` yalnızca CPU'da çalışıyor; **GPU'da desteklenmiyor**.
- Platformlar arası birebir aynılık garantisi yok.
- GPU'daki atomik indirgemeler deterministik değil.

**GPU:**
- CUDA opsiyonel; her platform için `-cpu-only` hazır ayar (preset) var.
- GPU sürümü: CUDA 12.8 ve Volta veya üstü kart gerekiyor.
- Rijit cisim ve artikülasyon GPU'da çalışabiliyor.
- **Direct-GPU API:** Toplu durum okuma/yazma, Jacobian ve kütle matrisi doğrudan CUDA tamponlarında. Isaac Lab bu yolu kullanıyor.
- PBD parçacık, FEM ve kumaş **yalnızca GPU'da** çalışıyor.
- SDF çarpışması için belgeler çelişiyor: bir yerde "sadece GPU", başka yerde "CPU'da da çalışır" deniyor. Test edilmeli.

**Akışkan / kaldırma / rüzgar:**
- PBD akışkanında `setWind` var, ama **rijit cisim için kaldırma kuvveti, hidrodinamik ve rotor aerodinamiği yok.**
- Bunlar için kendi `ForceModel` eklentilerimiz gerekecek (diğer motorlarda da öyle).

**Sorgular:**
- raycast, sweep ve overlap var. `PxBatchQueryExt` CPU'da çalışıyor ve kendi içinde çok iş parçacıklı değil.
- GPU'da sensör ışın sorgusu yok.
- Konveks ayrıştırma SDK'da yok (sadece Blast/V-HACD içinde).

**Rust'tan kullanım:**
- Yalnızca C++, C API yok. ovphysx'in C API'si USD'ye ve Carbonite'e bağlı olduğu için bize uymuyor.
- physx-rs / physx-sys (Embark) eski PhysX 4/5.1 dönemine ait, güncelliği doğrulanmadı.
- Gereken iş: Dar bir C++ köprüsü (`cxx`), `cmake` crate ile cpu-only derleme, CUDA opsiyonel feature olarak.
- **Tahmini maliyet:** 4–8 hafta.

**Öneri:**
- PhysX **opsiyonel arka uç** olsun: `physics-physx` feature'ı, GPU için `physx-gpu` alt feature'ı.
  - CPU sürümü: Çok robotlu ve mesh ağırlıklı dünyalarda yüksek doğruluklu artikülasyon.
  - GPU sürümü: **Toplu RL arka ucu** (Isaac Lab benzeri, RTX 4060 ile uyumlu).
- Jolt ve MuJoCo'nun yerini almasın.
- Flow, Blast, deformable ve PBD şimdilik atlanacak.
- **İHA, multikopter ve ArduSub için PhysX'in bir avantajı yok.** Bunlar tek rijit gövde + bizim kuvvet modellerimiz; Rapier yeterli.
- **Riskler:**
  - C++ API'si sık değişiyor.
  - GPU'da ve platformlar arasında determinizm yok.
  - GPU yolu yalnızca NVIDIA'da çalışıyor.
  - Belgelerde çelişkiler var.
