# 04 — Alan Modelleri: Uçak, Multikopter, Su Altı, Batarya, Kara Aracı, ArduPilot Köprüsü

> Kaynak: inceleme ajanı raporu, 23.11.2026.
> **G** = `D:\rust-gazebo-ref\gazebo\gz-sim\src\systems\`, **A** = `D:\rust-gazebo-ref\gazebo\ardupilot_gazebo\`.
> Notasyon: q = ½ρV², S alan, α hücum açısı, β yanal kayma, ω rotor hızı.

**Özet:** Buradaki tüm Gazebo modelleri yarı-durağan, çoğunlukla katsayılarda doğrusal ve durumsuz. Akış belleği, ESC, motorun elektriksel dinamiği ve endüklenmiş akış (inflow) yok.

## 1. Sabit kanat

### lift_drag (`G\lift_drag\LiftDrag.cc`)
- Hava hızı `v = v_link + ω×r_cp − v_rüzgar` (L301-306). Açıklık yönündeki bileşen atılıyor; ok açısı tüm katsayılarda `cos²Λ` çarpanı.
- **CL:** stall'a kadar `cla·α`; stall sonrası `cla_stall` eğimiyle doğrusal (tek kırılma).
- **CD = |cda·α|.** **CD0 yok, indüklenmiş sürükleme (CL²) yok** → α=0'da sürükleme sıfır (L452-455).
- `Cm` aynı şekilde, moment `cm·q·S` — **veter çarpanı yok** (L489).
- Kontrol yüzeyi sadece CL ve Cm'e ekleniyor; CD TODO (L431).
- `cp` CoM'a değil **link orijinine** göre (kaynak yorumu da kabul ediyor, L499-512).
- Her yüzey ayrı eklenti ve kuvvetler toplanıyor. **Downwash yok, girişim yok.**

### Zephyr (`A\models\zephyr_with_ardupilot\model.sdf`)
- 7 LiftDrag: gövde, 2 elevon, 2 dikey kanatçık, **2 pervane palası**.
- **İtki, dönen eklemdeki palaların LiftDrag kaldırmasından** geliyor; itki eğrisi yok.
- Gaz: VELOCITY PID (838·gaz rad/s, P=0.2). Elevon: POSITION PID (P=10).

### advanced_lift_drag (`G\advanced_lift_drag\AdvancedLiftDrag.cc`)
- AVL tarzı katsayı birikimi; stall sigmoid karışımı (M=15, L596-599).
- `CL = (1−σ)(CL0+CLa·α) + σ·2·sgn(α)sin²α·cosα + CLb·β + ΣCL_δ·δ`.
- `CD = (1−σ)(CD0 + CL²/(π·AR·e)) + σ·|CD_fp·(0.5−0.5cos2α)| + ΣCD_δ` (L692-696).
- CY, Cl, Cn: α, β, δ, p̂, q̂, r̂'de doğrusal. Kontrol türevleri **derece başına**.
- **Re/Mach yok, α̇ (dinamik stall) yok, histerezis yok.** Yine tek yüzey.

### wind_effects (`G\wind_effects\WindEffects.cc`)
- Hız: birinci derece filtre + sinüs + Gauss gürültü (L518-568).
- Kuvvet `F = m·k(pos)·(v_rüzgar − v_link)` — **kütleyle orantılı doğrusal sürükleme**, şekilden bağımsız, tork yok (L624-626).
- **Dryden/von Kármán yok, rüzgar kesmesi (shear) yok, ani rüzgar (gust) yok.**

## 2. Multikopter

### multicopter_motor_model (`G\multicopter_motor_model\MulticopterMotorModel.cc`)
- İtki `T = k_T·ω²` (k_T = 8.55e-6); tork `Q = k_M·T` (L565-648).
- Rotor sürüklemesi `F = −|ω|·λ1·V⊥`, yuvarlanma momenti `−|ω|·μ1·V⊥` (Martin & Salaün).
- Motor: komut ω'ya asimetrik birinci derece filtre (τ_yukarı 1/80, τ_aşağı 1/40 s), sonra **eklem hızına kinematik olarak yazılıyor**.
- **İtki hava hızına bağlı değil:** J yok, inflow yok.

### multicopter_control
- Lee geometrik hız kontrolcüsü; `ω×Iω` terimi yorum satırında; dağıtım `ω² = A†·[α; T]`.

### ArduPilot iris
- **Motor modelini kullanmıyor.** 8 LiftDrag pala + ApplyJointForce kullanıyor: endüklenmiş akışsız, kaba tek elemanlı pala teorisi.

## 3. Havadan hafif / uzay
- **LighterThanAirDynamics:** 6×6 eklenmiş kütle + çapraz akış viskoz terimi + eksenel sürükleme.
- **SpacecraftThruster:** PWM aç/kapa, yükselme süresi yok, kütle azalması yok.

## 4. Deniz / su altı

**buoyancy:**
- Uniform mod `F = −ρVg`, **her zaman tüm hacim** (kısmi batma yok, havada da yüzer).
- Graded mod: katman düzlemleriyle gerçek kısmi batma ve doğrultucu moment, ama **sadece temel şekiller** (kutu, küre, silindir, kapsül, elipsoit, koni). **Mesh desteklenmiyor**. Su yüzeyi düz ve sabit, dalga yok.

**buoyancy_engine:** Hacim değişim hızı sınırlı balast; kuvvet orijinde, tork yok.

**hydrodynamics (Fossen):**
- `v_r = Rᵀ(v − v_akıntı)`.
- **Eklenmiş kütle `M_A·v̇_r` ve v̇ adımlar arası sonlu farkla** hesaplanıyor → açık yöntem, kararsız olabilir. Alternatif yol: DART'ta örtük `<fluid_added_mass>`.
- `C_A` yalnızca köşegen terimlerle.
- Sönüm: doğrusal + karesel, tam çapraz bağlı.
- **Yok:** dalga kuvveti, radyasyon/bellek, serbest yüzey, yüzgeç kaldırması.

**Akıntı:** Sabit değer, topic ya da CSV ızgarası (`environment_preload`).

**thruster:**
- `T = ρ·K_T·D⁴·ω|ω|`. Opsiyonel K_T(J), ama J **araç toplam hızından** hesaplanıyor.
- **İtki anında uygulanıyor** (gecikme yok). **K_Q tork katsayısı yok.**
- Sabit `power_load`.

**dvl:** GPU ışın huzmeleri; ışın başına `(v_sensör − v_hedef)·b̂ + N`; SVD en küçük kareler. Akustik multipath yok.

## 5. Batarya (`G\battery_plugin\LinearBatteryPlugin.cc` L731-796)
- `I = ΣP/V`, filtre, `q −= I·dt/3600`, `V = e0 + e1(1 − q/c) − r·I`.
- Doğrusal OCV + tek seri direnç. RC çifti, sıcaklık ve Peukert yok.
- Yükler sabit güç. **Voltaj itkiye ve RPM'e hiç geri beslenmiyor.**

## 6. Kara araçları
Hepsi kinematik hız hedefi → `JointVelocityCmd`:
- diff_drive, ackermann (iç/dış teker açısı), mecanum, swerve, paletli (ICR sürtünme yönü).
- wheel_slip: ODE kayma uyumu, **sabit kullanıcı tanımlı F_N**. Pacejka değil.
- lookup_wheel_slip: RGB harita.

## 7. ArduPilotPlugin (`A\src\ArduPilotPlugin.cc`)

**Giriş (UDP, varsayılan port 9002):**
- Little-endian `{u16 magic; u16 frame_rate; u32 frame_count; u16 pwm[16|32]}`.
- Magic 18458 = 16 kanal, 29569 = 32 kanal.
- Soket boşaltılıyor, en yeni paket kullanılıyor.
- `frame_count` geri giderse reset, aynıysa tekrar, atlarsa log.

**Kanal → eklem eşleme:**
- `cmd = multiplier·(clamp((pwm−min)/(max−min), 0, 1) + offset)`.
- Tipler: VELOCITY / POSITION (PID → kuvvet), EFFORT, COMMAND (gimbal/paraşüt topic).
- **Hata:** `useForce` olmadan EFFORT kullanılırsa null pointer'a erişiyor (L1410-1413).

**Kilitli adım (lockstep):**
- PreUpdate servo paketi gelene kadar bekliyor. **Servo karesi başına tek fizik adımı**, yani ArduPilot döngü hızı = 1/adım.

**Çıkış (JSON, `"\n{...}\n"` çerçeve):**
- `timestamp`.
- `imu.gyro`, `imu.accel_body` (FRD, özgül kuvvet).
- `position` ve `velocity` NED.
- `quaternion` [w,x,y,z] NED→FRD.
- Opsiyonel `rng_1..6`, `windvane`.
- **Gönderilmeyenler:** hava hızı, GPS, manyetometre, baro, batarya. ArduPilot bunları kendisi sentezliyor.

**Diğer eklentiler:**
- GstCamera: H.264 RTP, 127.0.0.1:5600.
- Paraşüt: spawn + DetachableJoint.
- Gimbal: saf pozisyon kontrolü.

## Doğruluk boşlukları

| Alan | Gazebo'da var | Gerçeğe göre eksik |
|---|---|---|
| Kanat aero | Tek kırılmalı doğrusal / AVL + sigmoid | Temel modelde CD0 + indüklenmiş sürükleme; Cm veter; dinamik stall/histerezis; α̇, downwash gecikmesi; Re/Mach; yüzey CD/menteşe momenti; pervane rüzgarı (prop-wash); yer etkisi; spin |
| Bileşim | Bağımsız yüzeyler toplanıyor | Kanat–kuyruk downwash, gövde girişimi, CoM'a göre cp |
| Atmosfer | Filtre + sinüs + beyaz gürültü, kütle orantılı | **Dryden/von Kármán**, shear, gust, termik, ISA ρ(h), uzaysal alan, p/q/r türbülansı |
| Rotor | kω², k_M·T, doğrusal H-kuvveti | **CT(J)/CQ(J), VRS, yer etkisi**, translational lift, flapping, jiroskopik etki, pervaneler arası girişim |
| ESC/motor | ω filtresi, kinematik | **ESC dinamiği, BLDC elektrik modeli (Kv, R, back-EMF), voltaja bağlı max RPM**, ataletle hızlanma |
| Batarya | Doğrusal OCV + R | **Voltaj düşüşünün itkiye bağlanması**, RC, doğrusal olmayan OCV(SoC), sıcaklık, motordan akım |
| Deniz | Temel şekilde kısmi batma, Fossen | **Dalga kuvvetleri**, mesh kısmi batma, radyasyon, deniz durumu spektrumu, örtük eklenmiş kütle, itici–gövde etkileşimi |
| İtici | Anlık itki, |V| ile K_T(J) | Eksenel J, K_Q, 4 kadran, yüzeye yakın havalanma (ventilation), gecikme, voltaj |
| Kara | Kinematik, sabit F_N | Pacejka/fırça lastik, yük transferi, Bekker zemin mekaniği, tork limitli motor |
| Sensör/bağlantı | Ham IMU, NED | Rotor titreşimi, pitot, gecikme/jitter enjeksiyonu |

## Önce daha iyisini yapacaklarımız (öncelik sırası)
1. **Rotor/pervane modeli:**
   - CT(J), CQ(J) tabloları veya momentum inflow'lu pala teorisi (BEMT).
   - VRS, yer etkisi (Cheeseman–Bennett), H-kuvveti/flapping.
   - **Tek model** multikopter, sabit kanat itici pervanesi ve su altı iticisi için kullanılır (ρ değişir).
2. **Elektrikli güç aktarımı:**
   - Zincir: ESC (PWM→duty, slew) → BLDC (Kv, R, back-EMF, atalet) → batarya (OCV(SoC) tablosu + R0 + RC), kapalı döngü akım.
   - Kazanç: Voltaj düşüşü ve itki kaybı modellenir. ArduPilot failsafe ve ayar gerçekçiliği için en kritik madde bu.
3. **Sabit kanat aero:**
   - CL/CD/Cm(α, β, δ) tabloları, CD0 + CL²/πeAR, veterle ölçeklenen Cm.
   - α̇ / dinamik stall (Goman–Khrabrov), gecikmeli downwash ε(α), kuyrukta prop-wash, yer etkisi.
   - cp CoM'a göre alınır.
   - **Bertaraf'ın AeroSandbox çıktısı doğrudan bu tablolara girer.**
4. **Atmosfer:** ISA, Dryden (MIL-F-8785C) türbülansı p/q/r bileşenleriyle birlikte, shear, gust. Rüzgar, kütle orantılı kuvvet olarak değil havaya göre hız üzerinden uygulanır.
5. **Deniz:**
   - Örtük eklenmiş kütle ve tam C_A.
   - Mesh üçgenlerini dalga yüzeyiyle kırparak kısmi batma.
   - JONSWAP/PM spektrumundan Froude–Krylov dalga kuvvetleri.
   - Derinlik/basınç alanı.
   - İtici için eksenel J ile K_T/K_Q ve ventilation (ArduSub, Siren).
6. **ArduPilot JSON köprüsü:**
   - Aynı sözleşme birebir: magic, `frame_count` anlamı, `\n` çerçeve, NED/FRD quaternion, kare başına tek adım.
   - EFFORT hatası düzeltilir.
   - Opsiyonel olarak hava hızı ve IMU titreşimi eklenir.
7. **Kara araçları:** Gerçek normal yükle fırça/Pacejka lastik modeli ve tork limitli motor.
