# 01 — Çekirdek Mimari (gz-sim 11.0.0-pre3, gz-plugin, gz-transport, gz-msgs, sdformat)

> Kaynak: inceleme ajanı raporu, 23.11.2026. Yollar `D:\rust-gazebo-ref\gazebo\` altına göredir.

## 1. ECS
- **Entity** = `uint64_t`, `kNullEntity=0` (`gz-sim/include/gz/sim/Entity.hh:58-61`). Gömülü **EnTT** 64-bit ID + 8-bit nesil için yamalı (`gz-sim/include/gz/sim/detail/vendor/entt/GAZEBO_MODIFIED_CODE.patch:9-28`). `SetEntityCreateOffset` ile GUI (MAX_I32/2) ve log oynatma (MAX_I64/2) ID uzayı ayrılıyor.
- **ECM** = `entt::basic_registry` sarmalayıcısı (`gz-sim/src/EntityComponentManager.cc:113-240`). Hiyerarşi `ParentEntity`/`Children` bileşenleri + EnTT kancaları (`:58-87`). `Clone` elle düzeltme + TODO (`:185-228`).
- **Bileşen tipi** `Component<DataType, Tag, Serializer>` (`components/Component.hh:352`); tip ID derleme zamanında `hash64("gz_sim_components.Pose")` (`components/Factory.hh:478-490`). Her DLL statik nesneyle singleton Factory'ye kayıt olur (`Factory.hh:500-522`). ~127 bileşen başlığı.
- **Serileştirme** iostream `<<`/`>>` → protobuf bytes (`Component.hh:111-200`, `EntityComponentManager.cc:1276-1309`). Yavaş ve float'ı metne çevirdiği için kayıplı.
- **Değişim takibi:** `NoChange` / `PeriodicChange` (kaybolabilir) / `OneTimeChange` (mutlaka iletilir) (`Types.hh:65-78`), varlık başına `unordered_set` (`EntityComponentManager.cc:89-111`). **Tuzak:** `SetComponentData` değişti işaretlemez; elle `SetChanged` gerekir (`detail/EntityComponentManager.hh:128-139`).
- **View önbelleği kaldırıldı**, EnTT view/group'a geçildi (`detail/EntityComponentManager.hh:284-337`).
- **v11 kırılmaları** (`Migration.md:227-245`): `Each` sırası artık sırasız (determinizm kaybı), bileşen pointer'ları sarkabilir.
- **Performans sorunları:** varlık başına hash set'ler; her durum yayınında iostream; `ResetTo` tüm depoyu yeniden kuruyor (`:2082-2092`); özyinelemeli silme tüm DetachableJoint'leri tarıyor (`:650-697`).

## 2. Sistem (eklenti) arayüzü
- Ayrı mix-in arayüzler (`include/gz/sim/System.hh`): `ISystemConfigure` (:137), `ConfigurePriority` (:161), `ConfigureParameters` (:175), `Reset` (:187), `PreUpdate`/`Update` (ECM yazılabilir, :195-207), `PostUpdate` (`const ECM&`, :211).
- **Faz sözleşmesi** (:50-83): PreUpdate = komut/kontrol, Update = fizik, PostUpdate = salt okuma (sensör, yayıncı).
- **Öncelik:** işaretli int, küçük önce; eşitlikte yükleme sırası; `<gz:system_priority>` XML ile ezilir (`src/SystemManager.cc:111-131`). Sabitler: UserCommands -16384, PrePhysics -128, Physics -64, PostPhysicsSensor -32 (`System.hh:114-128`).
- **İş parçacıkları:** Pre/Update sıralı; **PostUpdate her sistem için ayrı OS thread + bariyer** (`src/SimulationRunner.cc:652-773`); sistem eklenince tüm thread seti yeniden kuruluyor. "Salt okuma" yalnızca `const` ile; `mutable` ile deliniyor.

## 3. Simülasyon döngüsü
- `Server` → her `<world>` için bir `SimulationRunner` (`src/Server.cc:191-199`); çok dünya 2 thread'lik havuzu paylaşır.
- Model/ışıklar arka planda indirilen varlıklar bitince oluşturuluyor → **ilk adımlarda hangi varlıkların olduğu belirsiz** (`src/ServerPrivate.cc:769-856`).
- `updatePeriod = step / RTF` (RTF=0 → olabildiğince hızlı) (`SimulationRunner.cc:175-216`); Windows'ta waitable timer ile uyku+spin (`:1007-1083`).
- `Step()` sırası (`:1098-1181`): GUI durumu → istatistik → level → bekleyen sistemler → silme/oluşturma → **Pre/Update/PostUpdate** → çoklu adım → `ProcessMessages` → bayrak temizliği.
- **Reset/geri sarma** = başlangıç ECM anlık görüntüsüne dön + `ISystemReset`; Reset'i olmayan sistemler yeniden yükleniyor. Model bazlı reset yok.
- **Dağıtık simülasyon:** primary/secondary kilitli adım, 10 s zaman aşımı (`src/network/NetworkManagerPrimary.cc:115-188`); en yavaş düğüm herkesi bekletiyor. Deneysel, az kullanılıyor.
- **Determinizm sorunları:** global `math::Rand` tohumu, sırasız `Each`, asenkron PostUpdate yayınları, duvar saatine bağlı varlık oluşturma, transport geri çağrılarının rastgele adımda komut sokması.

## 4. Eklenti yükleme (gz-plugin)
- Arama: `GZ_SIM_SYSTEM_PLUGIN_PATH` → config → `~/.gz/sim/plugins` → kurulum (`src/SystemLoader.cc:48-63`). `dlopen`; Windows'ta dlfcn-win32 (`gz-plugin/loader/src/Loader.cc:440-449`).
- ABI: tek `extern "C" GzPluginHook` ama `Info` yapısı `std::string/std::function/unordered_map` içeriyor → **gerçekte "aynı derleyici + aynı STL" şartı** (`gz-plugin/core/include/gz/plugin/Info.hh`).
- `python_system_loader`: pybind11 gömülü, `get_system()` ile duck-typing, çağrı başına GIL (`systems/python_system_loader/PythonSystemLoader.cc:58-229`).

## 5. SDFormat
- 1.0–1.12 şema sürümleri + dönüştürücü (`sdformat/src/Converter.cc`). Nested model, `<include>` + merge, ~20 sensör tipi, eklenti XML'i serbest.
- **Çerçeve semantiği** (1.7+): `relative_to` ve `attached_to` iki çizgesi (`sdformat/src/FrameSemantics.cc:969-1830`).
- URDF otomatik algılanır, sabit eklemler ebeveyne **sessizce birleştirilir** (`parser_urdf.cc:63,410`).
- Sıkıntılar: iki paralel API (DOM + ham Element), eklenti parametreleri tipsiz XML, `__default__` sentinel'leri, ~3000 satır parser.

## 6. Transport + mesajlar
- Konu öneki `@partition@/topic`; ZeroMQ PUB/SUB + ROUTER; özel UDP multicast keşif 239.255.0.7:10317/10318 (Windows ağlarında sık engellenir).
- **Zenoh arka ucu eklenmiş**: `GZ_TRANSPORT_IMPLEMENTATION=zenoh`, SHM sıfır kopya (`gz-transport/src/NodeShared.cc:307-345`, `Node.cc:436-466`). Gazebo da Zenoh'a gidiyor.
- Süreç içi yayın bile protobuf kopyalayıp asenkron kuyruğa atıyor (`Node.cc:476-555`).
- ~216 `.proto`; kilit mesaj `SerializedStateMap`.
- **GUI kendi ayna ECM'ini tutuyor**: bağlanınca tam durum, sonra `/world/<w>/state` diff'leri; SceneBroadcaster yeni varlık/one-time değişimde tam, diğerinde kısılmış diff yayınlar (`systems/scene_broadcaster/SceneBroadcaster.cc:391-453`).
- `websocket_server`: tarayıcı köprüsü, CSV çerçeve `op,topic,type,payload`.

## 7. Olaylar, kullanıcı komutları, log
- EventManager: süreç içi senkron tipli sinyal.
- UserCommands servisleri: create/remove/set_pose/light/physics/... Yanıt "kuyruğa alındı" demek, "yapıldı" değil (`systems/user_commands/UserCommands.cc:735-815`).
- **"Cmd bileşeni" deseni**: `WorldPoseCmd` yazılır, fizik tüketip siler — her yerde kullanılıyor.
- Kayıt: diff'ler SQLite `state.tlog`'a, **ara anahtar kare yok** → geri sarmak baştan oynatmak demek (`systems/log/LogPlayback.cc:483-503`).

## Rust tasarımı için dersler
1. Üç fazlı sözleşmeyi koru, **tiplerle zorla**: Pre/Update `&mut World`, PostUpdate `&World` → rayon ile gerçekten paralel.
2. Tek tamsayı öncelik yerine **okuma/yazma kümesi bildiren zamanlayıcı** + adlandırılmış çapalar (UserCommands < PrePhysics < Physics < Sensors).
3. **Otomatik değişim takibi** (`Mut<T>` tick); OneTime/Periodic bileşen başına *politika*.
4. **Baştan determinizm:** kararlı iterasyon sırası, dünya/sistem başına tohumlu RNG, dış komutlar yalnızca adım sınırında, tamsayı ns simülasyon zamanı.
5. Cmd-bileşen desenini koru ama komutlara ID ver, **"tamamlandı" onayı** döndür.
6. Sahte C ABI yok: statik bağlama (Cargo feature + `inventory`), `abi_stable`/C-ABI vtable, ya da WASM/Python sınırı. Sabit string ID'ler.
7. Bileşen kaydı: sabit isim → 64-bit hash, serde kodek zorunlu, **ikili serileştirme**.
8. **Ucuz anlık görüntü** (reset, seek, kayıt, dağıtık senkron tek mekanizma); kayıt = anahtar kare + diff.
9. Dünya başına runner + başsız `step(n)` API; **ArduPilot/PX4 kilitli adım birinci sınıf mod**.
10. SDF/URDF **içe aktarma formatı**; kendi tipli şemamız (RON/TOML). Çerçeve çizgeleri yükleme anında bir kez çözülür; eklenti parametreleri tipli.
11. Transport: **Zenoh doğrudan**; özel multicast keşif yok; `/world/<w>/...` konu düzeni kalsın.
12. GUI: aynı anlık görüntü/diff akışından ayna; WebSocket köprüsü transport katmanında.
13. Kaçın: ilk adımları değiştiren arka plan indirme, sarkan pointer'lar, sistem eklenince thread yeniden kurma, zorlanmayan salt okuma, asenkron süreç içi yayın.
14. Level/dağıtık simülasyonu **ertele**.
