# siriusrobotics · Swerve (WPILib 2026, Java, komut tabanlı)

4 adet **MK4i L2** modül, sürüş + dönüş **NEO / REV Spark MAX**, mutlak enkoder **CANcoder**, gyro **Pigeon 2**.
Xbox kumandayla **alan merkezli** sürüş.

| Kumanda | Görev |
|---|---|
| Sol çubuk | Hareket (ileri/geri, sağ/sol) |
| Sağ çubuk (yatay) | Dönüş |
| Start | Gyro sıfırla (robot o an nereye bakıyorsa orası "ileri") |
| Sol tampon (LB) basılı | Yavaş mod (%30) |

Ölü bölge 0.08. Kumanda Driver Station'da USB **0** numarada olmalı.

Derleme durumu: bu PC'de `gradlew build` → **BUILD SUCCESSFUL** (WPILib 2026.1.1, REVLib 2026.0.5, Phoenix6 26.3.0; vendordep json'ları `vendordeps/` içinde).

---

## 1. Doldurman gerekenler

Hepsi tek yerde: `src/main/java/frc/sirius/Constants.java` ("DOLDUR" yazan satırlar). Takım numarası ayrı dosyada.

| # | Ne | Nerede | Varsayılan |
|---|---|---|---|
| 1 | **Takım numarası** | `.wpilib/wpilib_preferences.json` → `"teamNumber"` | -1 (yazılmazsa deploy olmaz) |
| 2 | Gyro türü | Pigeon2 varsayılan. NavX ise aşağıdaki "NavX" bölümü | Pigeon2 |
| 3 | Pigeon2 CAN ID | `Gyro.PIGEON_CAN_ID` | 13 |
| 4 | CAN hattı | `Gyro.CAN_BUS`, `CANCODER_BUS` (CANivore varsa adı) | "rio" |
| 5 | Mutlak enkoder türü | CANcoder varsayılan. Thrifty ise aşağıdaki bölüm | CANcoder |
| 6 | Spark MAX + CANcoder ID'leri | `FRONT_LEFT … BACK_RIGHT` (sürüş, dönüş, CANcoder) | Ön Sol 1/2/3 · Ön Sağ 4/5/6 · Arka Sol 7/8/9 · Arka Sağ 10/11/12 |
| 7 | Teker merkezleri arası mesafe | `Chassis.TRACK_WIDTH_M`, `WHEEL_BASE_M` (metre) | 0.6 × 0.6 |
| 8 | Enkoder ofsetleri | `FRONT_LEFT … BACK_RIGHT` son sayı (derece) | 0.0 (ayarlanmadan robot düz gitmez!) |
| 9 | Sürüş yönü | `ModuleHw.DRIVE_INVERTED` | false |
| 10 | Gyro yönü | `Gyro.INVERT` | false |

Spark MAX ID'lerini **REV Hardware Client**, CANcoder/Pigeon ID'lerini **Phoenix Tuner X** ile ver/oku. Aynı ID iki cihazda olmasın (Spark MAX'lar ile CTRE cihazları farklı üretici olduğu için aynı numara çakışmaz, ama karışıklık olmasın diye ayrı tuttuk).

## 2. İlk açılış kontrolü (tekerler YERDEN KESİK, robot sehpada)

1. Robotu aç, Driver Station bağlan, **SmartDashboard / Elastic** aç (WPILib araçlarında).
2. Her modül için `OnSol/…`, `OnSag/…`, `ArkaSol/…`, `ArkaSag/…` değerleri görünmeli. Görünmüyorsa o modülün CAN ID'si yanlış.
3. `Gyro derece`: robotu elle **sola** (yukarıdan bakınca saat yönünün tersine) çevir → değer **artmalı**. Azalıyorsa `Gyro.INVERT = true`.

## 3. Enkoder ofseti ayarı (en önemli adım)

1. Ofsetler 0.0 iken kodu deploy et, robotu **Disabled** bırak.
2. Dört tekeri elle **tam düz ileri** çevir. Bir cetvel/alüminyum profili tekerlerin yanına dayayarak hizala. Bütün modüllerde dişli (bevel) **aynı tarafa** (örneğin hepsi robotun sol tarafına) baksın; yoksa o teker ters döner.
3. SmartDashboard'da her modülün **`Ham CANcoder derece`** değerini oku.
4. Bu sayıyı `Constants.java` içinde o modülün son alanına yaz. Örnek: Ön Sol 37.6 gösteriyorsa
   `new ModuleConfig("OnSol", 1, 2, 3, 37.6)`.
5. Yeniden deploy et. Tekerler düzken `Mutlak derece` ≈ 0 olmalı (±1°).
6. Teleop'ta sol çubuğu hafifçe ileri it: dört teker de ileri dönmeli.
   - Bir teker **geri** dönüyorsa: o modülün ofsetine **180 ekle** (ya da çıkar).
   - Hepsi geri dönüyorsa: `ModuleHw.DRIVE_INVERTED = true`.
   - Tekerler hedef açıya gitmiyor, sürekli dönüyorsa/titreşiyorsa: `STEER_INVERTED` değerini değiştir; titreme sürerse `STEER_P` değerini düşür (1.0 → 0.5).
7. Çubukla sağa-sola gidince robot dönmemeli. Sağ çubukla dönüp sonra sol çubuğu ileri itince robot **sahaya göre** ileri gitmeli (robotun baktığı yöne göre değil). Değilse gyro yönünü (`Gyro.INVERT`) kontrol et.

## 4. Deploy (robota yükleme)

1. Bilgisayara **WPILib 2026** kurulu olmalı (VS Code içinde gelir). Bu projeyi **WPILib VS Code** ile aç (`File > Open Folder > sirius-swerve`).
2. `.wpilib/wpilib_preferences.json` içine takım numaranı yaz (`"teamNumber": 1234` gibi).
3. Robota bağlan: USB-B kablo, Ethernet ya da robotun Wi-Fi'ı.
4. VS Code'da sağ üstteki **WPILib simgesi → "Deploy Robot Code"** (ya da `Shift+F5`).
   Komut satırından: `gradlew deploy` (Windows: `.\gradlew.bat deploy`).
5. "BUILD SUCCESSFUL" ve Driver Station'da **Robot Code** yeşil olmalı.
6. İlk sürüşte Driver Station'da hız düşük olsun diye **sol tamponu basılı tut** (yavaş mod).

İlk deploy internet ister (vendordep kütüphanelerini indirir). Yarışma alanında internet yoksa evde bir kez `gradlew build` çalıştırman yeter, kütüphaneler önbelleğe iner.

## 5. Gyro NavX ise

1. VS Code: `WPILib: Manage Vendor Libraries → Install new library (online)` → Studica NavX json adresi (Studica sitesinden 2026).
2. `SwerveSubsystem.java` içinde yalnız gyro ile ilgili satırları değiştir:
   - `Pigeon2 gyro = …` yerine `AHRS gyro = new AHRS(NavXComType.kMXP_SPI);`
   - `getYaw()` içi: `return Rotation2d.fromDegrees(-gyro.getAngle());` (NavX saat yönünde artar, eksi işaret bunu düzeltir)
   - `gyro.setYaw(0)` yerine `gyro.reset()`.

## 6. Mutlak enkoder Thrifty (analog) ise

`SwerveModule.java` içinde CANcoder yerine `AnalogEncoder` kullan (roboRIO analog portu, ID yerine port numarası):
`getRawAbsoluteRad()` → `return encoder.get() * 2 * Math.PI;` Ofset ayarı yukarıdaki adımlarla aynıdır.

## 7. Dosya haritası

```
src/main/java/frc/sirius/
  Main.java               başlangıç (dokunma)
  Robot.java              TimedRobot, komut zamanlayıcı
  RobotContainer.java     kumanda tuşları
  Constants.java          TÜM ayarlar (DOLDUR satırları)
  commands/TeleopSwerve.java     çubuk → hız (ölü bölge, yavaş mod)
  subsystems/SwerveSubsystem.java  4 modül + gyro + odometri + panel değerleri
  subsystems/SwerveModule.java     tek modül: NEO sürüş, NEO dönüş (Spark MAX içi PID), CANcoder
vendordeps/  REVLib.json, Phoenix6-frc2026-latest.json, WPILibNewCommands.json
```

Notlar:
- Sürüş açık çevrim (gerilim = hız / 4.5 m/s × 12 V): ilk gün için en güvenli ve ayarsız yol. Otonom gerekirse sonra hız PID'i eklenir.
- Dönüş motoru Spark MAX içinde PID ile çalışır; her Teleop başında dönüş enkoderi CANcoder'dan yeniden alınır.
- MK4i L2 sabitleri (robottaki eski koddan): teker 0,093 m, sürüş oranı 6.75:1, dönüş oranı 150/7:1, şasi 0,54 × 0,54 m, Pigeon ID 20.

## Yumuşak sürüş ve canlı ayar (10-24)
İlk sürüm çok sertti: çubuk doğrusaldı, küçük komutta teker sürtünmeyi yenemiyordu, sonra birden sıçrıyordu. Şimdi:
- **Eğri:** küçük itiş = çok küçük hız (kare eğri), tam itiş = tam hız.
- **İvme sınırı:** çubuğu bir anda itince robot sıçramaz.
- **kS ileri besleme:** küçük dönüş/sürüş komutu da tekeri gerçekten döndürür.

Robot AÇIKKEN, deploy etmeden ayarlanır: Elastic/Shuffleboard → **Preferences** tablosu → `Surus/...`:
| Ayar | Varsayılan | Ne zaman değiştir |
|---|---|---|
| Surus/HizCarpan | 0.5 | İlk denemede 0.5; rahatsa 0.7-0.8 |
| Surus/Egri | 2.0 | Hâlâ sertse 2.5–3, tepkisizse 1.5 |
| Surus/DonusEgri | 2.0 | Küçük dönüşler için aynı mantık |
| Surus/DonusMaxRadS | 4.71 | Dönüş çok hızlıysa 3.14 |
| Surus/IvmeMps2 | 4.0 | Sıçrıyorsa 3, ağırsa 6 |
| Surus/DonusIvmeRadS2 | 10.0 | Dönüş sertse 6 |
| Surus/kS_Volt | 0.25 | Tekerler havadayken: komut var ama teker dönmüyorsa artır (0.05 adım), titriyorsa azalt |
| Surus/kV | 2.45 | Tam çubukta hız ölçümü (Hiz mps) hedefin altındaysa artır |
| Surus/SahaYonlu | true | Robot dönerken sürüş yönü kayıyorsa önce false yap (robot yönlü). Düzelirse gyro ters: Constants Gyro.INVERT = true |

**Havada ayar sırası:** robotu takoza koy → ofsetleri doğrula (tekerler düz) → kS'yi tekerin zar zor döndüğü değere getir → yere indir, HizCarpan 0.5 ile sür → Egri/İvme ile his ayarla.

## Kumanda (10-25)
| Tuş | İş |
|---|---|
| Sol çubuk | Hareket (robot yönlü: ileri = robotun önü; Surus/SahaYonlu=true ile saha yönlü) |
| Sağ çubuk | Dönüş; bırakınca robot yönünü kendisi korur (Surus/YonTut) |
| Sağ tetik | TURBO: tam hız (4,5 m/s) |
| Sol tampon | Bas-bırak: yavaş kip AÇ/KAPA (hassas yanaşma) |
| Back (⧉) | Saha yönlü ↔ robot yönlü (kalıcı) |
| X (basılı) | Tekerler çapraz kilit: itilince kaymaz |
| Start | Gyro sıfırla (robot nereye bakıyorsa "ileri") |

Autonomous = kumandasız deneme sürüşü (robot HAVADA): ileri, sola, dönüş, geri; her adım FRC_UserProgram.log'a yazılır.
Yeni ayarlar: Surus/YonTut (true), Surus/YonTutP (4.0; salınıyorsa düşür).

## Ofset alma (10-25)
1. Robot DISABLE. Dört teker düz ileri, bütün dişliler (bevel) robotun SAĞINA baksın (cetvelle kontrol).
   İstersen önce Test kipinde Enable: tekerler kendiliğinden 0°'ye döner, sonra Disable.
2. roboRIO'da `touch /home/lvuser/ofset_al` → kayıtta "OFSET KAYDEDILDI" satırı; değerler Preferences "Ofset/<ad>"da kalıcıdır.

Panoda "Surus/Kip" (ROBOT/SAHA · YAVAŞ/TURBO) ve "Surus/Aku V". Akü 9 V altına inince hız otomatik %60'a kısılır (brownout koruması).
