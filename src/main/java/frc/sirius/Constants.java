package frc.sirius;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * ============================================================
 *  SIRIUS ROBOTICS · TÜM AYARLAR TEK YERDE
 *  "DOLDUR" yazan her satırı robota göre kontrol et / değiştir.
 *  (Takım numarası burada değil: .wpilib/wpilib_preferences.json → "teamNumber")
 * ============================================================
 */
public final class Constants {
  private Constants() {}

  // ---------------- Kumanda ----------------
  public static final class OI {
    public static final int DRIVER_PORT = 0;           // Driver Station'da USB sırası 0
    public static final double DEADBAND = 0.08;        // ölü bölge
    public static final double SLOW_MODE_SCALE = 0.30; // sol tampon basılıyken hız çarpanı
  }

  // ---------------- Gyro ----------------
  public static final class Gyro {
    // DOLDUR: Varsayılan Pigeon 2.0 (CTRE). NavX kullanıyorsan README "NavX" bölümüne bak.
    public static final int PIGEON_CAN_ID = 20;        // robotta
    public static final String CAN_BUS = "rio";        // CANivore yoksa "rio" kalsın
    public static final boolean INVERT = false;        // Robot sola dönerken açı ARTMIYORSA true yap
  }

  // ---------------- Modül donanımı (MK4i L2, robottaki eski koddan) ----------------
  public static final class ModuleHw {
    public static final double WHEEL_DIAMETER_M = 0.093; // eski kodda ölçülmüş (aşınmış 4" teker)
    public static final double DRIVE_RATIO = 6.75;      // L2: 6.75:1
    public static final double STEER_RATIO = 150.0 / 7.0; // MK4i dönüş: 21.43:1

    public static final boolean DRIVE_INVERTED = false;
    public static final boolean STEER_INVERTED = true;  // MK4i'de dönüş motoru ters (standart)

    public static final int DRIVE_CURRENT_LIMIT_A = 45; // 10-25: 6 brownout → 50'den indirildi
    public static final int STEER_CURRENT_LIMIT_A = 20;

    // Dönüş PID (Spark MAX içinde, birim: radyan). Titriyorsa P'yi düşür.
    public static final double STEER_P = 1.0;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.0;

    // Havada ölçülen kV 2.45 → serbest hız ~4,8 m/s. Tavan 4,5 (yön düzeltmesine pay kalsın).
    public static final double MAX_SPEED_MPS = 4.5;
    public static final double MAX_ANGULAR_RAD_PER_S = 2 * Math.PI; // 1 tur/sn
  }

  // ---------------- Şasi ölçüsü ----------------
  // 10-25 yerde görüldü: modül adlarının "ön"ü robotun gerçek ARKASI. Hareket komutları 180° çevrilir
  // (dönüş etkilenmez). Sürüş motorunu ters çevirmek YANLIŞ çözümdü: ileri-geriyi düzeltip dönüşü bozdu.
  public static final class Chassis {
    // DOLDUR: teker MERKEZLERİ arası mesafe (metre). Varsayılan 0.6 x 0.6.
    public static final boolean ON_TERS = false; // 10-25 yerde: false doğru (ileri = robotun önü, dönüş doğru)
    public static final double TRACK_WIDTH_M = 0.54; // sol teker ↔ sağ teker (robotta ölçüldü)
    public static final double WHEEL_BASE_M = 0.54;  // ön teker ↔ arka teker (robotta ölçüldü)

    // Sıra: ÖN SOL, ÖN SAĞ, ARKA SOL, ARKA SAĞ (+x ileri, +y sol)
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
        new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
        new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
        new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2));
  }

  // ---------------- Modüller: CAN ID + ofset ----------------
  /** Mutlak enkoder: varsayılan CTRE CANcoder. Thrifty kullanıyorsan README'ye bak. */
  public static final String CANCODER_BUS = "rio";

  /**
   * DOLDUR: her modül için (sürüş Spark MAX ID, dönüş Spark MAX ID, CANcoder ID, ofset derece).
   * Ofset: tekerler düz ileri bakarken SmartDashboard'daki "Ham CANcoder" değeri (README adım 3).
   */
  // ID'ler robotta doğrulandı. Ofsetler 10-25 elle hizalanarak alındı: düz ileri, DİŞLİLER SAĞDA.
  // Asıl değer roboRIO'daki Preferences "Ofset/<ad>"; yeniden almak için README "Ofset alma".
  public static final ModuleConfig FRONT_LEFT  = new ModuleConfig("OnSol",   12,  6, 2, -144.14);
  public static final ModuleConfig FRONT_RIGHT = new ModuleConfig("OnSag",   55, 15, 5, -36.21);
  public static final ModuleConfig BACK_LEFT   = new ModuleConfig("ArkaSol", 11, 13, 4, 74.36);
  public static final ModuleConfig BACK_RIGHT  = new ModuleConfig("ArkaSag", 16, 17, 3, -129.29);

  public record ModuleConfig(String name, int driveId, int steerId, int cancoderId,
      double offsetDeg) {}
}
