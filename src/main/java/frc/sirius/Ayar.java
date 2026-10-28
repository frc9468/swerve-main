package frc.sirius;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Canlı ayarlar: robot açıkken Elastic / Shuffleboard'daki "Preferences" tablosundan
 * değiştirilir, yeniden deploy GEREKMEZ, roboRIO'da kalıcı saklanır.
 * İyi değeri bulunca istersen buradaki varsayılanı da güncelle.
 */
public final class Ayar {
  private Ayar() {}

  /** Kumanda eğrisi üssü: 1 = düz (sert), 2 = kare (yumuşak başlangıç), 3 = küp (çok yumuşak). */
  public static double egri() { return al("Surus/Egri", 2.0); }
  /** Dönüş eğrisi üssü (küçük dönüşler için ayrı). */
  public static double donusEgri() { return al("Surus/DonusEgri", 2.0); }
  /** Normal kipte tam çubuk hız çarpanı (0-1). Sağ tetik TURBO = 1.0. */
  public static double hizCarpan() { return al("Surus/HizCarpan", 0.75); }
  /** Tam çubukta dönüş hızı (rad/s). 2π = saniyede 1 tur. */
  public static double donusMax() { return al("Surus/DonusMaxRadS", 2.0 * Math.PI); }
  /** İvme sınırı (m/s²): çubuğu bir anda itince robotun sıçramasını önler. */
  public static double ivme() { return al("Surus/IvmeMps2", 5.0); }
  /** Dönüş ivme sınırı (rad/s²). */
  public static double donusIvme() { return al("Surus/DonusIvmeRadS2", 14.0); }
  /** Statik sürtünme gerilimi (V): küçük komutta tekerin hiç dönmemesini önler. */
  public static double kS() { return al("Surus/kS_Volt", 0.25); }
  /** Hız başına gerilim (V / (m/s)). 10-25 havada ölçüldü: 2.9 ile 0,4 istendi 0,48 geldi → 2.45. */
  public static double kV() { return al("Surus/kV", 2.45); }

  /** Saha yönlü sürüş (true) ya da robot yönlü (false, sahip 10-25 varsayılanı: ileri = robotun önü). */
  public static boolean sahaYonlu() {
    Preferences.initBoolean("Surus/SahaYonlu", false);
    return Preferences.getBoolean("Surus/SahaYonlu", false);
  }

  /** Kumandada Back (⧉): saha yönlü ↔ robot yönlü, kalıcı. */
  public static void sahaYonluDegistir() {
    boolean yeni = !sahaYonlu();
    Preferences.setBoolean("Surus/SahaYonlu", yeni);
    System.out.println("KIP " + (yeni ? "SAHA yönlü" : "ROBOT yönlü"));
  }

  /** Yön tutma (sağ çubuk bırakılınca robot yönünü korur). */
  public static boolean yonTut() {
    Preferences.initBoolean("Surus/YonTut", true);
    return Preferences.getBoolean("Surus/YonTut", true);
  }

  /** Yön tutma sertliği (rad/s / rad). Salınıyorsa düşür, geç toparlıyorsa artır. */
  public static double yonTutP() { return al("Surus/YonTutP", 4.0); }

  /** roboRIO'da kalmış eski varsayılanları yenisine taşır (Preferences kalıcıdır, init üzerine yazmaz). */
  public static void eskiyiTasi() {
    if (Preferences.containsKey("Surus/kV") && Preferences.getDouble("Surus/kV", 0) == 2.9) {
      Preferences.setDouble("Surus/kV", 2.45);
    }
    if (!Preferences.containsKey("Surus/RobotYonluGecti")) { // 10-25: bir kez robot yönlüye geç
      Preferences.setBoolean("Surus/SahaYonlu", false);
      Preferences.setBoolean("Surus/RobotYonluGecti", true);
    }
    tasi("Surus/HizCarpan", 0.5, 0.75);
    tasi("Surus/IvmeMps2", 6.0, 5.0);
    tasi("Surus/IvmeMps2", 4.0, 6.0);
    tasi("Surus/DonusIvmeRadS2", 10.0, 14.0);
    tasi("Surus/DonusMaxRadS", 1.5 * Math.PI, 2.0 * Math.PI);
  }

  private static void tasi(String ad, double eski, double yeni) {
    if (Preferences.containsKey(ad) && Preferences.getDouble(ad, 0) == eski) {
      Preferences.setDouble(ad, yeni);
    }
  }

  private static double al(String ad, double varsayilan) {
    Preferences.initDouble(ad, varsayilan);
    return Preferences.getDouble(ad, varsayilan);
  }
}
