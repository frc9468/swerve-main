package frc.sirius.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class UpperMechanismSubsystem extends SubsystemBase {
  private static final int INTAKE_LEFT_ID = 8;    // alma motoru 1 (lider)
  private static final int INTAKE_RIGHT_ID = 31;  // alma motoru 2 (8'i takip eder)
  private static final int SHOOTER_ID = 2;        // atma motoru (lider)
  private static final int SHOOTER2_ID = 3;       // atma motoru 2 (2'yi takip eder)

  private static final double INTAKE_SPEED = -0.5;
  private static final double FEED_SPEED = -0.5;
  private static final double SHOOTER_MANUAL_SPEED = -0.5; // B tuşu

  private static final double SHOOTER_DIR = -1.0;

  private static final double SHOOTER_KV = 12.0 / 5676.0;
  private static final double SHOOTER_KP = 0.002;
  private static final double READY_ALT_RPM = 15;   // hedefin en fazla 15 RPM altında hazır say
  private static final double READY_UST_RPM = 100;  // hedefi çok aşarsa hazır sayma

  // Kalibrasyon bitince true yap: Y'ye basınca RPM tablodan (vision mesafesiyle) gelir.
  private static final boolean TABLO_VARSAYILAN = false;

  private final SparkMax intakeLeft = new SparkMax(INTAKE_LEFT_ID, MotorType.kBrushless);
  private final SparkMax intakeRight = new SparkMax(INTAKE_RIGHT_ID, MotorType.kBrushless);
  private final SparkMax shooter = new SparkMax(SHOOTER_ID, MotorType.kBrushless);
  private final SparkMax shooter2 = new SparkMax(SHOOTER2_ID, MotorType.kBrushless);

  // Mesafe (Vision/Mesafe m) -> RPM. ÖRNEK DEĞERLER: kendi ölçümlerinle değiştir.
  private final InterpolatingDoubleTreeMap rpmTablosu = new InterpolatingDoubleTreeMap();

  private final Debouncer hazirFiltre = new Debouncer(0.1, DebounceType.kRising);
  private boolean besleniyor = false;
  private boolean atisAktif = false;

  // Vision bağlantısı (RobotContainer'da setVision ile verilir)
  private DoubleSupplier mesafe = () -> Double.NaN;
  private BooleanSupplier besleIzni = () -> true;

  public UpperMechanismSubsystem() {
    rpmTablosu.put(1.0, 2500.0);
    rpmTablosu.put(2.0, 3000.0);
    rpmTablosu.put(3.0, 3500.0);

    SmartDashboard.setDefaultNumber("Atis/Hedef RPM", 2500);
    SmartDashboard.setDefaultNumber("Atis/Mesafe m", 2.0);
    SmartDashboard.setDefaultBoolean("Atis/Tablo kullan", TABLO_VARSAYILAN);

    SparkMaxConfig intakeLeftCfg = new SparkMaxConfig();
    intakeLeftCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

    SparkMaxConfig intakeRightCfg = new SparkMaxConfig();
    intakeRightCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30).follow(intakeLeft, true);

    SparkMaxConfig shooterCfg = new SparkMaxConfig();
    shooterCfg.idleMode(IdleMode.kCoast).smartCurrentLimit(40);

    SparkMaxConfig shooter2Cfg = new SparkMaxConfig();
    shooter2Cfg.idleMode(IdleMode.kCoast).smartCurrentLimit(40).follow(shooter, true);

    intakeLeft.configure(intakeLeftCfg, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeRight.configure(intakeRightCfg, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    shooter.configure(shooterCfg, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    shooter2.configure(shooter2Cfg, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** Vision'dan mesafe ve "besleme izni" bilgisini bağlar. */
  public void setVision(DoubleSupplier mesafe, BooleanSupplier besleIzni) {
    this.mesafe = mesafe;
    this.besleIzni = besleIzni;
  }

  public boolean atisAktifMi() {
    return atisAktif;
  }

  public boolean atisHazirMi() {
    return atisAktif && besleniyor;
  }

  /** Basılı tutulduğu sürece alır (A tuşu). */
  public Command intakeCommand() {
    return startEnd(() -> intakeLeft.set(INTAKE_SPEED), () -> intakeLeft.set(0));
  }

  /** Elle sabit güçle atma (B tuşu), besleme yok. */
  public Command shootCommand() {
    return startEnd(() -> shooter.set(SHOOTER_MANUAL_SPEED), () -> shooter.set(0));
  }

  /**
   * Y tuşu: shooter hedef hıza çıkar, hız tamam VE izin varsa intake besler.
   * Basılı tutulduğu sürece beslemeye devam eder. Bırakınca hepsi durur.
   */
  public Command shootWhenReadyCommand() {
    return runOnce(() -> {
      besleniyor = false;
      hazirFiltre.calculate(false);
    }).andThen(run(this::atisDongusu)).finallyDo(this::stop);
  }

  private double hedefRpm() {
    if (SmartDashboard.getBoolean("Atis/Tablo kullan", TABLO_VARSAYILAN)) {
      double m = mesafe.getAsDouble();
      if (Double.isNaN(m)) {
        m = SmartDashboard.getNumber("Atis/Mesafe m", 2.0); // hedef görünmüyorsa elle girilen
      }
      SmartDashboard.putNumber("Atis/Kullanilan mesafe", m);
      return rpmTablosu.get(m);
    }
    return SmartDashboard.getNumber("Atis/Hedef RPM", 2500);
  }

  private double shooterRpm() {
    return Math.abs(shooter.getEncoder().getVelocity());
  }

  private void atisDongusu() {
    atisAktif = true;
    double hedef = hedefRpm();
    double olculen = shooterRpm();
    double volt = SHOOTER_KV * hedef + SHOOTER_KP * (hedef - olculen);
    volt = MathUtil.clamp(volt, 0.0, 12.0);
    shooter.setVoltage(SHOOTER_DIR * volt);

    boolean hizTamam = hazirFiltre.calculate(
        olculen >= hedef - READY_ALT_RPM && olculen <= hedef + READY_UST_RPM);
    boolean izin = besleIzni.getAsBoolean();
    if (hizTamam && izin) {
      if (!besleniyor) {
        // Kalibrasyon için: konsola yapıştırmaya hazır satır yazar
        System.out.println(String.format("KALIBRASYON rpmTablosu.put(%.2f, %.0f);",
            SmartDashboard.getNumber("Atis/Kullanilan mesafe", -1), hedef));
      }
      besleniyor = true;
    }
    intakeLeft.set(besleniyor ? FEED_SPEED : 0.0);

    SmartDashboard.putNumber("Atis/Hedef (kullanilan) RPM", hedef);
    SmartDashboard.putNumber("Atis/Olculen RPM", olculen);
    SmartDashboard.putBoolean("Atis/Hiz tamam", hizTamam);
    SmartDashboard.putBoolean("Atis/Hizalama izni", izin);
    SmartDashboard.putBoolean("Atis/Hazir", besleniyor);
  }

  public void stop() {
    atisAktif = false;
    besleniyor = false;
    intakeLeft.set(0);
    shooter.set(0);
  }
}