package frc.sirius.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.sirius.Ayar;
import frc.sirius.Constants;
import frc.sirius.Constants.ModuleConfig;
import frc.sirius.Constants.ModuleHw;

/** Tek bir MK4i modülü: NEO sürüş + NEO dönüş (ikisi de Spark MAX) + CANcoder. */
public class SwerveModule {
  private final String name;
  private final SparkMax drive;
  private final SparkMax steer;
  private final RelativeEncoder driveEnc;
  private final RelativeEncoder steerEnc;
  private final SparkClosedLoopController steerPid;
  private final CANcoder cancoder;
  private double offsetRad;

  public SwerveModule(ModuleConfig cfg) {
    name = cfg.name();
    // Ofset önce roboRIO'daki kalıcı kayıttan (Ofset/<ad>), yoksa Constants'tan.
    edu.wpi.first.wpilibj.Preferences.initDouble("Ofset/" + name, cfg.offsetDeg());
    offsetRad = Units.degreesToRadians(
        edu.wpi.first.wpilibj.Preferences.getDouble("Ofset/" + name, cfg.offsetDeg()));

    drive = new SparkMax(cfg.driveId(), MotorType.kBrushless);
    steer = new SparkMax(cfg.steerId(), MotorType.kBrushless);
    cancoder = new CANcoder(cfg.cancoderId(), new CANBus(Constants.CANCODER_BUS));

    double wheelCirc = Math.PI * ModuleHw.WHEEL_DIAMETER_M;
    double drivePosFactor = wheelCirc / ModuleHw.DRIVE_RATIO; // motor turu → metre
    double steerPosFactor = 2 * Math.PI / ModuleHw.STEER_RATIO; // motor turu → radyan

    SparkMaxConfig driveCfg = new SparkMaxConfig();
    driveCfg.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ModuleHw.DRIVE_CURRENT_LIMIT_A)
        .inverted(ModuleHw.DRIVE_INVERTED);
    driveCfg.encoder.positionConversionFactor(drivePosFactor)
        .velocityConversionFactor(drivePosFactor / 60.0);
    drive.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig steerCfg = new SparkMaxConfig();
    steerCfg.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ModuleHw.STEER_CURRENT_LIMIT_A)
        .inverted(ModuleHw.STEER_INVERTED);
    steerCfg.encoder.positionConversionFactor(steerPosFactor)
        .velocityConversionFactor(steerPosFactor / 60.0);
    steerCfg.closedLoop.pid(ModuleHw.STEER_P, ModuleHw.STEER_I, ModuleHw.STEER_D)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .outputRange(-1, 1);
    steer.configure(steerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEnc = drive.getEncoder();
    steerEnc = steer.getEncoder();
    steerPid = steer.getClosedLoopController();

    driveEnc.setPosition(0);
    resyncSteer();
  }

  /** CANcoder'ın ham açısı (radyan, ofset UYGULANMAMIŞ). Ofset ayarında bunu oku. */
  public double getRawAbsoluteRad() {
    return cancoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }

  /** Ofset uygulanmış mutlak açı. */
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRadians(getRawAbsoluteRad() - offsetRad);
  }

  /** Dönüş motorunun göreli enkoderini mutlak açıya eşitle. */
  public void resyncSteer() {
    steerEnc.setPosition(getAbsoluteAngle().getRadians());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(steerEnc.getPosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEnc.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEnc.getPosition(), getAngle());
  }

  public void setDesiredState(SwerveModuleState desired) {
    // Durma kararı HAM hız üzerinden verilir. Önce cosineScale yapılırsa 90° hatalı teker
    // hızı ~0 sayılıp dönüş motoru da kapanır ve teker hiç dönmez (10-25 deneme kaydı).
    if (Math.abs(desired.speedMetersPerSecond) < 0.01) {
      stop();                           // dururken tekerleri merkezlemeye çalışma
      return;
    }
    Rotation2d current = getAngle();
    desired.optimize(current);          // 90°'den fazla dönmek yerine tekeri ters sür
    desired.cosineScale(current);       // açı oturana dek sürüş hızını azalt (dönüş sürer)
    // İleri besleme: kS sürtünmeyi yener (küçük komut da tekeri döndürür), kV hızla orantılı.
    double v = desired.speedMetersPerSecond;
    double volt = Math.copySign(Ayar.kS(), v) + Ayar.kV() * v;
    drive.setVoltage(Math.max(-12.0, Math.min(12.0, volt)));
    steerPid.setSetpoint(desired.angle.getRadians(), ControlType.kPosition);
  }

  /** Tekeri verilen açıya çevir, KISA YOL ARAMADAN (ofset hizalaması için), sürüş kapalı. */
  public void aciyaGit(Rotation2d aci) {
    drive.stopMotor();
    tut(aci.getRadians());
  }

  /** Hedefe 1,5° kala dönüş motorunu bırak: yerinde tutarken vızıldamasın (10-25 "trrrr"). */
  private void tut(double hedefRad) {
    double hata = MathUtil.angleModulus(hedefRad - getAngle().getRadians());
    if (Math.abs(hata) < Math.toRadians(1.5)) {
      steer.stopMotor();
    } else {
      steerPid.setSetpoint(MathUtil.angleModulus(hedefRad), ControlType.kPosition);
    }
  }

  /** Sürüşü durdur, tekeri verilen açıda tut (X kilidi). */
  public void kilitle(Rotation2d aci) {
    SwerveModuleState st = new SwerveModuleState(0, aci);
    st.optimize(getAngle());
    drive.stopMotor();
    tut(st.angle.getRadians());
  }

  public void stop() {
    drive.stopMotor();
    steer.stopMotor();
  }

  /** Teker ŞU AN düz ileri bakıyor kabul edilip ofset kalıcı kaydedilir. Döndürür: yeni ofset (derece). */
  public double ofsetAl() {
    double deg = Units.radiansToDegrees(getRawAbsoluteRad());
    edu.wpi.first.wpilibj.Preferences.setDouble("Ofset/" + name, deg);
    offsetRad = Units.degreesToRadians(deg);
    resyncSteer();
    return deg;
  }

  public String getName() {
    return name;
  }
}
