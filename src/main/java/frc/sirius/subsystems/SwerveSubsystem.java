package frc.sirius.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.sirius.Constants;
import frc.sirius.Constants.Chassis;
import frc.sirius.Constants.Gyro;
import frc.sirius.Constants.ModuleHw;

public class SwerveSubsystem extends SubsystemBase {
  // Sıra Constants.Chassis.KINEMATICS ile aynı: ÖN SOL, ÖN SAĞ, ARKA SOL, ARKA
  // SAĞ
  private final SwerveModule[] modules = {
      new SwerveModule(Constants.FRONT_LEFT),
      new SwerveModule(Constants.FRONT_RIGHT),
      new SwerveModule(Constants.BACK_LEFT),
      new SwerveModule(Constants.BACK_RIGHT)
  };

  private final Pigeon2 gyro = new Pigeon2(Gyro.PIGEON_CAN_ID, new CANBus(Gyro.CAN_BUS));
  private final SwerveDriveOdometry odometry;
  private final Field2d saha = new Field2d();
  private double sonYazilanGyro = 0;
  private int hizalaSayac = 0;

  private int gyroSifirSayac = 0;

  public int getGyroSifirSayac() {
    return gyroSifirSayac;
  }

  public SwerveSubsystem() {
    gyro.setYaw(0);
    odometry = new SwerveDriveOdometry(Chassis.KINEMATICS, getYaw(), getPositions());
    SmartDashboard.putData("Saha", saha);
  }

  /**
   * Gyro açısı (saat yönünün tersi pozitif). NavX'e geçersen yalnız burayı
   * değiştir.
   */
  public Rotation2d getYaw() {
    double deg = gyro.getYaw().getValueAsDouble();
    return Rotation2d.fromDegrees(Gyro.INVERT ? -deg : deg);
  }

  /** Start tuşu: robotun şu anki yönü "ileri" olur. */
  public void zeroGyro() {
  gyroSifirSayac++;
  gyro.setYaw(0);
  odometry.resetPosition(getYaw(), getPositions(),
      new Pose2d(getPose().getTranslation(), new Rotation2d()));
}

  /**
   * @param xMps ileri (+) m/s, @param yMps sola (+) m/s, @param omegaRadPs sola
   *             dönüş (+)
   */
  public void drive(double xMps, double yMps, double omegaRadPs, boolean fieldRelative) {
    if (Chassis.ON_TERS) { // robotun gerçek önü modül çerçevesinin -x yönü
      xMps = -xMps;
      yMps = -yMps;
    }
    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xMps, yMps, omegaRadPs, getYaw())
        : new ChassisSpeeds(xMps, yMps, omegaRadPs);
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] states = Chassis.KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleHw.MAX_SPEED_MPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  /**
   * Ofset hizalaması (Test kipi): dört teker düz ileri, DİŞLİLER SAĞDA (robotta
   * görüldü, 10-25).
   */
  public void hizala() {
    double[] aci = { 0, 0, 0, 0 }; // ÖN SOL, ÖN SAĞ, ARKA SOL, ARKA SAĞ
    for (int i = 0; i < 4; i++) {
      modules[i].aciyaGit(Rotation2d.fromDegrees(aci[i]));
    }
    if (hizalaSayac++ % 25 == 0) {
      System.out.println("HIZALA " + ozet());
    }
  }

  /**
   * Tekerleri X biçiminde kilitle (hepsi merkeze çapraz, hız 0): itilince kaymaz.
   */
  public void xKilit() {
    double[] aci = { 45, -45, -45, 45 }; // ÖN SOL, ÖN SAĞ, ARKA SOL, ARKA SAĞ
    for (int i = 0; i < 4; i++) {
      modules[i].kilitle(Rotation2d.fromDegrees(aci[i]));
    }
  }

  /**
   * Ofset alma: robot DISABLE iken /home/lvuser/ofset_al dosyası belirirse
   * (uzaktan `touch`),
   * tekerlerin şu anki hâli "düz ileri" sayılır ve dört ofset kalıcı kaydedilir.
   * Hizalama: dört teker düz ileri, BÜTÜN dişliler (bevel) robotun SAĞINA baksın.
   */
  private void ofsetIstegi() {
    java.io.File f = new java.io.File("/home/lvuser/ofset_al");
    if (!edu.wpi.first.wpilibj.DriverStation.isDisabled() || !f.exists()) {
      return;
    }
    f.delete();
    StringBuilder b = new StringBuilder("OFSET KAYDEDILDI");
    for (SwerveModule m : modules) {
      b.append(String.format(" | %s=%.2f", m.getName(), m.ofsetAl()));
    }
    System.out.println(b);
  }

  /** Deneme kaydı için tek satır: her modülün hedefe göre açısı ve hızı. */
  public String ozet() {
    StringBuilder b = new StringBuilder(String.format("gyro=%.1f", getYaw().getDegrees()));
    for (SwerveModule m : modules) {
      b.append(String.format(" | %s mutlak=%.1f motor=%.1f hiz=%.2f", m.getName(),
          m.getAbsoluteAngle().getDegrees(), m.getAngle().getDegrees(),
          m.getState().speedMetersPerSecond));
    }
    return b.toString();
  }

  public void stop() {
    for (SwerveModule m : modules) {
      m.stop();
    }
  }

  public void resyncModules() {
    for (SwerveModule m : modules) {
      m.resyncSteer();
    }
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] p = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      p[i] = modules[i].getPosition();
    }
    return p;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(getYaw(), getPositions());
    saha.setRobotPose(getPose());
    ofsetIstegi();
    double g = getYaw().getDegrees();
    if (edu.wpi.first.wpilibj.DriverStation.isDisabled() && Math.abs(g - sonYazilanGyro) > 10) { // gyro yön denetimi:
                                                                                                 // elle çevirince kayda
                                                                                                 // düşer
      System.out.println(String.format("GYRO %.1f derece (sola çevirince ARTMALI)", g));
      sonYazilanGyro = g;
    }
    SmartDashboard.putNumber("Gyro derece", getYaw().getDegrees());
    for (SwerveModule m : modules) {
      // Ofset ayarı için: tekerler düz ileri bakarken bu değeri Constants'a yaz.
      SmartDashboard.putNumber(m.getName() + "/Ham CANcoder derece",
          Units.radiansToDegrees(m.getRawAbsoluteRad()));
      SmartDashboard.putNumber(m.getName() + "/Mutlak derece", m.getAbsoluteAngle().getDegrees());
      SmartDashboard.putNumber(m.getName() + "/Motor derece", m.getAngle().getDegrees());
      SmartDashboard.putNumber(m.getName() + "/Hiz mps", m.getState().speedMetersPerSecond);
    }
  }
}
