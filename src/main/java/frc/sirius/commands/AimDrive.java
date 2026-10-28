package frc.sirius.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.sirius.Ayar;
import frc.sirius.Constants.ModuleHw;
import frc.sirius.Constants.OI;
import frc.sirius.subsystems.SwerveSubsystem;
import frc.sirius.subsystems.VisionSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/** Sol çubukla sür, robot kendini gyro ile hedefe hizalasın (sağ çubuk devre dışı). */
public class AimDrive extends Command {
  private static final double KP = 5.0;           // rad/s per rad. Salınıyorsa düşür, yavaşsa artır
  private static final double KD = 0.25;          // yaklaşırken frenleme. Titriyorsa düşür
  private static final double MAX_OMEGA = 4.0;    // rad/s
  private static final double MIN_OMEGA = 0.25;   // tekerler sürtünmesini aşsın diye alt sınır
  private static final double OLU_BOLGE_DEG = 0.6;
  private static final double DONUS_IVME = 20.0;  // rad/s² dönüş hızının değişim sınırı
  private static final double HEDEF_KAYIP_S = 0.4;
  private static final double ISARET = 1.0;       // robot hedeften KAÇIYORSA -1.0 yap
  private static final double HIZ_CARPANI = 0.5;  // nişan alırken sürüş hızı

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final DoubleSupplier forward;
  private final DoubleSupplier left;

  private final PIDController pid = new PIDController(KP, 0, KD);
  private final SlewRateLimiter omegaSinir = new SlewRateLimiter(DONUS_IVME);
  private final TimeInterpolatableBuffer<Rotation2d> yawGecmisi =
      TimeInterpolatableBuffer.createBuffer(1.0);

  private double hedefYawRad = 0;
  private double hedefZamani = -100;
  private boolean hedefVar = false;
  private int sonKare = -1;

  public AimDrive(SwerveSubsystem swerve, VisionSubsystem vision,
      DoubleSupplier forward, DoubleSupplier left) {
    this.swerve = swerve;
    this.vision = vision;
    this.forward = forward;
    this.left = left;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pid.reset();
    omegaSinir.reset(0);
    hedefVar = false;
    sonKare = vision.getKareNo();
  }

  @Override
  public void execute() {
    double simdi = Timer.getFPGATimestamp();
    Rotation2d yaw = swerve.getYaw();
    yawGecmisi.addSample(simdi, yaw);

    // Yeni Limelight karesi geldi: o karenin çekildiği andaki yönden hedef yönü hesapla
    if (vision.hedefVarMi() && vision.getKareNo() != sonKare) {
      sonKare = vision.getKareNo();
      Optional<Rotation2d> yawKare = yawGecmisi.getSample(simdi - vision.getGecikmeS());
      double kareYawRad = yawKare.orElse(yaw).getRadians();
      // tx pozitif = hedef sağda → hedef yön daha negatif (saat yönü)
      hedefYawRad = kareYawRad - Math.toRadians(vision.getTxDeg());
      hedefZamani = simdi;
      hedefVar = true;
    }
    if (simdi - hedefZamani > HEDEF_KAYIP_S) {
      hedefVar = false;
    }

    double hataRad = 0;
    double omega = 0;
    if (hedefVar) {
      hataRad = MathUtil.angleModulus(hedefYawRad - yaw.getRadians());
      if (Math.abs(Math.toDegrees(hataRad)) > OLU_BOLGE_DEG) {
        omega = pid.calculate(0, hataRad) * ISARET;
        if (Math.abs(omega) < MIN_OMEGA) {
          omega = Math.copySign(MIN_OMEGA, omega);
        }
        omega = MathUtil.clamp(omega, -MAX_OMEGA, MAX_OMEGA);
      } else {
        pid.reset();
      }
    } else {
      pid.reset();
    }
    omega = omegaSinir.calculate(omega);

    SmartDashboard.putNumber("Aim/Hata derece", Math.toDegrees(hataRad));
    SmartDashboard.putBoolean("Aim/Hedef var", hedefVar);

    double x = MathUtil.applyDeadband(forward.getAsDouble(), OI.DEADBAND);
    double y = MathUtil.applyDeadband(left.getAsDouble(), OI.DEADBAND);
    swerve.drive(x * ModuleHw.MAX_SPEED_MPS * HIZ_CARPANI,
        y * ModuleHw.MAX_SPEED_MPS * HIZ_CARPANI, omega, Ayar.sahaYonlu());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}