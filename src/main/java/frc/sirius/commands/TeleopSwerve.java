package frc.sirius.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.sirius.Ayar;
import edu.wpi.first.wpilibj2.command.Command;
import frc.sirius.Constants.ModuleHw;
import frc.sirius.Constants.OI;
import frc.sirius.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Alan merkezli sürüş: sol çubuk hareket, sağ çubuk dönüş.
 * Hız kipleri: normal (Surus/HizCarpan), sol tampon yavaş, sağ tetik TURBO (tam
 * hız).
 * Yön tutma: sağ çubuk bırakılınca robot o anki yönünü gyro ile korur; giderken
 * kaymaz.
 */
public class TeleopSwerve extends Command {
  private final SwerveSubsystem swerve;
  private final DoubleSupplier forward;
  private final DoubleSupplier left;
  private final DoubleSupplier turn;
  private final BooleanSupplier slow;
  private final BooleanSupplier turbo;
  private final PIDController yonPid = new PIDController(4.0, 0, 0);
  private final Timer donusBitti = new Timer();
  private boolean yavas = false;
  private int kayit = 0;
  private int gorulenSifir = -1;
  private boolean oncekiTampon = false;
  private Double hedefYon = null; // radyan; null = yön tutma beklemede
  private double ivme = Double.NaN;
  private double donusIvme = Double.NaN;
  private SlewRateLimiter xSinir;
  private SlewRateLimiter ySinir;
  private SlewRateLimiter rSinir;

  public TeleopSwerve(SwerveSubsystem swerve, DoubleSupplier forward, DoubleSupplier left,
      DoubleSupplier turn, BooleanSupplier slow, BooleanSupplier turbo) {
    this.swerve = swerve;
    this.forward = forward;
    this.left = left;
    this.turn = turn;
    this.slow = slow;
    this.turbo = turbo;
    yonPid.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    ivme = Double.NaN; // ivme sınırlayıcılarını sıfırdan kur
    hedefYon = null;
    donusBitti.restart();
    gorulenSifir = swerve.getGyroSifirSayac();
  }

  @Override
  public void execute() {
    sinirlariGuncelle();
    int s = swerve.getGyroSifirSayac();
    if (s != gorulenSifir) { // Start'a basıldı: eski hedef yönü geçersiz
      gorulenSifir = s;
      hedefYon = null;
      yonPid.reset();
      donusBitti.restart();
    }
    // Sol tampon: bas-bırak ile yavaş kip AÇ/KAPA (10-25: basılı tutmak fark
    // edilmiyordu).
    boolean tampon = slow.getAsBoolean();
    if (tampon && !oncekiTampon) {
      yavas = !yavas;
      System.out.println("KIP yavaş " + (yavas ? "AÇIK" : "KAPALI"));
    }
    oncekiTampon = tampon;
    boolean turboAcik = turbo.getAsBoolean() && !yavas;
    double scale = yavas ? OI.SLOW_MODE_SCALE
        : turboAcik ? 1.0
            : Ayar.hizCarpan();
    // Akü 9 V altına inince hızı kıs: brownout (10-25 kaydında 6 kez) robotu
    // sarsar.
    double volt = RobotController.getBatteryVoltage();
    if (volt < 9.0) {
      scale = Math.min(scale, 0.6);
    }
    SmartDashboard.putString("Surus/Kip", (Ayar.sahaYonlu() ? "SAHA" : "ROBOT")
        + (yavas ? " · YAVAŞ" : turboAcik ? " · TURBO" : ""));
    SmartDashboard.putNumber("Surus/Aku V", volt);

    // Sol çubuk: yönü koru, BÜYÜKLÜĞE eğri uygula (küçük itiş = küçük hız, tam itiş
    // = tam hız).
    double x = forward.getAsDouble();
    double y = left.getAsDouble();
    double mag = Math.min(1.0, Math.hypot(x, y));
    double magD = MathUtil.applyDeadband(mag, OI.DEADBAND);
    double egriMag = Math.pow(magD, Ayar.egri());
    double vx = mag > 1e-6 ? x / mag * egriMag : 0.0;
    double vy = mag > 1e-6 ? y / mag * egriMag : 0.0;

    double r = MathUtil.applyDeadband(turn.getAsDouble(), OI.DEADBAND);
    r = Math.copySign(Math.pow(Math.abs(r), Ayar.donusEgri()), r);

    double vxm = xSinir.calculate(vx * ModuleHw.MAX_SPEED_MPS * scale);
    double vym = ySinir.calculate(vy * ModuleHw.MAX_SPEED_MPS * scale);
    double omega = rSinir.calculate(r * Ayar.donusMax() * Math.max(scale, 0.5));
    omega = yonTut(r, omega, Math.hypot(vxm, vym));
    if (Math.hypot(vxm, vym) > 0.05 || Math.abs(omega) > 0.05) {
      if (kayit++ % 10 == 0) { // teşhis: komut ↔ gyro işaretleri
        System.out.println(String.format("SUR vx=%.2f vy=%.2f r=%.2f omega=%.2f gyro=%.1f hedef=%s kip=%s",
            vxm, vym, r, omega, swerve.getYaw().getDegrees(),
            hedefYon == null ? "-" : String.format("%.1f", Math.toDegrees(hedefYon)),
            Ayar.sahaYonlu() ? "SAHA" : "ROBOT"));
      }
    }
    swerve.drive(vxm, vym, omega, Ayar.sahaYonlu());
  }

  /**
   * Sağ çubuk boştaysa ve robot gidiyorsa yönü sabit tut. Dönüş bitince 0,25 sn
   * bekleyip
   * (savrulma dinsin) o anki yönü hedef alır. Gyro 10-25 doğrulandı: robot
   * yönlüde de çalışır.
   */
  private double yonTut(double r, double omega, double hiz) {
    if (!Ayar.yonTut()) {
      hedefYon = null;
      return omega;
    }
    if (r != 0.0) {
      hedefYon = null;
      donusBitti.restart();
      return omega;
    }
    if (hedefYon == null) {
      if (donusBitti.hasElapsed(0.25)) {
        hedefYon = swerve.getYaw().getRadians();
      }
      return omega;
    }
    if (hiz < 0.05) {
      return 0.0; // dururken düzeltme yapma, teker sürtmesin
    }
    yonPid.setP(Ayar.yonTutP());
    double duzeltme = yonPid.calculate(swerve.getYaw().getRadians(), hedefYon);
    return MathUtil.clamp(duzeltme, -2.0, 2.0);
  }

  /** Preferences'tan ivme değişirse sınırlayıcıyı yeniden kur (canlı ayar). */
  private void sinirlariGuncelle() {
    double a = Ayar.ivme();
    double b = Ayar.donusIvme();
    if (a != ivme || b != donusIvme) {
      ivme = a;
      donusIvme = b;
      xSinir = new SlewRateLimiter(a);
      ySinir = new SlewRateLimiter(a);
      rSinir = new SlewRateLimiter(b);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
