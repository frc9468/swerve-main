package frc.sirius;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.sirius.Constants.OI;
import frc.sirius.commands.AimDrive;
import frc.sirius.commands.TeleopSwerve;
import frc.sirius.subsystems.LedSubsystem;
import frc.sirius.subsystems.SwerveSubsystem;
import frc.sirius.subsystems.UpperMechanismSubsystem;
import frc.sirius.subsystems.VisionSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final UpperMechanismSubsystem upper = new UpperMechanismSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER_PORT);
  private final LedSubsystem led =
      new LedSubsystem(upper, vision, () -> driver.a().getAsBoolean());

  public RobotContainer() {
    // Etiket görünüyorsa hizalı olana kadar bekle, görünmüyorsa beklemeden besle.
    upper.setVision(vision::getMesafeM, () -> !vision.hedefVarMi() || vision.hizaliMi());

    // Xbox: çubuk ileri = negatif Y, sola = negatif X → işaretleri çeviriyoruz.
    swerve.setDefaultCommand(new TeleopSwerve(
        swerve,
        () -> -driver.getLeftY(),   // ileri/geri
        () -> -driver.getLeftX(),   // sağ/sol
        () -> -driver.getRightX(),  // dönüş
        () -> driver.leftBumper().getAsBoolean(),     // yavaş kip
        () -> driver.getRightTriggerAxis() > 0.5));   // TURBO

    // X basılıyken tekerler çapraz kilitlenir
    driver.x().whileTrue(Commands.run(swerve::xKilit, swerve));

    // A basılıyken al, B basılıyken elle at
    driver.a().whileTrue(upper.intakeCommand());
    driver.b().whileTrue(upper.shootCommand());

    // Y: nişan al + hıza çık + hazır olunca besle
    driver.y().whileTrue(Commands.parallel(
        upper.shootWhenReadyCommand(),
        new AimDrive(swerve, vision, () -> -driver.getLeftY(), () -> -driver.getLeftX())));

    // RB: sadece nişan al
    driver.rightBumper().whileTrue(
        new AimDrive(swerve, vision, () -> -driver.getLeftY(), () -> -driver.getLeftX()));

    // Start = gyro sıfırla
    driver.start().onTrue(Commands.runOnce(swerve::zeroGyro).ignoringDisable(true));

    // Back = saha yönlü ↔ robot yönlü
    driver.back().onTrue(Commands.runOnce(frc.sirius.Ayar::sahaYonluDegistir).ignoringDisable(true));
  }

  public Command denemeSurusu() {
    return Commands.sequence(
        adim("1-ileri", 0.4, 0, 0),
        adim("2-dur", 0, 0, 0),
        adim("3-sola", 0, 0.4, 0),
        adim("4-dur", 0, 0, 0),
        adim("5-sola-don", 0, 0, 1.0),
        adim("6-dur", 0, 0, 0),
        adim("7-geri", -0.4, 0, 0),
        Commands.runOnce(() -> {
          swerve.stop();
          System.out.println("DENEME bitti");
        }, swerve));
  }

  private Command adim(String ad, double vx, double vy, double omega) {
    int[] n = {0};
    return Commands.run(() -> {
      swerve.drive(vx, vy, omega, false);
      if (n[0]++ % 10 == 0) {
        System.out.println("DENEME " + ad + " " + swerve.ozet());
      }
    }, swerve).beforeStarting(() -> n[0] = 0).withTimeout(2.0);
  }

  public void hizala() {
    swerve.hizala();
  }

  public void stopDrive() {
    swerve.stop();
    upper.stop();
  }

  public void resyncModules() {
    swerve.resyncModules();
  }
}