package frc.sirius;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final RobotContainer container;

  public Robot() {
    Ayar.eskiyiTasi();
    container = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    container.stopDrive();
  }

  @Override
  public void autonomousInit() {
    container.resyncModules();
    CommandScheduler.getInstance().schedule(container.denemeSurusu());
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    // Dönüş motorlarının göreli enkoderini mutlak enkoderden (CANcoder) yeniden al.
    container.resyncModules();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    container.resyncModules();
  }

  /** Test kipi = ofset hizalaması: tekerler düz ileri döner, robot sürmez. */
  @Override
  public void testPeriodic() {
    container.hizala();
  }
}
