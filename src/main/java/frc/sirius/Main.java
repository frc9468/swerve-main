package frc.sirius;

import edu.wpi.first.wpilibj.RobotBase;

/** Başlangıç noktası. Burayı değiştirme. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
