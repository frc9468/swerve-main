package frc.sirius.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LedSubsystem extends SubsystemBase {
  private static final int PWM_PORT = 0;   // roboRIO PWM 0
  private static final int LED_COUNT = 30; // DOLDUR: şeritteki LED sayısı

  private final AddressableLED led = new AddressableLED(PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  private final UpperMechanismSubsystem upper;
  private final VisionSubsystem vision;
  private final BooleanSupplier intaking;

  private static LEDPattern p(Color c) {
    return LEDPattern.solid(c).atBrightness(Percent.of(50));
  }

  private final LEDPattern dusukAku = p(Color.kRed).blink(Seconds.of(0.1));
  private final LEDPattern dsYok = p(Color.kRed).blink(Seconds.of(0.5));
  private final LEDPattern kapaliHazir = p(Color.kBlue).breathe(Seconds.of(3));
  private final LEDPattern atisHazir = p(Color.kGreen);
  private final LEDPattern atisHazirlaniyor = p(Color.kYellow).blink(Seconds.of(0.1));
  private final LEDPattern alma = p(Color.kOrange);
  private final LEDPattern hizali = p(Color.kWhite);
  private final LEDPattern turbo = p(Color.kCyan);
  private final LEDPattern yavas = p(Color.kPurple);
  private final LEDPattern hedefGorunuyor = p(Color.kMagenta);
  private final LEDPattern bosta = p(Color.kBlue);

  public LedSubsystem(UpperMechanismSubsystem upper, VisionSubsystem vision,
      BooleanSupplier intaking) {
    this.upper = upper;
    this.vision = vision;
    this.intaking = intaking;
    led.setLength(LED_COUNT);
    led.start();
  }

  @Override
  public void periodic() {
    LEDPattern secilen = sec();
    secilen.applyTo(buffer);
    led.setData(buffer);
  }

  private LEDPattern sec() {
    boolean disabled = DriverStation.isDisabled();
    double volt = RobotController.getBatteryVoltage();

    if (volt < (disabled ? 11.8 : 9.0)) {
      return dusukAku;
    }
    if (disabled) {
      return DriverStation.isDSAttached() ? kapaliHazir : dsYok;
    }
    if (upper.atisHazirMi()) {
      return atisHazir;
    }
    if (upper.atisAktifMi()) {
      return atisHazirlaniyor;
    }
    if (intaking.getAsBoolean()) {
      return alma;
    }
    if (vision.hizaliMi()) {
      return hizali;
    }
    String kip = SmartDashboard.getString("Surus/Kip", "");
    if (kip.contains("TURBO")) {
      return turbo;
    }
    if (kip.contains("YAVAŞ")) {
      return yavas;
    }
    if (vision.hedefVarMi()) {
      return hedefGorunuyor;
    }
    return bosta;
  }
}