package frc.sirius.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static final String TABLO = "limelight"; // Limelight adını değiştirdiysen yaz
  private static final int PIPELINE = 0;           // AprilTag pipeline numarası
  private static final int HEDEF_TAG_ID = -1;      // hedef etiket ID'si, -1 = herhangi biri
  private static final double TX_OFFSET_DEG = 0.0; // atış hep sağa/sola kaçıyorsa buradan düzelt
  private static final double HIZA_TOLERANSI_DEG = 1.5;
  private static final double VERI_ZAMAN_ASIMI_S = 0.5;

  private final NetworkTable tablo = NetworkTableInstance.getDefault().getTable(TABLO);
  private final MedianFilter filtre = new MedianFilter(5);

  private boolean bagli = false;
  private boolean hedefVar = false;
  private double tx = 0;
  private double mesafe = Double.NaN;
  private String durum = "";

  private long sonTlZaman = -1;
  private int kareNo = 0;
  private double gecikmeS = 0;

  @Override
  public void periodic() {
    tablo.getEntry("pipeline").setNumber(PIPELINE);

    // Limelight her karede "tl" yazar. Verinin yaşına bakarak bağlantıyı anlarız.
    long tlZaman = tablo.getEntry("tl").getLastChange();
    long yasUs = RobotController.getFPGATime() - tlZaman;
    bagli = tablo.getEntry("tl").exists() && yasUs < VERI_ZAMAN_ASIMI_S * 1e6;

    // Yeni kare geldi mi? Gecikmeyi de hesapla (yakalama + işleme, ms → sn)
    if (tlZaman != sonTlZaman) {
      sonTlZaman = tlZaman;
      kareNo++;
      gecikmeS = (tablo.getEntry("tl").getDouble(0) + tablo.getEntry("cl").getDouble(0)) / 1000.0;
    }

    boolean tv = tablo.getEntry("tv").getDouble(0) == 1;
    int tid = (int) tablo.getEntry("tid").getDouble(-1);
    boolean dogruTag = HEDEF_TAG_ID < 0 || tid == HEDEF_TAG_ID;
    hedefVar = bagli && tv && dogruTag;

    if (!bagli) {
      durum = "Limelight verisi yok (kablo, IP, takım numarası)";
    } else if (!tv) {
      durum = "Etiket görünmüyor (pipeline AprilTag mı? kamera eğimi?)";
    } else if (!dogruTag) {
      durum = "Yanlış etiket ID: " + tid;
    } else {
      durum = "OK";
    }

    if (hedefVar) {
      tx = tablo.getEntry("tx").getDouble(0) - TX_OFFSET_DEG;

      // [x, y, z, roll, pitch, yaw]: kamera çerçevesinde etiketin konumu (m)
      double[] pose = tablo.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
      if (pose.length >= 3) {
        double d = Math.sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);
        mesafe = d > 0.05 ? filtre.calculate(d) : Double.NaN;
      } else {
        mesafe = Double.NaN;
      }
    } else {
      tx = 0;
      mesafe = Double.NaN;
      filtre.reset();
    }

    SmartDashboard.putBoolean("Vision/Bagli", bagli);
    SmartDashboard.putBoolean("Vision/Hedef var", hedefVar);
    SmartDashboard.putBoolean("Vision/Hizali", hizaliMi());
    SmartDashboard.putString("Vision/Durum", durum);
    SmartDashboard.putNumber("Vision/tx", tx);
    SmartDashboard.putNumber("Vision/Mesafe m", Double.isNaN(mesafe) ? -1 : mesafe);
  }

  public boolean hedefVarMi() {
    return hedefVar;
  }

  /** Hedef kameranın sağındaysa pozitif (derece). */
  public double getTxDeg() {
    return tx;
  }

  /** Kameradan etikete düz çizgi mesafesi (m). Yoksa NaN. */
  public double getMesafeM() {
    return mesafe;
  }

  public boolean hizaliMi() {
    return hedefVar && Math.abs(tx) < HIZA_TOLERANSI_DEG;
  }

  /** Limelight'tan her yeni kare geldiğinde artar. */
  public int getKareNo() {
    return kareNo;
  }

  /** Karenin çekilmesinden bu yana geçen yaklaşık süre (sn). */
  public double getGecikmeS() {
    return gecikmeS;
  }
}