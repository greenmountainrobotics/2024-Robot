package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.constants.Battery;
import java.time.Instant;
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;

public class BatteryTracker {
  private static Battery battery;

  public static Battery scanBatteryQR() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    CvSink sink = CameraServer.getVideo();
    QRCodeDetector detector = new QRCodeDetector();

    Mat mat = new Mat();
    long startTime = Instant.now().toEpochMilli();
    while (sink.grabFrame(mat) == 0) {
      System.out.println(sink.getError());
      if (!sink.getError().equals("timed out getting frame")
          || Instant.now().toEpochMilli() - startTime > 4000) {
        return Battery.NONE;
      }
    }
    String result = detector.detectAndDecode(mat);
    sink.close();
    camera.close();
    battery =
        switch (result) {
          case "1" -> Battery.CIABATTA;
          case "2" -> Battery.CLUNKY_NOISES;
          case "3" -> Battery.SOURDOUGH;
          case "4" -> Battery.CHALLAH;
          case "5" -> Battery.RYE;
          default -> Battery.NONE;
        };
    return battery;
  }

  public static Battery getBattery() {
    return battery;
  }
}
