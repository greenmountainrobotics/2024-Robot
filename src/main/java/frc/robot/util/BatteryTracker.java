package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import frc.robot.Constants.Battery;
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;

public class BatteryTracker {
  public static Battery scanBatteryQR() {
    try (UsbCamera camera = CameraServer.startAutomaticCapture();
        CvSink sink = CameraServer.getVideo()) {
      QRCodeDetector detector = new QRCodeDetector();

      Mat mat = new Mat();
      if (sink.grabFrame(mat) == 0) {
        return Battery.NONE;
      }
      String result = detector.detectAndDecode(mat);
      sink.close();
      camera.close();
      return switch (result) {
        case "1" -> Battery.CIABATTA;
        case "2" -> Battery.CLUNKY_NOISES;
        case "3" -> Battery.SOURDOUGH;
        case "4" -> Battery.CHALLAH;
        case "5" -> Battery.RYE;
        default -> Battery.NONE;
      };
    }
  }
}
