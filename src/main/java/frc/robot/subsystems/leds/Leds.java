package frc.robot.subsystems.leds;

import static frc.robot.constants.IdConstants.PWMId.LedsId;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LedConstants;
import frc.robot.util.Alliance;

public class Leds extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;
  private static final double PULSE_DIF = 256 / LedConstants.PULSE_TIME / 100;
  private boolean pulseUp = true;
  private double vValue = 255;
  private int hValue = Alliance.isRed() ? 0 : 120;
  private Timer timer = new Timer();
  private Color currentColor = Color.kGreen;

  public static class State {
    public static boolean AprilTagsPoseDetected = false;
    public static boolean RunningAuto = false;
    public static boolean DrivingToPose = false;

    public static boolean RunningTeleOp = false;
  }

  public Leds(AddressableLED leds) {
    this.leds = leds;
    ledBuffer = new AddressableLEDBuffer(LedConstants.LEDS_LENGTH);

    leds.setLength(ledBuffer.getLength());

    leds.setData(ledBuffer);
    leds.start();
  }

  private void autoSetColors() {
    Color allianceColor = Alliance.isRed() ? Color.kRed : Color.kBlue;
    hValue += 1;
    hValue %= 360;
    showSolidColor(Color.fromHSV(hValue, 255, 255));

    /*
    apriltags not connected -> yellow
    robot started -> green
    auto -> green pulse
    teleop -> alliance color static
    running to position -> alliance color pulse
    */

    /*    if (!State.AprilTagsPoseDetected) {
      showSolidColor(new Color(128, 128, 0));
    } else if (State.RunningAuto) {
      pulseColor(Color.kGreen);
    } else if (State.DrivingToPose) {
      pulseColor(allianceColor);
    } else if (State.RunningTeleOp) {
      showSolidColor(allianceColor);
    } else {
      showSolidColor(Color.kGreen);
    }*/
  }

  private void showSolidColor(Color color) {
    for (int i = 0; i < LedConstants.LEDS_LENGTH; i++) {
      ledBuffer.setLED(i, color);
    }
  }

  private void pulseColor(Color color) {
    if (timer.get() > LedConstants.PULSE_TIME) {
      pulseUp = !pulseUp;
      timer.restart();
    }
    if (!pulseUp) {
      vValue = Math.abs(vValue - PULSE_DIF);
    } else {
      vValue = Math.min(255, vValue + PULSE_DIF);
    }
    showSolidColor(Color.fromHSV(hValue, 255, (int) vValue));
  }

  @Override
  public void periodic() {
    autoSetColors();

    leds.setData(ledBuffer);
  }
}
