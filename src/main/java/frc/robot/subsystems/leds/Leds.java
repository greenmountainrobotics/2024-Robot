package frc.robot.subsystems.leds;

import static frc.robot.Constants.PWMIdConstants.LedsId;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alliance;

public class Leds extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;
  private static final int LEDS_LENGTH = 10;

  public static class State {
    public static boolean AprilTagsPoseDetected = false;
    public static boolean RunningAuto = false;
    public static boolean DrivingToPose = false;

    public static boolean RunningTeleOp = false;
  }

  public Leds() {
    leds = new AddressableLED(LedsId);
    ledBuffer = new AddressableLEDBuffer(LEDS_LENGTH);

    leds.setLength(ledBuffer.getLength());

    showSolidColor(Color.kGreen);

    leds.setData(ledBuffer);
    leds.start();
  }

  private void autoSetColors() {
    Color allianceColor = Alliance.isRed() ? Color.kRed : Color.kBlue;

    /*
    apriltags not connected -> yellow
    robot started -> green
    auto -> green pulse
    teleop -> alliance color static
    running to position -> alliance color pulse
    */

    if (!State.AprilTagsPoseDetected) {
      showSolidColor(new Color(128, 0, 0));
    } else if (State.RunningAuto) {
      pulseColor(Color.kGreen);
    } else if (State.DrivingToPose) {
      pulseColor(allianceColor);
    } else if (State.RunningTeleOp) {
      showSolidColor(allianceColor);
    } else {
      showSolidColor(Color.kGreen);
    }
  }

  private void showSolidColor(Color color) {
    for (int i = 0; i < LEDS_LENGTH; i++) {
      ledBuffer.setLED(i, color);
    }
  }

  private void pulseColor(Color color) {
    for (int i = 0; i < LEDS_LENGTH; i++) {
      ledBuffer.setLED(i, color); // TODO: actually pulse
    }
  }

  @Override
  public void periodic() {
    autoSetColors();

    leds.setData(ledBuffer);
  }
}
