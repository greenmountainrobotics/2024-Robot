package frc.robot.subsystems.leds;

import static frc.robot.constants.IdConstants.PWMId.LedsId;

import java.nio.channels.spi.SelectorProvider;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alliance;

public class Leds extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;
  private static final int LEDS_LENGTH = 10;

  private static final int PULSE_TIME = 500;

  private static final double PULSE_DIF = 256 / PULSE_TIME / 50;
  private boolean pulseUp = true;
  private double vValue = 255;
  private static final int hValue = Alliance.isRed() ? 0 : 120;
  private Timer timer = new Timer();

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
    if (timer.get() > PULSE_TIME) {
      pulseUp = !pulseUp;
      timer.restart();
    }
    if(!pulseUp) {
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
