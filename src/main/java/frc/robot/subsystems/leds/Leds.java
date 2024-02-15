package frc.robot.subsystems.leds;

import static frc.robot.Constants.PWMIdConstants.LedsId;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;
  private static final int LEDS_LENGTH = 71;

  public Leds() {
    leds = new AddressableLED(LedsId);
    ledBuffer = new AddressableLEDBuffer(LEDS_LENGTH);

    leds.setLength(ledBuffer.getLength());

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }

    leds.setData(ledBuffer);
    leds.start();
  }

  @Override
  public void periodic() {
    leds.setData(ledBuffer);
  }
}
