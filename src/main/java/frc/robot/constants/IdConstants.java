package frc.robot.constants;

public final class IdConstants {
  public static final class CANId {
    public static final int FrontLeftDriveId = 1;
    public static final int FrontLeftTurnId = 2;
    public static final int FrontLeftEncoderId = 3;

    public static final int FrontRightDriveId = 4;
    public static final int FrontRightTurnId = 5;
    public static final int FrontRightEncoderId = 6;

    public static final int BackLeftDriveId = 7;
    public static final int BackLeftTurnId = 8;
    public static final int BackLeftEncoderId = 9;

    public static final int BackRightDriveId = 10;
    public static final int BackRightTurnId =
        11; // This one just... randomly reset its CAN id?? 3/16/2024
    public static final int BackRightEncoderId = 12;

    public static final int TopSpinMotorId = 21;
    public static final int BottomSpinMotorId = 22;
    public static final int ShooterArticulationMotorId = 23;

    public static final int PigeonId = 31;
    public static final int RevPDHId = 32;

    public static final int RightIntakeExtensionMotorId = 41;
    public static final int LeftIntakeExtensionMotorId = 42;
    public static final int IntakeArticulationMotorId = 43;
    public static final int IntakeSpinMotorId = 44;
  }

  public static final class PWMId {
    public static final int LedsId = 0;
  }

  public static final class DIOId {
    public static final int IntakeArticulationEncoderId = 0;
    public static final int ShooterArticulationEncoderId = 1;
  }
}
