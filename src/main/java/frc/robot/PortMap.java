package frc.robot;

public class PortMap {
    public static class Intake {
        public static final int intakeMotor1ID = 14;
        public static final int intakeMotor2ID = 15;
        public static final int intakePositionMotorID = 13;
    
        public static final int closingHallEffectPort = 1;
      }
  
      public static class Spinner {
        public static final int stuckIRPort = 4;
        public static final int buttomIRPort = 0;
    
        public static final int spinnerMotorID = 16;
      }
  
      public static class Gripper {
        public static final int gripperMotorID = 17;
      }
  
      public static class ChameleonClimb {
        public static final int graspingMotorID = 20;
      }
  
      public static class Arm {
        public static final int extenstionMotorID = 18;
        public static final int extenstionHallEffectPort = 9;
        public static final int rotationMotorID = 19;
      }
  
      public static class Swerve {
        public static final int leftFrontAbsoluteEncoder = 21;
        public static final int leftFrontDriveID = 2;
        public static final int leftFrontTurningID = 3;
    
        public static final int leftBackAbsoluteEncoder = 22;
        public static final int leftBackDriveID = 4;
        public static final int leftBackTurningID = 5;
    
        public static final int rightFrontAbsoluteEncoder = 23;
        public static final int rightFrontDriveID = 7;
        public static final int rightFrontTurningID = 6;
    
        public static final int rightBackAbsoluteEncoder = 24;
        public static final int rightBackDriveID = 8;
        public static final int rightBackTurningID = 9;
      }

      public static final int ledPort = 5; //TODO
}
