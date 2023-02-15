package frc.robot;

public class PortMap {
    public static class Intake {
        public static final int intakeMotorID = 13; //TODO
        public static final int intakePositionMotorID = 14; //TODO
    
        public static final int closingHallEffectPort = 1; //TODO
      }
  
      public static class Spinner {
        public static final int stuckIRPort = 2; //TODO
        public static final int buttomIRPort = 3; //TODO
    
        public static final int spinnerMotorID = 15; //TODO
      }
  
      public static class Gripper {
        public static final int gripperMotorID = 16; //TODO
      }
  
      public static class ChameleonClimb {
        public static final int graspingMotorID = 17; //TODO
      }
  
      public static class Arm {
        public static final int extenstionMotorID = 18; //TODO
        public static final int extenstionHallEffectPort = 4; //TODO
        public static final int rotationMotorID = 19; //TODO
        public static final int rotationHallEffectPort = 5; //TODO
      }
  
      public static class Swerve {
        public static final int leftFrontAbsoluteEncoder = 9;
        public static final int leftFrontDriveID = 1;
        public static final int leftFrontTurningID = 2;
    
        public static final int leftBackAbsoluteEncoder = 10;
        public static final int leftBackDriveID = 3;
        public static final int leftBackTurningID = 4;
    
        public static final int rightFrontAbsoluteEncoder = 11;
        public static final int rightFrontDriveID = 6;
        public static final int rightFrontTurningID = 5;
    
        public static final int rightBackAbsoluteEncoder = 12;
        public static final int rightBackDriveID = 7;
        public static final int rightBackTurningID = 8;
      }

      public static final int ledPort = 5; //TODO
}
