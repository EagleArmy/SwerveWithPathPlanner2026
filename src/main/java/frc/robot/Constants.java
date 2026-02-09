package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
   
    public static final class TestingConstants 
  {
    public static final int testMotorID = 6;
  } 

    public static final class ElevatorConstants 
  {
    public static final int kElevatorLeftMotorID = 99;        //Pratice Robot ID: 3
    //public static final int kElevatorRightMotorID = 10;      //Pratice Robot ID: 44
    // public static final double kStartPosition = 0; 
    // public static final double kFirstPosition = 4.6; //originally 0
    // public static final double kSecondPosition = 9.2;             
    // public static final double kThirdPosition = 26.5;
    // public static final double kFourthPosition = 54; //originally 54
    //public static final double kFourthPositionAuto = 52;
  }

  public static final class ShooterConstants {
    public static final int ShooterMotorID = 10;
    public static final int ShooterMotorID2 = 2;
    
    public static final double shooterSpeed = 0.55;
  }

  public static final class IntakeConstants{
    public static final int IntakeMotorID = 8;
    public static final double intakeSpeed = 1.0;

    public static final int HopperMotorID = 11;
    public static final double hopperSpeed = 0.5;

  }


  }


