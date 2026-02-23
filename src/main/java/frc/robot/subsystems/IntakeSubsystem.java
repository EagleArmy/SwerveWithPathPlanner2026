package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.IntakeMotorID);
    public final double intakeSpeed = IntakeConstants.intakeSpeed;
    public IntakeSubsystem() 
    {
       addChild("intakeMotor", intakeMotor);

       var intakeMotorConfiguration = new TalonFXConfiguration();
       intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       intakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       intakeMotor.getConfigurator().apply( intakeMotorConfiguration );

      var slot0ConfigsLeft = intakeMotorConfiguration.Slot0;
    slot0ConfigsLeft.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsLeft.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsLeft.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsLeft.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsLeft.kI = 0; // no output for integrated error
    slot0ConfigsLeft.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    //  set Motion Magic settings
    var motionMagicConfigsLeft = intakeMotorConfiguration.MotionMagic;
    motionMagicConfigsLeft.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsLeft.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsLeft.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

    intakeMotor.getConfigurator().apply(intakeMotorConfiguration);

    }

    public void start()
    {
        intakeMotor.set(-intakeSpeed);
        System.out.println("intake Speed: " + intakeSpeed);
    }

    public void reverse() 
    {
        intakeMotor.set(intakeSpeed);
    }

    public void stop()
    {
        intakeMotor.set( 0 );
    }
    // public void increasetestingspeed() 
    // {
    //     hopperspeed += 0.05;
    //     System.out.println("Testing Speed: " + hopperspeed);
    // }
    // public void decreasetestingspeed() 
    // {
    //     hopperspeed -= 0.05;
    //     System.out.println("Testing Speed: " + hopperspeed);
    // }

    @Override
    public void periodic() 
    {
      //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
      //log();
    }
}
