    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SysIDShooter extends SubsystemBase {
  // The motor on the shooter wheel .
    private final TalonFX shooterFlywheelMotor = new TalonFX(ShooterConstants.ShooterFlywheelMotorID);
    private final TalonFX shooterMotor = new TalonFX(ShooterConstants.ShooterMotorID2);
    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
    
public SysIDShooter() {
     setName("Flywheel");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        // Set any necessary configs in the Feedback group here
        shooterFlywheelMotor.getConfigurator().apply(cfg);

        /* Speed up signals for better characterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            shooterFlywheelMotor.getPosition(),
            shooterFlywheelMotor.getVelocity(),
            shooterFlywheelMotor.getMotorVoltage());

        /* Optimize out the other signals, since they're not useful for SysId */
        // shooterFlywheelMotor.optimizeBusUtilization();

    // var ShooterMotor2Configuration = new TalonFXConfiguration();
    //    ShooterMotor2Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //    ShooterMotor2Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //    shooterMotor.getConfigurator().apply( ShooterMotor2Configuration );

    // var ShooterMotorFlywheelConfiguration = new TalonFXConfiguration();
    //    ShooterMotorFlywheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //    ShooterMotorFlywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //    shooterFlywheelMotor.getConfigurator().apply( ShooterMotorFlywheelConfiguration );

        shooterMotor.setControl(new Follower(ShooterConstants.ShooterMotorID2, MotorAlignmentValue.Opposed));

}


private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> shooterFlywheelMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

    public Command joystickDriveCommand(DoubleSupplier output) {
    return run(() -> shooterFlywheelMotor.setControl(m_joystickControl.withOutput(output.getAsDouble())));
}

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.dynamic(direction);
   }
}