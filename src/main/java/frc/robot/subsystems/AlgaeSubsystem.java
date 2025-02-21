// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import java.util.Map;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  SparkFlex algaeIntakeMotor = new SparkFlex(13, MotorType.kBrushless);
  SparkFlex algaeIntakeRotationMotor = new SparkFlex(14, MotorType.kBrushless);

  SparkFlexConfig algaeMotorConfig = new SparkFlexConfig();
  SparkFlexConfig algaeRotationMotorConfig = new SparkFlexConfig();

  SparkClosedLoopController algaeRotationLoopController = algaeIntakeRotationMotor.getClosedLoopController();
  RelativeEncoder algaeRotationEncoder = algaeIntakeRotationMotor.getExternalEncoder();

  public static final double kStoredPosition = 0;
  public static final double kGroundPickupPosition = .8;
  public static final double kReefPickupPosition = .5;
  public static final double kShootingPosition = .2;

  /** Subsystem-wide setpoints */
  public enum AlgaeSetpoint {
    kStoredPosition,
    kGroundPickupPosition,
    kReefPickupPosition,
    kShootingPosition
  }

  private double algaeRotationCurrentTarget = kStoredPosition;

  private CANrange intakeLoadSensor = new CANrange(20);

  public AlgaeSubsystem() {

    intakeLoadSensor.getConfigurator().apply(new CANrangeConfiguration());

    algaeMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    algaeIntakeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeRotationMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    algaeRotationMotorConfig.encoder
    .positionConversionFactor(1.0);

    algaeRotationMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-.05, .05);

        algaeRotationMotorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(.01);

    algaeIntakeRotationMotor.configure(algaeRotationMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeRotationEncoder.setPosition(0);
  }

  //#region Control Methods

  public boolean isIntakeLoaded() {
    // Assuming a very short distance indicates the intake is loaded
    double distance = intakeLoadSensor.getDistance().getValueAsDouble();
    boolean isDetected = intakeLoadSensor.getIsDetected().getValue();
    return distance < .1 && isDetected == true; // Adjust the threshold value as needed
}

// public double intakeLoadDistance() {
//   // Distance value from intake load sensor
//   double distance = intakeLoadSensor.getDistance().getValueAsDouble();
//   return distance;
// }

  public void runIntake(double speed) {
    algaeIntakeMotor.set(speed);
  }

  private void moveToSetpoint() {
    algaeRotationLoopController.setReference(
      algaeRotationCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToSetpoint();

    SmartDashboard.putNumber("Intake/MotorOutput", algaeIntakeMotor.get());

    SmartDashboard.putBoolean("Is Intake Loaded", isIntakeLoaded());

    SmartDashboard.putNumber("Intake Load Sensor Distance", intakeLoadSensor.getDistance().getValueAsDouble());

    SmartDashboard.putNumber("Algae Target Position", algaeRotationCurrentTarget);
    SmartDashboard.putNumber("Algae Actual Position", algaeRotationEncoder.getPosition());
  }

  /**
   * Sets the rollers at set speed
   */
  public Command runIntakeAtSpeedCommand(double speed) {
    return Commands.runOnce(() -> runIntake(speed));
  }

  /**
   * Sets the rollers to pick up a game piece at half speed
   */
  public Command pickUpGamePieceCommand() {
    return Commands.runOnce(() -> runIntake(.5));
  }

  /**
   * Sets the rollers to eject a game piece at half speed
   */
  public Command ejectGamePieceCommand() {
    return Commands.runOnce(() -> runIntake(-.5));
  }

  /**
   * Stops the intake motors
   */
  public Command stopIntakeAtSpeedCommand() {
    return Commands.runOnce(() -> algaeIntakeMotor.stopMotor());
  }

  /**
   * Command to set the subsystem setpoint. This will set the algae claw to the predefined
   * position for the given setpoint.
   */
  public Command setSetpointCommand(AlgaeSetpoint algaesetpoint) {
    return this.runOnce(
        () -> {
          switch (algaesetpoint) {
            case kStoredPosition:
            algaeRotationCurrentTarget = kStoredPosition;
              break;
            case kGroundPickupPosition:
            algaeRotationCurrentTarget = kGroundPickupPosition;
              break;
            case kReefPickupPosition:
            algaeRotationCurrentTarget = kReefPickupPosition;
              break;
            case kShootingPosition:
            algaeRotationCurrentTarget = kShootingPosition;
              break;
          }
        });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Pick_Up_Game_Piece",
      pickUpGamePieceCommand(),
      "Eject_Game_Piece",
      ejectGamePieceCommand(),
      "Stop_Intake",
      stopIntakeAtSpeedCommand()
    );
  }
}