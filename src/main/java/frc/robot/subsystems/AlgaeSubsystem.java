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
  public SparkFlex algaeIntakeMotor = new SparkFlex(13, MotorType.kBrushless);
  public SparkFlex algaeIntakeRotationMotor = new SparkFlex(14, MotorType.kBrushless);

  SparkFlexConfig algaeIntakeMotorConfig = new SparkFlexConfig();
  SparkFlexConfig algaeRotationMotorConfig = new SparkFlexConfig();

  SparkClosedLoopController algaeRotationLoopController = algaeIntakeRotationMotor.getClosedLoopController();
  RelativeEncoder algaeRotationEncoder = algaeIntakeRotationMotor.getExternalEncoder();

  public static final double kStowedPosition = 0;
  public static final double kGroundPickupPosition = .5;
  public static final double kProcessorPosition = .4;
  public static final double kReefPickupPosition = .5;
  public static final double kShootingPosition = .2;
  public static final double kSafePosition = .45;

  /** Subsystem-wide setpoints */
  public enum AlgaeSetpoint {
    kStowedPosition,
    kGroundPickupPosition,
    kProcessorPosition,
    kReefPickupPosition,
    kShootingPosition,
    kSafePosition
  }

  private double algaeRotationCurrentTarget = kStowedPosition;

  public final CANrange algaeIntakeLoadSensor = new CANrange(20);

  private boolean isLoadedLocked = false; // Track loaded state

  public AlgaeSubsystem() {

    algaeIntakeLoadSensor.getConfigurator().apply(new CANrangeConfiguration());

    algaeIntakeMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    algaeIntakeMotor.configure(algaeIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeRotationMotorConfig
    .inverted(true)
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

    moveToSetpoint();
  }

  public void setLoadedLocked(boolean loaded) {
    isLoadedLocked = loaded;
  }

  public boolean getLoadedLocked() {
    return isLoadedLocked;
  }

  public double getAlgaeRotationCurrentTarget() {
    return algaeRotationCurrentTarget;
  }

  public double getAlgaeRotationPosition() {
    return algaeRotationEncoder.getPosition();
  }

  //#region Control Methods

  public boolean isAlgaeIntakeLoaded() {
    // Assuming a very short distance indicates the intake is loaded
    double distance = algaeIntakeLoadSensor.getDistance().getValueAsDouble();
    boolean isDetected = algaeIntakeLoadSensor.getIsDetected().getValue();
    return distance < .1 && isDetected == true; // Adjust the threshold value as needed
}

  public void runIntake(double speed) {
    algaeIntakeMotor.set(speed);
  }

  public void holdAlgaeIntake() {
    algaeIntakeMotor.set(0.05); // Adjust the hold power as needed
  }

  private void moveToSetpoint() {
    algaeRotationLoopController.setReference(
      algaeRotationCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public double getAlgaeMotorOutput() {
    // Updated to reflect actual control output
    return algaeIntakeRotationMotor.getAppliedOutput(); // Use duty cycle for PositionVoltage
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //moveToSetpoint();

    // Debug current command state
    Command currentCmd = this.getCurrentCommand();
    SmartDashboard.putString("Algae Current Command", currentCmd != null ? currentCmd.getName() : "None");
    SmartDashboard.putNumber("Algae Intake Motor Output", algaeIntakeMotor.get());
    SmartDashboard.putBoolean("Is Algae Intake Loaded", isAlgaeIntakeLoaded());
    SmartDashboard.putNumber("Algae Intake Load Sensor Distance", algaeIntakeLoadSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Algae Target Position", algaeRotationCurrentTarget);
    SmartDashboard.putNumber("Algae Actual Position", algaeRotationEncoder.getPosition());
  }

  /**
   * Sets the rollers at set speed
   */
  public Command runIntakeAtSpeedCommand(double speed) {
    return Commands.startEnd(() -> runIntake(speed), () -> algaeIntakeMotor.stopMotor(), this)
            .withName("RunIntakeAtSpeed");
  }

  /**
   * Sets the rollers to pick up a game piece at half speed
   */
  public Command pickUpGamePieceCommand() {
    return Commands.startEnd(() -> runIntake(.1), () -> {}, this) // No stop action—default command handles it
            .withName("PickUpAlgae");
  }

  /**
   * Sets the rollers to eject a game piece at half speed
   */
  public Command ejectGamePieceCommand() {
    return Commands.startEnd(() -> runIntake(-.1), () -> {}, this) // No stop action—default command handles it
            .withName("EjectAlgae");
  }

  /**
   * Stops the intake motors
   */
  public Command stopIntakeAtSpeedCommand() {
    return Commands.runOnce(() -> algaeIntakeMotor.stopMotor()).withName("StopAlgaeIntake");
  }

  public Command setSetpointCommand(AlgaeSetpoint algaesetpoint) {
    return Commands.run(() -> {
        double target;
        boolean loaded = getLoadedLocked(); // Use locked state for consistency
        switch (algaesetpoint) {
            case kStowedPosition:
                target = loaded ? kSafePosition : kStowedPosition;
                break;
            case kGroundPickupPosition:
                target = kGroundPickupPosition;
                break;
            case kProcessorPosition:
                target = kProcessorPosition;
                break;
            case kReefPickupPosition:
                target = kReefPickupPosition;
                break;
            case kShootingPosition:
                target = kShootingPosition;
                break;
            case kSafePosition:
                target = kSafePosition;
                break;
            default:
                target = kStowedPosition;
        }
        algaeRotationCurrentTarget = target;
        moveToSetpoint();
        System.out.println("AlgaeSubsystem: Setting target to " + target + " (Locked Loaded: " + loaded + ", Live Loaded: " + isAlgaeIntakeLoaded() + ")");
    }, this).until(() -> Math.abs(getAlgaeRotationPosition() - algaeRotationCurrentTarget) < 0.01)
    .withTimeout(15.0) // Increased from 10.0
    .withName("AlgaeTo" + algaesetpoint);
}

// Overload for default (non-continuous)
// public Command setSetpointCommand(AlgaeSetpoint algaesetpoint) {
//   return setSetpointCommand(algaesetpoint, false);
// }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Pick_Up_Algae",
      pickUpGamePieceCommand(),
      "Eject_Algae",
      ejectGamePieceCommand(),
      "Stop_Algae_Intake",
      stopIntakeAtSpeedCommand(),
      "Set_Algae_Setpoint_Stowed",
      setSetpointCommand(AlgaeSetpoint.kStowedPosition),
      "Set_Algae_Setpoint_Ground_Pickup",
      setSetpointCommand(AlgaeSetpoint.kGroundPickupPosition),
      "Set_Algae_Setpoint_Processor",
      setSetpointCommand(AlgaeSetpoint.kProcessorPosition),
      "Set_Algae_Setpoint_Reef_Pickup",
      setSetpointCommand(AlgaeSetpoint.kReefPickupPosition),
      "Set_Algae_Setpoint_Shooting",
      setSetpointCommand(AlgaeSetpoint.kShootingPosition),
      "Set_Algae_Setpoint_Safe", 
      setSetpointCommand(AlgaeSetpoint.kSafePosition)
    );
  }
}