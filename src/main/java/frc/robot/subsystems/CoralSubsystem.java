// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANrange;
import java.util.Map;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */

  public SparkMax coralIntakeMotor = new SparkMax(15, MotorType.kBrushless);
  SparkFlex coralIntakeRotationMotor = new SparkFlex(16, MotorType.kBrushless);

  SparkMaxConfig coralMotorConfig = new SparkMaxConfig();
  SparkFlexConfig coralRotationMotorConfig = new SparkFlexConfig();

  SparkClosedLoopController coralRotationLoopController = coralIntakeRotationMotor.getClosedLoopController();
  RelativeEncoder coralRotationEncoder = coralIntakeRotationMotor.getExternalEncoder();

  public static final double kStartingPosition = 0;
  public static final double kStowedPosition = .05;
  public static final double kHumanPickupPosition = .1;
  public static final double kShootingPosition = .3;

  /** Subsystem-wide setpoints */
  public enum CoralSetpoint {
    kStartingPosition,
    kStowedPosition,
    kHumanPickupPosition,
    kShootingPosition
  }

  private double coralRotationCurrentTarget = kStartingPosition;

  private CANrange coralIntakeLoadSensor = new CANrange(21);


  public CoralSubsystem() {

    coralIntakeLoadSensor.getConfigurator().apply(new CANrangeConfiguration());

    coralMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    coralIntakeMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralRotationMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    coralRotationMotorConfig.encoder
    .positionConversionFactor(1.0);

    coralRotationMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-.05, .05);

        coralRotationMotorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(.01);

    coralIntakeRotationMotor.configure(coralRotationMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralRotationEncoder.setPosition(0);

  }

  public double getCoralRotationCurrentTarget() {
    return coralRotationCurrentTarget;
  }

  public double getCoralRotationPosition() {
    return coralRotationEncoder.getPosition();
  }

  //#region Control Methods

  public boolean isCoralIntakeLoaded() {
    // Assuming a very short distance indicates the intake is loaded
    double distance = coralIntakeLoadSensor.getDistance().getValueAsDouble();
    boolean isDetected = coralIntakeLoadSensor.getIsDetected().getValue();
    return distance < .1 && isDetected == true; // Adjust the threshold value as needed
  }

  public void runCoralIntake(double speed) {
    coralIntakeMotor.set(speed);
  }

  public void holdCoralIntake() {
    coralIntakeMotor.set(0.15); // Adjust the hold power as needed
  }

  private void moveCoralToSetpoint() {
    coralRotationLoopController.setReference(
      coralRotationCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  //#region Periodic

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveCoralToSetpoint();

    // Debug current command state
    Command currentCmd = this.getCurrentCommand();
    SmartDashboard.putString("Coral Current Command", currentCmd != null ? currentCmd.getName() : "None");
    SmartDashboard.putNumber("Coral Intake Motor Output", coralIntakeMotor.get());
    SmartDashboard.putBoolean("Is Coral Intake Loaded", isCoralIntakeLoaded());
    SmartDashboard.putNumber("Coral Intake Load Sensor Distance", coralIntakeLoadSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Coral Target Position", coralRotationCurrentTarget);
    SmartDashboard.putNumber("Coral Actual Position", coralRotationEncoder.getPosition());

  }

  //#region Commands

  /**
   * Sets the rollers at set speed
   */
  public Command runCoralIntakeAtSpeedCommand(double speed) {
    return Commands.startEnd(() -> runCoralIntake(speed), () -> coralIntakeMotor.stopMotor(), this)
            .withName("RunCoralIntakeAtSpeed");
  }

  /**
   * Sets the rollers to pick up a game piece at half speed
   */
  public Command pickUpCoralCommand() {
    return Commands.startEnd(() -> runCoralIntake(.5), () -> {}, this) // No stop action—default command handles it
            .withName("PickUpCoral");
  }

  /**
   * Sets the rollers to eject a game piece at half speed
   */
  public Command ejectCoralCommand() {
    return Commands.startEnd(() -> runCoralIntake(-.5), () -> {}, this) // No stop action—default command handles it
            .withName("EjectCoral");
  }

  /**
   * Stops the intake motors
   */
  public Command stopCoralIntakeAtSpeedCommand() {
    return Commands.runOnce(() -> coralIntakeMotor.stopMotor()).withName("StopCoralIntake");
  }

  /**
   * Command to set the subsystem setpoint. This will set the coral intake to the predefined
   * position for the given setpoint.
   */
  public Command setCoralSetpointCommand(CoralSetpoint coralSetpoint) {
    return this.runOnce(
        () -> {
          switch (coralSetpoint) {
            case kStartingPosition:
            coralRotationCurrentTarget = kStartingPosition;
              break;
            case kStowedPosition:
            coralRotationCurrentTarget = kStowedPosition;
              break;
            case kHumanPickupPosition:
            coralRotationCurrentTarget = kHumanPickupPosition;
              break;
            case kShootingPosition:
            coralRotationCurrentTarget = kShootingPosition;
              break;
          }
        });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Pick_Up_Coral",
      pickUpCoralCommand(),
      "Eject_Coral",
      ejectCoralCommand(),
      "Stop_Coral_Intake",
      stopCoralIntakeAtSpeedCommand(),
      "Set_Coral_Setpoint_Stowed",
      setCoralSetpointCommand(CoralSetpoint.kStowedPosition),
      "Set_Coral_Setpoint_Human_Pickup",
      setCoralSetpointCommand(CoralSetpoint.kHumanPickupPosition),
      "Set_Coral_Setpoint_Shooting",
      setCoralSetpointCommand(CoralSetpoint.kShootingPosition)
    );
  }
}
