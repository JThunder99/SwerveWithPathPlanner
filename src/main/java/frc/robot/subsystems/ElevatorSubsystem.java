// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  SparkFlex elevator1 = new SparkFlex(11, MotorType.kBrushless);
  SparkFlex elevator2 = new SparkFlex(12, MotorType.kBrushless);

  SparkFlexConfig elevator1Config = new SparkFlexConfig();
  SparkFlexConfig elevator2Config = new SparkFlexConfig();

  private DigitalInput intakeTopLimitSwitch;
  private DigitalInput intakeBottomLimitSwitch;

  SparkClosedLoopController  elevatorClosedLoopController = elevator1.getClosedLoopController();
  RelativeEncoder elevator1Encoder = elevator1.getEncoder();

  public static final int kFeederStation = 0;
  public static final int kLevel1 = 25;
  public static final int kLevel2 = 50;
  public static final int kLevel3 = 75;
  public static final int kLevel4 = 100;

  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4
  }

  private double elevatorCurrentTarget = kFeederStation;
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;

  public ElevatorSubsystem() {

    intakeTopLimitSwitch = new DigitalInput(0);
    intakeBottomLimitSwitch = new DigitalInput(1);

    elevator1Config
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    elevator1Config.encoder
    .positionConversionFactor(1.0);

    elevator1Config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-.5, 5);

        elevator1Config.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1);

    elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevator2Config
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .follow(elevator1, true);

    elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevator1Encoder.setPosition(0);
  }

  //#region Control Methods

  public void setElevatorSpeed(double speed) {
    elevator1.set(speed);
  }

  public void stopElevatorSpeed() {
    elevator1.stopMotor();
  }

  private void moveToSetpoint() {
    elevatorClosedLoopController.setReference(
      elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
      if (!wasResetByLimit && elevator1.getReverseLimitSwitch().isPressed()) {
      //if (!wasResetByLimit && intakeBottomLimitSwitch.get()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevator1Encoder.setPosition(0);
      wasResetByLimit = true;
      } else if (!elevator1.getReverseLimitSwitch().isPressed()) {
      //} else if (!intakeBottomLimitSwitch.get()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      elevator1Encoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  public Command setElevatorSpeedCommand(double speed) {
    return Commands.runOnce(() -> setElevatorSpeed(speed));
  }

  public Command stopElevatorSpeedCommand() {
    return Commands.runOnce(() -> stopElevatorSpeed());
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              elevatorCurrentTarget = kFeederStation;
              break;
            case kLevel1:
              elevatorCurrentTarget = kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = kLevel4;
              break;
          }
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();
    SmartDashboard.putBoolean("Intake Top Limit Switch", intakeTopLimitSwitch.get());
    SmartDashboard.putBoolean("Intake Bottom Limit Switch", intakeBottomLimitSwitch.get());
    SmartDashboard.putBoolean("Rev Intake Bottom Limit Switch",elevator1.getReverseLimitSwitch().isPressed());
    SmartDashboard.putNumber("Elevator Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator Actual Position", elevator1Encoder.getPosition());
  }
}
