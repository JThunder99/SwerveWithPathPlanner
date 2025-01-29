// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.Map;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private TalonFX intakeMotor = new TalonFX(10);
  private TalonFX leftIntakeRotationMotor = new TalonFX(11);
  private TalonFX rightIntakeRotationMotor = new TalonFX(12);

  private CANrange intakeLoadSensor = new CANrange(20);

  private DigitalInput intakeTopLimitSwitch;
  private DigitalInput intakeBottomLimitSwitch;

  private PIDController intakeAnglePid;
  private double intakeAngleStartPoint;
  public boolean intakeAngleToggledIn;
  private Debouncer intakeAngleToggleDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
  public static final int PositionDelta = 49;

  public IntakeSubsystem() {

    intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    leftIntakeRotationMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightIntakeRotationMotor.getConfigurator().apply(new TalonFXConfiguration());

    intakeLoadSensor.getConfigurator().apply(new CANrangeConfiguration());
    
    intakeTopLimitSwitch = new DigitalInput(0);
    intakeBottomLimitSwitch = new DigitalInput(1);

    intakeAngleStartPoint = getPositionRight();
    intakeAnglePid = new PIDController(0.2, 0.0, 0.0);
    intakeAnglePid.setSetpoint(intakeAngleStartPoint);
    intakeAngleToggledIn = true;
    

    // Set the default command for the subsystem so that it runs the PID loop
    setDefaultCommand(seekAngleSetpointCommand());
  }

  //#region Control Methods

  public boolean isIntakeLoaded() {
    // Assuming a very short distance indicates the intake is loaded
    double distance = intakeLoadSensor.getDistance().getValueAsDouble();
    boolean isDetected = intakeLoadSensor.getIsDetected().getValue();
    return distance < .1 && isDetected == true; // Adjust the threshold value as needed
}

public double intakeLoadDistance() {
  // Distance value from intake load sensor
  double distance = intakeLoadSensor.getDistance().getValueAsDouble();
  return distance;
}

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Gets the current position of the Intake Angle from the left Kraken's encoder
   * @return
   */
  public double getPositionLeft() {
    return leftIntakeRotationMotor.getRotorPosition().getValueAsDouble();
  }

  /**
   * Gets the current position of the Intake Angle from the right Kraken's encoder
   * @return
   */
  public double getPositionRight() {
    return rightIntakeRotationMotor.getRotorPosition().getValueAsDouble();
  }

  /**
   * Sets the speed of the Intake Angle Motors
   * @param speed
   */
  public void setAngleMotorSpeed(double speed) {
    leftIntakeRotationMotor.set(-speed);
    rightIntakeRotationMotor.set(speed);
  }

  /**
   * Sets the Intake Angle to a given position in rotations of the motor shaft
   * @param positionSetpoint
   */
  public void setIntakeRotation() {
    var currentPosition = getPositionRight();
    var setpoint = intakeAngleToggledIn ? intakeAngleStartPoint : (intakeAngleStartPoint - PositionDelta);
    SmartDashboard.putNumber("Intake/AngleSetpoint", setpoint);

    var pidOutput = intakeAnglePid.calculate(currentPosition, setpoint);
    SmartDashboard.putNumber("Intake/AnglePIDOutput", pidOutput);

    // artificial limits
    if (currentPosition < intakeAngleStartPoint && pidOutput > 0 && !intakeTopLimitSwitch.get()) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 1));
    } else if (
      currentPosition > (intakeAngleStartPoint - PositionDelta) && pidOutput < 0 && !intakeBottomLimitSwitch.get()
    ) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, -1, 0));
    } else {
      setAngleMotorSpeed(0);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake/ToggledIn", intakeAngleToggledIn);

    SmartDashboard.putNumber("Intake/ArmPositionRight", getPositionRight());
    SmartDashboard.putNumber("Intake/ArmPositionLeft", getPositionLeft());

    SmartDashboard.putNumber("Intake/LeftMotorOutput", leftIntakeRotationMotor.get());
    SmartDashboard.putNumber("Intake/RightMotorOutput", rightIntakeRotationMotor.get());

    SmartDashboard.putNumber("Intake/MotorOutput", intakeMotor.get());

    SmartDashboard.putBoolean("Is Intake Loaded", isIntakeLoaded());

    SmartDashboard.putNumber("Intake Load Sensor Distance", intakeLoadDistance());

    SmartDashboard.putBoolean("Intake Top Limit Switch", intakeTopLimitSwitch.get());
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
    return Commands.runOnce(() -> intakeMotor.stopMotor());
  }

  /**
   * Constantly seeks the angle setpoint for the intake. If interrupted, stops the rotate intake motors
   * @return
   */
  public Command seekAngleSetpointCommand() {
    return this.run(() -> setIntakeRotation()).finallyDo(() -> stopArmMotorsCommand());
  }

  /**
   * Sets the intake angle to scoring position
   */
  public Command setIntakeInCommand() {
    return Commands.runOnce(() -> intakeAngleToggledIn = true);
  }

  /**
   * Sets the intake angle setpoint to ground position
   */
  public Command setIntakeOutCommand() {
    return Commands.runOnce(() -> intakeAngleToggledIn = false);
  }

  /**
   * Toggles the intake angle setpoint between in/out
   * @return
   */
  public Command toggleIntakeInAndOutCommand() {
    return Commands.runOnce(() -> intakeAngleToggledIn = !intakeAngleToggleDebouncer.calculate(intakeAngleToggledIn));
  }

  /**
   * Stops the intake rotate motors
   * @return
   */
  public Command stopArmMotorsCommand() {
    return Commands.runOnce(() -> {
      leftIntakeRotationMotor.stopMotor();
      rightIntakeRotationMotor.stopMotor();
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Rotate_Intake_Out",
      setIntakeOutCommand(),
      "Rotate_Intake_In",
      setIntakeInCommand(),
      "Pick_Up_Game_Piece",
      pickUpGamePieceCommand(),
      "Eject_Game_Piece",
      ejectGamePieceCommand(),
      "Stop_Intake",
      stopIntakeAtSpeedCommand()
    );
  }
}