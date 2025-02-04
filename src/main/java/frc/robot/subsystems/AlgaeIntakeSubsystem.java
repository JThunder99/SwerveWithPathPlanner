// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
// import com.ctre.phoenix6.hardware.TalonFX;

import java.util.Map;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  // private TalonFX intakeMotor = new TalonFX(10);
  // private TalonFX leftIntakeRotationMotor = new TalonFX(11);
  // private TalonFX rightIntakeRotationMotor = new TalonFX(12);

  SparkFlex algaeIntakeMotor = new SparkFlex(13, MotorType.kBrushless);

  SparkFlexConfig algaeintakeMotorConfig = new SparkFlexConfig();

  private CANrange intakeLoadSensor = new CANrange(20);

  public AlgaeIntakeSubsystem() {

    // intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    // leftIntakeRotationMotor.getConfigurator().apply(new TalonFXConfiguration());
    // rightIntakeRotationMotor.getConfigurator().apply(new TalonFXConfiguration());

    intakeLoadSensor.getConfigurator().apply(new CANrangeConfiguration());

    algaeintakeMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    algaeIntakeMotor.configure(algaeintakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // Set the default command for the subsystem so that it runs the PID loop
    // setDefaultCommand(seekAngleSetpointCommand());
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
    algaeIntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/MotorOutput", algaeIntakeMotor.get());

    SmartDashboard.putBoolean("Is Intake Loaded", isIntakeLoaded());

    SmartDashboard.putNumber("Intake Load Sensor Distance", intakeLoadDistance());
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