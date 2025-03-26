// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public SparkFlex climberMotor = new SparkFlex(37, MotorType.kBrushless);

  SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

  public ClimberSubsystem() {

    climberMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

   //#region Control Methods
  public void setClimberSpeed(double speed) {
    climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Motor Output", climberMotor.get());
  }

  /**
   * Sets the climber motor at set speed
   */
  public Command runClimberAtSpeedCommand(double speed) {
    return Commands.runOnce(() -> setClimberSpeed(speed));
  }
}
