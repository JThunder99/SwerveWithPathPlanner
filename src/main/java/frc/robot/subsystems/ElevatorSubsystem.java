// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  SparkFlex elevator1 = new SparkFlex(11, MotorType.kBrushless);
  SparkFlex elevator2 = new SparkFlex(12, MotorType.kBrushless);

  SparkFlexConfig elevator1Config = new SparkFlexConfig();
  SparkFlexConfig elevator2Config = new SparkFlexConfig();

  private DigitalInput intakeTopLimitSwitch;
  private DigitalInput intakeBottomLimitSwitch;

  public ElevatorSubsystem() {

    intakeTopLimitSwitch = new DigitalInput(0);
    intakeBottomLimitSwitch = new DigitalInput(1);

    elevator1Config
    .inverted(false)
    .idleMode(IdleMode.kBrake);

    elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevator2Config
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .follow(elevator1, true);

    elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //#region Control Methods

  public void setElevatorSpeed(double speed) {
    elevator1.set(speed);
  }

  public void stopElevatorSpeed() {
    elevator1.stopMotor();
  }

  public Command setElevatorSpeedCommand(double speed) {
    return Commands.runOnce(() -> setElevatorSpeed(speed));
  }

  public Command stopElevatorSpeedCommand() {
    return Commands.runOnce(() -> stopElevatorSpeed());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Top Limit Switch", intakeTopLimitSwitch.get());
    SmartDashboard.putBoolean("Intake Bottom Limit Switch", intakeBottomLimitSwitch.get());
  }
}
