// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(22, ""); // CAN ID 22
    private final AlgaeSubsystem algaeSubsystem;
    private final CoralSubsystem coralSubsystem;

    // Track current state to detect changes
    private boolean lastAlgaeLoaded = false;
    private boolean lastCoralLoaded = false;
    private boolean lastShooting = false;

    public CANdleSubsystem(AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        this.coralSubsystem = coralSubsystem;
        // Initial state: Red briefly, then off
        candle.setLEDs(255, 0, 0, 0, 0, 8);
        candle.clearAnimation(0); // Clear any animation
        candle.setLEDs(0, 0, 0, 0, 0, 8); // Ensure off
    }

    @Override
    public void periodic() {
        boolean algaeLoaded = algaeSubsystem.isAlgaeIntakeLoaded();
        boolean coralLoaded = coralSubsystem.isCoralIntakeLoaded();
        boolean isShooting = algaeSubsystem.getAlgaeRotationCurrentTarget() == AlgaeSubsystem.kShootingPosition ||
                             coralSubsystem.getCoralRotationCurrentTarget() == CoralSubsystem.kShootingPosition;

        // Update CANdle only on state change
        if (algaeLoaded != lastAlgaeLoaded || coralLoaded != lastCoralLoaded || isShooting != lastShooting) {
            candle.clearAnimation(0); // Clear any running animation before new state
            if (algaeLoaded) {
                if (isShooting) {
                    candle.animate(new com.ctre.phoenix.led.StrobeAnimation(0, 255, 0, 0, 0.1, 8), 0);
                } else {
                    candle.setLEDs(0, 255, 0, 0, 0, 8);
                }
            } else if (coralLoaded) {
                if (isShooting) {
                    candle.animate(new com.ctre.phoenix.led.StrobeAnimation(255, 165, 0, 0, 0.1, 8), 0);
                } else {
                    candle.setLEDs(255, 165, 0, 0, 0, 8);
                }
            } else {
                candle.setLEDs(0, 0, 0, 0, 0, 8);
            }

            // Update last states
            lastAlgaeLoaded = algaeLoaded;
            lastCoralLoaded = coralLoaded;
            lastShooting = isShooting;
        }
    }
}