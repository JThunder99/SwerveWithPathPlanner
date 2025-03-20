// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(26, "");
    private final AlgaeSubsystem algaeSubsystem;
    private final CoralSubsystem coralSubsystem;

    // Track current state to detect changes
    private boolean lastAlgaeLoaded = false;
    private boolean lastCoralLoaded = false;
    private boolean lastShooting = false;
    private boolean lastCoralShooting = false;

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
        boolean algaeShooting = algaeSubsystem.getAlgaeRotationCurrentTarget() == AlgaeSubsystem.kShootingPosition;
        boolean coralShooting = coralSubsystem.getCoralRotationCurrentTarget() == CoralSubsystem.kShootingLevel1Position ||
                                coralSubsystem.getCoralRotationCurrentTarget() == CoralSubsystem.kShootingLevel2Position ||
                                coralSubsystem.getCoralRotationCurrentTarget() == CoralSubsystem.kShootingLevel3Position ||
                                coralSubsystem.getCoralRotationCurrentTarget() == CoralSubsystem.kShootingLevel4Position;

        if (algaeLoaded != lastAlgaeLoaded || coralLoaded != lastCoralLoaded || algaeShooting != lastShooting || coralShooting != lastCoralShooting) {
            candle.clearAnimation(0);
            if (algaeLoaded && coralLoaded) {
                // Both loaded (rare case): Prioritize algae for now, could refine further
                if (algaeShooting) {
                    candle.animate(new com.ctre.phoenix.led.StrobeAnimation(0, 255, 0, 0, 0.1, 8), 0); // Green flash
                } else {
                    candle.setLEDs(0, 255, 0, 0, 0, 8); // Solid green
                }
            } else if (algaeLoaded) {
                if (algaeShooting) {
                    candle.animate(new com.ctre.phoenix.led.StrobeAnimation(0, 255, 0, 0, 0.1, 8), 0); // Green flash
                } else {
                    candle.setLEDs(0, 255, 0, 0, 0, 8); // Solid green
                }
            } else if (coralLoaded) {
                if (coralShooting) {
                    candle.animate(new com.ctre.phoenix.led.StrobeAnimation(255, 165, 0, 0, 0.1, 8), 0); // Orange flash
                } else {
                    candle.setLEDs(255, 165, 0, 0, 0, 8); // Solid orange
                }
            } else {
                candle.setLEDs(0, 0, 0, 0, 0, 8); // Off
            }
            lastAlgaeLoaded = algaeLoaded;
            lastCoralLoaded = coralLoaded;
            lastShooting = algaeShooting; // Update to track algae shooting state
            lastCoralShooting = coralShooting; // Add tracking for coral shooting state
        }
    }
}