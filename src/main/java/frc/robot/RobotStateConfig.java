// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.AlgaeSubsystem.AlgaeSetpoint;
import frc.robot.subsystems.CoralSubsystem.CoralSetpoint;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetpoint;

public class RobotStateConfig {
    public final CoralSetpoint coralTarget;
    public final AlgaeSetpoint algaeTarget;
    public final ElevatorSetpoint elevatorTarget;

    public RobotStateConfig(
        CoralSetpoint coralTarget,
        AlgaeSetpoint algaeTarget,
        ElevatorSetpoint elevatorTarget
    ) {
        this.coralTarget = coralTarget;
        this.algaeTarget = algaeTarget;
        this.elevatorTarget = elevatorTarget;
    }

    public static final RobotStateConfig ALGAE_GROUND_PICKUP = new RobotStateConfig(
        CoralSetpoint.kStowedPosition,
        AlgaeSetpoint.kGroundPickupPosition,
        ElevatorSetpoint.kGroundPosition
    );

    public static final RobotStateConfig ALGAE_PROCESSOR = new RobotStateConfig(
        CoralSetpoint.kStowedPosition,
        AlgaeSetpoint.kProcessorPosition,
        ElevatorSetpoint.kStowedPosition
    );

    public static final RobotStateConfig ALGAE_REEF_LEVEL_1_PICKUP = new RobotStateConfig(
        CoralSetpoint.kStowedPosition,
        AlgaeSetpoint.kReefPickupPosition,
        ElevatorSetpoint.kLevel1
    );

    public static final RobotStateConfig ALGAE_REEF_LEVEL_2_PICKUP = new RobotStateConfig(
        CoralSetpoint.kStowedPosition,
        AlgaeSetpoint.kReefPickupPosition,
        ElevatorSetpoint.kLevel2
    );

    public static final RobotStateConfig ALGAE_SHOOTING = new RobotStateConfig(
        CoralSetpoint.kStowedPosition,
        AlgaeSetpoint.kShootingPosition,
        ElevatorSetpoint.kAlgaeShootingPosition
    );

    public static final RobotStateConfig CORAL_HUMAN_PICKUP = new RobotStateConfig(
        CoralSetpoint.kHumanPickupPosition,
        AlgaeSetpoint.kStowedPosition,
        ElevatorSetpoint.kStowedPosition
    );

    public static final RobotStateConfig CORAL_SHOOTING_LEVEL_1 = new RobotStateConfig(
        CoralSetpoint.kShootingPosition,
        AlgaeSetpoint.kStowedPosition,
        ElevatorSetpoint.kLevel1
    );

    public static final RobotStateConfig CORAL_SHOOTING_LEVEL_2 = new RobotStateConfig(
        CoralSetpoint.kShootingPosition,
        AlgaeSetpoint.kReefPickupPosition,
        ElevatorSetpoint.kLevel2
    );

    public static final RobotStateConfig CORAL_SHOOTING_LEVEL_3 = new RobotStateConfig(
        CoralSetpoint.kShootingPosition,
        AlgaeSetpoint.kReefPickupPosition,
        ElevatorSetpoint.kLevel3
    );

    public static final RobotStateConfig CORAL_SHOOTING_LEVEL_4 = new RobotStateConfig(
        CoralSetpoint.kShootingPosition,
        AlgaeSetpoint.kReefPickupPosition,
        ElevatorSetpoint.kLevel4
    );

    public static final RobotStateConfig ROBOT_STOWED = new RobotStateConfig(
        CoralSetpoint.kStowedPosition,
        AlgaeSetpoint.kStowedPosition,
        ElevatorSetpoint.kStowedPosition
    );
}