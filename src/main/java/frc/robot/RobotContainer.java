// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetpoint;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeSetpoint;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralSetpoint;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    //private final CommandXboxController joystick1 = new CommandXboxController(1);
    private final CommandGenericHID operatorControl = new CommandGenericHID(1); // SJ@JX kit on port 1

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public AlgaeSubsystem AlgaeSubsystem;
    public ElevatorSubsystem ElevatorSubsystem;
    public CoralSubsystem CoralSubsystem;
    public CANdleSubsystem CANdleSubsystem;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        AlgaeSubsystem = new AlgaeSubsystem();
        ElevatorSubsystem = new ElevatorSubsystem();
        CoralSubsystem = new CoralSubsystem();
        CANdleSubsystem = new CANdleSubsystem(AlgaeSubsystem, CoralSubsystem);

        // Default command for AlgaeSubsystem
        AlgaeSubsystem.setDefaultCommand(
            Commands.run(() -> {
                if (AlgaeSubsystem.isAlgaeIntakeLoaded()) {
                    AlgaeSubsystem.holdAlgaeIntake();
                    SmartDashboard.putString("Algae Default State", "Holding");
                } else {
                    AlgaeSubsystem.algaeIntakeMotor.stopMotor();
                    SmartDashboard.putString("Algae Default State", "Stopped");
                }
            }, AlgaeSubsystem).withName("AlgaeDefaultHold")
        );

        // Default command for CoralSubsystem
        CoralSubsystem.setDefaultCommand(
            Commands.run(() -> {
                if (CoralSubsystem.isCoralIntakeLoaded()) {
                    CoralSubsystem.holdCoralIntake();
                    SmartDashboard.putString("Coral Default State", "Holding");
                } else {
                    CoralSubsystem.coralIntakeMotor.stopMotor();
                    SmartDashboard.putString("Coral Default State", "Stopped");
                }
            }, CoralSubsystem).withName("CoralDefaultHold")
        );

        // Register the named commands from each subsystem that may be used in PathPlanner
        //NamedCommands.registerCommands(Drivetrain.getNamedCommands());
        NamedCommands.registerCommands(AlgaeSubsystem.getNamedCommands());
        NamedCommands.registerCommands(ElevatorSubsystem.getNamedCommands());
        NamedCommands.registerCommands(CoralSubsystem.getNamedCommands());
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        autoChooser.addOption("Mid_to_mid_reef", getAutonomousCommand());
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
        configureOperatorControls();

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
            
    }

    private void configureOperatorControls() {
        // Modifier buttons
        Trigger algaeModifier = operatorControl.button(1); // Algae modifier button
        Trigger coralModifier = operatorControl.button(2); // Coral modifier button

        // Scenario buttons (3-7 for 10 scenarios)
        operatorControl.button(3).and(algaeModifier).onTrue(transitionToStateCommand(RobotStateConfig.ALGAE_GROUND_PICKUP));
        operatorControl.button(3).and(coralModifier).onTrue(transitionToStateCommand(RobotStateConfig.CORAL_HUMAN_PICKUP));

        operatorControl.button(4).and(algaeModifier).onTrue(transitionToStateCommand(RobotStateConfig.ALGAE_PROCESSOR));
        operatorControl.button(4).and(coralModifier).onTrue(transitionToStateCommand(RobotStateConfig.CORAL_SHOOTING_LEVEL_1));

        operatorControl.button(5).and(algaeModifier).onTrue(transitionToStateCommand(RobotStateConfig.ALGAE_REEF_LEVEL_1_PICKUP));
        operatorControl.button(5).and(coralModifier).onTrue(transitionToStateCommand(RobotStateConfig.CORAL_SHOOTING_LEVEL_2));

        operatorControl.button(6).and(algaeModifier).onTrue(transitionToStateCommand(RobotStateConfig.ALGAE_REEF_LEVEL_2_PICKUP));
        operatorControl.button(6).and(coralModifier).onTrue(transitionToStateCommand(RobotStateConfig.CORAL_SHOOTING_LEVEL_3));

        operatorControl.button(7).and(algaeModifier).onTrue(transitionToStateCommand(RobotStateConfig.ALGAE_SHOOTING));
        operatorControl.button(7).and(coralModifier).onTrue(transitionToStateCommand(RobotStateConfig.CORAL_SHOOTING_LEVEL_4));

        // Intake control buttons (8-9 for 4 intake commands)
        operatorControl.button(8).and(algaeModifier).whileTrue(AlgaeSubsystem.pickUpGamePieceCommand());
        operatorControl.button(8).and(coralModifier).whileTrue(CoralSubsystem.pickUpCoralCommand());

        operatorControl.button(9).and(algaeModifier).whileTrue(AlgaeSubsystem.ejectGamePieceCommand());
        operatorControl.button(9).and(coralModifier).whileTrue(CoralSubsystem.ejectCoralCommand());

        // Dedicated Stow button
        operatorControl.button(10).onTrue(transitionToStateCommand(RobotStateConfig.ROBOT_STOWED));
    }

    /**
     * Transitions the robot to a specified state configuration, enforcing subsystem rules.
     * - Coral: Adjusted first to avoid interference (R1: elevator position, R3: algae position).
     * - Algae Safe: Moves algae to a safe intermediate position (0.5) to prevent Rule R1 violations.
     * - Elevator: Adjusted third, ensuring algae is safe (R1: algae position if at shooting height).
     * - Algae Target: Final adjustment with Rule R2 (no stow if loaded) and Rule R3 (coral position).
     * @param config The target state configuration (coral, algae, elevator setpoints)
     * @return Command to execute the transition
     */
    public Command transitionToStateCommand(RobotStateConfig config) {
        // Step 1: Adjust Coral Intake Position
        // - Ensures coral moves to target while respecting rules
        Command adjustCoral = Commands.either(
            Commands.none(), // Skip if coral is already at target
            Commands.sequence(
                Commands.runOnce(() -> System.out.println("Starting adjustCoral")).ignoringDisable(true),
                // Coral Rule R1: Wait until elevator is at target if coral targets kStartingPosition
                Commands.waitUntil(() -> {
                    boolean condition = config.coralTarget != CoralSetpoint.kStartingPosition ||
                                        Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) < 1;
                    System.out.println("Coral R1 wait: " + condition);
                    return condition;
                }).andThen(
                    Commands.runOnce(() -> {
                        if (config.coralTarget == CoralSetpoint.kStartingPosition &&
                            Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) >= 1) {
                            System.out.println("Moving elevator for Coral R1");
                            ElevatorSubsystem.setSetpointCommand(config.elevatorTarget).execute();
                            SmartDashboard.putString("Transition Info", "Moved elevator to target to allow coral to Starting");
                        }
                    }).ignoringDisable(true)
                ),
                // Check coral loaded state for kStartingPosition
                Commands.runOnce(() -> {
                    if (CoralSubsystem.isCoralIntakeLoaded() && config.coralTarget == CoralSetpoint.kStartingPosition) {
                        SmartDashboard.putString("Transition Error", "Coral cannot move to Starting position when loaded");
                    }
                }).ignoringDisable(true),
                // Coral Rule R3: Move algae to safe position if coral targets kShootingPosition and algae is at kShootingPosition
                Commands.sequence(
                    Commands.runOnce(() -> {
                        if (config.coralTarget == CoralSetpoint.kShootingPosition &&
                            Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kShootingPosition) < 0.01) {
                            System.out.println("Moving algae to safe position for Coral R3");
                        }
                    }).ignoringDisable(true),
                    Commands.either(
                        AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kReefPickupPosition)
                            .andThen(Commands.waitUntil(() -> {
                                boolean condition = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kReefPickupPosition) < 0.01;
                                System.out.println("Algae R3 safe wait: " + condition + " (Algae Pos: " + AlgaeSubsystem.getAlgaeRotationPosition() + ")");
                                return condition;
                            }).withTimeout(5.0)),
                        Commands.none(),
                        () -> config.coralTarget == CoralSetpoint.kShootingPosition &&
                              Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kShootingPosition) < 0.01
                    ),
                    Commands.runOnce(() -> {
                        if (config.coralTarget == CoralSetpoint.kShootingPosition &&
                            Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kShootingPosition) < 0.01) {
                            SmartDashboard.putString("Transition Info", "Moved algae to ReefPickup to allow coral Shooting");
                        }
                    }).ignoringDisable(true)
                ),
                // Wait until algae is clear of kShootingPosition for coral kShootingPosition
                Commands.waitUntil(() -> {
                    boolean condition = config.coralTarget != CoralSetpoint.kShootingPosition ||
                                        Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kShootingPosition) >= 0.01;
                    System.out.println("Coral R3 wait: " + condition);
                    return condition;
                }).withTimeout(5.0),
                // Conditional coral adjustments based on algae target
                Commands.either(
                    CoralSubsystem.setCoralSetpointCommand(config.coralTarget),
                    CoralSubsystem.setCoralSetpointCommand(CoralSetpoint.kStowedPosition),
                    () -> config.algaeTarget != AlgaeSetpoint.kGroundPickupPosition &&
                          config.algaeTarget != AlgaeSetpoint.kReefPickupPosition
                ),
                // Algae adjustment for kHumanPickupPosition (Rule R5)
                Commands.either(
                    Commands.none(),
                    Commands.sequence(
                        AlgaeSubsystem.setSetpointCommand(
                            AlgaeSubsystem.isAlgaeIntakeLoaded() ? AlgaeSetpoint.kReefPickupPosition : AlgaeSetpoint.kStowedPosition
                        ),
                        Commands.waitUntil(() -> {
                            boolean condition = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - 
                                (AlgaeSubsystem.isAlgaeIntakeLoaded() ? AlgaeSubsystem.kReefPickupPosition : AlgaeSubsystem.kStowedPosition)) < 0.01;
                            System.out.println("Algae coral human wait: " + condition + " (Algae Pos: " + AlgaeSubsystem.getAlgaeRotationPosition() + ")");
                            return condition;
                        }).withTimeout(5.0)
                    ),
                    () -> config.coralTarget != CoralSetpoint.kHumanPickupPosition ||
                          Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kStowedPosition) < 0.01
                ),
                Commands.either(
                    CoralSubsystem.setCoralSetpointCommand(config.coralTarget),
                    CoralSubsystem.setCoralSetpointCommand(CoralSetpoint.kStowedPosition),
                    () -> config.algaeTarget != AlgaeSetpoint.kProcessorPosition
                ),
                // Final coral move to target
                CoralSubsystem.setCoralSetpointCommand(config.coralTarget),
                Commands.waitUntil(() -> {
                    boolean condition = Math.abs(CoralSubsystem.getCoralRotationPosition() - getSetpointValue(config.coralTarget)) < 0.01;
                    return condition;
                }),
                Commands.runOnce(() -> System.out.println("Finished adjustCoral")).ignoringDisable(true)
            ),
            () -> Math.abs(CoralSubsystem.getCoralRotationPosition() - getSetpointValue(config.coralTarget)) < 0.01
        );

        // Step 2: Adjust Algae to Safe Position
        // - Moves algae to kReefPickupPosition (0.5) to avoid Rule R1 violations during elevator movement
        Command adjustAlgaeToSafe = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> System.out.println("Starting adjustAlgaeToSafe")).ignoringDisable(true),
                AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kReefPickupPosition),
                Commands.waitUntil(() -> {
                    boolean condition = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kReefPickupPosition) < 0.01;
                    System.out.println("Algae safe wait: " + condition + " (Algae Pos: " + AlgaeSubsystem.getAlgaeRotationPosition() + ")");
                    return condition;
                }).withTimeout(5.0),
                Commands.runOnce(() -> System.out.println("Finished adjustAlgaeToSafe")).ignoringDisable(true)
            ),
            () -> Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kReefPickupPosition) < 0.01 ||
                  (Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - getSetpointValue(config.algaeTarget)) < 0.01 &&
                   Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) < 1)
        );

        // Step 3: Adjust Elevator Position
        // - Moves elevator to target, ensuring algae is safe if moving away from kAlgaeShootingPosition
        Command adjustElevator = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> {
                    boolean needsSafeMove = Math.abs(ElevatorSubsystem.getElevatorPosition() - ElevatorSubsystem.kAlgaeShootingPosition) < 1 &&
                                            config.elevatorTarget != ElevatorSetpoint.kAlgaeShootingPosition &&
                                            Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kShootingPosition) < 0.1;
                    System.out.println("Starting adjustElevator, Elevator R1 check: " + needsSafeMove + " (Current Algae Pos: " + AlgaeSubsystem.getAlgaeRotationPosition() + ")");
                    if (needsSafeMove) {
                        System.out.println("Moving algae to ReefPickup");
                        SmartDashboard.putString("Transition Info", "Moved algae to Reef Pickup to unblock elevator from Algae Shooting");
                    }
                    SmartDashboard.putBoolean("NeedsSafeMoveDebug", needsSafeMove);
                }).ignoringDisable(true),
                // Elevator Rule R1: Move algae to safe position if needed
                Commands.either(
                    AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kReefPickupPosition)
                        .andThen(Commands.runOnce(() -> {
                            System.out.println("Post-set algae target: " + AlgaeSubsystem.getAlgaeRotationCurrentTarget());
                            SmartDashboard.putNumber("Debug Algae Target", AlgaeSubsystem.getAlgaeRotationCurrentTarget());
                        }).ignoringDisable(true)),
                    Commands.none(),
                    () -> SmartDashboard.getBoolean("NeedsSafeMoveDebug", false)
                ),
                Commands.waitUntil(() -> {
                    boolean needsSafeMove = SmartDashboard.getBoolean("NeedsSafeMoveDebug", false);
                    boolean algaeSafe = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kReefPickupPosition) < 0.01;
                    boolean condition = !needsSafeMove || algaeSafe;
                    System.out.println("Elevator R1 wait: " + condition + " (Algae Pos: " + AlgaeSubsystem.getAlgaeRotationPosition() + ", Target: " + AlgaeSubsystem.getAlgaeRotationCurrentTarget() + ", InitialNeedsSafeMove: " + needsSafeMove + ")");
                    return condition;
                }).withTimeout(5.0),
                Commands.runOnce(() -> System.out.println("Setting elevator target")).ignoringDisable(true),
                ElevatorSubsystem.setSetpointCommand(config.elevatorTarget),
                Commands.waitUntil(() -> {
                    boolean condition = Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) < 1;
                    System.out.println("Elevator target wait: " + condition + " (Elevator Pos: " + ElevatorSubsystem.getElevatorPosition() + ", Target: " + ElevatorSubsystem.getElevatorCurrentTarget() + ")");
                    return condition;
                }).withTimeout(5.0),
                Commands.runOnce(() -> System.out.println("Finished adjustElevator")).ignoringDisable(true)
            ),
            () -> Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) < 1
        );

        // Step 4: Adjust Algae to Final Target
        // - Moves algae to final target with Rule R2 enforcement (no stow if loaded)
        Command adjustAlgaeToTarget = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> System.out.println("Starting adjustAlgaeToTarget")).ignoringDisable(true),
                // Algae Rule R1: Wait until elevator is at kStowedPosition if targeting kStowedPosition
                Commands.waitUntil(() -> {
                    boolean condition = config.algaeTarget != AlgaeSetpoint.kStowedPosition ||
                                        Math.abs(ElevatorSubsystem.getElevatorPosition() - ElevatorSubsystem.kStowedPosition) < 1;
                    System.out.println("Algae R1 wait: " + condition);
                    return condition;
                }).andThen(
                    Commands.runOnce(() -> {
                        if (config.algaeTarget == AlgaeSetpoint.kStowedPosition &&
                            Math.abs(ElevatorSubsystem.getElevatorPosition() - ElevatorSubsystem.kStowedPosition) >= 1) {
                            System.out.println("Moving elevator for Algae R1");
                            ElevatorSubsystem.setSetpointCommand(ElevatorSetpoint.kStowedPosition).execute();
                            SmartDashboard.putString("Transition Info", "Moved elevator to Stowed to allow algae to Stow");
                        }
                    }).ignoringDisable(true)
                ),
                // Algae Rule R2: Warn if attempting to stow while loaded
                Commands.runOnce(() -> {
                    if (config.algaeTarget == AlgaeSetpoint.kStowedPosition && AlgaeSubsystem.isAlgaeIntakeLoaded()) {
                        SmartDashboard.putString("Transition Error", "Algae cannot move to Stowed when loaded");
                    }
                }).ignoringDisable(true),
                // Algae Rule R3: Wait until coral is clear if targeting kShootingPosition
                Commands.waitUntil(() -> {
                    boolean condition = config.algaeTarget != AlgaeSetpoint.kShootingPosition ||
                                        Math.abs(CoralSubsystem.getCoralRotationPosition() - CoralSubsystem.kShootingPosition) >= 0.01;
                    System.out.println("Algae R3 wait: " + condition);
                    return condition;
                }).andThen(
                    Commands.runOnce(() -> {
                        if (config.algaeTarget == AlgaeSetpoint.kShootingPosition &&
                            Math.abs(CoralSubsystem.getCoralRotationPosition() - CoralSubsystem.kShootingPosition) < 0.01) {
                            System.out.println("Moving coral for Algae R3");
                            CoralSubsystem.setCoralSetpointCommand(CoralSetpoint.kStowedPosition).execute();
                            SmartDashboard.putString("Transition Info", "Moved coral to Stowed to allow algae Shooting");
                        }
                    }).ignoringDisable(true)
                ),
                // Set and lock algae loaded state, apply initial setpoint
                Commands.runOnce(() -> {
                    boolean loaded = AlgaeSubsystem.isAlgaeIntakeLoaded();
                    System.out.println("Setting algae target");
                    System.out.println("Algae loaded: " + loaded + " (Distance: " + 
                        AlgaeSubsystem.algaeIntakeLoadSensor.getDistance().getValueAsDouble() + ", Detected: " + 
                        AlgaeSubsystem.algaeIntakeLoadSensor.getIsDetected().getValue() + ")");
                    AlgaeSubsystem.setLoadedLocked(loaded); // Lock state for consistency
                    AlgaeSetpoint target = config.algaeTarget == AlgaeSetpoint.kStowedPosition && loaded
                        ? AlgaeSetpoint.kReefPickupPosition : config.algaeTarget;
                    AlgaeSubsystem.setSetpointCommand(target).execute();
                }).ignoringDisable(true),
                // Continuously enforce setpoint for 5s, then wait until position matches
                Commands.sequence(
                    Commands.run(() -> {
                        boolean loaded = AlgaeSubsystem.getLoadedLocked();
                        AlgaeSetpoint target = config.algaeTarget == AlgaeSetpoint.kStowedPosition && loaded
                            ? AlgaeSetpoint.kReefPickupPosition : config.algaeTarget;
                        AlgaeSubsystem.setSetpointCommand(target, true).execute();
                        System.out.println("Continuous algae setpoint: " + getSetpointValue(target));
                    }).withTimeout(5.0),
                    Commands.waitUntil(() -> {
                        boolean loaded = AlgaeSubsystem.getLoadedLocked();
                        AlgaeSetpoint target = config.algaeTarget == AlgaeSetpoint.kStowedPosition && loaded
                            ? AlgaeSetpoint.kReefPickupPosition : config.algaeTarget;
                        double targetValue = getSetpointValue(target);
                        boolean condition = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - targetValue) < 0.01;
                        System.out.println("Algae target wait: " + condition + " (Algae Pos: " + 
                            AlgaeSubsystem.getAlgaeRotationPosition() + ", Target: " + targetValue + ")");
                        return condition;
                    }).withTimeout(5.0)
                ),
                Commands.runOnce(() -> System.out.println("Finished adjustAlgaeToTarget")).ignoringDisable(true)
            ),
            () -> Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - getSetpointValue(config.algaeTarget)) < 0.01
        );

        // Execute the full transition sequence
        return Commands.sequence(
            adjustCoral,
            adjustAlgaeToSafe,
            adjustElevator,
            adjustAlgaeToTarget
        ).handleInterrupt(() -> System.out.println("State transition interrupted"));
    }

    // Maps setpoint enums to their numerical values (uses subsystem constants)
    private double getSetpointValue(Enum<?> setpoint) {
        if (setpoint instanceof CoralSetpoint) {
            return switch ((CoralSetpoint) setpoint) {
                case kStartingPosition -> CoralSubsystem.kStartingPosition;
                case kStowedPosition -> CoralSubsystem.kStowedPosition;
                case kHumanPickupPosition -> CoralSubsystem.kHumanPickupPosition;
                case kShootingPosition -> CoralSubsystem.kShootingPosition;
            };
        } else if (setpoint instanceof AlgaeSetpoint) {
            return switch ((AlgaeSetpoint) setpoint) {
                case kStowedPosition -> AlgaeSubsystem.kStowedPosition;
                case kGroundPickupPosition -> AlgaeSubsystem.kGroundPickupPosition;
                case kProcessorPosition -> AlgaeSubsystem.kProcessorPosition;
                case kReefPickupPosition -> AlgaeSubsystem.kReefPickupPosition;
                case kShootingPosition -> AlgaeSubsystem.kShootingPosition;
            };
        } else if (setpoint instanceof ElevatorSetpoint) {
            return switch ((ElevatorSetpoint) setpoint) {
                case kStowedPosition -> ElevatorSubsystem.kStowedPosition;
                case kGroundPosition -> ElevatorSubsystem.kGroundPosition;
                case kFeederStation -> ElevatorSubsystem.kFeederStation;
                case kLevel1 -> ElevatorSubsystem.kLevel1;
                case kLevel2 -> ElevatorSubsystem.kLevel2;
                case kLevel3 -> ElevatorSubsystem.kLevel3;
                case kLevel4 -> ElevatorSubsystem.kLevel4;
                case kAlgaeShootingPosition -> ElevatorSubsystem.kAlgaeShootingPosition;
            };
        }
        return 0; // Default fallback
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
