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
     * - Coral: Adjusted to stowed first, then to final position after elevator/algae (R1: no starting pos when elevator moves, R2: no starting pos if loaded).
     * - Algae Safe: Moves algae to a safe intermediate position (ReefPickup) before elevator moves if needed.
     * - Elevator: Adjusted after coral stowed and algae safe, respecting algae position constraints.
     * - Algae Target: Adjusted after elevator, with Rule R2 (no stow if loaded) and Rule R3 (coral position).
     * - Coral Final: Adjusted last to target position.
     * @param config The target state configuration (coral, algae, elevator setpoints)
     * @return Command to execute the transition
     */
    public Command transitionToStateCommand(RobotStateConfig config) {
        Command debugStart = Commands.runOnce(() -> {
            System.out.println("Transition Start - Algae: " + AlgaeSubsystem.getAlgaeRotationPosition() + 
                ", Elevator: " + ElevatorSubsystem.getElevatorPosition() + 
                ", Coral: " + CoralSubsystem.getCoralRotationPosition() +
                ", Target: " + config.algaeTarget + "/" + config.elevatorTarget + "/" + config.coralTarget);
        });
    
        // Step 1: Move Coral to Stowed First (if elevator will move and coral not already stowed)
        Command adjustCoralToStowed = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> System.out.println("Step: Coral to Stowed (Initial)")),
                CoralSubsystem.setCoralSetpointCommand(CoralSetpoint.kStowedPosition),
                Commands.waitUntil(() -> Math.abs(CoralSubsystem.getCoralRotationPosition() - CoralSubsystem.kStowedPosition) < 0.01).withTimeout(10.0)
            ),
            () -> Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) < 1 ||
                  Math.abs(CoralSubsystem.getCoralRotationPosition() - CoralSubsystem.kStowedPosition) < 0.01
        );
    
        // Step 2: Adjust Algae to Safe Position (ReefPickup) if entering/leaving kShootingPosition or moving from kStowedPosition
        Command adjustAlgaeToSafe = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.either(
                    Commands.sequence(
                        Commands.runOnce(() -> System.out.println("Step: Elevator to Stowed for Algae Move")),
                        ElevatorSubsystem.setSetpointCommand(ElevatorSetpoint.kStowedPosition),
                        Commands.waitUntil(() -> Math.abs(ElevatorSubsystem.getElevatorPosition() - ElevatorSubsystem.kStowedPosition) < 1).withTimeout(10.0)
                    ),
                    Commands.none(),
                    () -> Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kStowedPosition) < 0.01 && 
                          config.algaeTarget != AlgaeSetpoint.kStowedPosition
                ),
                Commands.runOnce(() -> System.out.println("Step: Algae to Safe (ReefPickup)")),
                AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kReefPickupPosition),
                Commands.waitUntil(() -> Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kReefPickupPosition) < 0.01).withTimeout(10.0)
            ),
            () -> {
                boolean elevatorMoving = Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) > 1;
                boolean algaeAtTarget = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - getSetpointValue(config.algaeTarget)) < 0.01;
                boolean algaeEnteringShooting = config.algaeTarget == AlgaeSetpoint.kShootingPosition &&
                                               Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kStowedPosition) < 0.01;
                boolean algaeLeavingShooting = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kShootingPosition) < 0.01 &&
                                              Math.abs(ElevatorSubsystem.getElevatorPosition() - ElevatorSubsystem.kAlgaeShootingPosition) < 1 &&
                                              config.elevatorTarget != ElevatorSetpoint.kAlgaeShootingPosition;
                boolean algaeLeavingStowed = Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kStowedPosition) < 0.01 &&
                                            config.algaeTarget != AlgaeSetpoint.kStowedPosition;
                boolean needsSafeMove = algaeEnteringShooting || algaeLeavingShooting || algaeLeavingStowed;
                System.out.println("Algae Safe Check - Moving: " + elevatorMoving + ", At Target: " + algaeAtTarget + 
                                  ", Entering: " + algaeEnteringShooting + ", Leaving Shooting: " + algaeLeavingShooting + 
                                  ", Leaving Stowed: " + algaeLeavingStowed + ", Needs Safe: " + needsSafeMove);
                return !elevatorMoving || algaeAtTarget || !needsSafeMove;
            }
        );
    
        // Step 3: Adjust Algae to Target (including stowing if needed)
        Command adjustAlgaeToTarget = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> {
                    boolean loaded = AlgaeSubsystem.isAlgaeIntakeLoaded();
                    AlgaeSubsystem.setLoadedLocked(loaded);
                    System.out.println("Step: Algae to Target - Loaded: " + loaded);
                }),
                Commands.either(
                    Commands.sequence(
                        Commands.runOnce(() -> System.out.println("Step: Elevator to Stowed for Algae Stow")),
                        ElevatorSubsystem.setSetpointCommand(ElevatorSetpoint.kStowedPosition),
                        Commands.waitUntil(() -> Math.abs(ElevatorSubsystem.getElevatorPosition() - ElevatorSubsystem.kStowedPosition) < 1).withTimeout(10.0),
                        AlgaeSubsystem.setSetpointCommand(config.algaeTarget == AlgaeSetpoint.kStowedPosition && AlgaeSubsystem.getLoadedLocked() ? AlgaeSetpoint.kReefPickupPosition : config.algaeTarget),
                        Commands.waitUntil(() -> Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - getSetpointValue(config.algaeTarget == AlgaeSetpoint.kStowedPosition && AlgaeSubsystem.getLoadedLocked() ? AlgaeSetpoint.kReefPickupPosition : config.algaeTarget)) < 0.01).withTimeout(10.0)
                    ),
                    Commands.either(
                        Commands.sequence(
                            Commands.runOnce(() -> System.out.println("Step: Elevator to Algae Shooting")),
                            ElevatorSubsystem.setSetpointCommand(ElevatorSetpoint.kAlgaeShootingPosition),
                            Commands.waitUntil(() -> Math.abs(ElevatorSubsystem.getElevatorPosition() - ElevatorSubsystem.kAlgaeShootingPosition) < 1).withTimeout(10.0),
                            AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kShootingPosition)
                        ),
                        AlgaeSubsystem.setSetpointCommand(config.algaeTarget == AlgaeSetpoint.kStowedPosition && AlgaeSubsystem.getLoadedLocked() ? AlgaeSetpoint.kReefPickupPosition : config.algaeTarget),
                        () -> config.algaeTarget == AlgaeSetpoint.kShootingPosition
                    ),
                    () -> config.algaeTarget == AlgaeSetpoint.kStowedPosition && 
                          Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kStowedPosition) >= 0.01
                ),
                Commands.waitUntil(() -> Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - getSetpointValue(config.algaeTarget == AlgaeSetpoint.kStowedPosition && AlgaeSubsystem.getLoadedLocked() ? AlgaeSetpoint.kReefPickupPosition : config.algaeTarget)) < 0.01).withTimeout(10.0)
            ),
            () -> Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - getSetpointValue(config.algaeTarget)) < 0.01
        );
    
        // Step 4: Adjust Elevator to Target (after algae is set)
        Command adjustElevator = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> System.out.println("Step: Elevator to Target")),
                ElevatorSubsystem.setSetpointCommand(config.elevatorTarget),
                Commands.waitUntil(() -> Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) < 1).withTimeout(10.0)
            ),
            () -> Math.abs(ElevatorSubsystem.getElevatorPosition() - getSetpointValue(config.elevatorTarget)) < 1
        );
    
        // Step 5: Move Coral to Final Target (respecting loaded state and algae position)
        Command adjustCoralToFinal = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> {
                    boolean loaded = CoralSubsystem.isCoralIntakeLoaded();
                    if (config.coralTarget == CoralSetpoint.kStartingPosition && loaded) {
                        System.out.println("Transition Error: Coral cannot move to Starting position when loaded - staying at Stowed");
                    }
                }),
                Commands.waitUntil(() -> config.coralTarget != CoralSetpoint.kShootingPosition || 
                                       Math.abs(AlgaeSubsystem.getAlgaeRotationPosition() - AlgaeSubsystem.kShootingPosition) >= 0.01).withTimeout(10.0),
                CoralSubsystem.setCoralSetpointCommand(config.coralTarget == CoralSetpoint.kStartingPosition && CoralSubsystem.isCoralIntakeLoaded() ? CoralSetpoint.kStowedPosition : config.coralTarget),
                Commands.waitUntil(() -> Math.abs(CoralSubsystem.getCoralRotationPosition() - getSetpointValue(config.coralTarget == CoralSetpoint.kStartingPosition && CoralSubsystem.isCoralIntakeLoaded() ? CoralSetpoint.kStowedPosition : config.coralTarget)) < 0.01).withTimeout(10.0)
            ),
            () -> Math.abs(CoralSubsystem.getCoralRotationPosition() - getSetpointValue(config.coralTarget)) < 0.01
        );
    
        return Commands.sequence(
            debugStart,
            adjustCoralToStowed,
            adjustAlgaeToSafe,
            adjustAlgaeToTarget,
            adjustElevator,
            adjustCoralToFinal
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
