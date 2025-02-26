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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeSetpoint;
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

    private final CommandXboxController joystick1 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public AlgaeSubsystem AlgaeSubsystem;
    public ElevatorSubsystem ElevatorSubsystem;
    public CoralSubsystem CoralSubsystem;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        AlgaeSubsystem = new AlgaeSubsystem();
        ElevatorSubsystem = new ElevatorSubsystem();
        CoralSubsystem = new CoralSubsystem();

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
        // Configure your button bindings here
        // joystick1.a().whileTrue(AlgaeIntakeSubsystem.runIntakeAtSpeedCommand(0.1))
        // .onFalse(AlgaeIntakeSubsystem.stopIntakeAtSpeedCommand());
        
        // joystick1.b().whileTrue(AlgaeIntakeSubsystem.runIntakeAtSpeedCommand(-0.1))
        // .onFalse(AlgaeIntakeSubsystem.stopIntakeAtSpeedCommand());

        // joystick1.y().whileTrue(ElevatorSubsystem.setElevatorSpeedCommand(0.1))
        // .onFalse(ElevatorSubsystem.stopElevatorSpeedCommand());

        // joystick1.x().whileTrue(ElevatorSubsystem.setElevatorSpeedCommand(-0.1))
        // .onFalse(ElevatorSubsystem.stopElevatorSpeedCommand());

        // B Button -> Elevator/Arm to level 2 position
        joystick1.b().onTrue(ElevatorSubsystem.setSetpointCommand(Setpoint.kFeederStation));

         // A Button -> Elevator/Arm to level 2 position
         joystick1.a().onTrue(ElevatorSubsystem.setSetpointCommand(Setpoint.kLevel2));

        // X Button -> Elevator/Arm to level 3 position
        joystick1.x().onTrue(ElevatorSubsystem.setSetpointCommand(Setpoint.kLevel3));

        // Y Button -> Elevator/Arm to level 4 position
        joystick1.y().onTrue(ElevatorSubsystem.setSetpointCommand(Setpoint.kLevel4));

        joystick1.povLeft().onTrue(AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kStowedPosition));

        joystick1.povDown().onTrue(AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kGroundPickupPosition));

        joystick1.povRight().onTrue(AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kReefPickupPosition));

        joystick1.povUp().onTrue(AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kShootingPosition));

    }

    /**
     * Transitions the robot to a specified state configuration, enforcing all subsystem rules.
     * @param config The target state configuration
     * @return Command to execute the transition
     */
    public Command transitionToStateCommand(RobotStateConfig config) {
        Command adjustCoral = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> {
                    if (config.coralTarget == CoralSetpoint.kStartingPosition &&
                        ElevatorSubsystem.getElevatorCurrentTarget() != getSetpointValue(config.elevatorTarget)) {
                        throw new IllegalStateException("Coral cannot move to starting position while elevator is moving");
                    }
                }).ignoringDisable(true),
                Commands.runOnce(() -> {
                    if (CoralSubsystem.isCoralIntakeLoaded() && config.coralTarget == CoralSetpoint.kStartingPosition) {
                        throw new IllegalStateException("Coral cannot move to starting position when loaded");
                    }
                }).ignoringDisable(true),
                Commands.runOnce(() -> {
                    if (config.coralTarget == CoralSetpoint.kShootingPosition &&
                        config.algaeTarget == AlgaeSetpoint.kShootingPosition) {
                        throw new IllegalStateException("Coral and algae cannot both be in shooting positions simultaneously");
                    }
                }).ignoringDisable(true),
                Commands.either(
                    CoralSubsystem.setCoralSetpointCommand(config.coralTarget),
                    CoralSubsystem.setCoralSetpointCommand(CoralSetpoint.kStowedPosition),
                    () -> config.algaeTarget != AlgaeSetpoint.kGroundPickupPosition &&
                          config.algaeTarget != AlgaeSetpoint.kReefPickupPosition
                ),
                Commands.either(
                    Commands.none(),
                    AlgaeSubsystem.setSetpointCommand(AlgaeSetpoint.kStowedPosition),
                    () -> config.coralTarget != CoralSetpoint.kHumanPickupPosition ||
                          Math.abs(AlgaeSubsystem.getAlgaeRotationCurrentTarget() - AlgaeSubsystem.kStowedPosition) < 0.01
                ),
                Commands.either(
                    CoralSubsystem.setCoralSetpointCommand(config.coralTarget),
                    CoralSubsystem.setCoralSetpointCommand(CoralSetpoint.kStowedPosition),
                    () -> config.algaeTarget != AlgaeSetpoint.kProcessorPosition
                ),
                CoralSubsystem.setCoralSetpointCommand(config.coralTarget)
            ),
            () -> Math.abs(CoralSubsystem.getCoralRotationCurrentTarget() - getSetpointValue(config.coralTarget)) < 0.01
        );

        Command adjustElevator = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> {
                    if (Math.abs(ElevatorSubsystem.getElevatorCurrentTarget() - ElevatorSubsystem.kAlgaeShootingPosition) < 1 &&
                        config.elevatorTarget != Setpoint.kAlgaeShootingPosition &&
                        (Math.abs(AlgaeSubsystem.getAlgaeRotationCurrentTarget() - AlgaeSubsystem.kStowedPosition) < 0.01 ||
                         Math.abs(AlgaeSubsystem.getAlgaeRotationCurrentTarget() - AlgaeSubsystem.kShootingPosition) < 0.01)) {
                        throw new IllegalStateException("Elevator cannot move from algae shooting until algae is not stowed or shooting");
                    }
                }).ignoringDisable(true),
                Commands.either(
                    ElevatorSubsystem.setSetpointCommand(config.elevatorTarget),
                    Commands.waitUntil(() -> 
                        CoralSubsystem.getCoralRotationCurrentTarget() != CoralSubsystem.kStartingPosition &&
                        AlgaeSubsystem.getAlgaeRotationCurrentTarget() != AlgaeSubsystem.kStowedPosition &&
                        AlgaeSubsystem.getAlgaeRotationCurrentTarget() != AlgaeSubsystem.kShootingPosition
                    ).andThen(ElevatorSubsystem.setSetpointCommand(config.elevatorTarget)),
                    () -> CoralSubsystem.getCoralRotationCurrentTarget() != CoralSubsystem.kStartingPosition &&
                          AlgaeSubsystem.getAlgaeRotationCurrentTarget() != AlgaeSubsystem.kStowedPosition &&
                          AlgaeSubsystem.getAlgaeRotationCurrentTarget() != AlgaeSubsystem.kShootingPosition
                )
            ),
            () -> Math.abs(ElevatorSubsystem.getElevatorCurrentTarget() - getSetpointValue(config.elevatorTarget)) < 1
        );

        Command adjustAlgae = Commands.either(
            Commands.none(),
            Commands.sequence(
                Commands.runOnce(() -> {
                    if (config.algaeTarget == AlgaeSetpoint.kStowedPosition) {
                        if (config.elevatorTarget != Setpoint.kStowedPosition ||
                            ElevatorSubsystem.getElevatorCurrentTarget() != ElevatorSubsystem.kStowedPosition) {
                            throw new IllegalStateException("Algae cannot move to stowed unless elevator is stowed");
                        }
                        if (AlgaeSubsystem.isAlgaeIntakeLoaded()) {
                            throw new IllegalStateException("Algae cannot move to stowed when loaded");
                        }
                    }
                }).ignoringDisable(true),
                Commands.runOnce(() -> {
                    if (config.algaeTarget == AlgaeSetpoint.kShootingPosition &&
                        config.coralTarget == CoralSetpoint.kShootingPosition) {
                        throw new IllegalStateException("Algae and coral cannot both be in shooting positions simultaneously");
                    }
                }).ignoringDisable(true),
                Commands.either(
                    Commands.none(),
                    ElevatorSubsystem.setSetpointCommand(Setpoint.kAlgaeShootingPosition),
                    () -> config.algaeTarget != AlgaeSetpoint.kShootingPosition ||
                          Math.abs(ElevatorSubsystem.getElevatorCurrentTarget() - ElevatorSubsystem.kAlgaeShootingPosition) < 1
                ),
                AlgaeSubsystem.setSetpointCommand(config.algaeTarget)
            ),
            () -> Math.abs(AlgaeSubsystem.getAlgaeRotationCurrentTarget() - getSetpointValue(config.algaeTarget)) < 0.01
        );

        // Updated to replicate deadlineWith behavior without using it
        return Commands.parallel(
            adjustCoral,
            Commands.sequence(
                Commands.parallel(
                    adjustElevator,
                    adjustAlgae.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
                ).until(() -> adjustElevator.isFinished()), // Ends when adjustElevator finishes
                adjustAlgae.until(() -> Math.abs(AlgaeSubsystem.getAlgaeRotationCurrentTarget() - getSetpointValue(config.algaeTarget)) < 0.01)
            )
        ).handleInterrupt(() -> System.out.println("State transition interrupted"));
    }

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
        } else if (setpoint instanceof Setpoint) {
            return switch ((Setpoint) setpoint) {
                case kStowedPosition -> ElevatorSubsystem.kStowedPosition;
                case kFeederStation -> ElevatorSubsystem.kFeederStation;
                case kLevel1 -> ElevatorSubsystem.kLevel1;
                case kLevel2 -> ElevatorSubsystem.kLevel2;
                case kLevel3 -> ElevatorSubsystem.kLevel3;
                case kLevel4 -> ElevatorSubsystem.kLevel4;
                case kAlgaeShootingPosition -> ElevatorSubsystem.kAlgaeShootingPosition;
            };
        }
        return 0;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
