// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DiagnosticsCommand;
import frc.robot.commands.arm.ArmAim;
import frc.robot.commands.climb.ClimberOff;
import frc.robot.commands.climb.TelescopeClimbVoltage;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.intakeOff;
import frc.robot.commands.shoot.ShootAutoRPM;
import frc.robot.configuration.AutoModeSelector;
import frc.robot.configuration.CommandRegisterer;
import frc.robot.configuration.DriverControlsSticks;
import frc.robot.configuration.IDriverControls;
import frc.robot.configuration.OperatorControls;
import frc.robot.configuration.SysIDControls;
import frc.robot.configuration.swerve.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberTelescope;
import frc.robot.subsystems.DataLogger;
import frc.robot.subsystems.DataLogger.SwerveDL;
import frc.robot.subsystems.DataLogger.VisionDL;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStatus;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.vision.Vision;
import frc.team4646.SmartSubsystem;
import frc.team4646.drivers.Candle;

public class RobotContainer
{
    // subsystems
    public static SwerveDriveSubsystem SWERVE;
    public static Shooter SHOOTER;
    public static Intake INTAKE;
    public static Vision VISION;
    public static Vision VISION_FRONT;
    public static Climber CLIMBER;
    public static RobotStatus ROBOT_STATUS;
    public static Candle CANDLE;
    public static Diagnostics DIAGNOSTICS;
    public static ShooterArm SHOOTER_ARM;
    public static Feeder FEEDER;
    public static ClimberTelescope CLIMBER_TELESCOPE;

    // controls
    public static IDriverControls CONTROLS_DRIVE;
    public static OperatorControls CONTROLS_OPERATOR;

    public AutoModeSelector autoModeSelector;

    private final List<SmartSubsystem> allSubsystems = new ArrayList<SmartSubsystem>();

    private final PowerDistribution pdh;

    // misc
    Telemetry logger = new Telemetry(Constants.SWERVE_DRIVE.MaxSpeed);
    DataLogger dataLogger;

    boolean swerveDashCreated = false;

    public RobotContainer()
    {
        // initialize subsystems
        SWERVE = TunerConstants.getDriveTrain();
        VISION = new Vision("limelight", Vision.VisionPipeline.LOCALIZATION, true);
        VISION_FRONT = new Vision("limelight-front", Vision.VisionPipeline.LOCALIZATION, false);
        INTAKE = new Intake();
        FEEDER = new Feeder();
        SHOOTER = new Shooter();
        SHOOTER_ARM = new ShooterArm();
        // AMP_SCORER = new AmpScorer();
        ROBOT_STATUS = new RobotStatus();
        CANDLE = new Candle(Constants.CAN.CANDLE);
        DIAGNOSTICS = new Diagnostics();
        if (Constants.CLIMBER.ENABLED)
            CLIMBER = new Climber();
        if (Constants.CLIMBER_TELESCOPE.ENABLED)
            CLIMBER_TELESCOPE = new ClimberTelescope();

        allSubsystems.add(VISION);
        allSubsystems.add(VISION_FRONT);
        allSubsystems.add(INTAKE);
        allSubsystems.add(FEEDER);
        allSubsystems.add(SHOOTER);
        allSubsystems.add(SHOOTER_ARM);
        allSubsystems.add(ROBOT_STATUS);
        allSubsystems.add(CANDLE);
        allSubsystems.add(DIAGNOSTICS);
        if (Constants.CLIMBER.ENABLED)
            allSubsystems.add(CLIMBER);
        if (Constants.CLIMBER_TELESCOPE.ENABLED)
            allSubsystems.add(CLIMBER_TELESCOPE);

        // setup controls
        CONTROLS_DRIVE = new DriverControlsSticks();
        CONTROLS_OPERATOR = new OperatorControls();

        // set default commands
        SWERVE.setDefaultCommand(new DriveTeleop());
        SHOOTER.setDefaultCommand(new ShootAutoRPM());
        INTAKE.setDefaultCommand(new intakeOff());
        // AMP_SCORER.setDefaultCommand(new AmpOff());
        SHOOTER_ARM.setDefaultCommand(new ArmAim());
        DIAGNOSTICS.setDefaultCommand(new DiagnosticsCommand());
        if (Constants.CLIMBER.ENABLED)
            CLIMBER.setDefaultCommand(new ClimberOff());

        if (Constants.CLIMBER_TELESCOPE.ENABLED)
            CLIMBER_TELESCOPE.setDefaultCommand(new TelescopeClimbVoltage(0, 0));

        // misc
        if (Utils.isSimulation())
        {
            SWERVE.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        SWERVE.registerTelemetry(logger::telemeterize);

        if (Constants.TUNING.SYSID)
        {
            new SysIDControls();
        }

        // link auto commands to path planner
        CommandRegisterer.Register();

        // register auto modes with Smart Dashboard
        autoModeSelector = new AutoModeSelector();

        pdh = new PowerDistribution(Constants.CAN.POWER_DISTRIBUTION_PANEL, ModuleType.kRev);
        pdh.clearStickyFaults();

        if (Constants.DATA_LOGGING_ENABLED)
        {
            dataLogger = new DataLogger();
            dataLogger.addLogger(new SwerveDL(SWERVE));
            dataLogger.addLogger(new VisionDL("Limelight", VISION));
            dataLogger.addLogger(new VisionDL("LimelightFront", VISION_FRONT));
            allSubsystems.add(dataLogger);
        }
    }

    public void cacheSensors()
    {
        SWERVE.cacheSensors();
        allSubsystems.forEach(SmartSubsystem::cacheSensors);
    }

    public void updateHardware()
    {
        allSubsystems.forEach(SmartSubsystem::updateHardware);
    }

    public void onEnable(boolean isAutonomous)
    {
        allSubsystems.forEach(s -> s.onEnable(isAutonomous));
    }

    public void onDisable()
    {
        allSubsystems.forEach(SmartSubsystem::onDisable);
        // new DriveDisabled().schedule();

        autoModeSelector.reset();
        autoModeSelector.update();
    }

    boolean lastAllianceWasRed = false;

    public void whileDisabled()
    {
        if (autoModeSelector.update() || (lastAllianceWasRed != ROBOT_STATUS.isOnRedAlliance()))
        {
            logger.publishPath(autoModeSelector.getPathPreview());
        }
        allSubsystems.forEach(SmartSubsystem::whileDisabled);

        lastAllianceWasRed = ROBOT_STATUS.isOnRedAlliance();

        CONTROLS_DRIVE.whileDisabled();

        if (swerveDashCreated == false)
        {
            SWERVE.createDashboard();
            swerveDashCreated = true;
        }
    }

    public Command getAutonomousCommand()
    {
        Command autonomousCommand;

        Optional<Command> autoMode = autoModeSelector.getAutoMode();
        if (autoMode.isPresent())
        {
            // System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
            autonomousCommand = autoMode.get();
        }
        else
        {
            System.out.println("NO AUTO MODE SET");
            autonomousCommand = new WaitCommand(1);
        }
        return autonomousCommand;
    }
}
