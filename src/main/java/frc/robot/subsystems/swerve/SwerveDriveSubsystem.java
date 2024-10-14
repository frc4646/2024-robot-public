package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.team4646.util.ModifiedSignalLogger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem
{
    private SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private Pose2d poseCache = new Pose2d();

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules)
    {
        super(driveTrainConstants,
                250,
                // VecBuilder.fill(0.5, 0.5, 0.1),
                // VecBuilder.fill(0.2, 0.2, 0.9),
                modules);
        configurePathPlanner();
        setupSysId();
        setupModuleConfigs();
    }

    private void setupModuleConfigs()
    {
        configNeutralMode(NeutralModeValue.Coast);
        for (int i = 0; i < ModuleCount; i++)
        {
            SwerveModule module = getModule(i);

            module.getCANcoder().clearStickyFaults();

            module.getSteerMotor().clearStickyFaults();
            var config = module.getSteerMotor().getConfigurator();
            config.apply(Constants.SWERVE_DRIVE.TURN_CURRENT_LIMIT_CONFIG);

            module.getDriveMotor().clearStickyFaults();
            config = module.getDriveMotor().getConfigurator();
            config.apply(Constants.SWERVE_DRIVE.CURRENT_LIMIT_CONFIG);
            config.apply(Constants.SWERVE_DRIVE.RAMP_RATES);
        }
    }

    private void configurePathPlanner()
    {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations)
        {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> poseCache, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(12.5, 0, 0),
                        new PIDConstants(8, 0, 0),
                        Constants.SWERVE_DRIVE.MaxSpeedInAuto,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> isOnRedAlliance(),
                this); // Subsystem for requirements

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    private boolean isOnRedAlliance()
    {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field during auto only.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
        {
            return alliance.get() == DriverStation.Alliance.Red & !DriverStation.isTeleop();
        }
        return false;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
    {
        return new RunCommand(() ->
        {
            this.setControl(requestSupplier.get());
        }, this);
    }

    public Command getAutoPath(String pathName)
    {
        return new PathPlannerAuto(pathName);
    }

    @Override
    public void simulationPeriodic()
    {
        /* Assume */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    public void cacheSensors()
    {
        poseCache = getState().Pose;
    }

    public Pose2d getPose()
    {
        return poseCache;
    }

    /**
     * Takes the specified location and makes it the current pose for
     * field-relative maneuvers
     *
     * @param location
     *            Pose to make the current pose at.
     */
    @Override
    public void seedFieldRelative(Pose2d location)
    {
        Rotation2d yaw = Rotation2d.fromDegrees(m_pigeon2.getYaw().getValue());

        try
        {
            m_stateLock.writeLock().lock();

            m_odometry.resetPosition(yaw, m_modulePositions, location);
        } finally
        {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Get current speed of the robot
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds()
    {
        return getState().speeds;
    }

    /**
     * Get current velocity of the robot
     */
    public double getCurrentRobotVelocity()
    {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /**
     * Allows overriding the target rotation during Auto Paths.
     * For example, aiming at a target but still following the path.
     * 
     * @return
     */
    public Optional<Rotation2d> getRotationTargetOverride()
    {
        // set this when we want to override rotation
        if (Constants.SWERVE_DRIVE.AUTO_OVERRIDE && RobotContainer.ROBOT_STATUS.hasPiece())
        {
            // Return an optional containing the rotation override (this should be a field relative rotation)
            return Optional.of(RobotContainer.ROBOT_STATUS.getRotationToNudgedTarget(Constants.FIELD.SPEAKER, false));
        }
        else
        {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }

    // public void addVisionObservation(Pose2d pose, double timeInPast, )
    // {
    // // there might not be a need to have this separate from the super class's addVisionMeasurement

    // super.addVisionMeasurement(pose, timeInPast);
    // }

    public void createDashboard()
    {
        SmartDashboard.putData("Swerve Drive", new Sendable()
        {
            @Override
            public void initSendable(SendableBuilder builder)
            {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> getModule(0).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> getModule(0).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> getModule(1).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> getModule(1).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> getModule(2).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> getModule(2).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> getModule(3).getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> getModule(3).getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
            }
        });
    }

    ///////////////////////////////////////////////////
    /////////////////// SysID stuff ///////////////////
    ///////////////////////////////////////////////////

    private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);
    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    private SysIdRoutine m_driveSysIdRoutine;
    private SysIdRoutine m_steerSysIdRoutine;
    private SysIdRoutine m_slipSysIdRoutine;

    private void setupSysId()
    {
        if (Constants.TUNING.SYSID)
        {
            m_driveSysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState("Swerve Drive State")),
                    new SysIdRoutine.Mechanism(
                            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                            null,
                            this));

            m_steerSysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState("Swerve Steer State")),
                    new SysIdRoutine.Mechanism(
                            (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
                            null,
                            this));

            m_slipSysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState("Swerve Slip State")),
                    new SysIdRoutine.Mechanism(
                            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                            null,
                            this));
        }
    }

    public Command runDriveQuasiTest(Direction direction)
    {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(Direction direction)
    {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction)
    {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(Direction direction)
    {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public Command runDriveSlipTest()
    {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }
}
