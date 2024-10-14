// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.team4646.SmartSubsystem;

public class RobotStatus extends SmartSubsystem
{
    public static class DataCache
    {
        public double distanceRobotToSpeaker;
        public double angleRobotToSpeaker;

        public double distanceRobotToAmp;
        public double angleRobotToAmp;

        public boolean isRed;

        public boolean inIntake;
        public boolean inFeeder;
    }

    private final DigitalInput intakeRight, feeder, intakeMid;

    private final DataCache cache = new DataCache();
    // private Field2d m_field;

    /** Creates a new Diagnostics. */
    public RobotStatus()
    {
        intakeRight = new DigitalInput(Constants.DIGIN.INTAKE_RIGHT);
        feeder = new DigitalInput(Constants.DIGIN.FEEDER);
        intakeMid = new DigitalInput(Constants.DIGIN.INTAKE_MID);

        // field = new Field2d();
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.COMP_DASH_NAME);

        tab.addBoolean("Has Piece", () -> hasPiece()).withPosition(0, 0).withSize(1, 2);
        tab.addBoolean("Allign", () -> isAllignedToSpeaker()).withPosition(1, 0).withSize(1, 2);
        tab.addBoolean("Up To Speed", () -> RobotContainer.SHOOTER.isOnTarget()).withPosition(2, 0).withSize(1, 2);
        tab.addBoolean("Arm On Target", () -> RobotContainer.SHOOTER_ARM.isAtTargetAngle()).withPosition(3, 0).withSize(1, 2);
        tab.addBoolean("Can Shoot", () -> canShoot()).withPosition(4, 0).withSize(2, 2);

        tab.addBoolean("Intake", () -> inIntake()).withPosition(0, 2).withSize(1, 1);
        tab.addBoolean("Feeder", () -> inFeeder()).withPosition(1, 2).withSize(1, 1);
        tab.addBoolean("Vision Valid", () -> RobotContainer.VISION.isTargetPresent()).withPosition(2, 2).withSize(1, 1);

        tab.addNumber("Angle Speaker", () -> cache.angleRobotToSpeaker).withPosition(2, 3).withSize(1, 1);
        tab.addNumber("Distance Speaker", () -> cache.distanceRobotToSpeaker).withPosition(2, 2).withSize(1, 1);

        // m_field = new Field2d();
        // SmartDashboard.putBoolean("in zone", false);
    }

    public void cacheSensors()
    {
        cache.isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        // calculate distance to speaker for reuse
        // Transform2d transform = RobotContainer.ROBOT_STATE.getAngleToPoint(Constants.FIELD.SPEAKER, false);
        cache.angleRobotToSpeaker = RobotContainer.SWERVE.getPose().getRotation().getDegrees() - mirrorAndGetAngle(Constants.FIELD.SPEAKER, false).getDegrees();
        // SmartDashboard.putNumber("Speaker Degrees", cache.angleRobotToSpeaker);
        cache.distanceRobotToSpeaker = mirrorAndGetDistance(Constants.FIELD.SPEAKER);

        cache.angleRobotToAmp = mirrorAndGetAngle(Constants.FIELD.AMP_CLOSE, false).getDegrees();
        // SmartDashboard.putNumber("Speaker Degrees", cache.angleRobotToSpeaker);
        cache.distanceRobotToAmp = mirrorAndGetDistance(Constants.FIELD.AMP_CLOSE);

        // demo: how to use points to make "zones"
        Pose2d[] points =
        {
                Constants.FIELD.STAGE_CENTER, Constants.FIELD.STAGE_LEFT, Constants.FIELD.STAGE_RIGHT,
        };
        // Create and push Field2d to SmartDashboard.
        // SmartDashboard.putBoolean("in zone", inZone(points));

        // demo: how to mark points crudely on the map with trajectories
        // SmartDashboard.putData("StateField", m_field);
        // m_field.getObject("traj").setTrajectory(getCombinedTrajectoryReferencePoints());

        // store cache values so we only read once per loop
        cache.inFeeder = !feeder.get();
        cache.inIntake = !intakeRight.get() || !intakeMid.get();
    }

    @Override
    public void updateHardware()
    {
        // field.setRobotPose(RobotContainer.SWERVE.getState().Pose);
    }

    public boolean canShoot()
    {
        return RobotContainer.SHOOTER.isOnTarget() &&
                hasPiece() &&
                isAllignedToSpeaker() &&
                isInRange() &&
                RobotContainer.SHOOTER_ARM.isAtTargetAngle();
    }

    public boolean inFeeder()
    {
        return cache.inFeeder;
    }

    public boolean inIntake()
    {
        return cache.inIntake;
    }

    public boolean hasPiece()
    {
        return (inFeeder() || inIntake());
    }

    public boolean isInRange()
    {
        return cache.distanceRobotToSpeaker < Constants.SHOOTER.MAXIMUM_DISTANCE && cache.distanceRobotToSpeaker > Constants.SHOOTER.MINIMUM_DISTANCE;
    }

    public boolean isAllignedToSpeaker()
    {
        return Math.abs(cache.angleRobotToSpeaker) < Constants.SWERVE_DRIVE.ALLIGNED_DEGREES;
    }

    public double getAngleToSpeaker()
    {
        return cache.angleRobotToSpeaker;
    }

    /** distance to speaker in meters */
    public double getDistanceToSpeaker()
    {
        return cache.distanceRobotToSpeaker;
        // return getYawAngleToPoint(Constants.FIELD.SPEAKER, false).getTranslation().getNorm();
    }

    /** distance to amp in meters */
    public double getDistanceToAmp()
    {
        return cache.distanceRobotToAmp;
        // return getYawAngleToPoint(Constants.FIELD.AMP_CLOSE, false).getTranslation().getNorm();
    }

    public boolean isOnRedAlliance()
    {
        return cache.isRed;
    }

    public boolean inZone(Pose2d point, Pose2d[] points)
    {
        boolean result = false;
        if (point == null)
        {
            return false;
        }
        for (int i = 0, j = points.length - 1; i < points.length; j = i++)
        {
            if ((points[i].getY() > point.getY()) != (points[j].getY() > point.getY()) &&
                    (point.getX() < (points[j].getX() - points[i].getX()) * (point.getY() - points[i].getY()) / (points[j].getY() - points[i].getY())
                            + points[i].getX()))
            {
                result = !result;
            }
        }
        return result;
    }

    /***
     * Get angle for rotation to a point. Flips point if red.
     * 
     * @param point
     * @param flip
     *            if true, points using the back of the robot (180 deg offset)
     * @return
     */
    public Rotation2d getAngleToPoint(Pose2d point, boolean flip)
    {
        Pose2d swerve = RobotContainer.SWERVE.getPose();

        Rotation2d theta = new Rotation2d(point.getX() - swerve.getX(), point.getY() - swerve.getY());

        if (flip)
        {
            theta = theta.rotateBy(Rotation2d.fromDegrees(180.0));
        }
        return theta;
    }

    public Rotation2d mirrorAndGetAngle(Pose2d point, boolean flip)
    {
        point = flipFieldPointIfRed(point);
        return getAngleToPoint(point, flip);
    }

    /***
     * Get distance to a point. Flips point if red.
     * 
     * @param point
     * @return
     */
    public double getDistanceToPoint(Pose2d point)
    {
        Transform2d coord = RobotContainer.SWERVE.getPose().minus(point);
        return coord.getTranslation().getNorm();
    }

    public double mirrorAndGetDistance(Pose2d point)
    {
        point = flipFieldPointIfRed(point);
        return getDistanceToPoint(point);
    }

    public Pose2d getNudgedTarget(Pose2d desiredTarget)
    {
        ChassisSpeeds speeds = RobotContainer.SWERVE.getState().speeds;
        double rotationSpeed = speeds.omegaRadiansPerSecond; // todo: how to deal with rotation speed?

        Transform2d currentSwerveSpeeds = new Transform2d(speeds.vxMetersPerSecond * Constants.SWERVE_DRIVE.NUDGE_X,
                speeds.vyMetersPerSecond * Constants.SWERVE_DRIVE.NUDGE_Y, new Rotation2d());
        Pose2d adjustedTarget = flipFieldPointIfRed(desiredTarget).transformBy(currentSwerveSpeeds);
        return adjustedTarget;
    }

    public Rotation2d getRotationToNudgedTarget(Pose2d desiredTarget, boolean flip)
    {
        return getAngleToPoint(getNudgedTarget(desiredTarget), flip);
    }

    public Pose2d flipFieldPointIfRed(Pose2d point)
    {
        if (!cache.isRed)
        {
            return point;
        }
        return new Pose2d(Constants.FIELD.FIELD_LENGTH_M - point.getX(), point.getY(), point.getRotation());
    }

    public Trajectory trajectoryPoint(Pose2d point)
    {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                point,
                List.of(),
                point.transformBy(new Transform2d(0.01, 0.01, new Rotation2d(0))),
                new TrajectoryConfig(Units.feetToMeters(1), Units.feetToMeters(1)));

        return trajectory;
    }

    private Trajectory getCombinedTrajectoryReferencePoints()
    {
        // todo: loop this, couldn't figure out a good way to do it, points just wouldn't show up
        Trajectory traj_Center_to_Right = trajectoryPoint(flipFieldPointIfRed(Constants.FIELD.STAGE_CENTER));
        Trajectory traj_Right_to_Left = trajectoryPoint(flipFieldPointIfRed(Constants.FIELD.STAGE_LEFT));
        Trajectory traj_Left_to_Center = trajectoryPoint(flipFieldPointIfRed(Constants.FIELD.STAGE_RIGHT));
        Trajectory traj_Speaker = trajectoryPoint(flipFieldPointIfRed(Constants.FIELD.SPEAKER));
        Trajectory traj_Amp = trajectoryPoint(flipFieldPointIfRed(Constants.FIELD.AMP));
        Trajectory traj_Source_Close = trajectoryPoint(flipFieldPointIfRed(Constants.FIELD.SOURCE_CLOSE));
        Trajectory traj_Source_Far = trajectoryPoint(flipFieldPointIfRed(Constants.FIELD.SOURCE_FAR));
        Trajectory combined = traj_Center_to_Right.concatenate(traj_Right_to_Left).concatenate(traj_Left_to_Center).concatenate(traj_Speaker)
                .concatenate(traj_Amp).concatenate(traj_Source_Close).concatenate(traj_Source_Far);
        return combined;
    }

}
