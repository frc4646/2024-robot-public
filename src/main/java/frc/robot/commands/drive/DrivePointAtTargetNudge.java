// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DrivePointAtTargetNudge extends Command
{
    /** Creates a new DrivePointAtTargetNudge. */
    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE_DRIVE.PID_ROTATION_KP,
            Constants.SWERVE_DRIVE.PID_ROTATION_KI,
            Constants.SWERVE_DRIVE.PID_ROTATION_KD);

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);
    Pose2d desiredTarget;
    boolean flip;
    private Field2d m_field;

    public DrivePointAtTargetNudge(Pose2d target, boolean flipped)
    {
        desiredTarget = target;
        flip = flipped;
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        m_field = new Field2d();
        Shuffleboard.getTab("targeting").add("TargetField", m_field);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // ChassisSpeeds speeds = RobotContainer.SWERVE.getState().speeds;

        // Transform2d currentSwerveSpeeds = new Transform2d(speeds.vxMetersPerSecond * Constants.SWERVE_DRIVE.NUDGE_X,
        // speeds.vyMetersPerSecond * Constants.SWERVE_DRIVE.NUDGE_Y, new Rotation2d());
        // Pose2d adjustedTarget =
        // RobotContainer.ROBOT_STATUS.flipFieldPointIfRed(desiredTarget).transformBy(currentSwerveSpeeds);
        Rotation2d testRot = RobotContainer.ROBOT_STATUS.getRotationToNudgedTarget(desiredTarget, flip);
        m_field.getObject("traj").setTrajectory(RobotContainer.ROBOT_STATUS.trajectoryPoint(RobotContainer.ROBOT_STATUS.getNudgedTarget(desiredTarget)));

        RobotContainer.SWERVE.setControl(pointAtTarget
                .withTargetDirection(testRot)
                .withRotationalDeadband(.05)
                .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
                .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed()));

    }
}
