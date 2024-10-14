// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveGoToLocation extends Command
{

    private final double X_Y_ACTIVATION_DISTANCE_M = 1.5;

    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE_DRIVE.PID_ROTATION_KP,
            Constants.SWERVE_DRIVE.PID_ROTATION_KI,
            Constants.SWERVE_DRIVE.PID_ROTATION_KD);
    private final PhoenixPIDController driveX_PID = new PhoenixPIDController(
            2, // Constants.SWERVE_DRIVE.PID_TRANSLATION_KP / 3,
            0, // Constants.SWERVE_DRIVE.PID_TRANSLATION_KI,
            0); // Constants.SWERVE_DRIVE.PID_TRANSLATION_KD);
    private final PhoenixPIDController driveY_PID = new PhoenixPIDController(
            2, // Constants.SWERVE_DRIVE.PID_TRANSLATION_KP / 3,
            0, // Constants.SWERVE_DRIVE.PID_TRANSLATION_KI,
            0); // Constants.SWERVE_DRIVE.PID_TRANSLATION_KD);

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

    Pose2d position;
    double x, y;
    double rotation;
    boolean overrideX, overrideY;

    /**
     * 
     * @param targetPose
     * @param overrideX
     *            true to use controller inputs
     * @param overrideY
     *            true to use controller inputs
     */
    public DriveGoToLocation(Pose2d targetPose, boolean overrideX, boolean overrideY)
    {
        this(
                targetPose.getX(),
                targetPose.getY(),
                targetPose.getRotation().getDegrees(),
                overrideX,
                overrideY);
    }

    /**
     * 
     * @param x
     * @param y
     * @param rotation
     * @param overrideX
     *            true to use controller inputs
     * @param overrideY
     *            true to use controller inputs
     */
    public DriveGoToLocation(double x, double y, double rotation, boolean overrideX, boolean overrideY)
    {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.overrideX = overrideX;
        this.overrideY = overrideY;

        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {

        Pose2d tmpPoint = RobotContainer.ROBOT_STATUS.flipFieldPointIfRed(new Pose2d(x, y, new Rotation2d()));
        Pose2d swervePose = RobotContainer.SWERVE.getState().Pose;

        double xVelocity = driveX_PID.calculate(swervePose.getX(), tmpPoint.getX(), Utils.getCurrentTimeSeconds());
        double yVelocity = driveY_PID.calculate(swervePose.getY(), tmpPoint.getY(), Utils.getCurrentTimeSeconds());

        // use direct drive
        // xVelocity = driveX_PID.getPositionError() * 2;
        // yVelocity = driveY_PID.getPositionError() * 2;

        // cap them at the max swerve velocity
        xVelocity = MathUtil.clamp(xVelocity, -Constants.SWERVE_DRIVE.MaxSpeed, Constants.SWERVE_DRIVE.MaxSpeed);
        yVelocity = MathUtil.clamp(yVelocity, -Constants.SWERVE_DRIVE.MaxSpeed, Constants.SWERVE_DRIVE.MaxSpeed);

        // use the controller input if we want to have override control or if our distance is too large
        if (overrideX || Math.abs(driveX_PID.getPositionError()) > X_Y_ACTIVATION_DISTANCE_M)
        {
            xVelocity = RobotContainer.CONTROLS_DRIVE.getXSpeed();
        }
        if (overrideY || Math.abs(driveY_PID.getPositionError()) > X_Y_ACTIVATION_DISTANCE_M)
        {
            yVelocity = RobotContainer.CONTROLS_DRIVE.getYSpeed();
        }

        SmartDashboard.putNumber("DriveGoToLocation Rotation", rotation);
        SmartDashboard.putNumber("DriveGoToLocation xVelocity", xVelocity);
        SmartDashboard.putNumber("DriveGoToLocation yVelocity", yVelocity);

        RobotContainer.SWERVE.setControl(pointAtTarget
                .withRotationalDeadband(0.05)
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withTargetDirection(Rotation2d.fromDegrees(rotation)));
    }
}
