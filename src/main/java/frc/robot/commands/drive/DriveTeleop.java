// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTeleop extends Command
{

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE_DRIVE.PID_ROTATION_KP,
            Constants.SWERVE_DRIVE.PID_ROTATION_KI,
            Constants.SWERVE_DRIVE.PID_ROTATION_KD);

    public DriveTeleop()
    {
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute()
    {
        double distance = RobotContainer.ROBOT_STATUS.getDistanceToSpeaker();
        //boolean ready = distance < Constants.SHOOTER.MAXIMUM_DISTANCE && RobotContainer.ROBOT_STATUS.hasPiece() && distance > 2;
        //if (ready)
        //{
        //    Rotation2d testRot = RobotContainer.ROBOT_STATUS.mirrorAndGetAngle(Constants.FIELD.SPEAKER, false);
//
        //SmartDashboard.putNumber("Point At Target Angle", testRot.getDegrees());
//
        //RobotContainer.SWERVE.setControl(pointAtTarget
        //        .withTargetDirection(testRot)
        //        .withRotationalDeadband(.05)
        //        .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
        //        .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed()));
        //} else {
            RobotContainer.SWERVE.setControl(drive
                    .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
                    .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed())
                    .withRotationalRate(RobotContainer.CONTROLS_DRIVE.getRotationRate()));
        //}
    }
}
