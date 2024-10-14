// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveAllignToStage extends Command
{

    private final PhoenixPIDController turnPID = new PhoenixPIDController(
            Constants.SWERVE_DRIVE.STAGE_PID_KP,
            Constants.SWERVE_DRIVE.PID_ROTATION_KI,
            Constants.SWERVE_DRIVE.PID_ROTATION_KD);

    FieldCentricFacingAngle pointAtTarget = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);
    private double desiredAngleDegrees;

    public DriveAllignToStage(boolean isLeftStage)
    {
        if (isLeftStage)
        {
            desiredAngleDegrees = Constants.FIELD.STAGE_LEFT_ANGLE;
        } else {
            desiredAngleDegrees = Constants.FIELD.STAGE_RIGHT_ANGLE;
        }
        addRequirements(RobotContainer.SWERVE);
        pointAtTarget.HeadingController = turnPID;
        pointAtTarget.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void execute()
    {
        double allianceAdjustedAngle;
        if (RobotContainer.ROBOT_STATUS.isOnRedAlliance())
        {
            allianceAdjustedAngle = desiredAngleDegrees - 180;
        } else {
            allianceAdjustedAngle = desiredAngleDegrees;
        }
        RobotContainer.SWERVE.setControl(pointAtTarget
                .withTargetDirection(Rotation2d.fromDegrees(allianceAdjustedAngle))
                .withRotationalDeadband(.05)
                .withVelocityX(RobotContainer.CONTROLS_DRIVE.getXSpeed())
                .withVelocityY(RobotContainer.CONTROLS_DRIVE.getYSpeed()));
    }
}
