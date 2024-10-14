// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmPointAtSpeaker extends Command
{

    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;

    public ArmPointAtSpeaker()
    {
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        double distance_m = RobotContainer.ROBOT_STATUS.getDistanceToSpeaker();
        Rotation2d angle = Rotation2d.fromDegrees(Constants.SHOOTER.MAP.getAngle(distance_m));
        // Pose2d swervePose = RobotContainer.SWERVE.getState().Pose;
        // Transform2d distanceToShooter = new Transform2d(0.5, 0, swervePose.getRotation());
        // Pose2d shooterFront = RobotContainer.SWERVE.getState().Pose.transformBy(distanceToShooter);

        // Pose2d[] SpeakerPoints =
        // {
        // Constants.FIELD.STAGE_CENTER, Constants.FIELD.STAGE_LEFT, Constants.FIELD.STAGE_RIGHT
        // };
        // if (RobotContainer.ROBOT_STATE.inZone(shooterFront, SpeakerPoints))
        // {
        // angle = new Rotation2d(Constants.SHOOTER_ARM.SERVO.DEGREES_MIN);
        // }
        arm.setTargetAngle(angle);

        // double radians = Math.atan(Constants.SHOOTER_ARM.Y_PIVOT_TO_SPEAKER_M / distance_m);
        // Rotation2d rotation = new Rotation2d(radians);
        // arm.setTargetAngle(rotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
