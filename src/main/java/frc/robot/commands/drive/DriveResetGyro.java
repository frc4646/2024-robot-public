package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class DriveResetGyro extends InstantCommand
{
    SwerveRequest lockBrakes = new SwerveRequest.SwerveDriveBrake();

    public DriveResetGyro()
    {
        addRequirements(RobotContainer.SWERVE);
    }

    @Override
    public void execute()
    {
        RobotContainer.SWERVE.seedFieldRelative();
        // RobotContainer.SWERVE.seedFieldRelative(RobotContainer.SWERVE.getLocation());
    }
}
