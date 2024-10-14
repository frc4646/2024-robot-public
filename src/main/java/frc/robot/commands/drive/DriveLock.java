package frc.robot.commands.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveLock extends Command
{
    SwerveRequest lockBrakes = new SwerveRequest.SwerveDriveBrake();

    public DriveLock()
    {
        addRequirements(RobotContainer.SWERVE);
    }

    @Override
    public void execute()
    {
        RobotContainer.SWERVE.setControl(lockBrakes);
    }
}
