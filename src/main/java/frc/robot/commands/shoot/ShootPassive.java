// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShootPassive extends Command
{
    private static final Shooter shooter = RobotContainer.SHOOTER;

    /** Creates a new ShootPassive. */
    public ShootPassive()
    {
        addRequirements(shooter);
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
        if (RobotContainer.ROBOT_STATUS.getDistanceToSpeaker() < Constants.SHOOTER.MAXIMUM_DISTANCE && RobotContainer.ROBOT_STATUS.hasPiece())
        {
            shooter.setRPM(Constants.SHOOTER.PASSIVE_RPM, Constants.SHOOTER.PASSIVE_RPM);
        }
        else
        {
            shooter.setRPM(0, 0);
        }
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
