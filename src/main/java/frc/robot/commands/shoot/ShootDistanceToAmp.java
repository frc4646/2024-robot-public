// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* TODO Not done yet, change RPM based on distance to the speaker */
public class ShootDistanceToAmp extends Command
{
    private final Shooter shooterSubsystem = RobotContainer.SHOOTER;

    public ShootDistanceToAmp()
    {
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute()
    {
        shooterSubsystem.setRPM(Constants.SHOOTER.AMP_LAUNCH_RPM, Constants.SHOOTER.AMP_LAUNCH_RPM * Constants.SHOOTER.SPIN_PERCENT);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
