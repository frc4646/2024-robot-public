// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* No longer used, used to stop shooter and used as a default command */
public class ShootOff extends Command
{
    private final Shooter shooterSubsystem = RobotContainer.SHOOTER;

    public ShootOff()
    {
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute()
    {
        shooterSubsystem.setPercentOutput(0, 0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
