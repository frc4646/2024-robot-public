// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* Shooter default command so motors can get up to speed faster */
public class ShootAmp extends Command
{
    private final Shooter shooter = RobotContainer.SHOOTER;

    public ShootAmp()
    {
        addRequirements(shooter);
    }

    @Override
    public void execute()
    {
        // disabling this for now due to battery usage
        shooter.setRPM(Constants.SHOOTER.AMP_SCORE_RPM, Constants.SHOOTER.AMP_SCORE_RPM);
        // shooter.setPercentOutput(0, 0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
