// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* Sets percent output for shootermotors based on smartdashboard */
public class ShootFromSmartDashboard extends Command
{
    private final Shooter shooter = RobotContainer.SHOOTER;
    private double percentL, percentR;

    public ShootFromSmartDashboard()
    {
        addRequirements(shooter);
        percentL = SmartDashboard.getNumber("L Percent Output -1 to 1", Constants.SHOOTER.DEFAULT_PERCENT_SMARTDASH);
        SmartDashboard.putNumber("L Percent Output -1 to 1", Constants.SHOOTER.DEFAULT_PERCENT_SMARTDASH);
        percentR = SmartDashboard.getNumber("R Percent Output -1 to 1", Constants.SHOOTER.DEFAULT_PERCENT_SMARTDASH);
        SmartDashboard.putNumber("R Percent Output -1 to 1", Constants.SHOOTER.DEFAULT_PERCENT_SMARTDASH);
    }

    @Override
    public void execute()
    {
        percentL = SmartDashboard.getNumber("L Percent Output -1 to 1", percentL);

        percentR = SmartDashboard.getNumber("R Percent Output -1 to 1", percentR);
        shooter.setPercentOutput(percentL, percentR);

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
