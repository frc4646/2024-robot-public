// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* Shoots notes based on rpm for motors */
public class ShootRPM extends Command
{
    private final Shooter shooterSubsystem = RobotContainer.SHOOTER;
    private double rpmL = 4000, rpmR = 5000;

    public ShootRPM(double rpmInL, double rpmInR)
    {
        addRequirements(shooterSubsystem);
        if (Constants.TUNING.SHOOTER)
        {
            // Default values for left and right motors so note flies smooth
            SmartDashboard.putNumber("RPM In L", Constants.SHOOTER.DEFAULT_RPM_SMARTDASH);
            SmartDashboard.putNumber("RPM In R", Constants.SHOOTER.DEFAULT_RPM_SMARTDASH);
            rpmL = SmartDashboard.getNumber("RPM In L", Constants.SHOOTER.DEFAULT_RPM_SMARTDASH);
            // SmartDashboard.putNumber("RPM L", rpmL);
            rpmR = SmartDashboard.getNumber("RPM In R", Constants.SHOOTER.DEFAULT_RPM_SMARTDASH);
            // SmartDashboard.putNumber("RPM R", rpmR);
        }

        rpmL = rpmInL;
        rpmR = rpmInR;
    }

    @Override
    public void execute()
    {
        if (Constants.TUNING.SHOOTER)
        {
            rpmL = SmartDashboard.getNumber("RPM In L", Constants.SHOOTER.PASSIVE_RPM);
            rpmR = SmartDashboard.getNumber("RPM In R", Constants.SHOOTER.PASSIVE_RPM);
        }
        shooterSubsystem.setRPM(rpmL, rpmR);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
