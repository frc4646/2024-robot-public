// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShootDistance extends Command
{
    private final Shooter shooterSubsystem = RobotContainer.SHOOTER;

    public ShootDistance()
    {
        addRequirements(shooterSubsystem);
        SmartDashboard.putNumber("RPM Percentage", Constants.SHOOTER.SPIN_PERCENT);
    }

    @Override
    public void execute()
    {
        double distance = RobotContainer.ROBOT_STATUS.getDistanceToSpeaker();
    //    double rpmR = Constants.SHOOTER.MAP.getRPMRight(distance);
        double rpmL = Constants.SHOOTER.MAP.getRPMLeft(distance);
        double rpmR = rpmL * SmartDashboard.getNumber("RPM Percentage", Constants.SHOOTER.SPIN_PERCENT);
        shooterSubsystem.setRPM(rpmL, rpmR);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
