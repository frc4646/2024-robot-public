// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmDegrees extends Command
{

    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;
    private double degrees;

    public ArmDegrees(double degreesIn)
    {
        addRequirements(arm);
        degrees = degreesIn;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        Rotation2d rotation = Rotation2d.fromDegrees(degrees);
        arm.setTargetAngle(rotation);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
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
