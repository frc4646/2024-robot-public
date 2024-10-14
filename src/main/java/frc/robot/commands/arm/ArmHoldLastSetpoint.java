// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmHoldLastSetpoint extends Command
{
    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;

    public ArmHoldLastSetpoint()
    {
        addRequirements(arm);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        arm.setTargetAngle(arm.getLastSetpoint());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}