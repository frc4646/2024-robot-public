// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmAmpDegrees extends Command
{
    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;

    /** Creates a new ArmAmpDegrees. */
    public ArmAmpDegrees()
    {
        addRequirements(arm);
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
        if (RobotContainer.ROBOT_STATUS.hasPiece())
        {
            arm.setTargetAngle(Rotation2d.fromDegrees(Constants.SHOOTER_ARM.SERVO.AMP_DEGREES));
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
