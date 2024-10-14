// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmLaunchAtAmp extends Command
{

    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;

    public ArmLaunchAtAmp()
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
        double distance_m = RobotContainer.ROBOT_STATUS.getDistanceToAmp();
        // Rotation2d angle = Rotation2d.fromDegrees(Constants.SHOOTER.MAP.getAngle(distance_m));
        Rotation2d angle = Rotation2d.fromDegrees(Constants.SHOOTER_ARM.AMP_STOCKPILE_ANGLE);
        arm.setTargetAngle(angle);
        // double radians = Math.atan(Constants.SHOOTER_ARM.Y_PIVOT_TO_SPEAKER_M / distance_m);
        // Rotation2d rotation = new Rotation2d(radians);
        // arm.setTargetAngle(rotation);
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
