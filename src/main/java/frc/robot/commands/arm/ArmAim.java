// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotStatus;
import frc.robot.subsystems.ShooterArm;

public class ArmAim extends Command
{
    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;
    private final RobotStatus state = RobotContainer.ROBOT_STATUS;

    public ArmAim()
    {
        this.addRequirements(arm);
    }

    @Override
    public void execute()
    {
        boolean IsSafeToAim = state.hasPiece() && state.getDistanceToSpeaker() < Constants.SHOOTER.MAXIMUM_DISTANCE;

        if (IsSafeToAim)
        {
            double distance = RobotContainer.ROBOT_STATUS.getDistanceToSpeaker();
            double angle = Constants.SHOOTER.MAP.getAngle(distance);
            arm.setTargetAngle(Rotation2d.fromDegrees(angle));
        }
        else
        {
            arm.setTargetAngle(arm.getLastSetpoint());
        }
    }
}
