// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmCheckDegrees extends Command
{
    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;
    private final boolean customMinMaxWanted;
    private final double customMin;
    private final double customMax;

    /** Wait for Arm to become close to a target angle */
    public ArmCheckDegrees()
    {
        customMinMaxWanted = false;
        customMin = -1.0;
        customMax = -1.0;
    }

    /** Wait for Arm to be between two angles */
    public ArmCheckDegrees(double degreesMin, double degreesMax)
    {
        customMinMaxWanted = true;
        customMin = degreesMin;
        customMax = degreesMax;
    }

    @Override
    public boolean isFinished()
    {
        if (!customMinMaxWanted)
        {
            return arm.checkAngle(5);
        }

        double actual = arm.getAngle();
        return actual < customMax && actual > customMin;
    }
}
