// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterArm;

public class ArmSetBetweenDegrees extends Command
{
    private static final double EXTRA_DEGREES = 2;

    private final ShooterArm arm = RobotContainer.SHOOTER_ARM;
    private double degreesMin, degreesMax;

    public ArmSetBetweenDegrees(double degreesMinIn, double degreesMaxIn)
    {
        addRequirements(arm);
        degreesMin = degreesMinIn;
        degreesMax = degreesMaxIn;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        double actual = arm.getAngle();
        double setpoint = actual;
        if (actual > degreesMax)
        {
            setpoint = degreesMax;
        }
        else if (actual < degreesMin)
        {
            setpoint = degreesMin;
        }
        Rotation2d rotation = Rotation2d.fromDegrees(setpoint);
        arm.setTargetAngle(rotation);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        double actual = arm.getAngle();
        return actual < (degreesMax + EXTRA_DEGREES) && actual > (degreesMin - EXTRA_DEGREES);
    }
}
