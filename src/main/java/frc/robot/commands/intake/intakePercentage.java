// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/* Runs intake using given percentage */
public class intakePercentage extends Command
{
    private double percentage;

    private final Intake intake = RobotContainer.INTAKE;
    private final Feeder feeder = RobotContainer.FEEDER;

    public intakePercentage(double percent)
    {
        addRequirements(intake);
        addRequirements(feeder);
        percentage = percent;
    }

    @Override
    public void execute()
    {
        intake.setOutput(percentage);
        feeder.setOutput(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
