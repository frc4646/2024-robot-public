// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/* Runs both the feeder and the intake motors */
public class feederAndIntakePercentage extends Command
{
    double intakeSpeed;
    double feederSpeed;

    private final Intake intake = RobotContainer.INTAKE;
    private final Feeder feeder = RobotContainer.FEEDER;

    public feederAndIntakePercentage(double percent)
    {
        intakeSpeed = percent;
        feederSpeed = percent;
        addRequirements(intake);
        addRequirements(feeder);
    }

    public feederAndIntakePercentage(double percentI, double percentF)
    {
        intakeSpeed = percentI;
        feederSpeed = percentF;
        addRequirements(intake);
        addRequirements(feeder);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        intake.setOutput(intakeSpeed);
        feeder.setOutput(feederSpeed);
    }
}
