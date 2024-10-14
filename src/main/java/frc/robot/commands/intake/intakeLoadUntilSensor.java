// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/* Runs intake until have piece in intake */
public class intakeLoadUntilSensor extends Command
{
    private final Intake intake = RobotContainer.INTAKE;
    private final Feeder feeder = RobotContainer.FEEDER;

    public intakeLoadUntilSensor()
    {
        addRequirements(intake);
        addRequirements(feeder);
    }

    @Override
    public void execute()
    {
        intake.setOutput(Constants.INTAKE_FEEDER.INTAKE_LOAD_PERCENT);
        feeder.setOutput(0);
    }

    @Override
    public boolean isFinished()
    {
        return RobotContainer.ROBOT_STATUS.inIntake();
    }
}
