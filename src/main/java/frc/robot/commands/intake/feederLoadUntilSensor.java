// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.robotState.stopIfInFeeder;
import frc.robot.commands.robotState.stopIfInIntake;

/** Run feeder and intake until note is in feeder, then reverse intake so nothing is stuck */
public class feederLoadUntilSensor extends ParallelDeadlineGroup
{
    public feederLoadUntilSensor()
    {
        super(new stopIfInFeeder());

        addCommands(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new stopIfInIntake(),
                                new intakePercentage(Constants.INTAKE_FEEDER.INTAKE_LOAD_PERCENT)),

                        new ParallelDeadlineGroup(
                                new stopIfInFeeder(),
                                new feederAndIntakePercentage(Constants.INTAKE_FEEDER.FEED_PERCENT))));
    }
}
