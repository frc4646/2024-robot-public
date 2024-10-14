// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmSetBetweenDegrees;
import frc.robot.commands.intake.feederLoadUntilSensor;

public class SetArmThenIntake extends SequentialCommandGroup
{
    private final double HANDOFF_DEGREES_MIN = 15.0;
    private final double HANDOFF_DEGREES_MAX = 50.0;

    /** Pull note into feedback after arm is at a good angle */
    public SetArmThenIntake()
    {
        addCommands(
                new ArmSetBetweenDegrees(HANDOFF_DEGREES_MIN, HANDOFF_DEGREES_MAX),
                new feederLoadUntilSensor());
    }
}
