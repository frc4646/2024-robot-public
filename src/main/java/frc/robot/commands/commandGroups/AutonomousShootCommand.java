// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.DrivePointAtTargetAuto;
import frc.robot.commands.intake.feederLoadUntilSensor;
import frc.robot.commands.intake.intakeForceShoot;
import frc.robot.commands.robotState.stopIfNoPiece;
import frc.robot.commands.robotState.stopIfUpToSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousShootCommand extends SequentialCommandGroup
{
    /** Creates a new AutonomousShootCommand. */
    public AutonomousShootCommand()
    {
        // super(new InstantCommand());
        addCommands(
                new stopIfUpToSpeed(),
                new ParallelRaceGroup(
                        new WaitCommand(1.5),
                        new feederLoadUntilSensor(),
                        new DrivePointAtTargetAuto(Constants.FIELD.SPEAKER, false)),
                new ParallelDeadlineGroup(
                        new stopIfNoPiece(), // the wait we did before: new WaitCommand(Constants.INTAKE_FEEDER.AUTO_FEED_WAIT),
                        new intakeForceShoot(), new DrivePointAtTargetAuto(Constants.FIELD.SPEAKER, false)));
    }
}
