// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants;
import frc.robot.commands.arm.ArmDegrees;
import frc.robot.commands.arm.ArmPointAtSpeaker;
import frc.robot.commands.commandGroups.AutonomousShootCommand;
import frc.robot.commands.commandGroups.prepareToShootAndAim;
import frc.robot.commands.drive.DrivePointAtTargetAuto;
import frc.robot.commands.intake.FeedThenIntakeOff;
import frc.robot.commands.intake.intakeForceShoot;
import frc.robot.commands.intake.intakeLoadUntilSensor;
import frc.robot.commands.intake.intakeOff;
import frc.robot.commands.robotState.stopIfHasPiece;
import frc.robot.commands.robotState.stopIfNoPiece;
import frc.robot.commands.shoot.ShootDistance;
import frc.robot.commands.shoot.ShootOff;

/** Add your docs here. */

public class CommandRegisterer
{
    public static void Register()
    {
        NamedCommands.registerCommand("intake", new intakeLoadUntilSensor());
        NamedCommands.registerCommand("shoot", new intakeForceShoot());
        NamedCommands.registerCommand("start up shooter", new prepareToShootAndAim());
        NamedCommands.registerCommand("Intake Off", new intakeOff());
        NamedCommands.registerCommand("intake off", new intakeOff());
        NamedCommands.registerCommand("shoot off", new ShootOff());
        NamedCommands.registerCommand("intake and stage", new FeedThenIntakeOff());
        NamedCommands.registerCommand("RPM distance", new ShootDistance());
        NamedCommands.registerCommand("arm point at speeker", new ArmPointAtSpeaker());
        NamedCommands.registerCommand("stop if no piece", new stopIfNoPiece());
        NamedCommands.registerCommand("stop if has piece", new stopIfHasPiece());
        NamedCommands.registerCommand("arm zero", new ArmDegrees(Constants.SHOOTER_ARM.SERVO.DEGREES_MIN));
        NamedCommands.registerCommand("point at speaker", new DrivePointAtTargetAuto(Constants.FIELD.SPEAKER, false));
        NamedCommands.registerCommand("auto shoot", new AutonomousShootCommand());
        // NamedCommands.registerCommand("can shoot", new stopIfCanShoot());
    }
}
