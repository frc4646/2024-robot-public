// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmDegrees;
import frc.robot.commands.shoot.ShootRPM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualShoot extends ParallelCommandGroup {
  /** Creates a new ManualShoot. */
  public ManualShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ArmDegrees(Constants.SHOOTER.MAP.getAngle(Constants.SHOOTER.MINIMUM_DISTANCE)), 
    new ShootRPM(
        Constants.SHOOTER.MAP.getRPMLeft(Constants.SHOOTER.MINIMUM_DISTANCE),
        Constants.SHOOTER.MAP.getRPMRight(Constants.SHOOTER.MINIMUM_DISTANCE)));
  }
}
