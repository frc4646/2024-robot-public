package frc.robot.configuration;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.team4646.util.ModifiedSignalLogger;

public class SysIDControls
{
    CommandXboxController controller;

    public SysIDControls()
    {
        controller = new CommandXboxController(3);

        ///// Swerve
        // x + dpad up = drive forward ramp
        // x + dpad down = drive reverse ramp
        // x + dpad right = drive forward full speed
        // x + dpad left = drive reverse full speed
        controller.x().and(controller.pov(0)).whileTrue(RobotContainer.SWERVE.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
        controller.x().and(controller.pov(180)).whileTrue(RobotContainer.SWERVE.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));
        controller.x().and(controller.pov(90)).whileTrue(RobotContainer.SWERVE.runDriveDynamTest(SysIdRoutine.Direction.kForward));
        controller.x().and(controller.pov(270)).whileTrue(RobotContainer.SWERVE.runDriveDynamTest(SysIdRoutine.Direction.kReverse));

        // y + dpad up = steer forward ramp
        // y + dpad down = steer reverse ramp
        // y + dpad right = steer forward full speed
        // y + dpad left = steer reverse full speed
        controller.y().and(controller.pov(0)).whileTrue(RobotContainer.SWERVE.runSteerQuasiTest(SysIdRoutine.Direction.kForward));
        controller.y().and(controller.pov(180)).whileTrue(RobotContainer.SWERVE.runSteerQuasiTest(SysIdRoutine.Direction.kReverse));
        controller.y().and(controller.pov(90)).whileTrue(RobotContainer.SWERVE.runSteerDynamTest(SysIdRoutine.Direction.kForward));
        controller.y().and(controller.pov(270)).whileTrue(RobotContainer.SWERVE.runSteerDynamTest(SysIdRoutine.Direction.kReverse));

        // back + dpad up = slip test against wall
        controller.back().and(controller.pov(0)).whileTrue(RobotContainer.SWERVE.runDriveSlipTest());

        ///// Shooter
        // left bumper + dpad up = shooter forward ramp
        // left bumper + dpad down = shooter reverse ramp
        // left bumper + dpad right = shooter forward full speed
        // left bumper + dpad left = shooter reverse full speed
        controller.leftBumper().and(controller.pov(0)).whileTrue(RobotContainer.SHOOTER.runQuasiTest(SysIdRoutine.Direction.kForward));
        controller.leftBumper().and(controller.pov(180)).whileTrue(RobotContainer.SHOOTER.runQuasiTest(SysIdRoutine.Direction.kReverse));
        controller.leftBumper().and(controller.pov(90)).whileTrue(RobotContainer.SHOOTER.runDynamTest(SysIdRoutine.Direction.kForward));
        controller.leftBumper().and(controller.pov(270)).whileTrue(RobotContainer.SHOOTER.runDynamTest(SysIdRoutine.Direction.kReverse));

        ///// Arm
        // right bumper + dpad up = shooter forward ramp
        // right bumper + dpad down = shooter reverse ramp
        // right bumper + dpad right = shooter forward full speed
        // right bumper + dpad left = shooter reverse full speed
        controller.rightBumper().and(controller.pov(0)).whileTrue(RobotContainer.SHOOTER_ARM.runQuasiTest(SysIdRoutine.Direction.kForward));
        controller.rightBumper().and(controller.pov(180)).whileTrue(RobotContainer.SHOOTER_ARM.runQuasiTest(SysIdRoutine.Direction.kReverse));
        controller.rightBumper().and(controller.pov(90)).whileTrue(RobotContainer.SHOOTER_ARM.runDynamTest(SysIdRoutine.Direction.kForward));
        controller.rightBumper().and(controller.pov(270)).whileTrue(RobotContainer.SHOOTER_ARM.runDynamTest(SysIdRoutine.Direction.kReverse));

        ///// Stop Tests
        // back + start
        controller.back().and(controller.start()).onTrue(new RunCommand(ModifiedSignalLogger::stop));
        controller.leftStick().and(controller.rightStick()).onTrue(new RunCommand(ModifiedSignalLogger::start));
    }
}
