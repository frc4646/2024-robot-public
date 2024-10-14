package frc.robot.configuration;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmThumbstick;
import frc.robot.commands.climb.ClimbThumbstick;
import frc.robot.commands.climb.TelescopeClimbThumbstick;
import frc.robot.commands.commandGroups.ManualShoot;
import frc.robot.commands.commandGroups.SetArmThenIntake;
import frc.robot.commands.commandGroups.SmartClimb;
import frc.robot.commands.commandGroups.SmartShoot;
import frc.robot.commands.commandGroups.SmartTelescopeClimb;
import frc.robot.commands.commandGroups.prepareAmp;
import frc.robot.commands.commandGroups.shootAcrossFieldAmp;
import frc.robot.commands.intake.JiggleNote;
import frc.robot.commands.intake.feederAndIntakePercentage;
import frc.robot.commands.shoot.ShootDistance;
import frc.robot.subsystems.ClimberTelescope.ClimbPosition;
import frc.team4646.util.Util;

public class OperatorControls
{
    private final CommandXboxController controller;
    private final double thumbstickDeadband = 0.2;

    public OperatorControls()
    {
        controller = new CommandXboxController(1);
        controller.leftBumper().whileTrue(new ManualShoot());
        controller.rightBumper().whileTrue(new SmartShoot());
        controller.a().whileTrue(new SetArmThenIntake());
        controller.b().whileTrue(new feederAndIntakePercentage(Constants.INTAKE_FEEDER.REJECT_PERCENT));
        controller.x().whileTrue(new prepareAmp());
        controller.y().whileTrue(new ShootDistance());
        if (Constants.CLIMBER.ENABLED)
        {
            controller.povLeft().whileTrue(new SmartClimb());
        }
        if (Constants.CLIMBER_TELESCOPE.ENABLED)
        {
            controller.povLeft().whileTrue(new SmartTelescopeClimb(ClimbPosition.LEFT));
            controller.povRight().whileTrue(new SmartTelescopeClimb(ClimbPosition.RIGHT));
            controller.povDown().whileTrue(new SmartTelescopeClimb(ClimbPosition.CENTER));

        }
        controller.povUp().whileTrue(new JiggleNote());
        controller.axisLessThan(Axis.kLeftY.value, -thumbstickDeadband).whileTrue(new ArmThumbstick());
        controller.axisGreaterThan(Axis.kLeftY.value, thumbstickDeadband).whileTrue(new ArmThumbstick());
        controller.back()
                .whileTrue(new feederAndIntakePercentage(Constants.INTAKE_FEEDER.FORCE_FEED_OVERRIDE_PERCENT));
        controller.start().whileTrue(new shootAcrossFieldAmp());
        if (Constants.CLIMBER.ENABLED)
        {
            controller.axisLessThan(Axis.kRightY.value, -thumbstickDeadband).whileTrue(new ClimbThumbstick());
            controller.axisGreaterThan(Axis.kRightY.value, thumbstickDeadband).whileTrue(new ClimbThumbstick());
        }
        if (Constants.CLIMBER_TELESCOPE.ENABLED)
        {
            controller.axisLessThan(Axis.kRightY.value, -thumbstickDeadband).and(controller.leftTrigger().or(controller.rightTrigger()))
                    .whileTrue(new TelescopeClimbThumbstick());
            controller.axisGreaterThan(Axis.kRightY.value, thumbstickDeadband).and(controller.leftTrigger().or(controller.rightTrigger()))
                    .whileTrue(new TelescopeClimbThumbstick());
        }
    }

    public void setRumble(boolean leftSide, double percent)
    {
        controller.getHID().setRumble(leftSide ? RumbleType.kLeftRumble : RumbleType.kRightRumble, percent);
    }

    public void setRumble(double percent)
    {
        setRumble(true, percent);
        setRumble(false, percent);
    }

    public boolean isOverridePressed()
    {
        return controller.getHID().getBackButton();
    }

    public boolean isLeftTriggerPressed()
    {
        return controller.getHID().getLeftTriggerAxis() > thumbstickDeadband;
    }

    public boolean isRightTriggerPressed()
    {
        return controller.getHID().getRightTriggerAxis() > thumbstickDeadband;
    }

    /**
     * positive value when pressed forward
     * 
     * @return -1 to 1
     */
    public double getArmJoystick()
    {
        return -Util.handleDeadband(controller.getLeftY(), thumbstickDeadband);
    }

    /**
     * positive value when pressed forward
     * 
     * @return -1 to 1
     */
    public double getClimberJoystick()
    {
        return -Util.handleDeadband(controller.getRightY(), thumbstickDeadband);
    }

    public boolean isRightThumbstickDown()
    {
        return controller.getRightY() > thumbstickDeadband;
    }

}
