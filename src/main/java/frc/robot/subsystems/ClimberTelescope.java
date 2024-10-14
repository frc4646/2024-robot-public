// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.team4646.SmartSubsystem;
import frc.team4646.drivers.servo.PositionServo;
import frc.team4646.drivers.servo.ServoMode;

public class ClimberTelescope extends SmartSubsystem
{
    public static enum ClimbPosition
    {
        LEFT,
        RIGHT,
        CENTER
    };

    private PositionServo leftClimber, rightClimber;
    private double setPointL = 0.0, setPointR = 0.0;
    private String DASH_NAME = "Climber Telescope";

    /** Creates a new ClimberTelescope. */
    public ClimberTelescope()
    {
        leftClimber = new PositionServo(Constants.CLIMBER_TELESCOPE.SERVO_CONFIG_L);
        rightClimber = new PositionServo(Constants.CLIMBER_TELESCOPE.SERVO_CONFIG_R);
        leftClimber.setMotor(ServoMode.OPEN_LOOP, 0);
        rightClimber.setMotor(ServoMode.OPEN_LOOP, 0);

        createDashboard();
    }

    public void setTargetVoltage(double percentL, double percentR)
    {
        double currentPositionL = leftClimber.getCurrent();
        double currentPositionR = rightClimber.getCurrent();
        // protect if it's near the min/max limits
        // if (percentL > 0.0 && leftClimber.isSaturatedMax(currentPositionL))
        // {
        // percentL = 0.0;
        // }
        // if (percentL < 0.0 && leftClimber.isSaturatedMin(currentPositionL))
        // {
        // percentL = 0.0;
        // }

        // if (percentR > 0.0 && rightClimber.isSaturatedMax(currentPositionR))
        // {
        // percentR = 0.0;
        // }
        // if (percentR < 0.0 && rightClimber.isSaturatedMin(currentPositionR))
        // {
        // percentR = 0.0;
        // }

        leftClimber.setMotor(ServoMode.OPEN_LOOP, percentL);
        rightClimber.setMotor(ServoMode.OPEN_LOOP, percentR);
    }

    public void setTargetVoltageL(double percent)
    {
        double currentPositionL = leftClimber.getCurrent();
        leftClimber.setMotor(ServoMode.OPEN_LOOP, percent);
    }

    public void setTargetVoltageR(double percent)
    {
        double currentPositionR = rightClimber.getCurrent();
        rightClimber.setMotor(ServoMode.OPEN_LOOP, percent);
    }

    public void setTargetPosition(double inchesL, double inchesR)
    {
        if (inchesL > Constants.CLIMBER_TELESCOPE.MAX_HEIGHT)
        {
            inchesL = Constants.CLIMBER_TELESCOPE.MAX_HEIGHT;
        }
        if (inchesL < Constants.CLIMBER_TELESCOPE.MIN_HEIGHT)
        {
            inchesL = Constants.CLIMBER_TELESCOPE.MIN_HEIGHT;
        }

        if (inchesR > Constants.CLIMBER_TELESCOPE.MAX_HEIGHT)
        {
            inchesR = Constants.CLIMBER_TELESCOPE.MAX_HEIGHT;
        }
        if (inchesR < Constants.CLIMBER_TELESCOPE.MIN_HEIGHT)
        {
            inchesR = Constants.CLIMBER_TELESCOPE.MIN_HEIGHT;
        }

        leftClimber.setMotor(ServoMode.POSITION, inchesL);
        rightClimber.setMotor(ServoMode.POSITION, inchesR);
        setPointL = inchesL;
        setPointR = inchesR;
    }

    /**
     * Call once per robot code loop. <b>Actually</b> sets the motor to the
     * previously requested setpoint.
     */
    @Override
    public void updateHardware()
    {
        // if tuning sys id this gets set in the setMotor call
        if (!Constants.TUNING.SYSID)
        {
            leftClimber.updateHardware();
            rightClimber.updateHardware();
        }
    }

    @Override
    public void cacheSensors()
    {
        leftClimber.cacheSensors();
        rightClimber.cacheSensors();
    }

    @Override
    public void whileDisabled()
    {
        // store current angle as setpoint to prevent motion on enable
        setPointL = getPositionL();
        setPointR = getPositionR();
    }

    public double getPositionL()
    {
        return leftClimber.getCurrent();
    }

    public double getPositionR()
    {
        return rightClimber.getCurrent();
    }

    public double getLastSetpointL()
    {
        return setPointL;
    }

    public double getLastSetpointR()
    {
        return setPointR;
    }

    public boolean checkPositionL(double tolerance)
    {
        return (Math.abs(getPositionL() - setPointL)) < tolerance;
    }

    public boolean checkPositionR(double tolerance)
    {
        return (Math.abs(getPositionR() - setPointR)) < tolerance;
    }

    public boolean isAtTargetPositionL()
    {
        return leftClimber.isAtTarget();
    }

    public boolean isAtTargetPositionR()
    {
        return rightClimber.isAtTarget();
    }

    public double getTorqueCurrentL()
    {
        return leftClimber.getTorqueCurrent();
    }

    public double getTorqueCurrentR()
    {
        return rightClimber.getTorqueCurrent();
    }

    public void createDashboard()
    {
        if (Constants.TUNING.CLIMBER)
        {
            Shuffleboard.getTab(DASH_NAME).add(this).withPosition(0, 0).withSize(2, 1);
            Shuffleboard.getTab(DASH_NAME).addBoolean("On Target L", () -> leftClimber.isAtTarget()).withPosition(2, 0);
            Shuffleboard.getTab(DASH_NAME).addDouble("Current Angle L", () -> leftClimber.getCurrent()).withPosition(3, 0).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addString("Set Mode L", () -> leftClimber.getSetpoint().wantedMode.toString()).withPosition(0, 1).withSize(2, 1);
            Shuffleboard.getTab(DASH_NAME).addDouble("Set Value L", () -> leftClimber.getSetpoint().wantedValue).withPosition(2, 1).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addDouble("Error L", () -> leftClimber.getError()).withPosition(3, 1).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Min L", () -> leftClimber.isSaturatedMin(leftClimber.getCurrent())).withPosition(0, 2);
            Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Max L", () -> leftClimber.isSaturatedMax(leftClimber.getCurrent())).withPosition(1, 2);

            Shuffleboard.getTab(DASH_NAME).addBoolean("On Target R", () -> rightClimber.isAtTarget()).withPosition(6, 0);
            Shuffleboard.getTab(DASH_NAME).addDouble("Current Angle R", () -> rightClimber.getCurrent()).withPosition(7, 0).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addString("Set Mode R", () -> rightClimber.getSetpoint().wantedMode.toString()).withPosition(4, 1).withSize(2, 1);
            Shuffleboard.getTab(DASH_NAME).addDouble("Set Value R", () -> rightClimber.getSetpoint().wantedValue).withPosition(6, 1).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addDouble("Error R", () -> rightClimber.getError()).withPosition(7, 1).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Min R", () -> rightClimber.isSaturatedMin(rightClimber.getCurrent())).withPosition(4, 2);
            Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Max R", () -> rightClimber.isSaturatedMax(rightClimber.getCurrent())).withPosition(5, 2);
        }
    }
}