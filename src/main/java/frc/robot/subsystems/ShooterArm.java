package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.team4646.SmartSubsystem;
import frc.team4646.drivers.servo.AngleServo;
import frc.team4646.drivers.servo.Servo.Slot;
import frc.team4646.drivers.servo.ServoMode;
import frc.team4646.util.ModifiedSignalLogger;

public class ShooterArm extends SmartSubsystem
{
    public final String DASH_NAME = "Arm";

    private AngleServo servo;
    private double setPoint = 0.0;

    public ShooterArm()
    {
        servo = new AngleServo(Constants.SHOOTER_ARM.SERVO_CONFIG);
        servo.setMotor(ServoMode.OPEN_LOOP, 0);
        createDashboard();
        setupSysId();
    }

    public void setTargetVoltage(double percent)
    {
        double currentAngles = servo.getCurrent();

        // protect if it's near the min/max limits
        if (percent > 0.0 && servo.isSaturatedMax(currentAngles))
        {
            percent = 0.0;
        }
        if (percent < 0.0 && servo.isSaturatedMin(currentAngles))
        {
            percent = 0.0;
        }

        servo.setMotor(ServoMode.OPEN_LOOP, percent, chooseGainSlot(currentAngles));
    }

    public void setTargetAngle(Rotation2d wanted)
    {
        setTargetAngle(wanted.getDegrees());
    }

    /** Control arms by updating motor setpoints using wanted <b>position control</b> */
    public void setTargetAngle(double wanted)
    {
        double wantedAngle = wanted;
        servo.setMotor(ServoMode.POSITION, wantedAngle, chooseGainSlot(wantedAngle));
        setPoint = wanted;
    }

    /*
     * Change the gain slot based on our current angle.
     * This fixes the oscillations at a vertical 90deg angle
     * 
     * @return
     */
    private Slot chooseGainSlot(double wantedAngle)
    {
        double currentAngle = servo.getCurrent();
        double difference = Math.abs(wantedAngle - currentAngle);
        Slot slot = Slot.SLOT_0;
        if (difference < 5 &&
                (currentAngle > Constants.SHOOTER_ARM.SLOT_MIN_DEGREES
                        && currentAngle < Constants.SHOOTER_ARM.SLOT_MAX_DEGREES))
        {
            slot = Slot.SLOT_1;
        }
        return slot;
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
            servo.updateHardware();
        }
    }

    @Override
    public void cacheSensors()
    {
        servo.cacheSensors();
    }

    @Override
    public void whileDisabled()
    {
        // store current angle as setpoint to prevent motion on enable
        setPoint = getAngle();
    }

    public double getAngle()
    {
        return servo.getCurrent();
    }

    public double getLastSetpoint()
    {
        return setPoint;
    }

    public boolean checkAngle(double angleTolerance)
    {
        return (Math.abs(getAngle() - setPoint)) < angleTolerance;
    }

    public boolean isAtTargetAngle()
    {
        return servo.isAtTarget();
    }

    public void createDashboard()
    {
        if (Constants.TUNING.SHOOTER_ARM)
        {
            Shuffleboard.getTab(DASH_NAME).add(this).withPosition(0, 0).withSize(2, 1);
            Shuffleboard.getTab(DASH_NAME).addBoolean("On Target", () -> servo.isAtTarget()).withPosition(2, 0);
            Shuffleboard.getTab(DASH_NAME).addDouble("Current Angle", () -> servo.getCurrent()).withPosition(3, 0).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addString("Set Mode", () -> servo.getSetpoint().wantedMode.toString()).withPosition(0, 1).withSize(2, 1);
            Shuffleboard.getTab(DASH_NAME).addDouble("Set Value", () -> servo.getSetpoint().wantedValue).withPosition(2, 1).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addDouble("Error", () -> servo.getError()).withPosition(3, 1).withSize(2, 1);

            Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Min", () -> servo.isSaturatedMin(servo.getCurrent())).withPosition(0, 2);
            Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Max", () -> servo.isSaturatedMax(servo.getCurrent())).withPosition(1, 2);
        }
    }

    ///////////////////////////////////////////////////
    /////////////////// SysID stuff ///////////////////
    ///////////////////////////////////////////////////
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    private SysIdRoutine m_SysIdRoutine;

    private void setupSysId()
    {
        if (Constants.TUNING.SYSID)
        {
            m_SysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null, null, null,
                            ModifiedSignalLogger.logState("Arm State")),
                    new SysIdRoutine.Mechanism(
                            (Measure<Voltage> volts) ->
                            {
                                servo.getMotor().setControl(m_sysidControl.withOutput(volts.in(Volts)));
                            },
                            null,
                            this));
        }
    }

    public Command runQuasiTest(Direction direction)
    {
        return m_SysIdRoutine.quasistatic(direction);
    }

    public Command runDynamTest(Direction direction)
    {
        return m_SysIdRoutine.dynamic(direction);
    }
}
