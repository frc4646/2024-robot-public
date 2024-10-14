package frc.team4646.drivers.servo;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

/**
 * Position/Angle Control Servo class - abstracts the TalonFX and CANcoder objects to consolidate configuration.
 */
public class AngleServo extends Servo
{

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(true);

    /**
     * Create a new TalonFX/CANcoder paired object for Servo uses
     * 
     * @param config
     */
    public AngleServo(ServoConfig config)
    {
        super(config);
    }

    /**
     * Call once per robot code loop. <b>Actually</b> sets the motor to the
     * previously requested setpoint.
     */
    @Override
    public void updateHardware()
    {
        final double feedforward = setpoint.wantedArbitraryFeedForward;
        switch (setpoint.wantedMode)
        {
            case OPEN_LOOP:
                motor.setControl(dutyCycleControl.withOutput(setpoint.wantedValue));
                break;

            case POSITION:
                motionMagicVoltage.Slot = setpoint.slot.ordinal();
                motor.setControl(motionMagicVoltage.withPosition(setpoint.wantedValue / 360.0).withFeedForward(feedforward));
                break;

            default:
                break;
        }
    }

    @Override
    public double getCurrent()
    {
        return motor.getPosition().getValue() * 360.0;
    }

}
