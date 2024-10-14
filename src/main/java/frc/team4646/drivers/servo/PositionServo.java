package frc.team4646.drivers.servo;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class PositionServo extends Servo
{
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(true);

    public PositionServo(ServoConfig config)
    {
        super(config);
    }

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
                motor.setControl(motionMagicVoltage.withPosition(setpoint.wantedValue).withFeedForward(feedforward));
                break;

            default:
                break;
        }
    }

    @Override
    public double getCurrent()
    {
        return motor.getPosition().getValue();
    }

    public double getTorqueCurrent()
    {
        return motor.getTorqueCurrent().getValue();
    }
}
