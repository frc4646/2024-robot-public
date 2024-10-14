package frc.team4646.drivers.servo;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.robot.Constants;
import frc.team4646.drivers.TalonFXFactory;
import frc.team4646.util.StabilityCounter;

/**
 * Position/Angle Control Servo class - abstracts the TalonFX and CANcoder objects to consolidate configuration.
 */
public abstract class Servo
{
    protected final TalonFX motor;
    protected final ServoConfig config;

    protected final Setpoint setpoint;
    private final StabilityCounter stabilityCounter;

    /**
     * Create a new TalonFX/CANcoder paired object for Servo uses
     * 
     * @param config
     */
    public Servo(ServoConfig config)
    {
        this.config = config;
        motor = TalonFXFactory.createTalon(config.TALON_CAN_ID, config.CAN_BUS_NAME, config.TALON_FX_CONFIG);

        if (config.TALON_FOLLOWER_CAN_ID != -1)
        {
            // create the folower motor, but no need to keep it around
            TalonFXFactory.createPermanentFollowerTalon(config.TALON_FOLLOWER_CAN_ID, config.CAN_BUS_NAME, motor, config.TALON_FOLLOWER_INVERT);
        }

        BaseStatusSignal.setUpdateFrequencyForAll(100, motor.getPosition());

        if (config.CANCODER_CAN_ID != -1 && config.CANCODER_CONFIG != null)
        {
            CANcoder cancoder = new CANcoder(config.CANCODER_CAN_ID, config.CAN_BUS_NAME);
            cancoder.getConfigurator().apply(config.CANCODER_CONFIG);
            BaseStatusSignal.setUpdateFrequencyForAll(250, cancoder.getPosition());
        }

        setpoint = new Setpoint();
        stabilityCounter = new StabilityCounter(config.AT_TARGET_STABILITY_COUNT);
    }

    /**
     * Changes the motor's setpoint. Call updateHardware() later to apply to motor.
     */
    public void setMotor(ServoMode modeWanted, double value)
    {
        setMotor(modeWanted, value, 0.0, Slot.SLOT_0);
    }

    /**
     * Changes the motor's setpoint. Call updateHardware() later to apply to motor.
     */
    public void setMotor(ServoMode modeWanted, double value, Slot slot)
    {
        setMotor(modeWanted, value, 0.0, slot);
    }

    /**
     * Changes the motor's setpoint. Call updateHardware() later to apply to motor.
     */
    public void setMotor(ServoMode modeWanted, double value, double arbitraryFeedForward)
    {
        setMotor(modeWanted, value, arbitraryFeedForward, Slot.SLOT_0);
    }

    /**
     * Changes the motor's setpoint. Call updateHardware() later to apply to motor.
     */
    public void setMotor(ServoMode modeWanted, double value, double arbitraryFeedForward, Slot slot)
    {
        setpoint.wantedMode = modeWanted;
        setpoint.wantedValue = value;
        setpoint.wantedArbitraryFeedForward = arbitraryFeedForward;
        setpoint.slot = slot;

        if (Constants.TUNING.SYSID)
        {
            updateHardware();
        }
    }

    public Setpoint getSetpoint()
    {
        return setpoint;
    }

    /**
     * Call once per robot code loop.
     * <b>Actually</b> sets the motor to the previously requested setpoint.
     * Override to provide custom scaling to the command, ie for angles in degrees instead of rotations
     */
    public abstract void updateHardware();

    /**
     * Get the current position/angle.
     * Override to provide custom scaling to the value, ie for angles in degrees instead of rotations
     * 
     * @return
     */
    public abstract double getCurrent();

    public void cacheSensors()
    {
        stabilityCounter.calculate(Math.abs(getError()) < config.ON_TARGET);
    }

    /**
     * Get the target
     */
    public double getTarget()
    {
        return setpoint.wantedValue;
    }

    /**
     * Get the error, or the difference between the target and the current angle
     */
    public double getError()
    {
        return getCurrent() - getTarget();
    }

    /**
     * Is our current within our "on target" tolerance?
     */
    public boolean isAtTarget()
    {
        return stabilityCounter.isStable();
    }

    /**
     * Is the wanted is past our max?
     */
    public boolean isSaturatedMax(double wanted)
    {
        return wanted > config.MAX;
    }

    /**
     * Is the wanted is past our min?
     */
    public boolean isSaturatedMin(double wanted)
    {
        return wanted < config.MIN;
    }

    /**
     * Is the hardware limit switch at the max triggered?
     */
    public boolean isHardwareLimitMax()
    {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    /**
     * Is the hardware limit switch at the min triggered?
     */
    public boolean isHardwareLimitMin()
    {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * get the TalonFX motor object
     */
    public TalonFX getMotor()
    {
        return motor;
    }

    public static class Setpoint
    {
        public ServoMode wantedMode;
        public double wantedValue;
        public double wantedArbitraryFeedForward;
        public Slot slot;

        public Setpoint()
        {
            this.wantedMode = ServoMode.OPEN_LOOP;
            this.wantedValue = 0.0;
            this.wantedArbitraryFeedForward = 0.0;
            this.slot = Slot.SLOT_0;
        }
    }

    public static enum Slot
    {
        SLOT_0,
        SLOT_1
    }

}
