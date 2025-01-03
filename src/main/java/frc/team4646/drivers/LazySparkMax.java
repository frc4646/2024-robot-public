package frc.team4646.drivers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands.
 */
public class LazySparkMax extends CANSparkMax
{
    protected double mLastSet = Double.NaN;
    protected ControlType mLastControlType = null;

    // Set if is a follower
    protected CANSparkMax mLeader = null;

    public LazySparkMax(int deviceNumber)
    {
        super(deviceNumber, MotorType.kBrushless);
    }

    public CANSparkMax getLeader()
    {
        return mLeader;
    }

    @Override
    public REVLibError follow(final CANSparkBase leader)
    {
        mLeader = (CANSparkMax) leader; // TODO is this the right place to cast?
        return super.follow(leader);
    }

    /**
     * wrapper method to mimic TalonSRX set method
     */
    public void set(ControlType type, double setpoint)
    {
        if (setpoint != mLastSet || type != mLastControlType)
        {
            mLastSet = setpoint;
            mLastControlType = type;
            super.getPIDController().setReference(setpoint, type);
        }
    }
}