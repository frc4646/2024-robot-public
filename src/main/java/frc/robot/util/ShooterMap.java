package frc.robot.util;

import frc.robot.Constants;
import frc.team254.util.InterpolatingDouble;
import frc.team254.util.InterpolatingTreeMap;

public class ShooterMap
{
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mapLeft = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mapRight = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mapAngle = new InterpolatingTreeMap<>();
    private double distanceMin = Double.MAX_VALUE;
    private double distanceMax = Double.MIN_VALUE;
    private double rpmLeftMin = Double.MAX_VALUE;
    private double rpmRightMin = Double.MAX_VALUE;
    private double angleMin = Double.MIN_VALUE;

    public void add(double distance_m, double rpmLeft, double rpmRight, double angle, boolean useDefaultSpin)
    {
        mapLeft.put(new InterpolatingDouble(distance_m), new InterpolatingDouble(rpmLeft));
        if (useDefaultSpin)
        {
            rpmRight = Constants.SHOOTER.SPIN_PERCENT * rpmLeft;
        }
        mapRight.put(new InterpolatingDouble(distance_m), new InterpolatingDouble(rpmRight));
        mapAngle.put(new InterpolatingDouble(distance_m), new InterpolatingDouble(angle));
        updateMinMax(distance_m, rpmLeft, rpmRight, angle);
    }

    public boolean isInRange(double distance_m)
    {
        return distance_m > getDistanceMin() && distance_m < getDistanceMax();
    }

    public double getRPMLeft(double distance_m)
    {
        return mapLeft.getInterpolated(new InterpolatingDouble(distance_m)).value;
    }

    public double getRPMRight(double distance_m)
    {
        return mapRight.getInterpolated(new InterpolatingDouble(distance_m)).value;
    }

    public double getAngle(double distance_m)
    {
        return mapAngle.getInterpolated(new InterpolatingDouble(distance_m)).value;
    }

    public double getDistanceMin()
    {
        return distanceMin;
    }

    public double getDistanceMax()
    {
        return distanceMax;
    }

    public double getRPMLeftMin()
    {
        return rpmLeftMin;
    }

    public double getRPMRightMin()
    {
        return rpmRightMin;
    }

    public double getAngleMin()
    {
        return angleMin;
    }

    public double getDistanceDefault()
    {
        return (getDistanceMin() + getDistanceMax()) / 2.0;
    }

    public double getRPMLeftDefault()
    {
        return getRPMLeft(getDistanceDefault());
    }

    public double getRPMRightDefault()
    {
        return getRPMRight(getDistanceDefault());
    }

    public double getAngleDefault()
    {
        return getAngle(getDistanceDefault());
    }

    private void updateMinMax(double distance, double rpmLeft, double rpmRight, double angle)
    {
        if (distance < distanceMin)
        {
            distanceMin = distance;
        }
        if (distance > distanceMax)
        {
            distanceMax = distance;
        }
        if (rpmLeft < rpmLeftMin)
        {
            rpmLeftMin = rpmLeft;
        }
        if (rpmRight < rpmRightMin)
        {
            rpmRightMin = rpmRight;
        }
        if (angle < angleMin)
        {
            angleMin = angle;
        }
    }
}
