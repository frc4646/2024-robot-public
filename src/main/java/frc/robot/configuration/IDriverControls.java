// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

public interface IDriverControls
{
    /**
     * Get the x speed, with left as -X, in mps.
     */
    public double getXSpeed();

    /**
     * Get the y speed, with forward as -Y, in mps.
     */
    public double getYSpeed();

    /**
     * Get the rate of angular rotation, with counterclockwise as -X, in radians/sec.
     */
    public double getRotationRate();

    public int getSnapAngle();

    public void whileDisabled();
}
