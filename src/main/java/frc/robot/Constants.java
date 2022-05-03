// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    // Motor IDs
    public static final int FRONT_LEFT = 1;
    public static final int BACK_LEFT = 2;
    public static final int FRONT_RIGHT = 3;
    public static final int BACK_RIGHT = 4;

    // Default PID
    public static final double DEFAULT_P = 0.00435;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.003;

    // Default FF
    public static final double DEFAULT_S = 0.0;
    public static final double DEFAULT_V = 0.1335;

    // Default maximum velocity
    public static final double DEFAULT_VEL = 2000;

    // Robot stuff
    public static final double CIRCUMFERENCE = Units.inchesToMeters(6 * Math.PI);

    // Joystick port
    public static final int JOYSTICK = 0;
}
