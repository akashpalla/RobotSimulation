/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {
	
	public final static class Elevator{
	//Elevator Constants
	public static final double kGearRatio = 10.0;  // 40:1 Reduction
	public static final double kPulleyRadius = Units.inchesToMeters(2);
	public static final double kCarriageMass = 6.803;  // in KG  (15 pounds)
	public static final double kMinHeight = 0;
	public static final double kMaxHeight = 3.35;  // SI Units is meters == 11 ft
	public static final double kTicksPerRevolution = 4096;

	public static final double kDistancePerTick = 2 * Math.PI * kPulleyRadius / kTicksPerRevolution / kGearRatio;

	public static final double kP  = .9;
	public static final double kI = 0;
	public static final double kD = .3;

	public static final double kBottomSetpoint = 0;
	public static final double kMidSetpoint = 2;
	public static final double kHightSetpoint = 3;

	}
}