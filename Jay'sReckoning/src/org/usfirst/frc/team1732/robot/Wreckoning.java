package org.usfirst.frc.team1732.robot;

public class Wreckoning {
	private long start_time;
	private int right_value;
    private int left_value;
	private final static double drive_base = 24.5;
	private final static double wheel_circumference = 8*Math.PI;
	private final static double counts_per_rotation = 100;
	private final static double turning_circumference = drive_base*Math.PI;
	
	public void reset(int lv, int rv) {
		right_value = rv;
		left_value = lv;
		start_time = System.currentTimeMillis();
	}
	
	public double getDistance(int count) {
		return count*wheel_circumference/counts_per_rotation;
	}
	
	public double getAngle(int lv, int rv) {
		return ((getDistance(right_value) - getDistance(rv)+getDistance(left_value)-getDistance(lv))/turning_circumference)/2*Math.PI*2;
		// return (2*wheel_circumference*(rv-right_value)/counts_per_rotation)/drive_base;
	}
}
