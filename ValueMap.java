package org.usfirst.frc.team97.robot;

public class ValueMap {
	Vec2 range1;
	Vec2 range2;
	
	public ValueMap(double x1, double y1, double x2, double y2) {
		remap1(x1, y1);
		remap2(x2, y2);
	}
	
	public double getVal (double input) {
		double i = (input - range1.x)/(range1.y - range1.x);
		double newVal = range2.x + ((range2.y - range2.x) * i);
		return newVal;
	}
	public void remap1 (double newX, double newY) {
		range1 = new Vec2(newX, newY);
	}
	public void remap2 (double newX, double newY) {
		range2 = new Vec2(newX, newY);
	}
	public static double getVal(double x1, double y1, double x2, double y2, double input) {
		ValueMap VM = new ValueMap(x1, y1, x2, y2);
		return VM.getVal(input);
	}
}
class Vec2 {
	double x;
	double y;
	public Vec2 (double xVal, double yVal) {
		x = xVal;
		y = yVal;
	}
}