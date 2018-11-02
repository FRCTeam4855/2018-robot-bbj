package org.usfirst.frc.team4855.robot;

import edu.wpi.first.wpilibj.Encoder;

public class SwerveWheel {
	private double eSet=0,flip=0,addto=0,last=0;
	boolean lockFlip = false;
	final double ETD = 1.158333; //ENCODER TO DEGREES
	Encoder enc;
	
	public SwerveWheel(Encoder e) {
		enc = e;
	}
	
	public double calculateWheelAngle(double dL1, double dL2) {
		eSet = -(Math.atan2(dL1,dL2)*180/Math.PI)*ETD;
		if (eSet==0) {eSet=last;}
		eSet += addto;
		eSet += flip;
		if (Math.abs(enc.get()-eSet) > 90*ETD && Math.abs(enc.get()-eSet) < 270*ETD) {
			eSet -= flip;
			if (flip==0) flip=180*ETD; else flip=0;
			eSet += flip;
		}
		
		//This is just for autonomous rotation to angle
		if (lockFlip) {
			if (flip!=0) {
				eSet -= flip;
				flip=0;
			}
		}
		
		if (last - eSet > 185*ETD) {addto += 360*ETD;eSet += 360*ETD;} //For magnetic encoders, USE 412 (ABS 4048)
		if (last - eSet < -185*ETD) {addto -= 360*ETD;eSet -= 360*ETD;}
		if (enc.get() - eSet > 380*ETD) {addto += 360*ETD;eSet += 360*ETD;}
		if (enc.get() - eSet < -380*ETD) {addto -= 360*ETD;eSet -= 360*ETD;}
		last = eSet;
		return eSet;
	}
	
	public int getFlip() {
		if (flip==0) return 1; else return -1;
	}
	
	public void lockFlip(boolean f) {
		lockFlip = f;
	}
	
	public void reset() {
		eSet=0;flip=0;last=0;addto=0;
	}
}