package frc.utilities;

import frc.robot.Robot;

public class SoftwareTimer {

	private double expirationTime;

	public void setTimer(double timerValue) {
		expirationTime = Robot.systemTimer.get() + timerValue;
	}

	public boolean isExpired() {
		return (Robot.systemTimer.get() > expirationTime);
		// if robotTime exceeds expirationTime, then this returns true
	}
}
