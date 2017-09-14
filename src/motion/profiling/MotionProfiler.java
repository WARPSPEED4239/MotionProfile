package motion.profiling;

import java.util.ArrayList;

import motion.constants.MotionConstants;

public class MotionProfiler {
	
	public MotionProfiler() {}
	
	public MotionProfile getProfile(double targetPosition, double cruiseVelocity, double accelerationRate) {
		
		// Adjusts signs to achieve targetPositon
		if (!isSameSign(targetPosition, cruiseVelocity))
			cruiseVelocity *= -1;
		if (!isSameSign(targetPosition, accelerationRate))
			accelerationRate *= -1;
		
		boolean isReverse = false;
		if (targetPosition < 0) {
			isReverse = true;
			targetPosition *= -1;
			cruiseVelocity *= -1;
			accelerationRate *= -1;
		}
		
		double accelerationDistance = 0.5 * Math.pow(cruiseVelocity, 2) / accelerationRate;
		double decelerationDistance = accelerationDistance;
		double cruisingDistance = targetPosition - accelerationDistance - decelerationDistance;
		
		ArrayList<MotionData> profileData;
		if (cruisingDistance >= 0) {
			profileData = generateTrapezoidal(targetPosition, cruiseVelocity, accelerationRate);
		}
		else {
			profileData = generateTriangle(targetPosition, accelerationRate);
		}
		
		
		if (isReverse && profileData != null && profileData.size() > 0) {
			for (MotionData data : profileData) {
				if (data.position != 0) data.position *= -1;
				if (data.velocity != 0) data.velocity *= -1;
				if (data.acceleration != 0) data.acceleration *= -1;
			}
		}
		
		if (isReverse)
			targetPosition *= -1;
		return new MotionProfile(profileData, targetPosition, profileData.get(profileData.size() - 1).time);
	}
	
	private ArrayList<MotionData> generateTrapezoidal(double targetPosition, double cruiseVelocity, double accelerationRate) {
		double accelerationTime = cruiseVelocity / accelerationRate;
		double accelerationDistance = 0.5 * accelerationRate * Math.pow(accelerationTime, 2);
		double decelerationTime = accelerationTime;
		double decelerationDistance = accelerationDistance;
		double cruisingDistance = targetPosition - accelerationDistance - decelerationDistance;
		double cruisingTime = cruisingDistance / cruiseVelocity;
		
		double totalTime = accelerationTime + cruisingTime + decelerationTime;
		
		double currentPosition = 0.0;
		double currentVelocity = 0.0;
		double currentAcceleration = 0.0;
		
		double previousVelocity;
		
		ArrayList<MotionData> profileData = new ArrayList<MotionData>();	
		for (double currentTime = 0.0; currentTime < totalTime + MotionConstants.DELTA_TIME; currentTime += MotionConstants.DELTA_TIME) {
			// accelerating
			if (currentTime < accelerationTime) {
				currentPosition = 0.5 * accelerationRate * Math.pow(currentTime, 2);
				currentVelocity = accelerationRate * currentTime;
				currentAcceleration = accelerationRate;
			}
			// cruising
			else if (currentTime < accelerationTime + cruisingTime) {
				currentPosition += cruiseVelocity * MotionConstants.DELTA_TIME;
				currentVelocity = cruiseVelocity;
				currentAcceleration = 0.0;
			}
			// decelerating
			else if (currentTime < totalTime) {
				currentAcceleration = -accelerationRate;
				previousVelocity = currentVelocity;
				currentVelocity += currentAcceleration * MotionConstants.DELTA_TIME;
				currentPosition += currentVelocity * MotionConstants.DELTA_TIME;
				currentPosition += 0.5 * (previousVelocity - currentVelocity) * MotionConstants.DELTA_TIME;
			}
			// stopped
			else {
				currentPosition = targetPosition;
				currentVelocity = 0.0;
				currentAcceleration = 0.0;
			}
			
			MotionData data = new MotionData();
			data.time = currentTime;
			data.position = currentPosition;
			data.velocity = currentVelocity;
			data.acceleration = currentAcceleration;
			profileData.add(data);
		}
		
		return profileData;
	}
	
	private ArrayList<MotionData> generateTriangle(double targetPosition, double accelerationRate) {
		double accelerationTime = Math.abs(Math.sqrt(targetPosition / accelerationRate));
		double decelerationTime = accelerationTime;
		double totalTime = accelerationTime + decelerationTime;
		
		double currentPosition = 0.0;
		double currentVelocity = 0.0;
		double currentAcceleration = 0.0;
		
		double previousVelocity;
		
		ArrayList<MotionData> profileData = new ArrayList<MotionData>();	
		for (double currentTime = 0.0; currentTime < totalTime + MotionConstants.DELTA_TIME; currentTime += MotionConstants.DELTA_TIME) {
			// accelerating
			if (currentTime < accelerationTime) {
				currentPosition = 0.5 * accelerationRate * Math.pow(currentTime, 2);
				currentVelocity = accelerationRate * currentTime;
				currentAcceleration = accelerationRate;
			}
			// decelerating
			else if (currentTime < totalTime) {
				currentAcceleration = -accelerationRate;
				previousVelocity = currentVelocity;
				currentVelocity += currentAcceleration * MotionConstants.DELTA_TIME;
				currentPosition += currentVelocity * MotionConstants.DELTA_TIME;
				currentPosition += 0.5 * (previousVelocity - currentVelocity) * MotionConstants.DELTA_TIME;
			}
			// stopped
			else {
				currentPosition = targetPosition;
				currentVelocity = 0.0;
				currentAcceleration = 0.0;
			}
			
			MotionData data = new MotionData();
			data.time = currentTime;
			data.position = currentPosition;
			data.velocity = currentVelocity;
			data.acceleration = currentAcceleration;
			profileData.add(data);
		}
		
		return profileData;
	}
	
	private boolean isSameSign(double d1, double d2) {
		return (d1 >= 0 && d2 >= 0) || (d1 <= 0 && d2 <= 0);
	}
	
}
