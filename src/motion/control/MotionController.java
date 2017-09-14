package motion.control;

import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import motion.constants.MotionConstants;
import motion.logger.MotionLogger;
import motion.profiling.MotionData;
import motion.profiling.MotionProfile;

public class MotionController {

	private MotionSource m_MotionSource;
	private MotionOutput m_MotionOuput;
	private double m_Kv, m_Ka, m_Kp, m_Ki, m_Kd;
	
	private MotionProfile m_MotionProfile;
	
	private double m_Tolerance = 0.0;
	private boolean m_OnTarget = false;
	private boolean m_Enabled = false;
	
	private double m_Error = 0.0;
	private double m_ErrorPrev = 0.0;
	private double m_ErrorSum = 0.0;
	
	private double m_DistanceFromDestination = 0.0;
	
	private long m_TimeCurrentMillis;
	private long m_TimePrevMillis;
	private long m_TimeStartMillis;
	
	private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
	
	public MotionController(MotionSource source, MotionOutput output, double Kv, double Ka, double Kp, double Ki, double Kd) {
		m_MotionSource = source;
		m_MotionOuput = output;
		m_Kv = Kv;
		m_Ka = Ka;
		m_Kp = Kp;
		m_Ki = Ki;
		m_Kd = Kd;
	}
	
	public void setMotionProfile(MotionProfile profile) {
		m_MotionProfile = profile;
	}
	
	public void setTolerance(double tolerance) {
		m_Tolerance = tolerance;
	}
	
	public boolean onTarget() {
		return m_OnTarget;
	}
	
	public boolean isEnabled() {
		return m_Enabled;
	}
	
	public double getDistanceFromDestination() {
		return m_DistanceFromDestination;
	}
	
	public void enable() {
		if (m_Tolerance <= 0.0) {
			MotionLogger.print("Cannot enable. Tolerance is currently " + m_Tolerance + ". Please set with setTolerance()");
			return;
		}
		
		if (m_MotionProfile == null) {
			MotionLogger.print("Cannot enable. MotionProfile is currently null. Please set with setMotionProfile()");
			return;
		}
		
		if (m_MotionSource == null) {
			MotionLogger.print("Cannot enable. MotionSource is currently null. Please pass a non-null MotionSource when constructing a MotionController.");
			return;
		}
		
		if (m_MotionOuput == null) {
			MotionLogger.print("Cannot enable. MotionOutput is currently null. Please pass a non-null MotionOutput when constructing a MotionController.");
			return;
		}
		
		m_TimeStartMillis = System.currentTimeMillis();
		m_TimePrevMillis = System.currentTimeMillis();
		m_TimeCurrentMillis = System.currentTimeMillis();
		m_OnTarget = false;
		m_Enabled = true;
		
		executor.scheduleAtFixedRate(motionTask, 0, MotionConstants.DELTA_TIME_MILLIS, TimeUnit.MILLISECONDS);
	}
	
	public void disable() {
		m_Enabled = false;
		executor.shutdownNow();
		m_Tolerance = 0.0;
		m_MotionProfile = null;
	}
	
	private TimerTask motionTask = new TimerTask() {
		@Override
		public void run() {
			m_TimePrevMillis = m_TimeCurrentMillis;
			m_TimeCurrentMillis = System.currentTimeMillis();
			
			if (m_Enabled)
				calculate();
		}
	};
	
	private synchronized void calculate() {
		double elapsedTime = ((double) m_TimeCurrentMillis - m_TimeStartMillis) / 1000;
		double dt = ((double) m_TimeCurrentMillis - m_TimePrevMillis) / 1000;
		
		MotionLogger.printDebug("elapsedTime = " + elapsedTime);
		MotionLogger.printDebug("dt = " + dt);
		
		MotionData setpoint = m_MotionProfile.getData(elapsedTime);
		MotionLogger.printDebug("Setpoint: position = " + setpoint.position + ", velocity = " + setpoint.velocity + ", acceleration = " + setpoint.acceleration);
		
		double currentPosition = m_MotionSource.motionGet();
		
		m_ErrorPrev = m_Error;
		m_Error = setpoint.position - currentPosition;
		
		m_DistanceFromDestination = m_MotionProfile.getTargetPosition() - currentPosition;
		if (Math.abs(m_DistanceFromDestination) < m_Tolerance) {
			m_OnTarget = true;
			disable();
			return;
		}
		
		if (m_Error >= 0 && m_ErrorPrev >= 0) {
			double large = Math.max(m_Error, m_ErrorPrev);
			double small = Math.min(m_Error, m_ErrorPrev);
			m_ErrorSum += small * dt;
			m_ErrorSum += 0.5 * (large - small) * dt;
		}
		else if (m_Error <= 0 && m_ErrorPrev <= 0) {
			double large = Math.min(m_Error, m_ErrorPrev);
			double small = Math.max(m_Error, m_ErrorPrev);
			m_ErrorSum += small * dt;
			m_ErrorSum += 0.5 * (large - small) * dt;
		}
		else {
			double slope = (m_Error - m_ErrorPrev) / dt;
			double timeToZero = -m_ErrorPrev / slope;
			m_ErrorSum += 0.5 * (m_ErrorPrev) * timeToZero;
			m_ErrorSum += 0.5 * (m_Error) * (dt - timeToZero);
		}
		
		MotionLogger.printDebug("m_Error = " + m_Error);
		MotionLogger.printDebug("m_ErrorSum = " + m_ErrorSum);
		MotionLogger.printDebug("m_ErrorPrev = " + m_ErrorPrev);
		
		double output = m_Kv * setpoint.velocity + m_Ka * setpoint.acceleration + m_Kp * m_Error + m_Ki * m_ErrorSum + m_Kd * (m_Error - m_ErrorPrev) / dt;
		
		if (Double.isNaN(output) || Double.isInfinite(output)) {
			MotionLogger.printDebug("output (before) = " + output);
			output = 0;
		}
		
		if (output > 1) {
			MotionLogger.printDebug("output (before) = " + output);
			output = 1;
		}
		else if (output < -1) {
			MotionLogger.printDebug("output (before) = " + output);
			output = -1;
		}
		
		MotionLogger.printDebug("output = " + output);
		m_MotionOuput.updateMotors(output);
	}
	
}
