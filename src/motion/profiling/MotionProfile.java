package motion.profiling;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import motion.constants.MotionConstants;
import motion.logger.MotionLogger;

public class MotionProfile {
	
	private ArrayList<MotionData> profileData;
	private double targetPosition;
	private double timeToExecute;
	
	public MotionProfile(ArrayList<MotionData> profileData, double targetPosition, double timeToExecute) {
		this.profileData = profileData;
		this.targetPosition = targetPosition;
		this.timeToExecute = timeToExecute;
	}
	
	public double getTargetPosition() {
		return targetPosition;
	}
	
	public double getEstimatedTimeToExecute() {
		return timeToExecute;
	}
	
	public ArrayList<MotionData> getProfileData() {
		return profileData;
	}
	
	public MotionData getData(double time) {
		int index = getIndex(time);
		
		if (index >= 0 && index < profileData.size()) {
			return profileData.get(index);
		}
		
		return profileData.get(profileData.size() - 1);
	}
	
	private int getIndex(double time) {
		double freq = 1 / MotionConstants.DELTA_TIME;
		time = (double) Math.round(time * freq) / freq;
		int index = (int) (time / MotionConstants.DELTA_TIME);
		return index;
	}
	
	public void writeCSV(String filepath) {
		FileWriter fw = null;
		BufferedWriter bw = null;
		try {
			fw = new FileWriter(filepath);
			bw = new BufferedWriter(fw);
			
			bw.write(getCSVString());
			MotionLogger.print("CSV written to " + filepath);
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				if (bw != null)
					bw.close();
				if (fw != null)
					fw.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
	
	public String getCSVString() {
		StringBuilder sb = new StringBuilder();
		sb.append("time, position, velocity, acceleration\r\n");
		for (MotionData data : profileData) {
			String line = String.format("%f, %f, %f, %f\r\n", data.time, data.position, data.velocity, data.acceleration);
			sb.append(line);
		}
		return sb.toString();
	}
	
}

