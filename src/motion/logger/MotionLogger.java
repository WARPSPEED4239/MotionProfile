package motion.logger;

public class MotionLogger {

	public static int DEBUG = 1;
	public static int VERBOSE = 2;
	public static int NORMAL = 3;
	public static int SILENT = 4;
	
	private static int logLevel = NORMAL;
	
	public static void setLogLevel(int level) {
		logLevel = level;
	}
	
	public static void printDebug(String str) {
		if (logLevel <= DEBUG) {
			log(str, "(DEBUG)");
		}
	}
	
	public static void printVerbose(String str) {
		if (logLevel <= VERBOSE) {
			log(str, "(VERBOSE)");
		}
	}
	
	public static void print(String str) {
		if (logLevel <= NORMAL) {
			log(str, "");
		}
	}
	
	private static void log(String str, String type) {
		System.out.println("MotionLogger" + type + ": " + str);
	}
	
}
