package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RRLogger {

//    private static ConcurrentLinkedQueue<String> m_PowerBuffer = new ConcurrentLinkedQueue<String>();
//    private static ConcurrentLinkedQueue<String> m_DataBuffer = new ConcurrentLinkedQueue<String>();
    private static StringBuilder  m_dataAdder = new StringBuilder();
//    private static PrintWriter m_LogFile;
    private static PrintWriter m_DataLogFile;
    private static String directory = "/home/lvuser/logger/";
//    private static String logFileName = "power";
    private static String dataDumpFileName = "data";
    private static long startTime;
    private static File f = new File(directory + dataDumpFileName + ".csv");

    public static void start() {
        startTime = System.nanoTime();

        int i = 1;
        while (f.exists()) { // check if file exists
            f = new File(directory + dataDumpFileName + "_" + i + ".csv");
            i++;
        }
			try {
				f.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
            System.out.println(f);

//		} else {
//			try {
//				f.createNewFile();
//			} catch (IOException e) {
//				e.printStackTrace();
//			}
//            System.out.println("2");
//            System.out.println(f);
//		}

//        try {
//            m_LogFile = new PrintWriter(new BufferedWriter(new FileWriter(directory + logFileName + ".csv", false)));
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

		try {
			m_DataLogFile = new PrintWriter(new BufferedWriter(new FileWriter(f, true)));
			System.out.println(f);
        } catch (IOException e) {
			e.printStackTrace();
		}
        System.out.println("Done constructing logger");
		m_dataAdder.append("Autonomous Is Mean");
		m_dataAdder.append("\n");
    }

    //Create name of file based off of Real Time
    public static void addData(String dataType, double value) {

        String sep = ",";
        String str = dataType + sep + value + sep + ((double) (System.nanoTime() - startTime) / 1000000000.0) + sep;
        m_dataAdder.append(str);
    }

    public static void newLine() {
        m_dataAdder.append("\n");
    }

    public static void writeFromQueue() {

        try {
            if (m_dataAdder.length() > 0) {
                m_DataLogFile.write(m_dataAdder.toString());
                m_dataAdder.delete(0, m_dataAdder.length() - 1);
            }
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
    }
    public static void close() {
        m_DataLogFile.flush();
        m_DataLogFile.close();
    }

    public static String getFileName() {
        return f.getAbsolutePath();
    }
}