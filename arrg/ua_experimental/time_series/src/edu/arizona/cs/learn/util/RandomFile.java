package edu.arizona.cs.learn.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.util.Vector;

import org.apache.log4j.Logger;

public class RandomFile {
    private static Logger logger = Logger.getLogger(RandomFile.class);

	public static String getPID()  {  
		String defaultStr = ((int) Math.floor(Math.random() * 1000)) + "";
		try { 
			Vector<String> commands=new Vector<String>();  
			commands.add("/bin/bash");  
			commands.add("-c");
			commands.add("echo $PPID");  
			ProcessBuilder pb=new ProcessBuilder(commands);  

			Process pr=pb.start();  
			pr.waitFor();  
			if (pr.exitValue()==0) {  
				BufferedReader outReader=new BufferedReader(new InputStreamReader(pr.getInputStream()));  
				return outReader.readLine().trim();  
			} else {  
				logger.error("Error while getting PID - " + pr.exitValue());  
				return defaultStr;
			}  
		} catch (Exception e) { 
			logger.error("Error while getting PID " + e.getMessage());
		}
		return defaultStr;
	} 
	
	public static File sequenceAlignmentFile() throws FileNotFoundException { 
		String pid = getPID();
		String threadName = Thread.currentThread().getName();
		File f = new File("/tmp/" + pid + "-" + threadName + "-sa.dat");
		return f;
	}
	
	public static String alignFile(String key) {
		String pid = getPID();
		String threadName = Thread.currentThread().getName();
		String s = "/tmp/" + pid + "-" + threadName + "-" + key + ".align";
		return s;
	}
}
