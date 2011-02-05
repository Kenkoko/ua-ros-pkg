package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class UCRTimeSeries {
	
	private String _key;
	private Map<Integer,List<List<Double>>> _timeSeries;
	
	/**
	 * Load the actual file since there is a test group and a training
	 * group
	 * @param f
	 */
	private void load(File f) { 
		try {
			BufferedReader in = new BufferedReader(new FileReader(f));
			while (in.ready()) { 
				String line = in.readLine();
				String[] tokens = line.split("[ ]");

				int startIndex = 0;
				while (tokens[startIndex].trim().equals("")) { 
					++startIndex;
				}
				
				double d = Double.parseDouble(tokens[startIndex]);
				System.out.println("Class: " + d);
				int classId = (int) d;
				
				List<Double> episode = new ArrayList<Double>();
				for (int i = startIndex+1; i < tokens.length; ++i) {
					if (!"".equals(tokens[i].trim()))
						episode.add(Double.parseDouble(tokens[i]));
				}
				
				List<List<Double>> episodes = _timeSeries.get(classId);
				if (episodes == null) { 
					episodes = new ArrayList<List<Double>>();
					_timeSeries.put(classId, episodes);
				}
				episodes.add(episode);
			}
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}

	/**
	 * This method reads in the training data and converts
	 * it into a CSV format with one file per class.  This
	 * will allow me to look at the classes and determine 
	 * if there is a good pattern to learn from them.
	 * @param iPath
	 * @param key
	 * @param loadTest
	 */
	public void load(String iPath, String key, boolean loadTest) { 
		_key = key;
		_timeSeries = new TreeMap<Integer,List<List<Double>>>();
		
		load(new File(iPath + key + "/" + key + "_TRAIN"));
		if (loadTest)
			load(new File(iPath + key + "/" + key + "_TEST"));
	}
	
	/**
	 * Write the time series data to a CSV file that can be 
	 * easily loaded into R or JMP and will allow us to visualize
	 * the original time series data.
	 * @param oPath
	 * @throws Exception
	 */
	public void writeCSV(String oPath) throws Exception { 
		PrintStream out = new PrintStream(new File(oPath + _key + "/" + _key + ".csv"));
		out.println("class,episode,time,value");
		for (Integer classId : _timeSeries.keySet()) { 
			List<List<Double>> episodes = _timeSeries.get(classId);
			for (int i = 0; i < episodes.size(); ++i) { 
				List<Double> episode = episodes.get(i);
				for (int j = 0; j < episode.size(); ++j) { 
					out.println(classId + "," + i + "," + j + "," + episode.get(j));
				}
			}
		}
		out.close();
	}
	
	public void printStats() { 
		System.out.println("Key: " + _key);
		for (Integer key : _timeSeries.keySet()) { 
			System.out.println("\tClass: " + key + " -- " + _timeSeries.get(key).size());
		}
	}
	
	public static void main(String[] args) throws Exception { 
		UCRTimeSeries ucr = new UCRTimeSeries();
		
		ucr.load("/Users/wkerr/Sync/data/UCR-TimeSeries/", "FaceFour", false);
		ucr.printStats();
		ucr.writeCSV("/Users/wkerr/Sync/data/UCR-TimeSeries/");
	}
}
