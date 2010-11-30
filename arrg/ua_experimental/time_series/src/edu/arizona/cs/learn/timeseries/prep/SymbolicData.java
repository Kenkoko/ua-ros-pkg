package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import edu.arizona.cs.learn.timeseries.model.Interval;

public class SymbolicData {

	public static int numEpisodes = 60;
	public static int numVars = 6;
	
	public static List<Interval> toEpisode(String file) { 
		List<Interval> intervals = new ArrayList<Interval>();
		try { 
			BufferedReader in = new BufferedReader(new FileReader(file));
			List<List<String>> timeSeries = new ArrayList<List<String>>();
			for (int i = 0; i < numVars; ++i) { 
				timeSeries.add(new ArrayList<String>());
			}
			
			while (in.ready()) { 
				String line = in.readLine().replaceAll("[\"]", "");
				String[] tokens = line.split("[ ]");
				
				for (int i = 0; i < tokens.length; ++i) { 
					timeSeries.get(i).add(tokens[i]);
				}
			}
			in.close();
			
			for (int i = 0; i < numVars; ++i) { 
				intervals.addAll(TimeSeries.toIntervals("var-" + (i+1), timeSeries.get(i)));
			}
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return intervals;
	}
	
	public static void convert(String prefix, String output, int numEpisodes) { 
		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter(output));
			for (int i = 1; i <= numEpisodes; ++i) { 
				List<Interval> intervals = toEpisode(prefix + i);
				out.write("(" + i + "\n");
				out.write(" (\n");
				for (Interval interval : intervals) { 
					out.write("(\"" + interval.name + "\" " + 
							interval.start + " " +
							interval.end + ")\n");
				}
				out.write(" )\n");
				out.write(")\n");
			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}
	
	public static void main(String[] args) { 
		convert("data/raw-data/wes/round-5/f", "data/input/niall-a.lisp", numEpisodes);
		convert("data/raw-data/wes/round-5/g", "data/input/niall-b.lisp", numEpisodes);
	}
}
