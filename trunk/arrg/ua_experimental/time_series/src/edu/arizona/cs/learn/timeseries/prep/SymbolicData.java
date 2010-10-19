package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import edu.arizona.cs.learn.timeseries.model.Interval;

public class SymbolicData {

	public static int numEpisodes = 30;
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
	
	public static void main(String[] args) { 

		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("data/input/niall-a.lisp"));
			for (int i = 1; i <= 30; ++i) { 
				List<Interval> intervals = toEpisode("data/raw-data/wes/engine-2/f" + i);
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

		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("data/input/niall-b.lisp"));
			for (int i = 1; i <= 30; ++i) { 
				List<Interval> intervals = toEpisode("data/raw-data/wes/engine-2/g" + i);
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
}
