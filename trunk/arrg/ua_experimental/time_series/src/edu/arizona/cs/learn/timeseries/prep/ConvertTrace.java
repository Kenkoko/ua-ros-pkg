package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.Utils;

public class ConvertTrace {

	public static int _numEpisodes = 100;
	
	public static void main(String[] args) { 
		// Trace Dataset has 16 classes and each class 100 examples.
		// Each example consists of 4 variables.
		multithread();
	}
	
	public static void singlethread() { 
		
		for (int i = 1; i <= 16; ++i) { 
			doit(i);
		}
	}
	
	public static void multithread() { 
		ExecutorService execute  = Executors.newFixedThreadPool(Utils.numThreads);
		
		class TraceCallable implements Callable<Object> {
			private int _index;
			
			public TraceCallable(int index) { 
				_index = index;
			}
			
			@Override
			public Object call() throws Exception {
				doit(_index);
				return null;
			} 
		}
		
		List<Future<Object>> future = new ArrayList<Future<Object>>();
		for (int i = 1; i <= 16; ++i) { 
			future.add(execute.submit(new TraceCallable(i)));
		}
		
		for (Future<Object> results : future) {
			try {
				Object o = results.get();
			} catch (InterruptedException e) {
				e.printStackTrace();
			} catch (ExecutionException e) {
				e.printStackTrace();
			}
		}

		execute.shutdown();
	}
	
	private static void doit(int i) { 
		String className = i + "";
		if (i < 10) 
			className = "0" + className;

//		System.out.println("Beginning: " + className);
		SummaryStatistics ss = new SummaryStatistics();
		List<List<Interval>> episodes = new ArrayList<List<Interval>>();
		for (int j = 1; j <= _numEpisodes; ++j) { 
//			System.out.println("  Episode: " + j);
			List<Interval> episode = loadExample(className, j);
			ss.addValue(episode.size());
			
			episodes.add(episode);
		}

		System.out.println("[" + className + "] Average Episode Size: " + ss.getMean() + " " + ss.getStandardDeviation());
		Utils.writeEpsiodes("data/input/trace-" + i + ".lisp", episodes);
	}
	
	private static List<Interval> loadExample(String className, int episode) { 
		String dataFile = "data/raw-data/trace/c" + className + "_" + episode + ".dat";

		Map<String,List<Double>> map = new HashMap<String,List<Double>>();
		for (int i = 1; i <= 4; ++i) 
			map.put("var" + i, new ArrayList<Double>());

		try { 
			BufferedReader in = new BufferedReader(new FileReader(dataFile));
			while (in.ready()) { 
				String line = in.readLine();
				String[] tokens = line.split("[ ]");
				
				// the first variable is just the time step.
				// Technically there are exactly 4 variables so just process them
				for (int i = 1; i <= 4; ++i) 
					map.get("var"+i).add(Double.parseDouble(tokens[i]));
			}
			in.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		
		List<Interval> intervals = new ArrayList<Interval>();
		for (Map.Entry<String, List<Double>> entry : map.entrySet()) { 
			String key = entry.getKey();
			List<Double> column = entry.getValue();
			
			// Now for each variable, we first smooth it.  
//			column = TimeSeries.linearFilter(column, 5);
			column = TimeSeries.standardize(column);

//			List<String> sax = TimeSeries.sax(column, 6);
//			intervals.addAll(TimeSeries.toIntervals(key, sax));
//			
//			List<Double> delta = TimeSeries.diff(column);
//			List<String> sdl = TimeSeries.sdl(delta, 0.0025);
//			intervals.addAll(TimeSeries.toIntervals(key, sdl));
			
			List<Double> breakpoints = Arrays.asList(-2.0, -1.0, 1.0, 2.0);
			List<String> classes = Arrays.asList("steep-down", "down", "stable", "up", "steep-up");

			List<String> regression = TimeSeries.regression(column, breakpoints, classes);
			intervals.addAll(TimeSeries.toIntervals(key, regression));
		}
		
		return intervals;
	}
	
	//
	// Params filterWindow = 50 SAX = 8 SDL 0.0025   --- 43.84% correct
	// Params filterWindow = 30 SAX = 8 SDL 0.0025   --- 48.44% correct
	// Params filterWindow = 25 SAX = 8 SDL 0.0025   --- 52.25% correct
	// Params filterWindow = 20 SAX = 8 SDL 0.0025   --- 45.86% correct
	// Params filterWindow = 5  SAX = 6 SDL 0.0025   --- 59.39% correct
	
	// Params filterWindow 00 regression --- 47.50% correct
	// Params filterWindow 00 regression 0.5,2.0 -- 45.28% correct
	// Params filterWindow 00 regression 0.75,2.0 -- 51.19% correct
	// Params filterWindow 00 regression 	1.0,2.0  -- 46.79% correct
	// Params filterWindow 05 regression 0.75,2.0 -- 30.28% correct
	
	// Params filterWindow 05 regression --- 32.97% correct
	// Params filterWindow 10 regression --- 39.86% correct
	// Params filterWindow 25 regression --- 40.96% correct
	
	// Params filterWindow = 25 SAX = 8 regression --- 41.64% correct
	// Params filterWindow = 25 SAX = 6 regression --- 40.44% correct
	
	// Params filterWindow = 5 SAX = 6 SDL 0.0025   --- 
}
