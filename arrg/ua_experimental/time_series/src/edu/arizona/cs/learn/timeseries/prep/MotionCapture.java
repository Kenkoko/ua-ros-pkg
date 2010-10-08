package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.Utils;

/**
 * Convert motion capture data into a format more friendly
 * for use by the signature systems.
 * @author wkerr
 *
 */
public class MotionCapture {

	public static void multithread() { 
		ExecutorService execute  = Executors.newFixedThreadPool(Utils.numThreads);
		
		class MultiCallable implements Callable<Object> {
			private int _subjectId;
			private String _className;
			private List<Integer> _ids;
			
			public MultiCallable(int subjectId, String className, List<Integer> ids) { 
				_subjectId = subjectId;
				_className = className;
				_ids = ids;
			}
			
			@Override
			public Object call() throws Exception {
				doit(_subjectId, _className, _ids);
				return null;
			} 
		}
		
		List<Future<Object>> future = new ArrayList<Future<Object>>();
		Map<String,List<Integer>> classMap = new HashMap<String,List<Integer>>();
		classMap.put("jump", new ArrayList<Integer>());
		classMap.put("walk", new ArrayList<Integer>());
		classMap.put("run", new ArrayList<Integer>());

		for (int i = 1; i <= 7; ++i) 
			classMap.get("jump").add(i);
		classMap.get("run").add(8);
		classMap.get("jump").add(9);
		classMap.get("jump").add(10);
		for (int i = 11; i <= 34; ++i) 
			classMap.get("walk").add(i);
		for (int i = 35; i <= 46; ++i) 
			classMap.get("run").add(i);
		classMap.get("walk").add(47);
		for (int i = 48; i <= 57; ++i) 
			classMap.get("run").add(i);
		classMap.get("walk").add(58);
		
		for (String className : classMap.keySet()) {
			MultiCallable mc = new MultiCallable(16, className, classMap.get(className));
			future.add(execute.submit(mc));
		}
		
//		classMap = new HashMap<String,List<Integer>>();
//		classMap.put("jump", new ArrayList<Integer>());
//		classMap.put("walk", new ArrayList<Integer>());
//		
//		for (int i = 1; i <= 36; ++i) 
//			classMap.get("walk").add(i);
//		classMap.get("walk").add(57);
//		classMap.get("walk").add(62);
//		for (int i = 37; i <= 57; ++i) 
//			classMap.get("jump").add(i);
//		for (int i = 58; i <= 61; ++i) 
//			classMap.get("jump").add(i);
//		
//		for (String className : classMap.keySet()) {
//			MultiCallable mc = new MultiCallable(91, className, classMap.get(className));
//			future.add(execute.submit(mc));
//		}

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
	
	
	
	public static void doit(int subjectId, String className, List<Integer> ids) {
		List<List<Interval>> episodes = new ArrayList<List<Interval>>();
		SummaryStatistics ss = new SummaryStatistics();
		for (Integer id : ids) { 
			Map<String,List<Double>> map = loadEpisode(subjectId, className, id);

			List<Interval> intervals = new ArrayList<Interval>();
			for (Map.Entry<String, List<Double>> entry : map.entrySet()) { 
				String key = entry.getKey();
				List<Double> column = entry.getValue();

				List<Double> column2 = TimeSeries.standardize(column);
				List<Double> breakpoints = Arrays.asList(-2.0, -0.5, 0.5, 2.0);
				List<String> classes = Arrays.asList("steep-down", "down", "stable", "up", "steep-up");

				List<String> regression = TimeSeries.regression(column2, breakpoints, classes);
				intervals.addAll(TimeSeries.toIntervals(key, regression));

				// Now for each variable, we first smooth it.  
				column = TimeSeries.linearFilter(column, 10);
				column = TimeSeries.standardize(column);

				List<String> sax = TimeSeries.sax(column, 5);
				intervals.addAll(TimeSeries.toIntervals(key, sax));

//				List<Double> breakpoints = Arrays.asList(-0.0025, 0.0025);
//				List<String> classes = Arrays.asList("down", "stable", "up");
//				List<Double> delta = TimeSeries.diff(column);
//				List<String> sdl = TimeSeries.sdl(delta, breakpoints, classes);
//				intervals.addAll(TimeSeries.toIntervals(key, sdl));

			}
			episodes.add(intervals);
			ss.addValue(intervals.size());
		}
		System.out.println("Average size: " + ss.getMean() + " " + ss.getStandardDeviation());
		Utils.writeEpsiodes("data/input/mocap-" + subjectId + "-" + className + ".lisp", episodes);
	}
	
	private static Map<String,List<Double>> loadEpisode(int subjectId, String className, int id) { 
		Map<String,List<Double>> eMap = new HashMap<String,List<Double>>();
		
		String subject = subjectId + "";
		if (subjectId < 10)
			subject = "0" + subject;
		
		String episode = id + "";
		if (id < 10)
			episode = "0" + episode;
		
		try { 
			String base = "data/raw-data/cmu-mocap/";
			String inF = base + "original/" + subject + "/" + subject + "_" + episode + ".amc";
			BufferedReader in = new BufferedReader(new FileReader(inF));
			
			String outF = base + "updated/" + subjectId + "-" + className + "-" + id + ".csv"; 
			BufferedWriter out = new BufferedWriter(new FileWriter(outF));

			// The first three lines actually contain nothing of import to this 
			// tool.
			in.readLine();
			in.readLine();
			in.readLine();
			
			while (in.ready()) { 
				Map<String,Double> map = singleStep(in,out);
				for (String s : map.keySet()) {
					if (!eMap.containsKey(s)) { 
						eMap.put(s, new ArrayList<Double>());
					}
					eMap.get(s).add(map.get(s));
				}
			}
			
			in.close();
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		return eMap;
	}
	
	private static Map<String,Double> singleStep(BufferedReader in, BufferedWriter out) throws Exception { 
		Map<String,Double> map = new TreeMap<String,Double>();

		int timeStep = Integer.parseInt(in.readLine());
//		System.out.println("Time Step: " + timeStep);

		// These are the anchor points for the motion data.
		for (int i = 0; i < 29; ++i) { 
			String line = in.readLine();
			String[] tokens = line.split("[ ]");
			String varBase = tokens[0];
			for (int j = 1; j < tokens.length; ++j) { 
				String var = varBase + "" + j;
				double value = Double.parseDouble(tokens[j]);

				map.put(var, value);
			}
		}


		// we need to write a header
		if (timeStep == 1) { 
			out.write("\"timeStep\"");
			for (String key : map.keySet()) { 
				out.write(",\"" + key + "\"");
			}
			out.write("\n");
		}

//		System.out.println("  " + map.size());
		out.write(timeStep + "");
		for (String key : map.keySet()) { 
			out.write("," + map.get(key));
		}
		out.write("\n");

		return map;
	}
	
	
	public static void main(String[] args) { 
		multithread();
	}
}
