package edu.arizona.cs.learn.timeseries.prep;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.model.Interval;

/**
 * This class contains a set of helper functions 
 * that take as input a time-series represented
 * as a list of doubles
 * @author wkerr
 *
 */
public class TimeSeries {
	
	private static Map<Integer,List<Double>> breaksMap;
	
	static { 
		breaksMap = new HashMap<Integer,List<Double>>();
		breaksMap.put(2, Arrays.asList(0.0));
		breaksMap.put(3, Arrays.asList(-0.43, 0.43));
		breaksMap.put(4, Arrays.asList(-0.67, 0.0, 0.67));
		breaksMap.put(5, Arrays.asList(-0.84, -0.25, 0.25, 0.84));
		breaksMap.put(6, Arrays.asList(-0.97, -0.43, 0.00, 0.43, 0.97));
		breaksMap.put(7, Arrays.asList(-1.07, -0.57, -0.18, 0.18, 0.57, 1.07));
		breaksMap.put(8, Arrays.asList(-1.15, -0.67, -0.32, 0.00, 0.32, 0.67, 1.15));
 	}
	
	/**
	 * When you know that the time series only contains doubles and
	 * each line represents a single value in the time series
	 * then call load.
	 * @param file
	 * @param hasHeaders
	 * @param sep
	 * @return
	 */
	public static Map<String,List<Double>> load(String file, boolean hasHeaders, String sep) { 
		Map<String,List<Double>> map = new HashMap<String,List<Double>>();
		List<String> headers = new ArrayList<String>();

		try { 
			BufferedReader in = new BufferedReader(new FileReader(file));
			
			if (hasHeaders) { 
				String line = in.readLine();
				String[] tokens = line.split(sep);
				
				for (String token : tokens) {
					map.put(token, new ArrayList<Double>());
					headers.add(token);
				}
			}
			
			boolean firstTime = true;
			while (in.ready()) { 
				String line = in.readLine();
				String[] tokens = line.split("[ ]");
				
				if (!hasHeaders && firstTime) { 
					for (int i = 0; i < tokens.length; ++i) { 
						String header = "var" + i;
						map.put(header, new ArrayList<Double>());
						headers.add(header);
					}
					firstTime = false;
				}

				for (int i = 0; i < tokens.length; ++i) { 
					map.get(headers.get(i)).add(Double.parseDouble(tokens[i]));
				}
			}
			in.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
		
		return map;
	}

	/**
	 * standardize the time series by subtracting the mean
	 * and dividing by the standard deviation
	 * 
	 * Result is a mean of 0 and a standard deviation of 1
	 * @param timeSeries
	 * @return
	 */
	public static List<Double> standardize(List<Double> timeSeries) { 
		SummaryStatistics ss = new SummaryStatistics();
		for (Double d : timeSeries)
			if (Double.compare(d, Double.NaN) != 0)
				ss.addValue(d);
		
		double mean = ss.getMean();
		double sd = ss.getStandardDeviation();
		// if the standard deviation is 0 then we don't divide
		List<Double> result = new ArrayList<Double>();
		if (Double.compare(sd, 0.0) == 0) { 
			for (Double d : timeSeries) {
				if (Double.compare(d, Double.NaN) == 0)
					result.add(Double.NaN);
				else
					result.add(d - mean);
				
			}
		} else { 
			for (Double d : timeSeries) {
				if (Double.compare(d, Double.NaN) == 0)
					result.add(Double.NaN);
				else
					result.add((d - mean) / sd);
			}
		}
		return result;
	}
	
	/**
	 * Return the first difference of the given time series
	 * Subtract each value from it's predecessor.  We insert a 
	 * NaN at the front of the list to ensure that there are the
	 * same number in the delta 
	 * @param timeSeries
	 * @return
	 */
	public static List<Double> diff(List<Double> timeSeries) { 
		List<Double> results = new ArrayList<Double>();
		
		results.add(Double.NaN);
		for (int i = 0; i < timeSeries.size()-1; ++i) {
			double v1 = timeSeries.get(i);
			double v2 = timeSeries.get(i+1);

			if (Double.compare(v1,Double.NaN) == 0 || Double.compare(v2,Double.NaN) == 0)
				results.add(Double.NaN);
			else
				results.add(v2-v1);
		}
		
		return results;
	}
	
	/**
	 * The given time series is circular and care must be taken
	 * to avoid getting the wrong diff values when it rolls over
	 * 
	 * The boundary values are given
	 * @param timeSeries
	 * @param min
	 * @param max
	 * @return
	 */
	public static List<Double> circularDiff(List<Double> timeSeries, double min, double max) { 
		List<Double> results = new ArrayList<Double>();
		
		double delta = max - min;
		results.add(Double.NaN);
		for (int i = 0; i < timeSeries.size()-1; ++i) { 
			double v1 = timeSeries.get(i);
			double v2 = timeSeries.get(i+1);
			
			if (Double.compare(v1,Double.NaN) == 0 || Double.compare(v2,Double.NaN) == 0)
				results.add(Double.NaN);
			else { 
				double d1 = v2 - v1;
				double d2 = (v2 + delta) - v1;
				double d3 = v2 - (v1 + delta);
				
				if (Math.abs(d1) <= Math.abs(d2) && Math.abs(d1) <= Math.abs(d3))
					results.add(d1);
				else if (Math.abs(d2) <= Math.abs(d1) && Math.abs(d2) <= Math.abs(d3))
					results.add(d2);
				else if (Math.abs(d3) <= Math.abs(d1) && Math.abs(d3) <= Math.abs(d2))
					results.add(d3);
				else
					throw new RuntimeException("Unknown: " + d1 + " " + d2 + " " + d3);
			}
		}
		
		return results;
	}
	
	/**
	 * Filter the time series in order to do some smoothing and make it
	 * less jaggy.
	 * @param timeSeries
	 * @param k
	 * @return
	 */
	public static List<Double> linearFilter(List<Double> timeSeries, int k) { 
		List<Double> results = new ArrayList<Double>();
		
		// First add all of the NaN that will not be calculated
		for (int i = 0; i < k; ++i) 
			results.add(Double.NaN);

		for (int i = k; i < timeSeries.size()-k; ++i) { 
			
			// calculate y_i by summing over the window
			double y = 0;
			for (int j = -k; j <= k; ++j) { 
				y += timeSeries.get(i+j);
			}
			y /= (2*k + 1);

			results.add(y);
		}
		
		for (int i = 0; i < k; ++i)  
			results.add(Double.NaN);
		
		return results;
	}

	/**
	 * Generate a list of SAX symbols from the time series.  The number
	 * of candidate symbols is determined by the number of breaks selected.
	 * @param timeSeries
	 * @param numBreaks
	 * @return
	 */
	public static List<String> sax(List<Double> timeSeries, int numBreaks) { 
		List<String> results = new ArrayList<String>();

		List<Double> breaks = breaksMap.get(numBreaks);
		for (Double d : timeSeries) { 
			if (Double.compare(d, Double.NaN) == 0) {
				results.add("NaN");
			} else {
				int index = 0;
				while (index < breaks.size() && d > breaks.get(index))
					++index;
				
				results.add((numBreaks-index)+"");
			}
		}
		return results;
	}
	
	/**
	 * Generate a list of symbols based on the shape of the time series.
	 * @param deltaTimeSeries
	 * @return
	 */
	public static List<String> sdl(List<Double> deltaTimeSeries) { 
		List<Double> breakpoints = Arrays.asList(-0.001, 0.001);
		List<String> classes = Arrays.asList("down", "stable", "up");
		return sdl(deltaTimeSeries, breakpoints, classes);
	}
	
	
	/**
	 * Generate a list of symbols based on the shape of the the time series
	 * The time series is assumed to be standardized so that we can reason
	 * about the deltas in a uniform way
	 * @param deltaTimeSeries
	 * @param breakpoints
	 * @param classes
	 * @return
	 */
	public static List<String> sdl(List<Double> deltaTimeSeries, List<Double> breakpoints, List<String> classes) { 
		List<String> results = new ArrayList<String>();

		for (Double d : deltaTimeSeries) { 
			if (Double.compare(d, Double.NaN) == 0) {
				results.add("NaN");
			} else {

				boolean found = false;
				for (int i = 0; i < breakpoints.size() && !found; ++i) { 
					if (d < breakpoints.get(i)) {
						results.add(classes.get(i));
						found = true;
					}
				}
				
				if (!found)
					results.add(classes.get(classes.size()-1));
			}
		}
		return results;
	}
	
	/**
	 * Fit regression lines to the time series.
	 * @param timeSeries
	 * @return
	 */
	public static List<String> regression(List<Double> timeSeries) { 
		List<Double> breakpoints = Arrays.asList(-0.5, 0.5);
		List<String> classes = Arrays.asList("down", "stable", "up");
		return regression(timeSeries, breakpoints, classes);
	}
	
	/**
	 * Fit regression lines to the time series. 
	 * @param timeSeries
	 * @param breakpoints
	 * @param classes
	 * @return
	 */
	public static List<String> regression(List<Double> timeSeries, List<Double> breakpoints, List<String> classes) { 
		List<String> results = new ArrayList<String>();

		List<Edge> edges = LinearRegression.fitRegressionLines(timeSeries);
		
		for (Edge edge : edges) { 
			double x0 = edge.startIndex;
			double x1 = edge.endIndex;

			if (edge.isNaN()) { 
				for (int i = 0; i < (x1-x0); ++i) 
					results.add("NaN");
				continue;
			}
			
			double y0 = edge.regression.predict(x0);
			double y1 = edge.regression.predict(x1);

			double theta = Math.atan2((y1-y0), (x1-x0));
			double degrees = Math.toDegrees(theta);
			
			String symbol = null;
			for (int i = 0; i < breakpoints.size() && symbol == null; ++i) { 
				if (degrees < breakpoints.get(i)) { 
					symbol = classes.get(i);
				}
			}
			
			if (symbol == null)
				symbol = classes.get(classes.size()-1);
			
			for (int i = 0; i < (x1-x0); ++i) { 
				results.add(symbol);
			}
		}
		return results;
	}
	
	
	public static List<Interval> toIntervals(String key, List<String> symbolTS) {
		List<Interval> results = new ArrayList<Interval>();
		
		String expected = symbolTS.get(0);
		int idx = 0;
		
		for (int i = 1; i < symbolTS.size(); ++i) { 
			if (expected.equals(symbolTS.get(i))) 
				continue;
			
			if (!"NaN".equals(expected)) 
				results.add(Interval.make("(" + key + " " + expected + ")", idx, i));
			
			idx = i;
			expected = symbolTS.get(i);
		}
		
		if (!"NaN".equals(expected)) 
			results.add(Interval.make("(" + key + " " + expected + ")", idx, symbolTS.size()));
		
		return results;
	}
	
	public static List<Interval> booleanToIntervals(String key, List<Boolean> values) { 
		List<Interval> results = new ArrayList<Interval>();
		
		boolean expected = values.get(0);
		int idx = 0;
		
		for (int i = 1; i < values.size(); ++i) { 
			if (expected == values.get(i)) 
				continue;

			if (expected != false) 
				results.add(Interval.make(key, idx, i));
			
			idx = i;
			expected = values.get(i);
		}
		
		if (expected != false) 
			results.add(Interval.make(key, idx, values.size()));
		
		return results;
	}
	
	public static void main(String[] args) { 
		// test the filtering....
		List<Double> ts = Arrays.asList(
				12.0, 1.0, 5.0, 4.0, 13.0, 
				10.0, 13.0, 7.0, 3.0, 4.0, 
				13.0, 12.0, 15.0, 9.0, 4.0, 
				10.0, 10.0, 3.0, 5.0, 9.0);
		
		List<Double> r1 = linearFilter(ts, 5);
		System.out.println(r1);
		System.out.println(r1.size());
		
		// test the first-difference....
		List<Double> r2 = diff(r1);
		System.out.println(r2);
		System.out.println(r2.size());
		
		List<Double> r3 = diff(ts);
		System.out.println(r3);
		System.out.println(r3.size());
		
		List<Double> s1 = standardize(r1);
		System.out.println(s1);
		
		System.out.println(sax(s1, 6));
		System.out.println(sdl(r2));
		
		System.out.println(toIntervals("test", sax(s1, 6)));
		System.out.println(toIntervals("test", sdl(r2)));
		
		List<Double> s2 = standardize(ts);
		
		System.out.println(sax(s2, 6));
		System.out.println(regression(s2));
	}
}
