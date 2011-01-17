package edu.arizona.cs.learn.algorithm.bpp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.markov.BPPNode;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.graph.Edge;
import edu.uci.ics.jung.graph.DirectedGraph;

public class BPPFactory {
    private static Logger logger = Logger.getLogger(BPPFactory.class);
	
    public static String bpp(List<Interval> intervals, Comparator<Interval> sorter) {
    	List<Interval> bpp = compress(intervals, sorter);
    	
    	StringBuffer buf = new StringBuffer();
    	int max = 0;
    	for (Interval interval : bpp) 
    		max = Math.max(max, interval.end);
    	
    	for (Interval interval : bpp) { 
    		buf.append("[" + interval.name + " ");
    		
    		for (int i = 0; i < interval.start; ++i)  
    			buf.append('0');
    		
    		for (int i = interval.start; i < interval.end; ++i) 
    			buf.append('1');
    		
    		for (int i = interval.end; i < max; ++i)
    			buf.append('0');
    		
    		buf.append("]");
    	}
    	return buf.toString();
    }
    
	/**
	 * This simplifies the provided intervals so that there is a 
	 * canonical bit pattern that ignores duration.
	 * @param intervals
	 */
	public static List<Interval> compress(List<Interval> intervals, Comparator<Interval> sorter) { 
		 // this class stores some local information about the bit pattern
		 // so that we can link pairs of stuff together.
		class Data { 
			public String prop;
			public List<Boolean> stream;
			public List<Interval> intervals;
			
			public Data(String prop) {
				this.prop = prop;
				this.stream = new ArrayList<Boolean>();
				this.intervals = new ArrayList<Interval>();
			}
			
			public void add(Interval i) { 
				intervals.add(i);
			}
			
			public boolean isOn(int time) { 
				for (Interval i : intervals) { 
					if (i.on(time))
						return true;
				}
				return false;
			}
		}
		
		int minTime = Integer.MAX_VALUE;
		int maxTime = 0;
		Map<String,Data> propMap = new TreeMap<String,Data>();
		for (Interval i : intervals) { 
			Data d = propMap.get(i.name);
			if (d == null) { 
				d = new Data(i.name);
				propMap.put(i.name, d);
			}
			d.add(i);

			minTime = Math.min(minTime, i.start);
			maxTime = Math.max(maxTime, i.end);
		}
		
		int oIndex = minTime;
		boolean[] column = new boolean[propMap.size()];
		boolean[] next = new boolean[propMap.size()];
		
		// find the real start point of this pattern
		for (; allOff(intervals, oIndex); ++oIndex);
		
		// set up the initial column
		int idx = 0;
		for (Data d : propMap.values()) { 
			boolean on = d.isOn(oIndex);
			column[idx] = on;
			d.stream.add(on);
			++idx;
		}
		++oIndex;
		
		// continue this pattern adding the column when there
		// are changes (otherwise the column is ignored)
		for (; oIndex < maxTime; ++oIndex) { 
			// build the next column so that we can compare them.
			idx = 0;
			for (Data d : propMap.values()) { 
				next[idx] = d.isOn(oIndex);
				++idx;
			}
			
			// when column and next differ we need to add
			// this to the name for each row.
			if (!Arrays.equals(column, next)) {
				idx = 0;
				for (Data d : propMap.values()) { 
					column[idx] = next[idx];
					d.stream.add(next[idx]);
					++idx;
				}
			}
		}

		// we now have a relation between the intervals that should be small.
		// Now we have to convert it to a set of intervals.
		List<Interval> results = new ArrayList<Interval>();
		for (Map.Entry<String,Data> e : propMap.entrySet()) { 
			int startPos = 0;
			boolean expected = false;

			String prop = e.getKey();
			Data d = e.getValue();
			
			for (int i = 0; i < d.stream.size(); ++i) { 
				boolean on = d.stream.get(i);
				if (on != expected) { 
					// if we have just turned off.
					if (expected)  
						results.add(Interval.make(prop, startPos, i));

					expected = on;
					startPos = i;
				}
			}
			
			if (startPos < d.stream.size() && expected) { 
				results.add(Interval.make(prop, startPos, d.stream.size()));
			}
		}

		Collections.sort(results, sorter);
		return results;
	}
		
	/**
	 * Test the list of intervals to see if they are all
	 * off at the given time.
	 * @param intervals
	 * @param time
	 * @return
	 */
	public static boolean allOff(List<Interval> intervals, int time) { 
		for (Interval i : intervals) { 
			if (i.on(time))
				return false;
		}
		return true;
	}	
	
	/**
	 * Take the intervals and convert them into the minimum timeline
	 * @param props
	 * @param intervals
	 * @return
	 */
	public static char[][] timeLine(List<String> props, List<Interval> intervals) { 
		int tmpSize = 0;
		int endTime = 0;
		int startTime = Integer.MAX_VALUE;
		Map<String,List<Interval>> propMap = new TreeMap<String,List<Interval>>();
		for (Interval i : intervals) { 
			List<Interval> propIntervals = propMap.get(i.name);
			if (propIntervals == null) { 
				propIntervals = new ArrayList<Interval>();
				propMap.put(i.name, propIntervals);
			}
			
			propIntervals.add(i);
			startTime = Math.min(i.start, startTime);
			endTime = Math.max(i.end, endTime);
			
			tmpSize = Math.max(tmpSize, i.name.length());
		}
		tmpSize += 1;

		int time = (endTime - startTime);

		char[][] timeLine = new char[props.size()][time];
		// march forward through time determining the current state
		for (int i = 0; i < props.size(); ++i) { 
			String prop = props.get(i);
			for (int j = 0; j < time; ++j) { 
				timeLine[i][j] = '0';
			}

			List<Interval> propIntervals = propMap.get(prop);
			if (propIntervals == null) {
				// if we know nothing about this proposition
				// then it should all be * values.
//				for (int j = 0; j < time; ++j) { 
//					timeLine[i][j] = '*';
//				}
				continue;
			}
			
			
			for (Interval interval : propIntervals) { 
				for (int j = interval.start; j < interval.end; ++j) { 
					timeLine[i][j-startTime] = '1';
				}
			}
			
			// check the start state.
//			if (timeLine[i][0] == '0' && timeLine[i][1] == '0') 
//				timeLine[i][0] = '*';
			
			for (int j = 1; j < time-1; ++j) { 
				char left = timeLine[i][j-1];
				char us = timeLine[i][j];
				char right = timeLine[i][j+1];

//				if ((left == '*' || left == '0') && us =='0' && right == '0')
//					timeLine[i][j] = '*';
			}
			
			// check the end state.
//			if ((timeLine[i][time-2] == '0' || timeLine[i][time-2] == '*') && timeLine[i][time-1] == '0') 
//				timeLine[i][time-1] = '*';

			// I want to print the bpp just so that I can visually make sure
			// that it is correct for the time being.
//			System.out.print(prop);
//			for (int k = prop.length(); k <= tmpSize; ++k) { 
//				System.out.print(" ");
//			}
//			for (int k = 0; k < time; ++k) { 
//				System.out.print(timeLine[i][k]);
//			}
//			System.out.println();
		}

		return timeLine;
	}
	
	/**
	 * Use this method when you want to merge different paths to accepting
	 * states into one graph.
	 * @param graph
	 * @param vertices
	 * @param acceptingState
	 * @param propList
	 * @param instances
	 * @return
	 */
	public static DirectedGraph<BPPNode,Edge> graph(DirectedGraph<BPPNode,Edge> graph, Map<String,BPPNode> vertices, 
												    String acceptingState, List<String> propList, List<List<Interval>> instances) { 
		throw new RuntimeException("Revisit this in once we learn more");
	}

	public static void main(String[] args) { 
		String name = "data/input/chpt1.lisp";
		
//		Map<Integer,List<Interval>> map = SequenceFactory.load(new File(name));
//		List<Interval> episode = map.get(1);		
//		BPPFactory.renderBPP(compress(episode, Interval.eff), "/Users/wkerr/Desktop/chpt1.png");
//		BPPFactory.renderBPP(compress(episode, Interval.eff), "/Users/wkerr/Desktop/gesture-a-bpp.png");
		
//		episode = map.get(2);
//		BPPFactory.renderBPP(episode, "/Users/wkerr/Desktop/synthetic2.png");
//		BPPFactory.renderBPP(compress(episode, Interval.eff), "/Users/wkerr/Desktop/synthetic2-bpp.png");
		
		
//		sample1();
//		sample2();
		
		bppTest1();
	}
	
	public static void bppTest1() {
		List<Interval> intervals = new ArrayList<Interval>();
		intervals.add(Interval.make("a", 0, 10));
		intervals.add(Interval.make("b", 20, 25));
		intervals.add(Interval.make("c", 22, 24));
		
		List<Interval> bpp = compress(intervals, Interval.eff);
		System.out.println(bpp);
		
		char[][] timeLine = timeLine(Arrays.asList("a","b","c"), bpp);
	}
}
