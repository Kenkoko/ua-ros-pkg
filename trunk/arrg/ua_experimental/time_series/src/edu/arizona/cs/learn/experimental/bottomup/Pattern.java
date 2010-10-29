package edu.arizona.cs.learn.experimental.bottomup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import edu.arizona.cs.learn.timeseries.model.Interval;

public class Pattern {

	private String _name;
	private int _size;
	
	private Map<Integer,Map<String,IntervalSet>> _examples;
	
	public Pattern(String name, int size) { 
		_name = name;
		_size = size;
		
		_examples = new TreeMap<Integer,Map<String,IntervalSet>>();
	}
	
	public String name() { 
		return _name;
	}
	
	public int size() { 
		return _size;
	}
	
	public int episodeCount() {
		return _examples.size();
	}
	
	public String toString() { 
		return _name;
	}
	
	public int hashCode() { 
		return _name.hashCode();
	}
	
	/**
	 * Return the examples for a given episode.
	 * @param episodeId
	 * @return
	 */
	public List<IntervalSet> getExamples(int episodeId) { 
		Map<String,IntervalSet> map = _examples.get(episodeId);
		if (map == null)
			return null;
		return new ArrayList<IntervalSet>(map.values());
	}
	
	/**
	 * See if this episode is part of the examples already.
	 * If not add it.
	 * @param episodeId
	 * @return
	 */
	private Map<String,IntervalSet> findOrAdd(int episodeId) { 
		Map<String,IntervalSet> map = _examples.get(episodeId);
		if (map == null) { 
			map = new HashMap<String,IntervalSet>();
			_examples.put(episodeId, map);
		}
		return map;
	}
	
	/**
	 * Add the list of intervals as an example of this pattern.
	 * @param episodeId
	 * @param example
	 */
	public void add(int episodeId, IntervalSet example) { 
		Map<String,IntervalSet> map = findOrAdd(episodeId);
		if (map.containsKey(example.intervals().toString()))  
			throw new RuntimeException("Adding something already defined: " + example.intervals());

		map.put(example.intervals().toString(), example);
	}

	/**
	 * Return the "Pattern" name of the list of intervals.
	 * Based on  the paper by Iyad Batal that uses a subset
	 * of the Allen relations.
	 * @param intervals
	 * @return
	 */
	public static String getName(List<Interval> intervals) { 
//		System.out.println("getName(): " + intervals);
		Collections.sort(intervals, Interval.esf);
		StringBuilder buf = new StringBuilder(intervals.get(0).name);
		for (int i = 1; i < intervals.size(); ++i) { 
			Interval i1 = intervals.get(i-1);
			Interval i2 = intervals.get(i);
			
			if (i1.end <= i2.start)
				buf.append(" b " + i2.name);
			else 
				buf.append(" o " + i2.name);
		}
		return buf.toString();
	}
	
	/**
	 * Return the "Pattern" name of the list of intervals.
	 * Based on  the paper by Iyad Batal that uses a subset
	 * of the Allen relations.
	 * @param interval
	 * @return
	 */
	public static String getName(Interval interval) { 
		return interval.name;
	}
}
