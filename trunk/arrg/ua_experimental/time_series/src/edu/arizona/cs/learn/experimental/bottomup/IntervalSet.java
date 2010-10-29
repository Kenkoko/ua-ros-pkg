package edu.arizona.cs.learn.experimental.bottomup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.arizona.cs.learn.timeseries.model.Interval;

public class IntervalSet {

	private int _start;
	private int _end;
	
	private List<Interval> _intervals;
	
	public IntervalSet() { 
		_intervals = new ArrayList<Interval>();
		_start = Integer.MAX_VALUE;
		_end = 0;
	}
	
	public IntervalSet(IntervalSet is1, IntervalSet is2) { 
		this();
		
		if (is1.size() != is2.size())  
			throw new RuntimeException("ERROR: " + 
					is1.intervals() + " .... " + 
					is2.intervals());

		
		_intervals.addAll(is1._intervals);
		for (Interval i : is2._intervals) { 
			if (!_intervals.contains(i))
				_intervals.add(i);
		}
		
		if (_intervals.size() != is1.size()+1)  
			throw new RuntimeException("ERROR: " + 
					is1.intervals() + " .... " + 
					is2.intervals() + " -- " + 
					_intervals);
		
		Collections.sort(_intervals, Interval.eff);
		_start = Math.min(is1._start, is2._start);
		_end = Math.min(is1._end, is2._end);
	}
	
	public int start() { 
		return _start;
	}
	
	public int end() { 
		return _end;
	}
	
	public List<Interval> intervals() { 
		return _intervals;
	}
	
	public int size() { 
		return _intervals.size();
	}
	
	public void add(Interval interval) { 
		_intervals.add(interval);
		_start = Math.min(interval.start, _start);
		_end = Math.max(interval.end, _end);
	}
	
	/**
	 * Does this IntervalSet contain any of the other
	 * intervals.
	 * @param intervals
	 * @return
	 */
	public boolean contains(IntervalSet intervalSet) { 
		for (Interval i1 : _intervals) { 
			if (intervalSet._intervals.contains(i1))
				return true;
		}
		return false;
	}
		
	/**
	 * A generic comparator that will compare two temporal objects in a sortable
	 * fashion. eff --> earliest finishing first.
	 */
	public static Comparator<IntervalSet> eff = new Comparator<IntervalSet>() {
		public int compare(IntervalSet o1, IntervalSet o2) {
			if (o1._end > o2._end)
				return 1;
			if (o1._end < o2._end)
				return -1;

			if (o1._start > o2._start)
				return 1;
			if (o1._start < o2._start)
				return -1;

			return 0;
		}

	};
	
	/**
	 * Are these two intervals sets candidates for generate a pattern of 
	 * set1.size()+1
	 * @param set1
	 * @param set2
	 * @return
	 */
	public static boolean candidate(IntervalSet set1, IntervalSet set2) { 
		int nonOverlap = 0;
		for (Interval i1 : set1.intervals()) {
			if (!set2.intervals().contains(i1))
				++nonOverlap;
		}
		
		if (nonOverlap != 1)
			return false;
		
		nonOverlap = 0;
		for (Interval i2 : set2.intervals()) { 
			if (!set1.intervals().contains(i2))
				++nonOverlap;
		}
		
		if (nonOverlap != 1)
			return false;
		
		return true;
	}
	
}
