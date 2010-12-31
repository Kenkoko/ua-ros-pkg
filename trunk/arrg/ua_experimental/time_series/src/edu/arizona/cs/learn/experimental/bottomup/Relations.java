package edu.arizona.cs.learn.experimental.bottomup;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;
import edu.arizona.cs.learn.util.Utils;

public class Relations {
	
	public static Map<String,Integer> count(List<Interval> intervals, Comparator<Interval> sorter) { 
		Collections.sort(intervals, sorter);
		
		Map<String,Integer> map = new HashMap<String,Integer>();
		for (int i = 0; i < intervals.size(); ++i) { 
			Interval i1 = intervals.get(i);
			for (int j = i+1; j < intervals.size(); ++j) { 
				Interval i2 = intervals.get(j);
				
				if (!Utils.LIMIT_RELATIONS || i2.start - i1.end < Utils.WINDOW) { // or 5 for most things....
					String relation = AllenRelation.get(i1, i2);
					AllenRelation allen = new AllenRelation(relation, i1, i2);
					
					Integer count = map.get(allen.getKey());
					if (count == null) {
						count = 0;
					}
					
					map.put(allen.getKey(), count+=1);
				}
			}
		}
		
		return map;
	}

	/**
	 * Count the Allen relations that occur in the intervals.
	 * @param intervals
	 * @return
	 */
	public static Map<AllenRelation,Integer> count(List<Interval> intervals) { 
		Map<AllenRelation,Integer> map = new HashMap<AllenRelation,Integer>();

		Collections.sort(intervals, Interval.eff);
		for (int i = 0; i < intervals.size(); ++i) { 
			Interval i1 = intervals.get(i);
			for (int j = i+1; j < intervals.size(); ++j) { 
				Interval i2 = intervals.get(j);
				
				if (!Utils.LIMIT_RELATIONS || i2.start - i1.end < Utils.WINDOW) { // or 5 for most things....
					String relation = AllenRelation.get(i1, i2);
					AllenRelation allen = new AllenRelation(relation, i1, i2);
					
					Integer count = map.get(allen);
					if (count == null)
						count = 0;
					map.put(allen, count+1);
				}
			}
		}
		return map;
	}
}
