package edu.arizona.cs.learn.experimental.bottomup;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;
import edu.arizona.cs.learn.util.Utils;

public class RandomizedFluents {
	
	public static int SAMPLES = 1000;

	private Map<Integer,List<Variable>> _vMap;
	private Map<Integer,Double> _timeMap;
	
	private Map<AllenRelation,Integer> _cMap;
	private Map<AllenRelation,List<Integer>> _sMap;
	
	private Map<AllenRelation,DistroInfo> _dMap;
	
	/**
	 * Convert from the list of intervals into a list of
	 * variables representation.
	 * @param eMap
	 */
	public void convert(Map<Integer,List<Interval>> eMap) { 
		_vMap = new TreeMap<Integer,List<Variable>>();
		_timeMap = new TreeMap<Integer,Double>();
		
		_cMap = count(eMap);
		_sMap = new HashMap<AllenRelation,List<Integer>>();
		
		for (Integer key : eMap.keySet()) { 

			double maxTime = 0;
			Map<String,List<Interval>> propMap = new HashMap<String,List<Interval>>();

			for (Interval interval : eMap.get(key)) { 
				maxTime = Math.max(maxTime, interval.end);

				List<Interval> examples = propMap.get(interval.name);
				if (examples == null) { 
					examples = new ArrayList<Interval>();
					propMap.put(interval.name, examples);
				}
				examples.add(interval);
			}
			_timeMap.put(key, maxTime);
			
			// At this point all of the propositions are mapped to its 
			// list of propositions... therefore we can begin to make ..
			// on off lists.
			List<Variable> varList = new ArrayList<Variable>();
			for (String propName : propMap.keySet()) { 
				List<Interval> list = propMap.get(propName);
				Collections.sort(list, Interval.starts);
				
				Variable v = new Variable(propName);
				
				int last = 0;
				for (Interval interval : list) {
					double length = interval.end - interval.start;
					v.onList.add(length / maxTime);
					
					double offLength = interval.start - last;
					if (offLength > 0) 
						v.offList.add(offLength / maxTime);
					last = interval.end;
				}
				
				// Make sure we catch the boundary conditions
				Interval e = list.get(list.size()-1);
				if (e.end < maxTime) {
					double offLength = maxTime - e.end;
					v.offList.add(offLength / maxTime);
				}
				varList.add(v);
			}
			_vMap.put(key, varList);
		}
	}
	
	/**
	 * After converting the data into the lengths of the sticks (on and off)
	 * then this method will generate a single shuffling of *all* of the
	 * episodes
	 */
	public void shuffle() { 
		// Task 1: shuffle the variables into random episodes
		//   Subtask 1: Organize variables by proposition name
		Map<String,List<Variable>> propMap = new HashMap<String,List<Variable>>();
		for (List<Variable> list : _vMap.values()) { 
			for (Variable v : list) {
				List<Variable> dest = propMap.get(v.prop);
				if (dest == null) {
					dest = new ArrayList<Variable>();
					propMap.put(v.prop, dest);
				}
				dest.add(v);
			}
		}
		
		//  Subtask 2: Assign variables to random episodes
		Map<Integer,List<Variable>> rMap = new HashMap<Integer,List<Variable>>();
		List<Integer> eids = new ArrayList<Integer>(_vMap.keySet()); 
		for (List<Variable> vars : propMap.values()) { 
			// Shuffle the episode ids to change the order
			Collections.shuffle(eids);
			for (int i = 0; i < vars.size(); ++i) {
				List<Variable> list = rMap.get(eids.get(i));
				if (list == null) { 
					list = new ArrayList<Variable>();
					rMap.put(eids.get(i), list);
				}
				list.add(vars.get(i));
			}
		}
		
		// Task 2: For each of the newly created episodes, we need
		// to shuffle the variables.
		Map<Integer,List<Interval>> map = new TreeMap<Integer,List<Interval>>();
		for (Integer key : rMap.keySet()) { 
			map.put(key, shuffle(rMap.get(key), _timeMap.get(key)));
		}
		
		// Task 3: Count all of the allen relations and add this shuffle to
		// the _sMap so that we can build our sampling distribution of AllenRelations
		Map<AllenRelation,Integer> cMap = count(map);
		for (AllenRelation key : cMap.keySet()) { 
			List<Integer> dist = _sMap.get(key);
			if (dist == null) {
				dist = new ArrayList<Integer>();
				_sMap.put(key, dist);
			}
			dist.add(cMap.get(key));
		}
	}

		
	/**
	 * Generate a new episode out of the given variables 
	 * that is as long as the max time
	 * @param variables
	 * @param maxTime
	 */
	public List<Interval> shuffle(List<Variable> variables, double maxTime) { 
		List<Interval> intervals = new ArrayList<Interval>();
		for (Variable v : variables) { 
			
			// we need to alternate generating intervals until
			// there are no longer any sticks in the list
			List<Double> on = new LinkedList<Double>(v.onList);
			List<Double> off = new LinkedList<Double>(v.offList);
			boolean offList = true;

			if (on.size() > off.size())
				offList = false;
			
			if (on.size() == off.size() && Utils.random.nextDouble() < 0.1)
				offList = false;

			Collections.shuffle(on);
			Collections.shuffle(off);
			
			int currentTime = 0;
			while (on.size() > 0 && off.size() > 0) { 
				// select a new guy....
				if (offList) { 
					double ratio = off.remove(0);
					int length = Math.max(1, (int) Math.round(ratio * maxTime));
					currentTime += length;
					offList = false;
				}
				
				if (!offList) { 
					double ratio = on.remove(0);
					int length = Math.max(1, (int) Math.round(ratio * maxTime));
					intervals.add(Interval.make(v.prop, currentTime, currentTime+length));
					currentTime += length;
					offList = true;
				}
			}
		}
		return intervals;
	}
	
	private Map<AllenRelation,Integer> count(Map<Integer,List<Interval>> eMap) { 
		Map<AllenRelation,Integer> map = new HashMap<AllenRelation,Integer>();
		for (Integer key : eMap.keySet()) { 
			Map<AllenRelation,Integer> cntMap = count(eMap.get(key));
			for (AllenRelation ar : cntMap.keySet()) { 
				Integer count = map.get(ar);
				if (count == null)
					count = 0;
				map.put(ar, count+cntMap.get(ar));
			}
		}
		return map;
	}
	
	/**
	 * Count the Allen relations that occur in the intervals.
	 * @param intervals
	 * @return
	 */
	private Map<AllenRelation,Integer> count(List<Interval> intervals) { 
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
	
	public void updateDistroInfo() { 
		_dMap = new HashMap<AllenRelation,DistroInfo>();
		for (AllenRelation key : _cMap.keySet()) { 
			List<Integer> dist = _sMap.get(key);
			
			_dMap.put(key, new DistroInfo(key, _cMap.get(key), dist));
		}
	}
	
	/**
	 * Print out some information about the shuffling and the distribution
	 */
	public void print() { 
		System.out.println("Counts --- " + _cMap.size() + " ** Shuffle --- " + _sMap.size());
		
		// Just print out the relations that we actually care about.
		List<DistroInfo> values = new ArrayList<DistroInfo>(_dMap.values());
		Collections.sort(values, new Comparator<DistroInfo>() {
			@Override
			public int compare(DistroInfo o1, DistroInfo o2) {
				int retValue = Double.compare(o1.zC, o2.zC);
				if (retValue != 0)
					return retValue*-1;
				
				return o1.relation.getKey().compareTo(o2.relation.getKey());
			} 
			
		});

		for (DistroInfo d : values) { 
//			if (d.seenCount < 5)
//				continue;
			
			if (d.zC < d.zCU)
				continue; 
			
			System.out.println("  " + d.relation + " -- " + d.seenCount + " -- " + d.zC);
		}
	}
	
	public static void main(String[] args) { 
		RandomizedFluents rf = new RandomizedFluents();
		Map<Integer,List<Interval>> eMap = Utils.load(new File("data/input/chpt1-approach.lisp"));
		
		rf.convert(eMap);
		for (int i = 0; i < SAMPLES; ++i)  
			rf.shuffle();
		
		rf.updateDistroInfo();
		rf.print();
	}
	
}

class Variable { 
	public String prop;
	
	// onList and offList will store all
	// of the percentages that this proposition
	// was on for a specific episode.
	public List<Double> onList;
	public List<Double> offList;
	
	public Variable(String prop) { 
		this.prop = prop;
		
		onList = new ArrayList<Double>();
		offList = new ArrayList<Double>();
	}
}

class DistroInfo { 
	
	public AllenRelation relation;

	public int seenCount;
	public List<Integer> distribution;
	
	public SummaryStatistics summary;
	
	public double mean;
	public double sd;
	
	public double zC;
	public double zCU;
	public double zCL;
	
	public DistroInfo(AllenRelation relation, int seenCount, List<Integer> distro) { 
		this.relation = relation;
		this.seenCount = seenCount;
		
		if (distro == null)  
			distribution = new ArrayList<Integer>();
		else  
			distribution = new ArrayList<Integer>(distro);

		while (distribution.size() < RandomizedFluents.SAMPLES) 
			distribution.add(0);

		SummaryStatistics ss = new SummaryStatistics();
		for (Integer i : distribution)
			ss.addValue(i);
		mean = ss.getMean();
		sd = ss.getStandardDeviation();
		
		zC = ((double) this.seenCount - mean) / sd;
		
		Collections.sort(distribution);
		int quantile = (int) Math.floor((double) distribution.size() * 0.95);
		
		double upper = distribution.get(quantile);
		zCU = (upper - mean) / sd;
		
		double lower = distribution.get(distribution.size() - quantile);
		zCL = (lower - mean) / sd;
	}
}