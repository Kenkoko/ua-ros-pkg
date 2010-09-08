package edu.arizona.cs.learn.timeseries.model;

import java.util.List;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.util.SequenceType;

public class Episode {

	private String _name;
	private int    _id;
	
	private List<Interval> _intervals;
	
	public Episode(String name, int id, List<Interval> intervals) { 
		_name = name;
		_id = id;
		
		_intervals = intervals;
	}
	
	public String name() { 
		return _name;
	}
	
	public int id() { 
		return _id;
	}
	
	public List<Interval> intervals() { 
		return _intervals;
	}
	
	public Instance toInstance(SequenceType type) { 
		List<WeightedObject> sequence = type.getSequence(_intervals);
		Instance instance = new Instance(_name, _id, sequence);
		return instance;
	}
	
}
