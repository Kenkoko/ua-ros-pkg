package edu.arizona.cs.learn.timeseries.model;

import java.util.List;

import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

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
		List<Symbol> sequence = type.getSequence(_intervals);
		Instance instance = new Instance(_name, _id, _intervals, sequence);
		return instance;
	}
}
