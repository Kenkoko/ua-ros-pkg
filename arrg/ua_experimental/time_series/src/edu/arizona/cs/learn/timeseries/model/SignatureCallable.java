package edu.arizona.cs.learn.timeseries.model;

import java.util.List;
import java.util.concurrent.Callable;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;

public class SignatureCallable implements Callable<SignatureCallable> {
	private String _key;
	private boolean _prune;
	private int _min;
	
	private List<Instance> _training;

	private Signature _signature;
	private Long _duration;
	
	public SignatureCallable(String key, boolean prune, int min, List<Instance> training) { 
		_key = key;
		_prune = prune;
		_min = min;
		_training = training;
	}
	
	@Override
	public SignatureCallable call() throws Exception {
		long start = System.currentTimeMillis();
		Signature s = new Signature(_key);
		for (int i = 1; i <= _training.size(); i++) {
			s.update(_training.get(i-1).sequence());
			if (_prune && (i % 10 == 0)) {
				s = s.prune(_min);
			}
		}
		
		// Fifty percent pruning....
		_signature = s.prune(s.trainingSize()/2);
		_duration = System.currentTimeMillis() - start;
		return this;
	}
	
	public Signature signature() { 
		return _signature;
	}
	
	public Long duration() { 
		return _duration;
	}
}