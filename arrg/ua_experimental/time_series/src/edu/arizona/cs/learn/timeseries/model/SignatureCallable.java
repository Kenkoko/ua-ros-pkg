package edu.arizona.cs.learn.timeseries.model;

import java.util.List;
import java.util.concurrent.Callable;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;

public class SignatureCallable implements Callable<Signature> {
	private String _key;
	private boolean _prune;
	private int _min;
	
	private List<Instance> _training;
	
	public SignatureCallable(String key, boolean prune, int min, List<Instance> training) { 
		_key = key;
		_prune = prune;
		_min = min;
		_training = training;
	}
	
	@Override
	public Signature call() throws Exception {
		Signature s = new Signature(_key);
		for (int i = 1; i <= _training.size(); i++) {
			s.update(_training.get(i-1).sequence());
			if (_prune && (i % 10 == 0)) {
				s = s.prune(_min);
			}
		}
		
		// Fifty percent pruning....
		s = s.prune(s.trainingSize()/2);
		return s;
	}
}