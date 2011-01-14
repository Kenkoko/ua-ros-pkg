package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.model.SignatureCallable;
import edu.arizona.cs.learn.util.Utils;

public class BatchSignatures {

	private Map<String,Signature> _signatures;
	private Map<String,Long> _timing;
	
	private Map<String,List<Instance>> _training;
	
	private boolean _prune;
	private int _min;
	
	private Similarity _sim;
		
	public BatchSignatures(Map<String,List<Instance>> training) {
		this(training, false, 0);
	}
	
	public BatchSignatures(Map<String,List<Instance>> training, boolean prune, int min) { 
		this(training, false, 0, Similarity.strings);
	}
	
	public BatchSignatures(Map<String,List<Instance>> training, boolean prune, int min, Similarity sim) { 
		_training = training;
		_prune = prune;
		_min = min;
		_sim = sim;
	}
	
	/**
	 * return the learned signature
	 * @return
	 */
	public Map<String,Signature> signatures() { 
		return _signatures;
	}
	
	/**
	 * return the timing information from the learned signatures.
	 * @return
	 */
	public Map<String,Long> timing() { 
		return _timing;
	}
	
	public void run() { 
		ExecutorService execute = Executors.newFixedThreadPool(Utils.numThreads);

		List<Future<SignatureCallable>> futureList = new ArrayList<Future<SignatureCallable>>();
		for (String key : _training.keySet()) {
			SignatureCallable sc = new SignatureCallable(key, _prune, _min, _sim, _training.get(key));
			futureList.add(execute.submit(sc));
		}
		
		_signatures = new HashMap<String,Signature>();
		_timing = new HashMap<String,Long>();
		for (Future<SignatureCallable> results : futureList) {
			try {
				SignatureCallable sc = results.get();
				Signature signature = sc.signature();

				_timing.put(signature.key(), sc.duration());
				_signatures.put(signature.key(), signature);
			} catch (Exception e) { 
				e.printStackTrace();
			}
		}
		execute.shutdown();
	}
	
}
