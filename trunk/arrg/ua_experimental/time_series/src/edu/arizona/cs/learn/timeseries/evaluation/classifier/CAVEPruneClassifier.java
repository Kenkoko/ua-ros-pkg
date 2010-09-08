package edu.arizona.cs.learn.timeseries.evaluation.classifier;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.evaluation.cluster.Clustering;
import edu.arizona.cs.learn.timeseries.model.AgglomerativeNode;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.visualization.graph.Node;

/**
 * CAVESortClassifier only differs from the CAVE Classifier in
 * training.  Rather than randomly starting the training process, 
 * we actually try to do some sort of agglomerative clustering
 * in order to improve performance.
 * @author wkerr
 *
 */
public class CAVEPruneClassifier extends CAVEClassifier {

	public CAVEPruneClassifier(int percent) { 
		super(percent);
	}
	
	public String getName() { 
		return "cave-prune-" + _percent;
	}
	
	private void doTrain(Map<String,List<Instance>> instances) { 
		// At this point we have a map that maps from the class to the
		// training instances associated with it.
		Clustering c = new Clustering();
		
		for (String key : instances.keySet()) { 
			Signature s = new Signature(key);
			List<Instance> list = instances.get(key);
			for (int i = 1; i <= list.size(); ++i) { 
				s.update(list.get(i-1).sequence());
				if (i % 10 == 0) { 
					s = s.prune(3);
				}
			}
			_map.put(key, s);
		}		
	}
	
	public void train(List<List<Instance>> data, int x) {
		_map = new HashMap<String,Signature>();
		
		Map<String,List<Instance>> instances = new HashMap<String,List<Instance>>();
		for (int i = 0; i < data.size(); ++i) { 
			if (i==x) 
				continue;
			
			for (Instance instance : data.get(i)) { 
				List<Instance> list = instances.get(instance.name());
				if (list == null) { 
					list = new ArrayList<Instance>();
					instances.put(instance.name(), list);
				}
				
				list.add(instance);
			}
		}
		
		doTrain(instances);
	}

	public void train(List<Instance> data) {
		_map = new HashMap<String,Signature>();
		
		Map<String,List<Instance>> instances = new HashMap<String,List<Instance>>();
		for (Instance instance : data) { 
			List<Instance> list = instances.get(instance.name());
			if (list == null) { 
				list = new ArrayList<Instance>();
				instances.put(instance.name(), list);
			}
				
			list.add(instance);
		}
		
		doTrain(instances);
	}	
}
