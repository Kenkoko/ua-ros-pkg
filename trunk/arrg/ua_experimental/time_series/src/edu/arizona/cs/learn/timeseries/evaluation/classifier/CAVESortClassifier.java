package edu.arizona.cs.learn.timeseries.evaluation.classifier;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.distance.Distances;
import edu.arizona.cs.learn.timeseries.evaluation.cluster.Clustering;
import edu.arizona.cs.learn.timeseries.model.AgglomerativeNode;
import edu.arizona.cs.learn.timeseries.model.Signature;

/**
 * CAVESortClassifier only differs from the CAVE Classifier in
 * training.  Rather than randomly starting the training process, 
 * we actually try to do some sort of agglomerative clustering
 * in order to improve performance.
 * @author wkerr
 *
 */
public class CAVESortClassifier extends CAVEClassifier {

	private String _method;
	
	public CAVESortClassifier(String method) { 
		super(50);
		_method = method;
	}
	
	public String getName() { 
		return "cave-" + _method + "-" + _percent;
	}
	
	private void doTrain(Map<String,List<Instance>> instances) { 
		// At this point we have a map that maps from the class to the
		// training instances associated with it.
		Clustering c = new Clustering();
		
		for (String key : instances.keySet()) { 
			
			List<Instance> list = instances.get(key);
			double[][] matrix = Distances.distances(list);
			
			List<AgglomerativeNode> nodes = new ArrayList<AgglomerativeNode>();
			for (int i = 0; i < list.size(); ++i) { 
				nodes.add(new AgglomerativeNode(i, list.get(i)));
			}
			
			while (nodes.size() > 1) { 
				
				// find the smallest distance between nodes.
				AgglomerativeNode minN1 = null;
				AgglomerativeNode minN2 = null;
				double min = Double.POSITIVE_INFINITY;
				
				for (int i = 0; i < nodes.size(); ++i) { 
					AgglomerativeNode n1 = nodes.get(i);
					for (int j = i+1; j < nodes.size(); ++j) { 
						AgglomerativeNode n2 = nodes.get(j);
						double distance = n1.distance(n2, matrix, _method);
						if (distance < min) { 
							distance = min;
							minN1 = n1;
							minN2 = n2;
						}
					}
				}
				
				// now that we've found the minimum one, we can combine
				// them into a new node.
				nodes.remove(minN1);
				nodes.remove(minN2);
				
				nodes.add(new AgglomerativeNode(minN1, minN2));
			}
			
			// there should only be a single node in nodes
			// that contains the signature (possibly really good)
//			Signature s = new Signature(key);
//			s.heuristicTraining(instances.get(key));
			Signature s = nodes.get(0).signature;
			
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
