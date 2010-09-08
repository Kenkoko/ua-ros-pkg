package edu.arizona.cs.learn.timeseries.evaluation.classifier;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.evaluation.FoldStatistics;
import edu.arizona.cs.learn.timeseries.model.Score;
import edu.arizona.cs.learn.timeseries.model.Signature;

public class CAVEClassifier implements Classifier {
	
	protected int _percent;
	protected double _pct;
	
	protected int _fold;
	
	protected Map<String,Signature> _map;
	
	public CAVEClassifier(int percent) { 
		_percent = percent;
		_pct = (double) _percent / 100.0;
	}
	
	public String getName() { 
		return "cave-" + _percent;
	}
	
	public void addData(FoldStatistics fold) { 
		for (Signature signature : _map.values()) {
			int minSeen = (int) Math.round((double) signature.trainingSize() * _pct);
			fold.signatureSize.addValue(SequenceAlignment.subset(signature.signature(), minSeen).size());
		}
	}
	
	public String test(Instance instance) {
		
		Params params = new Params();
		params.setMin(5, 0);
		params.setBonus(1, 0);
		params.setPenalty(-1, 0);

		List<Score> scores = new ArrayList<Score>();
		for (Signature signature : _map.values()) { 
			params.seq1 = signature.signature();
			params.min1 = (int) Math.round((double) signature.trainingSize() * _pct);
			params.seq2 = instance.sequence();
			
			Score score = new Score();
			score.key = signature.key();
			score.distance = SequenceAlignment.distance(params);
			scores.add(score);
		}
		
		Collections.sort(scores, new Comparator<Score>() {
			public int compare(Score o1, Score o2) {
				return Double.compare(o1.distance, o2.distance);
			} 
		});
		
		return scores.get(0).key;
	}
	
	private void doTrain(Map<String,List<Instance>> instances) { 
		// At this point we have a map that maps from the class to the
		// training instances associated with it.
		for (String key : instances.keySet()) { 
			Signature s = new Signature(key);
			List<Instance> list = instances.get(key);
			for (int i = 0; i < list.size(); ++i) { 
				s.update(list.get(i).sequence());
//				s.toFile("/tmp/learning/" + getName() + "-" + _fold + "-" + key + "-" + i + ".sig", 0);
			}
//			s.train(instances.get(key));
//			s.toFile("/tmp/cave-" + s.key() + ".signature", s.trainingSize() / 2);
			
			_map.put(key, s);
		}
	}

	public void train(List<List<Instance>> data, int x) {
		_map = new HashMap<String,Signature>();
		_fold = x;

		File saveDir = new File("/tmp/learning/");
		if (!saveDir.exists()) { 
			saveDir.mkdir();
		}
		
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
	
	public void printError(Instance instance, int id) { 
		try { 
			BufferedWriter out = new BufferedWriter(new FileWriter("/tmp/printing/" + instance.name() + "-" + id + ".txt"));
			out.write("correct: " + instance.name() + " " + instance.id() + "\n");
			
			Params params = new Params();
			params.setMin(5, 0);
			params.setBonus(1, 0);
			params.setPenalty(-1, 0);

			List<Score> scores = new ArrayList<Score>();
			for (Signature signature : _map.values()) { 
				params.seq1 = signature.signature();
				params.min1 = signature.trainingSize() / 2;
				params.seq2 = instance.sequence();
				
				Score score = new Score();
				score.key = signature.key();
				score.distance = SequenceAlignment.distance(params);
				score.signature = signature;
				scores.add(score);
			}
			
			Collections.sort(scores, new Comparator<Score>() {
				public int compare(Score o1, Score o2) {
					return Double.compare(o1.distance, o2.distance);
				} 
			});
			
			for (int i = 0; i < scores.size(); ++i) { 
				Score score = scores.get(i);
				out.write("\t" + score.key + " " + score.distance + "\n");
				if (i == 0 || score.key.equals(instance.name())) { 
					params.seq1 = score.signature.signature();
					params.min1 = score.signature.trainingSize() / 2;
					params.seq2 = instance.sequence();
		
					Report report = SequenceAlignment.align(params);
//					for (int j = 0; j < report.results1.size(); ++j) { 
//						if (report.results1.get(j) != null &&
//							report.results2.get(j) != null) { 
//							out.write("\t\t" + report.results1.get(j).toStream() + "\n");
//						}
//					}
				}
			}
			
			Signature c1 = _map.get(instance.name());
			Signature c2 = _map.get(scores.get(0).key);
			Params plite = new Params(c1.signature(), c2.signature());
			plite.setMin(c1.trainingSize()/2, c2.trainingSize()/2);
			plite.setBonus(1, 1);
			plite.setPenalty(-1, -1);
			
			out.write("*********\n");
			Report report = SequenceAlignment.align(plite);
//			for (int i = 0; i < report.results1.size(); ++i) { 
//				if (report.results1.get(i) != null &&
//					report.results2.get(i) != null) { 
//					out.write("\t" + report.results1.get(i).toStream() + "\n");
//				}
//			}
			out.close();
		} catch (Exception e) { 
			e.printStackTrace();
		}
	}
}
