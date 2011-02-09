package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Normalize;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.timeseries.distance.Distances;
import edu.arizona.cs.learn.timeseries.evaluation.cluster.Clustering;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class RandomStarts {
	
	/**
	 * Explore how the sequences change as we randomly turn them on and
	 * off.
	 */
	public static void changes(String prefix) { 
    	
    	Map<String,List<Instance>> map = Utils.load(prefix, SequenceType.starts);
    	
		// test all the instances within a given prefix
		// to see how much the random positioning starts
		// changes the ordering within the sequence
		for (String className : map.keySet()) { 
			List<Instance> list = map.get(className);

			// Each episode has it's own summary statistics so 
			// that we can look at the average change by episode
			List<SummaryStatistics> listSS = new ArrayList<SummaryStatistics>();
			List<Double> selfDistance = new ArrayList<Double>();
			for (int i = 0; i < list.size(); ++i) { 
				listSS.add(new SummaryStatistics());
				Params params = new Params();
				params.setMin(0, 0);
				params.setBonus(1,0);
				params.setPenalty(0, 0);
				params.normalize = Normalize.none;
				params.seq1 = list.get(i).sequence();
				params.seq2 = list.get(i).sequence();
				selfDistance.add(GeneralAlignment.distance(params));
			}

			for (int i = 0; i < 1000; ++i) { 
				List<Instance> randList = Utils.sequences(className, "data/input/" + className + ".lisp", SequenceType.randomStarts);
				
				for (int j = 0; j < list.size(); ++j) { 
					Instance i1 = list.get(j);
					Instance i2 = randList.get(j);
					
					if (i1.id() != i2.id()) 
						throw new RuntimeException("Instances don't match: " + className + " " + i1.id() + " " + i2.id());
					
					Params params = new Params();
					params.setMin(0, 0);
					params.setBonus(1,0);
					params.setPenalty(0, 0);
					params.normalize = Normalize.none;
					params.seq1 = i1.sequence();
					params.seq2 = i2.sequence();
					
					double d = GeneralAlignment.distance(params);
					listSS.get(j).addValue(d);
				}
			}
			
			System.out.println("Class: " + className);
			for (int i = 0; i < list.size(); ++i) { 
				System.out.println("  Episode " + list.get(i).id() + " - " + list.get(i).sequence().size() + " - " + selfDistance.get(i));
				System.out.println("    min: " + listSS.get(i).getMin() + " max: " + listSS.get(i).getMax());
				System.out.println("    mean: " + listSS.get(i).getMean() + " sd: " + listSS.get(i).getStandardDeviation());
			}
		}
	}
	
	// first we are going to write the original distance matrix...
	// then we will do 5 randomStarts distance matrix....
	public static void distanceMatrices(String prefix) { 
    	Clustering c = new Clustering();

    	double[][] original = Distances.distances(prefix, SequenceType.starts);
    	Distances.save("/tmp/" + prefix + "-distance-matrix.csv", original);

    	for (int i = 0; i < 10; ++i) { 
    		double[][] random = Distances.distances(prefix, SequenceType.randomStarts);
    		Distances.save("/tmp/" + prefix + "-distance-matrix" + i + ".csv", random);
    	}
	}
	

	public static void main(String[] args) { 		
    	String prefix = "ww3d";
//    	distanceMatrices(prefix);

    	Clustering c = new Clustering();
    	double[][] original = Distances.distances(prefix, SequenceType.allen);
    	Distances.save("/tmp/" + prefix + "-distance-matrix-allen.csv", original);
	}
}
