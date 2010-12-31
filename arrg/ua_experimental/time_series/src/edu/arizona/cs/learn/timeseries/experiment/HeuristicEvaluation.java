package edu.arizona.cs.learn.timeseries.experiment;

import java.util.List;

import org.apache.commons.math.stat.descriptive.SummaryStatistics;
import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.Utils;

public class HeuristicEvaluation {
    private static Logger logger = Logger.getLogger(HeuristicEvaluation.class);

    public static void main(String[] args) { 
		Utils.LIMIT_RELATIONS = true;
		Utils.WINDOW = 5;

		String name = "ww-jump-over";
		
    	List<Instance> instances = Utils.sequences(name, "data/input/" + name + ".lisp", SequenceType.allen);
    	Signature s = new Signature(name);
    	s.heuristicTraining(instances);
    	
    	logger.debug("Training size : " + instances.size() + " " + s.trainingSize());
		Params params = new Params();
		params.setMin(s.trainingSize()/2, 0);
		params.setBonus(1, 0);
		params.setPenalty(-1, 0);

    	instances = Utils.sequences(name, "data/input/" + name + ".lisp", SequenceType.allen);
		SummaryStatistics ss = new SummaryStatistics();
		for (Instance instance : instances) { 
			params.seq1 = s.signature();
			params.seq2 = instance.sequence();
			
			ss.addValue(GeneralAlignment.distance(params));
		}
		logger.debug("Statistics: " + ss.getMean() + " " + ss.getStandardDeviation() + " [" + ss.getMin() + "," + ss.getMax() + "]");
	}
}
