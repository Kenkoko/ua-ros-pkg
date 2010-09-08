package edu.arizona.cs.learn.timeseries.evaluation;

import java.util.List;
import java.util.Map;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.util.SequenceType;
import edu.arizona.cs.learn.util.Utils;

public class PruningExperiment {

	public static void main(String[] args) { 
//		Utils.numThreads = 4;
//		
//		String[] prefixes = new String[] { "wes", "nicole", "derek", "ecg", "wafer", "ww3d" };
//		for (String prefix : prefixes) { 
//			for (int i = 40; i <= 100; i+= 10) { 
//				CrossValidation cv = new CrossValidation("all", "prune");
//				cv.setCAVEPercent(i);
//				
//				cv.runComparison(prefix, false);
//				
//			}
//		}
		
		Map<String,List<Instance>> map = Utils.load("chpt1-approach", SequenceType.bpp);
		List<Instance> list = map.get("chpt1-approach");
		
		System.out.println(list.get(0).sequence());
		
	}
}
