package edu.arizona.cs.learn.algorithm.heatmap;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Symbol;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.algorithm.render.Paint;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.SequenceType;

public class HeatmapImage {
    private static Logger logger = Logger.getLogger(HeatmapImage.class);
	
    public static Map<String,Double> intensityMap(List<WeightedObject> signature, int min, List<Interval> episode, SequenceType type) { 
		List<WeightedObject> sequence = type.getSequence(episode);
		logger.debug("Episode: " + episode);
		logger.debug("Sequence: " + sequence);
		
		Map<String,Double> map = new HashMap<String,Double>();
		Map<String,Double> maxWeightMap = new HashMap<String,Double>();
		for (Interval i : episode) { 
			map.put(i.toString(), new Double(0.0));
		}

		Params params = new Params(signature, sequence);
		params.setMin(min, 0);
		params.setBonus(1, 0);
		params.setPenalty(-1, 0);
		
		Report report = SequenceAlignment.align(params);
		logger.debug("Matches: " + report.numMatches + " Score: " + report.score);
		
		double maxWeight = 0;
		for (WeightedObject obj : signature) { 
			Symbol s = obj.key();
			logger.debug(" --- signature: " + s.getKey());
			for (String prop : s.getProps()) { 
				Double d = maxWeightMap.get(prop);
				if (d == null) 
					d = 0.0;
				logger.debug(" ---- prop: " + prop + " --- " + (d+obj.weight()));
				maxWeightMap.put(prop, d+obj.weight());
			}
		}
		
		double maxPossible = 0;
		for (Double d : maxWeightMap.values())
			maxPossible = Math.max(maxPossible, d);
		
		logger.debug("Max Weight: " + maxWeight + " Max Possible: " + maxPossible);
		
		double maxSeen = 0;
		double pctCovered = 0;
		for (int i = 0; i < report.results1.size(); ++i) { 
			WeightedObject obj1 = report.results1.get(i);
			WeightedObject obj2 = report.results2.get(i);
			
			if (obj1 == null || obj2 == null)
				continue;
			
			pctCovered += obj1.weight();
			
			for (Interval interval : obj2.key().getIntervals()) { 
				Double d1 = map.get(interval.toString());
				double d = d1 + obj1.weight();

				maxSeen = Math.max(maxSeen, d);
				map.put(interval.toString(), d);
			}
		}
		pctCovered /= maxWeight;
		
		Map<String,Double> tmp = new HashMap<String,Double>();
		for (Map.Entry<String,Double> entry : map.entrySet()) { 
			tmp.put(entry.getKey(), entry.getValue() / maxPossible);
		}
		map = tmp;
		return map;
    }

    public static void makeHeatmap(String imageFile, List<WeightedObject> signature, int min, List<Interval> episode, SequenceType type) { 
    	Map<String,Double> map = intensityMap(signature, min, episode, type);
		Paint.render(episode, map, imageFile);
	}
}
