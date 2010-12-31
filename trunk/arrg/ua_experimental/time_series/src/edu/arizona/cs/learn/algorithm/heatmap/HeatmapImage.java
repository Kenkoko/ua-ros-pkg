package edu.arizona.cs.learn.algorithm.heatmap;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.render.Paint;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.StringSymbol;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class HeatmapImage {
    private static Logger logger = Logger.getLogger(HeatmapImage.class);

    /**
     * build the intensity map for the episode.
     * @param signature
     * @param min
     * @param episode
     * @param type
     * @return
     */
    public static Map<String,Double> intensityMap(List<Symbol> signature, int min, List<Interval> episode, SequenceType type) { 
		List<Symbol> sequence = type.getSequence(episode);
		
		Map<String,Double> map = new HashMap<String,Double>();
		for (Interval i : episode)  
			map.put(i.toString(), new Double(0.0));


		Params params = new Params(signature, sequence);
		params.setMin(min, 0);
		params.setBonus(1, 0);
		params.setPenalty(-1, 0);
		
		Report report = GeneralAlignment.align(params);
		logger.debug("Matches: " + report.numMatches + " Score: " + report.score);
		
		for (int i = 0; i < report.results1.size(); ++i) { 
			StringSymbol obj1 = (StringSymbol) report.results1.get(i);
			StringSymbol obj2 = (StringSymbol) report.results2.get(i);
			
			if (obj1 == null || obj2 == null)
				continue;
			
			for (Interval interval : obj2.getIntervals()) { 
				Double d1 = map.get(interval.toString());
				double d = d1 + obj1.weight();

				map.put(interval.toString(), d);
			}
		}
		return map;
    }
    
    /**
     * Determine the best possible score and then scale the intensity map by
     * that value.
     * @param map
     * @param signature
     * @param min
     * @return
     */
    public static Map<String,Double> scale(Map<String,Double> map, List<Symbol> signature, int min) { 
		Map<String,Double> maxWeightMap = new HashMap<String,Double>();
		for (Symbol obj : signature) { 
			if (obj.weight() < min)
				continue;
			
			if (!(obj instanceof StringSymbol))
				throw new RuntimeException("Not yet supported!");
			
			StringSymbol s = (StringSymbol) obj;
			for (String prop : s.getProps()) { 
				Double d = maxWeightMap.get(prop);
				if (d == null) 
					d = 0.0;
				maxWeightMap.put(prop, d+obj.weight());
			}
		}
		
		double maxPossible = 0;
		for (Double d : maxWeightMap.values())
			maxPossible = Math.max(maxPossible, d);
		
		return scale(map, maxPossible);
    }
    
    /**
     * Scale the intensity map by a known value.
     * @param map
     * @param value
     * @return
     */
    public static Map<String,Double> scale(Map<String,Double> map, double value) { 
		Map<String,Double> scaledMap = new HashMap<String,Double>();
		for (String key : map.keySet()) 
			scaledMap.put(key, map.get(key) / value);

		return scaledMap;
    }
    
    
    /**
     * Determine the intensity map, scale it by the best possible score and then
     * render the results to file.
     * @param imageFile
     * @param signature
     * @param min
     * @param episode
     * @param type
     */
    public static void makeHeatmap(String imageFile, List<Symbol> signature, int min, List<Interval> episode, SequenceType type) { 
    	Map<String,Double> map = intensityMap(signature, min, episode, type);
    	map = scale(map, signature, min);

    	Paint.render(episode, map, imageFile);
	}
}
