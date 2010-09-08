package edu.arizona.cs.learn.timeseries.visualization.graphics.color;

import java.awt.Color;
import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;

/**
 * The HeatModel generates colors based on the number
 * of times the interval is seen in the Alignment.
 * @author wkerr
 *
 */
public class HeatRelativeModel extends HeatModel {
    private static Logger logger = Logger.getLogger(HeatRelativeModel.class);

    private DataModel _model;
    
    private Interval _selected = null;
    private Map<String,Float> _intensityMap;
	
	public HeatRelativeModel() { 
		_intensityMap = new HashMap<String,Float>();
	}
	
	public void modelChanged(DataModel dm) { 
		_model = dm;
	}
	
	public void setSelected(int i) {
		logger.debug("episodeChanged");
		
		_intensityMap = new HashMap<String,Float>();
		_selected = _model.episode().get(i);
		
//		float maxSeen = 0;
//		List<AllenObject>[] align = _model.getEpisodeAlignment();
//		for (int j = 0; j < align[0].size(); ++j) { 
//			AllenObject obj1 = align[0].get(j);
//			AllenObject obj2 = align[1].get(j);
//			
//			if (obj1 != null && obj2 != null) { 
//				for (Interval[] pair : obj2.getExamples().values()) { 
//					// There will always only be 1 example... our example.
//					if (pair[0] == _selected) { 
//						String name = pair[1].toString();
//						Float d2 = _intensityMap.get(name);
//						if (d2 == null)
//							d2 = new Float(0.0f);
//						float d = d2 + (float) obj1.getWeight();
//						_intensityMap.put(name, d);
//						maxSeen = Math.max(maxSeen, d);
//					}
//					
//					if (pair[1] == _selected) { 
//						String name = pair[0].toString();
//						Float d1 = _intensityMap.get(name);
//						if (d1 == null) 
//							d1 = new Float(0.0f);
//						float d = d1 + (float) obj1.getWeight();
//						_intensityMap.put(name, d);
//						maxSeen = Math.max(maxSeen, d);
//					}
//				}
//			}
//		}
//		
//		logger.debug("MaxSeen: " + maxSeen);
//		Map<String,Float> tmp = new HashMap<String,Float>();
//		for (Map.Entry<String,Float> entry : _intensityMap.entrySet()) { 
//			logger.debug(entry.getKey() + " " + entry.getValue());
//			tmp.put(entry.getKey(), entry.getValue() / maxSeen);
//		}
//		_intensityMap = tmp;
	}

	
	public void episodeChanged() { 
		_selected = null;
	}
	
	public Color getColor(int i) {
		if (_selected == null)  
			return super.getColor(i);
		
		Interval interval = _model.episode().get(i);
		if (interval == _selected) { 
			return Color.red;
		} else {
			String name = interval.toString();
			logger.debug("\tColor: " + name);
			Float f = _intensityMap.get(name);
			if (f == null) 
				return Color.black;
			
			logger.debug("\t\tFound: " + f);
			return new Color(f, 0.0f, 0.0f);
		}
	}
}
