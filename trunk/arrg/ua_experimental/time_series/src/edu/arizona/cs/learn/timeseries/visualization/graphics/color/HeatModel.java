package edu.arizona.cs.learn.timeseries.visualization.graphics.color;

import java.awt.Color;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.heatmap.HeatmapImage;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;

/**
 * The HeatModel generates colors based on the number
 * of times the interval is seen in the Alignment.
 * @author wkerr
 *
 */
public class HeatModel implements TimelineColor {
    private static Logger logger = Logger.getLogger(HeatModel.class);

    private DataModel _model;
	private Map<String,Double> _intensityMap;
	
	public HeatModel() { 
		_intensityMap = new HashMap<String,Double>();
	}
	
	public void modelChanged(DataModel dm) { 
		_model = dm;
	}
	
	public void episodeChanged() { 
//		logger.debug("episodeChanged");
		
		Signature signature = _model.signature();
		if (signature == null)
			return;
		
		int min = signature.trainingSize() / 2;

		_intensityMap = HeatmapImage.intensityMap(signature.signature(), min, 
				_model.episode(), _model.sequenceType());
	}
	
	public Color getColor(int i) {
		if (_model == null)
			return Color.lightGray;
		
		if (_model.signature() == null)
			return Color.red;
		
		logger.debug("Model: " + _model);
		logger.debug(" Episode: " + _model.episode() + " index: " + i);
		String name = _model.episode().get(i).toString();
		Double d = _intensityMap.get(name);
		if (d == null) { 
			return Color.white;
		} else { 
			float f = (float) d.doubleValue();
			return new Color(1,1-f,1-f);
		}
	}

	public void setSelected(int i) {
		// TODO maybe do something?
	}

	public void receiveMessage(String message) {
		// TODO Auto-generated method stub
		
	}

	public void repaint() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void modelSelected(File file) {
		// TODO Auto-generated method stub
		
	}

	
	
}
