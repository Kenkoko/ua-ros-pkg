package edu.arizona.cs.learn.timeseries.visualization.graphics.color;

import java.awt.Color;
import java.io.File;

import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;

public class SimpleModel implements TimelineColor {
	private Color[] _colors = { new Color(209, 42, 42, 200), new Color(237, 52, 52, 200) };

	public Color getColor(int i) {
		return _colors[i%2];
	}

	public void setSelected(int i) {
		// TODO maybe do something?
		
	}

	public void episodeChanged() {
		// TODO Auto-generated method stub
		
	}

	public void modelChanged(DataModel dm) {
		// TODO Auto-generated method stub
		
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
