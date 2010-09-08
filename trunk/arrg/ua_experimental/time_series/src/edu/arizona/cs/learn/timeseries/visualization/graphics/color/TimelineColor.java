package edu.arizona.cs.learn.timeseries.visualization.graphics.color;

import java.awt.Color;

import edu.arizona.cs.learn.timeseries.visualization.graphics.DataComponent;

public interface TimelineColor extends DataComponent {

	public void setSelected(int i);
	public Color getColor(int i);
}
