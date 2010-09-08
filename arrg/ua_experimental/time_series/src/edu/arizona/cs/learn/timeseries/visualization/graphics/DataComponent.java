package edu.arizona.cs.learn.timeseries.visualization.graphics;

import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;

public interface DataComponent {

	public void modelChanged(DataModel dm);
	public void episodeChanged();

	public void receiveMessage(String message);
	
	public void repaint();
}
