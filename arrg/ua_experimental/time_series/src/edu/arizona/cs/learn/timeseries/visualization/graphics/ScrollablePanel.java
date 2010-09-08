package edu.arizona.cs.learn.timeseries.visualization.graphics;

public interface ScrollablePanel {
	/**
	 * The user changed the scroll bar.  This
	 * update contains the new settings.
	 * @param row
	 * @param col
	 */
	public void scrolled(int row, int col);
}
