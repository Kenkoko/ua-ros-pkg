package edu.arizona.cs.learn.timeseries.visualization.graphics;

import java.awt.BorderLayout;
import java.awt.event.AdjustmentEvent;
import java.awt.event.AdjustmentListener;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.JScrollBar;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.visualization.Controller;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;

public class ScrolledCanvas extends JPanel implements DataComponent {
	
	/** These are the panels notified when something changes. */
	protected List<ScrollablePanel> _panels;
	
	/** This is the panel containing the others. */
	protected JPanel _panel;
	
	protected JScrollBar _horizontalBar;
	protected JScrollBar _verticalBar;

	// this represents the northernmost corner
	protected int _row = 0;
	protected int _column = 0;
	
	private DataModel _model;
	private boolean _compressed;
	
	public ScrolledCanvas(Controller dc, JPanel panel, boolean compressed) { 
		if (dc != null)
			dc.add(this);
		_panel = panel;
		_compressed = compressed;
		_panels = new ArrayList<ScrollablePanel>();
		
		addComponents();
		addListeners();
	}
	
	public void addPanel(ScrollablePanel panel) {
		_panels.add(panel);
	}
	
	protected void addComponents() {
		setLayout(new BorderLayout());
		
		_horizontalBar = new JScrollBar(JScrollBar.HORIZONTAL);
		_verticalBar = new JScrollBar(JScrollBar.VERTICAL);
		
		_horizontalBar.setUnitIncrement(1);
		_verticalBar.setUnitIncrement(1);
		
		add(_horizontalBar, BorderLayout.SOUTH);
		add(_verticalBar, BorderLayout.EAST);
		add(_panel, BorderLayout.CENTER);
	}		
	
	protected void addListeners() {
		_horizontalBar.addAdjustmentListener(new AdjustmentListener() {
			public void adjustmentValueChanged(AdjustmentEvent e) {
				_column = e.getValue();
				notifyPanels();
				repaint();
			}
		});
		
		_verticalBar.addAdjustmentListener(new AdjustmentListener() {
			public void adjustmentValueChanged(AdjustmentEvent e) {
				_row = e.getValue();
				notifyPanels();
				repaint();
			}
		});
	}
	
	private void reset() { 
		List<Interval> compressed = _model.compressed();
		if (compressed != null) { 
			int max = 0;
			for (Interval interval : compressed) 
				max = Math.max(max, interval.end);
			_horizontalBar.setMaximum(max);
			_verticalBar.setMaximum(_model.propSet().size());

			_horizontalBar.setValue(0);
			_verticalBar.setValue(0);	
			
		}
	}
	
	private void notifyPanels() { 
		for (ScrollablePanel sp : _panels) { 
			sp.scrolled(_row, _column);
		}
	}

	public void modelChanged(DataModel dm) {
		_model = dm;
		reset();
	}

	public void episodeChanged() {
		reset();
	}

	public void redraw() {
		// TODO Auto-generated method stub
		
	}

	public void receiveMessage(String message) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void modelSelected(File file) {
		// TODO Auto-generated method stub
		
	}
}
