package edu.arizona.cs.learn.timeseries.visualization.graphics;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JPanel;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.render.Paint;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.visualization.Controller;
import edu.arizona.cs.learn.timeseries.visualization.graphics.color.TimelineColor;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;

public class TimelineCanvas extends JPanel implements DataComponent, ScrollablePanel, MouseListener {
	private static Logger logger = Logger.getLogger(TimelineCanvas.class);

	private static final long serialVersionUID = 1L;

	public static final int TICK_WIDTH = 5;
	public static final int ROW_HEIGHT  = 12;

	protected int _xOffset = 0;
	protected int _yOffset = 0;
	
	protected TimelineColor _colorModel;
	protected DataModel     _model;
	
	protected int _row;
	protected int _col;

	private Color _gridColor = new Color(143, 133, 133);

	private boolean _compressed;
	
	private boolean _drawGrid = true;
	
	private boolean _highlightSelected = false;
	private Stroke   _highlightStroke = null;
	private Interval _selected = null;

	public TimelineCanvas(Controller dc, TimelineColor tc, boolean compressed) {
		if (dc != null)
			dc.add(this);
		_colorModel = tc;
		_compressed = compressed;
		
		_highlightStroke = new BasicStroke(2.0f);
		
		setBorder(BorderFactory.createLineBorder(Color.black));
		addMouseListener(this);
	}
	
	/**
	 * Our parent scroll controller notified us of a change
	 * to the scrolling.
	 */
	public void scrolled(int row, int col) { 
		_row = row;
		_col = col;
		repaint();
	}
	
	/**
	 * Should the selected interval be highlighted?
	 * @param value
	 */
	public void setHighlightSelected(boolean value) { 
		_highlightSelected = value;
	}
	
	public void paintComponent(Graphics graphics) {
		Graphics2D g = (Graphics2D) graphics;
		g.setBackground(Color.white);
		g.setColor(Color.white);
		g.fillRect(0, 0, getWidth(), getHeight());

		int width = getWidth();
		int height = getHeight();

		int imgWidth = width - _xOffset;
		int imgHeight = height - _yOffset;

		if (_model == null)
			return;

		int numRows = imgHeight / ROW_HEIGHT;
		int numCols = imgWidth / TICK_WIDTH;

		g.setColor(Color.lightGray);
		g.fillRect(0, 0, _xOffset, height);
		if (_drawGrid) { 
			drawGrid(g, numRows, numCols);
		}
		
		Font font = g.getFont();
		g.setFont(font.deriveFont(9.0f));
		
		g.setColor(Color.black);
		
		List<String> props = new ArrayList<String>(_model.propSet());
		for (int i = 0; i < props.size(); ++i) { 
			if (i < _row)
				continue;
			
			g.drawString(props.get(i), 2, transformY(i-_row)+11);
		}
		
//		logger.debug("window: (" + _row + " " + _col + ") (" + _row+numRows + " " + (_col + numCols) + ")");
		int[] spec = new int[4];
		boolean found = false;

		List<Interval> intervals = _model.episode();
		if (_compressed)
			intervals = _model.compressed();
		
		for (int i = 0; i < intervals.size(); ++i) { 
			Interval interval = intervals.get(i);
			int start = Math.min(_col+numCols, Math.max(interval.start, _col));
			int end = Math.max(_col, Math.min(interval.end, _col + numCols));
			
			if (start == end)
				continue;
			
			int index = props.indexOf(interval.name);
			if (index < _row) 
				continue; 
			
			int row = index - _row;
			g.setColor(_colorModel.getColor(i));
			
			int w = (end - start) * TICK_WIDTH;
			int h = ROW_HEIGHT;
			
			g.fillRect(transformX(start-_col), transformY(row)+2, w, h-3);
			
			if (_highlightSelected && interval == _selected) {
				spec[0] = transformX(start-_col);
				spec[1] = transformY(row);
				spec[2] = w;
				spec[3] = h;
				found = true;
			}
		}
		
		if (found) { 
			Stroke save = g.getStroke();

			g.setColor(new Color(0, 0, 1.0f, 0.8f));
			g.setStroke(_highlightStroke);
			g.drawRect(spec[0], spec[1], spec[2], spec[3]);

			g.setStroke(save);
			
		}
	}

	protected void drawGrid(Graphics2D g, int numRows, int numColumns) {
		int width = getWidth();
		int height = getHeight();

		g.setColor(_gridColor);
		for (int i = 0; i <= numRows; ++i) {
			g.drawLine(0, transformY(i), width, transformY(i));
		}

		for (int i = 0; i <= numColumns; ++i) {
			g.drawLine(transformX(i), 0, transformX(i), height);
		}
	}

	protected int transformY(int y) {
		return y * ROW_HEIGHT + _yOffset;
	}

	protected int transformX(int x) {
		return x * TICK_WIDTH + _xOffset;
	}
	
	public void modelChanged(DataModel dm) {
		_model = dm;
		_colorModel.modelChanged(dm);
	}
	
	public void episodeChanged() {
		// Need to recalculate the X_OFFSET
	    Paint._timeWidth = TICK_WIDTH;
	    Paint._rowHeight = ROW_HEIGHT;
	    Paint._fontSize = 9;

		_xOffset = Paint.determinePropArea(_model.propSet());
		
		_selected = null;
		_colorModel.episodeChanged();
		repaint();
	}
		
	public void redraw() { 
	}

	public void receiveMessage(String message) {
		if (message.startsWith("set drawgrid")) { 
			String[] tokens = message.split("[ ]");
			_drawGrid = Boolean.parseBoolean(tokens[2]);
			repaint();
		}
	}

	public void mouseClicked(MouseEvent e) {
		// We are going to determine if this click was done on an
		// interval.... 
		if (_model == null || _model.episode() == null) { 
			return;
		}

		int x = e.getX();
		int y = e.getY();
		
		int row = ((y - _yOffset) / ROW_HEIGHT) + _row;
		int time = ((x - _xOffset) / TICK_WIDTH) + _col;

		if (row > _model.propSet().size() || time < 0)
			return;

		List<String> props = new ArrayList<String>(_model.propSet());
		String prop = props.get(row);

		int index = 0;
		_selected = null;
		
		List<Interval> intervals = _model.episode();
		if (_compressed)
			intervals = _model.compressed();
		
		for (int i = 0; i < intervals.size(); ++i) { 
			Interval interval = intervals.get(i);
			if (interval.name.equals(prop) && 
					time >= interval.start && 
					time < interval.end) { 
				_selected = interval;
				index = i;
				break;
			}
		}
		
		if (_selected != null) {
			_colorModel.setSelected(index);
			repaint();
		}
		
		logger.debug("Row: " + prop + " Time: " + time);
	}

	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	public void mousePressed(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	public void mouseReleased(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void modelSelected(File file) {
		// TODO Auto-generated method stub
		
	}
}

