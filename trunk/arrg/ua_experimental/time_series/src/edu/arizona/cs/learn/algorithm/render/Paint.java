package edu.arizona.cs.learn.algorithm.render;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.RenderingHints;
import java.awt.Transparency;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import javax.imageio.ImageIO;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.AllenRelation;

public class Paint {
    private static Logger logger = Logger.getLogger(Paint.class);

    public static int _timeWidth = 10;
    public static int _rowHeight = 20;
    public static float _fontSize = 20;
    
    public static int _fontStyle = Font.BOLD;

    private static Color _gridColor = new Color(143, 133, 133);


	/**
	 * take the given set of intervals (assumed to be a bit pattern) and render
	 * them as an image.
	 * @param intervals
	 * @param fileName
	 */
	public static void render(List<Interval> intervals, String fileName) { 
		render(intervals, null, fileName);
	}
	
	/**
	 * take the given set of intervals (assumed to be a bit pattern) and render
	 * them as an image.
	 * @param intervals
	 * @param fileName
	 */
	public static void render(List<Interval> intervals, Map<String,Double> intensityMap, String fileName) { 
		BufferedImage img = paint(intervals, intensityMap);
		try {
			ImageIO.write(img, "PNG", new File(fileName));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
	
	public static int determinePropArea(Set<String> propSet) { 
		GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
		GraphicsDevice gs = ge.getDefaultScreenDevice();
		GraphicsConfiguration gc = gs.getDefaultConfiguration();

		BufferedImage img = gc.createCompatibleImage(200, 200, Transparency.BITMASK);
		Graphics2D g= img.createGraphics();
		Font font = g.getFont();
		g.setFont(font.deriveFont(_fontStyle, _fontSize));

		FontMetrics fm = g.getFontMetrics();
		int width = 0;
		for (String s : propSet) { 
			width = Math.max(fm.stringWidth(s), width);
		}
		// add some padding
		return width + 5;
	}
	
	
	public static BufferedImage paint(List<Interval> intervals, Map<String,Double> intensityMap) { 
		GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
		GraphicsDevice gs = ge.getDefaultScreenDevice();
		GraphicsConfiguration gc = gs.getDefaultConfiguration();

		
		// first determine the number of rows that will be needed,
		// how much time will be needed and how much space we must
		// reserve for the proposition names
		Set<String> propSet = new TreeSet<String>();
		int startTime = Integer.MAX_VALUE;
		int endTime = 0;
		for (Interval i : intervals) { 
			propSet.add(i.name);
			startTime = Math.min(i.start, startTime);
			endTime = Math.max(i.end, endTime);
		}
		
		// create a default bufferedimage so that we can get the graphics
		// and determine the size necessary for rendering the Text.
		int propArea = determinePropArea(propSet);
		int width = propArea + ((endTime-startTime) * _timeWidth) + 1;
		int height = propSet.size() * _rowHeight + 1;
		
//		logger.debug("start: " + startTime + " end: " + endTime + " width: " + width + " height: " + height);
		
		BufferedImage img = gc.createCompatibleImage(width, height, Transparency.BITMASK);

		Graphics2D g = img.createGraphics();
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setBackground(Color.white);
		g.setColor(Color.white);
		g.fillRect(0, 0, width, height);

		// fill in a darker gray area for the propositions
		g.setColor(Color.lightGray);
		g.fillRect(0, 0, propArea, height);

		// Draw the grid underneath us.
		g.setColor(_gridColor);
		for (int i = 0; i <= propSet.size(); ++i) 
			g.drawLine(0, i*_rowHeight, width, i*_rowHeight);
		for (int i = 0; i <= (endTime-startTime); ++i) {
			g.drawLine(i*_timeWidth+propArea, 0, i*_timeWidth+propArea, height);
		}
		
		Font font = g.getFont();
		g.setFont(font.deriveFont(_fontStyle, _fontSize));		
		g.setColor(Color.black);
		List<String> props = new ArrayList<String>(propSet);
		for (int i = 0; i < props.size(); ++i) { 
			g.drawString(props.get(i), 2, (i*_rowHeight)+_fontSize);
		}
		
		for (int i = 0; i < intervals.size(); ++i) { 
			Interval interval = intervals.get(i);
			int start = interval.start - startTime;
			int end = interval.end - startTime;			
			
			if (end == start) 
				continue;
			
			// Some process earlier decided to ignore rendering this interval....
			if (intensityMap != null && intensityMap.get(interval.toString()) == null)
				continue;
			
			int index = props.indexOf(interval.name);

			int iw = (end - start) * _timeWidth;
			int ih = _rowHeight;
			
			int x = (start*_timeWidth)+propArea;
			int y = (index*_rowHeight)+2;

			g.setColor(Color.black);
			g.drawRect(x, y, iw, ih-3);
			
			if (intensityMap == null || intensityMap.size() == 0) { 
				g.setColor(Color.lightGray);
			} else { 
				String name = interval.toString();
				Double d = intensityMap.get(name);
				if (d == null) { 
					g.setColor(Color.white);
				} else { 
					float f = (float) d.doubleValue();
					g.setColor(new Color(1,1-f,1-f));
				}
			}
			g.fillRect(x+1, y+1, iw-1, ih-4);
		}
		return img;
	}

	
	public static void sample1() { 
		List<Interval> intervals = new ArrayList<Interval>();
		intervals.add(Interval.make("a", 0, 10));
		intervals.add(Interval.make("a", 14, 18));
		intervals.add(Interval.make("b", 3, 7));
		intervals.add(Interval.make("c", 0, 4));
		intervals.add(Interval.make("c", 9, 16));
		intervals.add(Interval.make("d", 6, 12));
		
		Paint.render(intervals, "/Users/wkerr/Desktop/synthetic1.png");
		
		AllenRelation.text = AllenRelation.fullText;
		
		
		Collections.sort(intervals, Interval.esf);
		logger.debug(SequenceType.allen.getSequence(intervals));
	}
	
	public static void sample2() { 
		List<Interval> set1 = new ArrayList<Interval>();
		set1.add(Interval.make("a", 0, 10));
		set1.add(Interval.make("b", 5, 15));
		set1.add(Interval.make("c", 3, 7));

		Paint.render(set1, "/Users/wkerr/Desktop/example1.png");
		Collections.sort(set1, Interval.esf);
		logger.debug(SequenceType.allen.getSequence(set1));
	
		List<Interval> set2 = new ArrayList<Interval>();
		set2.add(Interval.make("a", 0, 10));
		set2.add(Interval.make("b", 5, 15));
		set2.add(Interval.make("c", 3, 7));
		set2.add(Interval.make("d", 0, 4));

		Paint.render(set2, "/Users/wkerr/Desktop/example2.png");
		Collections.sort(set2, Interval.esf);
		logger.debug(SequenceType.allen.getSequence(set2));

	
		List<Interval> set3 = new ArrayList<Interval>();
		set3.add(Interval.make("a", 0, 6));
		set3.add(Interval.make("b", 3, 7));
		set3.add(Interval.make("c", 2, 5));
		set3.add(Interval.make("d", 6, 9));

		Paint.render(set3, "/Users/wkerr/Desktop/example3.png");
		Collections.sort(set3, Interval.esf);
		logger.debug(SequenceType.allen.getSequence(set3));
		
		List<Interval> set4 = new ArrayList<Interval>();
		set4.add(Interval.make("a", 2, 8));
		set4.add(Interval.make("b", 5, 10));
		set4.add(Interval.make("c", 4, 7));
		set4.add(Interval.make("e", 0, 4));

		Paint.render(set4, "/Users/wkerr/Desktop/example4.png");
		Collections.sort(set4, Interval.esf);
		logger.debug(SequenceType.allen.getSequence(set4));
		
	}	
}
