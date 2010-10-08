package edu.arizona.cs.learn.timeseries.visualization;

import static java.awt.GridBagConstraints.BOTH;

import java.awt.Container;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.visualization.graphics.DataComponent;
import edu.arizona.cs.learn.timeseries.visualization.graphics.EpisodePanel;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;
import edu.arizona.cs.learn.util.graphics.GBC;

public class HeatmapComparisonTool extends Controller {
    private static Logger logger = Logger.getLogger(HeatmapComparisonTool.class);
	protected JFrame    _dataFrame;
	
	boolean _loadingModel = false;
	
	public HeatmapComparisonTool() {		
		loadProperties();
	}
	
	/**
	 * This sets the current DataModel to a new model created
	 * @param model
	 */
	public void setModel(DataModel model) { 
	}
	
	public void redraw() { 
		for (DataComponent dc : _listeners) { 
			dc.repaint();
		}
	}
	
	public void start() {
//		_dataModel = new DataModel();
		_dataFrame = new JFrame("Heatmap Comparison Tool");
		_dataFrame.setSize(1024,768);

		Container content = _dataFrame.getContentPane();
		content.setLayout(new GridBagLayout());

		JPanel splitTop = new JPanel();
		splitTop.setLayout(new GridBagLayout());

		final JLabel fileLabel = new JLabel("");
		final JLabel sigLabel = new JLabel("");

		JButton loadButton = new JButton("Load...");
		loadButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				String path = _properties.getProperty("load-path");
				JFileChooser jf = new JFileChooser(path);
				if (jf.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
					for (DataComponent dc : _listeners)  
						dc.modelSelected(jf.getSelectedFile());

					fileLabel.setText(jf.getSelectedFile().getName());
					saveDirectory("load-path", jf.getSelectedFile().getPath());
				}
				
			}
		});		
		
		JButton sigButton = new JButton("Signature...");
		sigButton.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
				String path = _properties.getProperty("msa-path");
				JFileChooser jf = new JFileChooser(path);
				if (jf.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
					_dataModel.loadSignature(jf.getSelectedFile());
					
					sigLabel.setText(jf.getSelectedFile().getName());
					
					saveDirectory("msa-path", jf.getSelectedFile().getPath());
				}
			}
		});
				
		JCheckBox drawGrid = new JCheckBox("Draw Grid", true);
		drawGrid.addActionListener(new ActionListener() { 
			boolean selected = true;
			public void actionPerformed(ActionEvent e) { 
				selected = !selected;
				sendMessage("set drawgrid " + selected);
			}
		});
		
		JPanel topPanel = new JPanel();
		topPanel.setLayout(new GridBagLayout());
		topPanel.add(loadButton, GBC.makeGBC(0,0,BOTH,0,0));
		topPanel.add(sigButton, GBC.makeGBC(1,0,BOTH,0,0));
		topPanel.add(new JPanel(), GBC.makeGBC(2,0,BOTH,1,0));
		topPanel.add(drawGrid, GBC.makeGBC(3,0,BOTH,1,0));
		
		content.add(topPanel, GBC.makeGBC(0,0,2,1,BOTH,1,0));
		
		content.add(new EpisodePanel(this), GBC.makeGBC(0,1,BOTH,1,1));
		content.add(new EpisodePanel(this), GBC.makeGBC(1,1,BOTH,1,1));
		content.add(new EpisodePanel(this), GBC.makeGBC(0,2,BOTH,1,1));
		content.add(new EpisodePanel(this), GBC.makeGBC(1,2,BOTH,1,1));
		
		JPanel spLeft = new JPanel();
		spLeft.setLayout(new GridBagLayout());
		spLeft.add(new JLabel("File: "), GBC.makeGBC(0,0,BOTH,0,0));
		spLeft.add(fileLabel, GBC.makeGBC(1,0,BOTH,0,0));
		spLeft.add(new JPanel(), GBC.makeGBC(2,0,BOTH,1,0));
		content.add(spLeft, GBC.makeGBC(0, 3, BOTH, 1, 0));

		JPanel spRight = new JPanel();
		spRight.add(new JLabel("Signature: "), GBC.makeGBC(0,0,BOTH,0,0));
		spRight.add(sigLabel, GBC.makeGBC(1,0,BOTH,0,0));
		spRight.add(new JPanel(), GBC.makeGBC(2,0,BOTH,1,0));
		content.add(spRight, GBC.makeGBC(1, 3, BOTH, 1, 0));
		
				
		_dataFrame.addWindowListener(new WindowAdapter() {
			public void windowClosed(WindowEvent e) {
				System.exit(0);
			}
		});
		
		_dataFrame.setVisible(true);
	}
	
	public static void main(String[] args) {
		HeatmapComparisonTool dc = new HeatmapComparisonTool();
		dc.start();
	}
}
