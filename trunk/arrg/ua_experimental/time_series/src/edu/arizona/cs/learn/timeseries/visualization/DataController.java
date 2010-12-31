package edu.arizona.cs.learn.timeseries.visualization;

import static java.awt.GridBagConstraints.BOTH;
import static java.awt.GridBagConstraints.NONE;

import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.DefaultComboBoxModel;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.timeseries.model.Episode;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.timeseries.visualization.graphics.DataComponent;
import edu.arizona.cs.learn.timeseries.visualization.graphics.ScrolledCanvas;
import edu.arizona.cs.learn.timeseries.visualization.graphics.TimelineCanvas;
import edu.arizona.cs.learn.timeseries.visualization.graphics.color.HeatModel;
import edu.arizona.cs.learn.timeseries.visualization.graphics.color.HeatRelativeModel;
import edu.arizona.cs.learn.timeseries.visualization.graphics.color.SimpleModel;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;
import edu.arizona.cs.learn.util.graphics.GBC;

public class DataController extends Controller {
    private static Logger logger = Logger.getLogger(DataController.class);
	protected JFrame    _dataFrame;
	
	boolean _loadingModel = false;
	
	protected DefaultComboBoxModel _episodes;
	protected JTextArea            _sequenceText;
		
	public DataController() {
		loadProperties();
	}
	
	/**
	 * This sets the current DataModel to a new model created
	 * @param model
	 */
	public void setModel(DataModel model) { 
		_loadingModel = true;
		_dataModel = model;
	
		_episodes.removeAllElements();
		for (Episode e : _dataModel.episodes()) { 
			_episodes.addElement(e.id());
		}
		
		for (DataComponent dc : _listeners) { 
			dc.modelChanged(_dataModel);
		}
		_loadingModel = false;
	}
	
	public void sendEpisodeChanged() { 
		for (DataComponent dc : _listeners) { 
			dc.episodeChanged();
		}
	}
	
	public void setEpisode(int id) { 
		_dataModel.setEpisode(id);
		sendEpisodeChanged();
		
		Instance instance = _dataModel.instance();
		StringBuffer buf = new StringBuffer();
		for (Symbol obj : instance.sequence()) { 
			buf.append(obj.weight() + "," + obj.toString() +"\n");
		}
		_sequenceText.setText(buf.toString());
	}
	
	public void redraw() { 
		for (DataComponent dc : _listeners) { 
			dc.repaint();
		}
	}
	
	public void start() {
//		_dataModel = new DataModel();
		_dataFrame = new JFrame("Multiple Sequence Alignment");
		_dataFrame.setSize(1024,768);

		Container content = _dataFrame.getContentPane();
		content.setLayout(new GridBagLayout());

		final JLabel fileLabel = new JLabel("");

		JPanel statusPanel = new JPanel();
		statusPanel.setLayout(new GridBagLayout());
		statusPanel.add(new JLabel("File: "), GBC.makeGBC(0,0,BOTH,0,0));
		statusPanel.add(fileLabel, GBC.makeGBC(1,0,BOTH,0,0));
		statusPanel.add(new JPanel(), GBC.makeGBC(2,0,BOTH,1,0));
		
		content.add(statusPanel, GBC.makeGBC(0, 1, 2, 1, BOTH, 0, 0));
		
		JPanel splitTop = new JPanel();
		splitTop.setLayout(new GridBagLayout());

		JButton loadButton = new JButton("Load...");
		loadButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				String path = _properties.getProperty("load-path");
				JFileChooser jf = new JFileChooser(path);
				if (jf.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
					DataModel model = new DataModel();
					model.load(jf.getSelectedFile());
					setModel(model);
					setEpisode(0);
					
					fileLabel.setText(jf.getSelectedFile().getName());
					
					saveDirectory("load-path", jf.getSelectedFile().getPath());
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
		topPanel.add(loadButton, GBC.makeGBC(0, 0, GridBagConstraints.NONE, 0, 0));
		topPanel.add(new JPanel(), GBC.makeGBC(1, 0, BOTH, 1, 1));
		topPanel.add(drawGrid, GBC.makeGBC(2, 0, NONE, 0, 0));

		splitTop.add(topPanel, GBC.makeGBC(0, 0, BOTH, 1, 0));
		
		TimelineCanvas bpCanvas = new TimelineCanvas(this, new SimpleModel(), true);
		TimelineCanvas canvas = new TimelineCanvas(this, new SimpleModel(), false);

		ScrolledCanvas sc1 = new ScrolledCanvas(this, canvas, false);
		sc1.addPanel(canvas);

		ScrolledCanvas sc2 = new ScrolledCanvas(this, bpCanvas, false);
		sc2.addPanel(bpCanvas);
		
		_sequenceText = new JTextArea();
		JScrollPane sequenceScroll = new JScrollPane(_sequenceText);
		
		JTabbedPane tabbedPane = new JTabbedPane(JTabbedPane.BOTTOM);
		tabbedPane.add("BitPattern", sc2);
		tabbedPane.add("Episode", sc1);
		tabbedPane.add("Sequence", sequenceScroll);
		splitTop.add(tabbedPane, GBC.makeGBC(0, 1, BOTH, 1, 1));

		JPanel splitBottom = new JPanel();
		splitBottom.setLayout(new GridBagLayout());
		
		final JLabel learnedLabel = new JLabel("0");
		final JLabel alignmentLabel = new JLabel("0");
		final JLabel scoreLabel = new JLabel("0");

		_episodes = new DefaultComboBoxModel();
		final JComboBox box = new JComboBox(_episodes);
		box.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent e) {
				// only process the selection event.
				if (!_loadingModel && e.getStateChange() == ItemEvent.SELECTED) { 
					setEpisode((Integer) box.getSelectedItem());
					
					// TODO: score changed...
//					scoreLabel.setText(_dataModel.getScore() + "");
				}
			}
		});

		JButton nextButton = new JButton("Next");
		nextButton.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
				if (_dataModel == null) 
					return;
				
//				_dataModel.nextAlignment();
//				learnedLabel.setText(_dataModel.getSeenCount() + "");
//				alignmentLabel.setText(_dataModel.getConceptSize() + "");
				
				sendEpisodeChanged();
				redraw();
//				scoreLabel.setText(_dataModel.getScore() + "");
			}
		});
		JButton resetButton = new JButton("Reset");
		resetButton.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
				if (_dataModel == null) 
					return;
				
//				_dataModel.resetAlignment();
//
//				learnedLabel.setText(_dataModel.getSeenCount() + "");
//				alignmentLabel.setText(_dataModel.getConceptSize() + "");
//				scoreLabel.setText(_dataModel.getScore() + "");
			}
		});
		JButton seeButton = new JButton("Alignment");
		seeButton.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
				if (_dataModel == null)
					return;
				
				StringBuffer buf = new StringBuffer();
				for (Symbol obj : _dataModel.signature().signature()) {
					buf.append(obj.weight() + "\t" + obj.toString() + "\n");
				}
				JTextArea textArea = new JTextArea(buf.toString());
				JScrollPane pane = new JScrollPane(textArea);
				JDialog dialog = new JDialog(_dataFrame, "Alignment", false);
				dialog.getContentPane().add(pane);
				dialog.setSize(200, 400);
				dialog.setVisible(true);
			}
		});

		splitBottom.add(box, GBC.makeGBC(0, 0, BOTH, 0, 1));
		splitBottom.add(new JPanel(), GBC.makeGBC(1, 0, BOTH, 1, 1));

		JPanel p = new JPanel();
		p.setLayout(new GridLayout(1,4));
		p.add(new JLabel("Training Count:"));
		p.add(learnedLabel);

		p.add(new JLabel("Alignment Size: "));
		p.add(alignmentLabel);
		
		JButton msa = new JButton("MSA Score");
		msa.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
//				if (_dataModel == null || _dataModel.getScoreReport() == null)
//					return;
				
//				StringBuffer buf = new StringBuffer();
//				ScoreReport score = _dataModel.getScoreReport();
//				buf.append("Score: " + score.score + "\n");
//				buf.append("Matches: " + score.numMatches + "\n");
//				buf.append("Sequence 1 Mismatches: " + score.s1Mismatch + " size: " + score.s1Size + "\n");
//				buf.append("Sequence 2 Mismatches: " + score.s2Mismatch + " size: " + score.s2Size + "\n");
//
//				JTextArea textArea = new JTextArea(buf.toString());
//				JScrollPane pane = new JScrollPane(textArea);
//				JDialog dialog = new JDialog(_dataFrame, "Score Report", false);
//				dialog.getContentPane().add(pane);
//				dialog.setSize(200, 200);
//				dialog.setVisible(true);
				
			}
		});
		JButton save = new JButton("Save");
		save.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
				if (_dataModel == null)
					return;
				
				String path = _properties.getProperty("msa-path");
				JFileChooser jf = new JFileChooser(path);
				if (jf.showSaveDialog(null) == JFileChooser.APPROVE_OPTION) {
					String fileName = jf.getSelectedFile().getAbsolutePath();
//					_dataModel.saveAlignment(fileName);
					saveDirectory("msa-path", jf.getSelectedFile().getPath());
				}
			}
		});
		JButton load = new JButton("Load");
		load.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
				String path = _properties.getProperty("msa-path");
				JFileChooser jf = new JFileChooser(path);
				if (jf.showOpenDialog(null) == JFileChooser.APPROVE_OPTION) {
					_dataModel.loadSignature(jf.getSelectedFile());
					saveDirectory("msa-path", jf.getSelectedFile().getPath());
				}
			}
		});
		JButton prune = new JButton("Prune");
		prune.addActionListener(new ActionListener() { 
			public void actionPerformed(ActionEvent e) { 
				if (_dataModel == null)
					return;
				
				_dataModel.signature().prune(3);
//				_dataModel.pruneConcept(3);
			}
		});
		
		splitBottom.add(p, GBC.makeGBC(0, 1, BOTH, 0, 1));

		p = new JPanel();
		p.add(msa);
		p.add(save);
		p.add(load);
		p.add(prune);
		
		splitBottom.add(p, GBC.makeGBC(0, 2, BOTH, 0, 1));

		p = new JPanel();
		p.add(nextButton);
		p.add(resetButton);
		p.add(seeButton);
		splitBottom.add(p, GBC.makeGBC(0, 3, BOTH, 1, 0));
		
		JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, splitTop, splitBottom);
		splitPane.setOneTouchExpandable(true);
		splitPane.setDividerLocation(1.0);
		splitPane.setResizeWeight(1);
		
		content.add(splitPane, GBC.makeGBC(0, 0, BOTH, 1, 1));
		
		TimelineCanvas heat1Canvas = new TimelineCanvas(this, new HeatModel(), true);
		TimelineCanvas heat2Canvas = new TimelineCanvas(this, new HeatRelativeModel(), true);
		
		heat2Canvas.setHighlightSelected(true);

		Insets insets = new Insets(2,2,2,2);
		JPanel panel = new JPanel();
		panel.setLayout(new GridBagLayout());
		panel.add(heat1Canvas, GBC.makeGBC(0, 0, BOTH, insets, 1, 1));
		panel.add(heat2Canvas, GBC.makeGBC(0, 1, BOTH, insets, 1, 1));
		
		ScrolledCanvas sc3 = new ScrolledCanvas(this, panel, true);
		sc3.addPanel(heat1Canvas);
		sc3.addPanel(heat2Canvas);
		
		content.add(sc3, GBC.makeGBC(1, 0, BOTH, 1, 1));
		
		_dataFrame.addWindowListener(new WindowAdapter() {
			public void windowClosed(WindowEvent e) {
				System.exit(0);
			}
		});
		
		_dataFrame.setVisible(true);
	}
	
	public static void main(String[] args) {
		DataController dc = new DataController();
		dc.start();
	}
}
