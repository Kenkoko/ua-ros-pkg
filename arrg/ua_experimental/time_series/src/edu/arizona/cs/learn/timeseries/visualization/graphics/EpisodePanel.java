package edu.arizona.cs.learn.timeseries.visualization.graphics;

import static java.awt.GridBagConstraints.BOTH;

import java.awt.GridBagLayout;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.io.File;

import javax.swing.DefaultComboBoxModel;
import javax.swing.JComboBox;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;

import edu.arizona.cs.learn.timeseries.model.Episode;
import edu.arizona.cs.learn.timeseries.visualization.Controller;
import edu.arizona.cs.learn.timeseries.visualization.graphics.color.HeatModel;
import edu.arizona.cs.learn.timeseries.visualization.model.DataModel;
import edu.arizona.cs.learn.util.graphics.GBC;

public class EpisodePanel extends JPanel implements DataComponent {
	
	protected DefaultComboBoxModel _episodes;
	protected JTextArea            _sequenceText;
	
	protected boolean _loading;
	
	protected DataModel _dataModel;
	protected TimelineCanvas _rawCanvas;
	protected TimelineCanvas _bppCanvas;
	
	protected int _eId;

	public EpisodePanel(Controller controller) { 
		super();
		controller.add(this);
		
		setLayout(new GridBagLayout());
		
		_loading = false;
		_episodes = new DefaultComboBoxModel();
		final JComboBox box = new JComboBox(_episodes);
		box.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent e) {
				// only process the selection event.
				if (!_loading && e.getStateChange() == ItemEvent.SELECTED) { 					
					_eId = (Integer) box.getSelectedItem();
					_dataModel.setEpisode(_eId);
					
					_rawCanvas.episodeChanged();
					_bppCanvas.episodeChanged();
				}
			}
		});
		
		add(box, GBC.makeGBC(0, 0, BOTH, 1, 0));
		
		_rawCanvas = new TimelineCanvas(null, new HeatModel(), false);
		_bppCanvas = new TimelineCanvas(null, new HeatModel(), true);

		ScrolledCanvas sc1 = new ScrolledCanvas(controller, _rawCanvas, false);
		sc1.addPanel(_rawCanvas);

		ScrolledCanvas sc2 = new ScrolledCanvas(controller, _bppCanvas, false);
		sc2.addPanel(_bppCanvas);
		
		_sequenceText = new JTextArea();
		JScrollPane sequenceScroll = new JScrollPane(_sequenceText);
		
		JTabbedPane tabbedPane = new JTabbedPane(JTabbedPane.BOTTOM);
		tabbedPane.add("BitPattern", sc2);
		tabbedPane.add("Episode", sc1);
		tabbedPane.add("Sequence", sequenceScroll);
		add(tabbedPane, GBC.makeGBC(0, 1, BOTH, 1, 1));

	}

	@Override
	public void modelSelected(File file) {
		_loading = true;
		
		_dataModel = new DataModel();
		_dataModel.load(file);
		_dataModel.setEpisode(0);
		
		_rawCanvas.modelChanged(_dataModel);
		_bppCanvas.modelChanged(_dataModel);

		_rawCanvas.episodeChanged();
		_bppCanvas.episodeChanged();

		_episodes.removeAllElements();
		for (Episode e : _dataModel.episodes()) { 
			_episodes.addElement(e.id());
		}
		
		_loading = false;
	}
	
	@Override
	public void modelChanged(DataModel dm) {
		throw new RuntimeException("Shouldn't be calling model changed within this program");
	}

	@Override
	public void episodeChanged() {
		throw new RuntimeException("Shouldn't be calling episode changed within this program");
	}

	@Override
	public void receiveMessage(String message) {
		// TODO Auto-generated method stub
		
	}
}
