package edu.arizona.cs.learn.timeseries.visualization.model;

import java.io.File;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.apache.log4j.Logger;

import edu.arizona.cs.learn.algorithm.bpp.BPPFactory;
import edu.arizona.cs.learn.timeseries.model.Episode;
import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.cs.learn.util.Utils;

public class DataModel {
    private static Logger logger = Logger.getLogger(DataModel.class);

    protected boolean _finish = true;
    
    protected int _uniqueCount = 0;
	
	protected SequenceType _sequenceType;
	
	/** maps the episode id to the index in the episode list */
	protected List<Episode> _episodes;
	protected Signature _signature;

	protected int _currentId;
	protected List<Interval> _episode;
	protected List<Interval> _compressed;
	protected Instance _instance;
	protected Set<String> _propSet;
	
	public DataModel() {
		// the default is Allen sequences.
		_sequenceType = SequenceType.allen;
	}

	/**
	 * Load in all of the episodes found in some file.
	 * @param file
	 */
	public void load(File file) { 
		String name = file.getName();
		_episodes = Utils.loadEpisodes(name, file);
	}
	
	/**
	 * Load in a specific signature in order to see heatmaps
	 * for the given Signature.
	 * @param file
	 */
	public void loadSignature(File file) { 
		_signature = Signature.fromXML(file.getAbsolutePath());
	}
	
	/**
	 * The the active episode for the DataModel
	 * @param id
	 */
	public void setEpisode(int id) { 
		_currentId = id;
		
		Episode e = _episodes.get(_currentId);
		_episode = e.intervals();
		_compressed = BPPFactory.compress(_episode, Interval.eff);
		_instance = e.toInstance(_sequenceType);
		
		_propSet = new TreeSet<String>();
		for (Interval interval : _episode)
			_propSet.add(interval.name);
	}
	
	/**
	 * Retrieve the episode with the given id
	 * @param id
	 * @return
	 */
	public Episode getEpisode(int id) { 
		return _episodes.get(id);
	}
	
	/**
	 * Get the episodes found inside of this
	 * world.
	 * @return
	 */
	public List<Episode> episodes() { 
		return _episodes;
	}
	
	/**
	 * Returns the signature associated with this
	 * DataModel
	 */
	public Signature signature() {
		return _signature;
	}
	
	/**
	 * Set the sequence type of this model.
	 * @param type
	 */
	public void sequenceType(SequenceType type) {
		_sequenceType = type;
	}
	
	/**
	 * return the current sequence type of this model
	 * @return
	 */
	public SequenceType sequenceType() { 
		return _sequenceType;
	}
	
	/**
	 * Retrieve the original list of Intervals for an episode
	 * @return
	 */
	public List<Interval> episode() { 
		return _episode;
	}
	
	/**
	 * Retrieve the compressed list of intervals for an episode
	 * @return
	 */
	public List<Interval> compressed() { 
		return _compressed;
	}
	
	/**
	 * Return the set of propositions in the currently
	 * selected episode
	 * @return
	 */
	public Set<String> propSet() { 
		return _propSet;
	}
	
	/**
	 * Return the sequence associated with the selected episode
	 * @return
	 */
	public Instance instance() { 
		return _instance;
	}
}
