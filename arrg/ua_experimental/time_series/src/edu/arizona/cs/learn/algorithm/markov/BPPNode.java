package edu.arizona.cs.learn.algorithm.markov;

import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.apache.log4j.Logger;

public class BPPNode {
	private static Logger logger = Logger.getLogger(BPPNode.class);
	public static int _counter = 0;
	private int _id;
	
	private List<String> _propList;
	private Set<String>  _props;

	private String _values;
	private String _color;
	private String _fontColor;
	
	private boolean _isFinal;
	private int _activeDepth;			// Distance from start in active path
	private int _distanceToFinal;		// Distance to 'nearest' final state
	
	private int _timeSpentOff;
	
	private BPPNode _startNode;

	protected BPPNode(BPPNode startNode) {
		_id = (_counter++);

		_startNode = startNode;
		_color = "white";
		_fontColor = "black";

		_timeSpentOff = 0;
		_isFinal = false;
		_activeDepth = 0;
		_distanceToFinal = Integer.MAX_VALUE;

		_props = new TreeSet<String>();
	}

	public BPPNode(List<String> propList, StringBuffer buf, BPPNode startNode,
			boolean isFinalState) {
		
		this(startNode);

		_propList = propList;
		_values = buf.toString();

		_props = new TreeSet<String>();
		for (int i = 0; i < this._propList.size(); i++) {
			if (_values.charAt(i) == '1') {
				_props.add(_propList.get(i));
			}
		}
		
		if (_props.size() == 0) {
			logger.debug("Created empty node: " + _id);
		}

		_color = "white";
		_fontColor = "black";
		
		_isFinal = isFinalState;
	}
	
	public BPPNode(List<String> propList, StringBuffer buf, BPPNode startNode) {
		this(propList, buf, startNode, false);
	}

	public int id() {
		return this._id;
	}

	public boolean isStart() {
		return this._startNode == null;
	}
	
	public boolean isFinal() {
		return _isFinal;
	}

	public BPPNode getStartState() {
		return this._startNode;
	}
	
	public int timeSpentOff() { 
		return _timeSpentOff;
	}
	
	public boolean satisfied(Set<String> props) {
		return props.containsAll(this._props);
	}

	/**
	 * Returns true if this node is activated by
	 * the set of propositions.  If false, the _timeSpentOff
	 * counter is incremented, if true then the _timeSpentOff
	 * counter is reset.
	 * @param activeProps
	 * @return
	 */
	public boolean active(Set<String> activeProps) {
		for (int i = 0; i < this._propList.size(); i++) {
			if ((this._values.charAt(i) == '1')
					&& (!activeProps.contains(this._propList.get(i)))) {
				++_timeSpentOff;
				return false;
			}
		}
		_timeSpentOff = 0;
		return true;
	}

	public boolean equals(Object o) {
		if (!(o instanceof BPPNode)) {
			return false;
		}
		BPPNode node = (BPPNode) o;
		return _id == node._id;
	}

	public void color(String color) {
		_color = color;
	}

	public String color() {
		return _color;
	}

	public void fontColor(String color) {
		_fontColor = color;
	}

	public String fontColor() {
		return this._fontColor;
	}

	public String label() {
		StringBuffer buf = new StringBuffer();
		for (int i = 0; i < _propList.size(); i++) {
			if (_values.charAt(i) == '1') {
				buf.append((String) _propList.get(i) + " \\n");
			}
		}

		return buf.toString();
	}

	public String toDot() {
		if ((this._color.equals("white")) || (this._color.equals("#FFFFFF"))) {
			String node = "\t\"" + this._id + "\" [fontcolor=\""
					+ this._fontColor + "\",label=\"distanceToFinal: " +
					_distanceToFinal + "\\n" + label() + "\"" +
					(_isFinal ? ",shape=\"octagon\"" : "") + "];\n";
			String selfLoop = "\t\"" + this._id + "\" -> \"" + this._id
					+ "\" [label=\"" + label() + "\"];\n";
			return node + selfLoop;
		}
		return "\t\"" + this._id + "\" [label=\"distanceToFinal: " +
				_distanceToFinal + "\\n" + label() +
				"\",style=\"filled\",color=\"" + this._color
				+ "\",fontcolor=\"" + this._fontColor + "\"" +
				(_isFinal ? ",shape=\"octagon\"" : "") + "];\n";
	}

	public static String id(List<String> propList, StringBuffer buf) {
		StringBuffer id = new StringBuffer();
		for (int i = 0; i < propList.size(); i++) {
			id.append(buf.charAt(i) + " " + (String) propList.get(i) + "|");
		}
		return id.toString();
	}
	
	public List<String> getPropList() {
		return _propList;
	}
	
	public String getValues() {
		return _values;
	}
	
	public Set<String> getProps() {
		return _props;
	}
	
	public String getColor() {
		return _color;
	}
	
	public void setColor(String color) {
		_color = color;
	}
	
	public void setIsFinal(boolean isfinal) {
		_isFinal = isfinal;
	}
	
	public int getActiveDepth() {
		return _activeDepth;
	}
	
	public void setActiveDepth(int depth) {
		_activeDepth = depth;
	}
	
	public int getDistanceToFinal() {
		return _distanceToFinal;
	}
	
	public void setDistanceToFinal(int dist) {
		_distanceToFinal = dist;
	}
}