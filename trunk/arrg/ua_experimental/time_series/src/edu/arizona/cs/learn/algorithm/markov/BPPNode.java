package edu.arizona.cs.learn.algorithm.markov;

import java.util.List;
import java.util.Set;
import java.util.TreeSet;

public class BPPNode {
	public static int _counter = 0;
	private int _id;
	
	private List<String> _propList;
	private Set<String>  _props;

	private String _values;
	private String _color;
	private String _fontColor;
	
	private BPPNode _startNode;

	protected BPPNode(BPPNode startNode) {
		_id = (_counter++);

		_startNode = startNode;
		_color = "white";
		_fontColor = "black";

		_props = new TreeSet<String>();
	}

	public BPPNode(List<String> propList, StringBuffer buf, BPPNode startNode) {
		this(startNode);

		_propList = propList;
		_values = buf.toString();

		_props = new TreeSet<String>();
		for (int i = 0; i < this._propList.size(); i++) {
			if (_values.charAt(i) == '1') {
				_props.add(_propList.get(i));
			}
		}

		_color = "white";
		_fontColor = "black";
	}

	public int id() {
		return this._id;
	}

	public boolean isStart() {
		return this._startNode == null;
	}

	public BPPNode getStartState() {
		return this._startNode;
	}

	public boolean satisfied(Set<String> props) {
		return props.containsAll(this._props);
	}

	public boolean active(Set<String> activeProps) {
		for (int i = 0; i < this._propList.size(); i++) {
			if ((this._values.charAt(i) == '1')
					&& (!activeProps.contains(this._propList.get(i)))) {
				return false;
			}
		}
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
					+ this._fontColor + "\",label=\"" + label() + "\"];\n";
			String selfLoop = "\t\"" + this._id + "\" -> \"" + this._id
					+ "\" [label=\"" + label() + "\"];\n";
			return node + selfLoop;
		}
		return "\t\"" + this._id + "\" [label=\"" + label()
				+ "\",style=\"filled\",color=\"" + this._color
				+ "\",fontcolor=\"" + this._fontColor + "\"];\n";
	}

	public static String id(List<String> propList, StringBuffer buf) {
		StringBuffer id = new StringBuffer();
		for (int i = 0; i < propList.size(); i++) {
			id.append(buf.charAt(i) + " " + (String) propList.get(i) + "|");
		}
		return id.toString();
	}
	
	public Set<String> getProps() {
		return _props;
	}
}