package edu.arizona.cs.learn.timeseries.visualization.graph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

public class Node {

	private List<Object> _keys;
	private int _id;
	
	private String _color;
	private String _fontColor;
	
	public Node(Object key, int id) { 
		_keys = new LinkedList<Object>();
		_keys.add(key);
		_id = id;
		
		_color = "white";
		_fontColor = "black";
	}
	
	public boolean equals(Object o) { 
		if (!(o instanceof Node))
			return false;
		
		return _id == ((Node) o)._id;
	}
	
	public void merge(Node n) { 
		if (n._keys.get(0).equals("start"))
			return;
		
		List<Object> tmp = new ArrayList<Object>(n._keys);
		Collections.reverse(tmp);
		for (Object o : tmp) { 
			_keys.add(0, o);
		}
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
		return _fontColor;
	}

	public boolean containsKey(Object key) { 
		for (Object o : _keys) 
			if (o.equals(key))
				return true;
		return false;
	}
	
	public int id() { 
		return _id;
	}
	
	public String label() { 
		StringBuffer buf = new StringBuffer();
		for (Object o : _keys) { 
			buf.append(o + "\\n");
		}
		return buf.toString();
	}

	public String toDot() { 
		if (_color.startsWith("#") && _color.length() < 7) { 
			while (_color.length() < 7) { 
				System.out.println(_color);
				_color = _color + "FF";
			}
		}
		if (_color.equals("white") || _color.equals("#FFFFFF")) { 
			return "\t\"" + id() + "\" [fontcolor=\"" + _fontColor + "\",label=\"" + label() + "\"];\n";
		}
		return "\t\"" + id() + "\" [label=\"" + label() + "\",style=\"filled\",color=\"" + _color + "\",fontcolor=\"" + _fontColor + "\"];\n";
	}
}
