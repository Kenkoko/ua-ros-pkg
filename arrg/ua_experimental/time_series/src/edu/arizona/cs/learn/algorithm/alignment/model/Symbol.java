package edu.arizona.cs.learn.algorithm.alignment.model;

import edu.arizona.cs.learn.timeseries.model.AllenRelation;
import edu.arizona.cs.learn.timeseries.model.CBA;
import edu.arizona.cs.learn.timeseries.model.Event;
import edu.arizona.cs.learn.timeseries.model.Interval;
import java.util.List;
import org.dom4j.Element;

public class Symbol {
	private String _name;

	public Symbol() {
	}

	public Symbol(String name) {
		_name = name;
	}

	public String getKey() {
		return _name;
	}
	
	public boolean equals(Object o) {
		if (!(o instanceof Symbol)) {
			return false;
		}
		Symbol symbol = (Symbol) o;
		return _name.equals(symbol._name);
	}
	
	public String toString() { 
		return _name;
	}
	
	public String latex() { 
		throw new RuntimeException("Latex is not defined for base class Symbol");
	}

	public List<String> getProps() {
		throw new RuntimeException("No propositions associated with a base class Symbol");
	}

	public List<Interval> getIntervals() {
		throw new RuntimeException("No intervals associated with a base class symbol");
	}

	public void toXML(Element e) {
		throw new RuntimeException("This constructor should never be used for XML purposes");
	}

	public static Symbol fromXML(Element e) {
		String type = e.attributeValue("type");

		if ("Event".equals(type)) {
			return Event.fromXML(e);
		}
		if ("AllenRelation".equals(type)) {
			return AllenRelation.fromXML(e);
		}
		if ("CBA".equals(type)) {
			return CBA.fromXML(e);
		}
		throw new RuntimeException("Unknown symbol type: " + type);
	}
}