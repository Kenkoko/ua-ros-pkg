package edu.arizona.cs.learn.timeseries.model;

import java.util.ArrayList;
import java.util.List;

import org.dom4j.Element;

import edu.arizona.cs.learn.algorithm.alignment.model.Symbol;

/**
 * The event class essentially wraps the interval class so that we don't have to
 * make Intervals implement Symbol
 * 
 * @author wkerr
 * 
 */

public class Event extends Symbol {
	private String _key;
	private List<String> _props;
	private List<Interval> _intervals;

	public Event(Interval interval) {
		_key = interval.name;

		_props = new ArrayList<String>();
		_props.add(interval.name);

		_intervals = new ArrayList<Interval>();
		_intervals.add(interval);
	}

	public Event(String name, Interval interval) {
		this(interval);
		this._key = name;
	}

	public boolean equals(Object o) {
		if (!(o instanceof Event)) {
			return false;
		}
		Event event = (Event) o;
		return this._key.equals(event._key);
	}

	public List<Interval> getIntervals() {
		return this._intervals;
	}

	public String getKey() {
		return this._key;
	}

	public List<String> getProps() {
		return this._props;
	}

	public String toString() {
		return this._key;
	}

	public void toXML(Element e) {
		Element evt = e.addElement("Symbol")
			.addAttribute("type", "Event")	
			.addAttribute("key", _key);

		_intervals.get(0).toXML(evt);
	}

	public static Event fromXML(Element e) {
		String key = e.attributeValue("key");
		Interval i = Interval.fromXML(e.element("Interval"));

		return new Event(key, i);
	}
}