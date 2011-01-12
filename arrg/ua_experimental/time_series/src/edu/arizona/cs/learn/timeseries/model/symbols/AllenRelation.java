package edu.arizona.cs.learn.timeseries.model.symbols;

import java.util.ArrayList;
import java.util.List;

import org.dom4j.Element;

import edu.arizona.cs.learn.timeseries.model.Interval;
import edu.arizona.cs.learn.util.Utils;


public class AllenRelation extends StringSymbol {
	
	public static final String[] fullText = new String[] { 
		"equals", "starts-with", "finishes-with", "before",
		"none", "meets", "overlaps", "contains", "during"
	};

	public static final String[] partialText = new String[] { 
		"eq", "s", "f", "b", "n", "m", "o", "c", "d"
	};
	
	public static String[] text = partialText;
	
	private String _prop1;
	private String _relation;
	private String _prop2;
	
	private String _key;
	
	private Interval _interval1;
	private Interval _interval2;
	
	public AllenRelation(String p1, String r, String p2) { 
		this(p1, r, p2, 1.0);
	}
	
	public AllenRelation(String relation, Interval i1, Interval i2) { 
		this(relation, i1, i2, 1.0);
	}

	public AllenRelation(String relation, Interval i1, Interval i2, double weight) { 
		this(i1.name, relation, i2.name, weight);
		
		_interval1 = i1;
		_interval2 = i2;
	}
	
	public AllenRelation(String p1, String r, String p2, double weight) { 
		super(weight);
		
		_prop1 = p1;
		_relation = r;
		_prop2 = p2;
		
		_key = "(" + _prop1 + " " + _relation + " " + _prop2 + ")";
	}
	
	public String prop1() { 
		return _prop1;
	}

	public String prop2() { 
		return _prop2;
	}
	
	public String relation() { 
		return _relation;
	}
	
	public String toString() { 
		return _key + "--" + _weight;
	}
	
	/**
	 * The hash code for this object is just the hash code of
	 * the key itself...
	 */
	public int hashCode() { 
		return _key.hashCode();
	}
	
	public String latex() { 
		return "\\ar{" + _prop1 + "}{" + _relation + "}{" + _prop2 + "} [$\\cdot$]";
	}
	
	public Interval interval1() { 
		if (_interval1 == null)
			throw new RuntimeException("Used the wrong constructor for this call");
		return _interval1;
	}
	
	public Interval interval2() { 
		if (_interval2 == null)
			throw new RuntimeException("Used the wrong constructor for this call");
		return _interval2;
	}
	
	public boolean equals(Object o) { 
		if (!(o instanceof AllenRelation))
			return false;
		
		AllenRelation r = (AllenRelation) o;
		return _key.equals(r._key);
	}
	
	/**
	 * Called when Interval i1 finishes 
	 * @param i1
	 * @param i2
	 * @return
	 */
	public static String forward(Interval i1, Interval i2) { 
		if (i1.start == i2.start && i1.end == i2.end)
			return text[0];
		if (i1.start == i2.start)
			return text[1];
		if (i1.end == i2.end)
			return text[2];

		if (i1.end < i2.start && i2.start - i1.end < Utils.WINDOW)
			return text[3];
		if (i1.end < i2.start)
			return text[4];
		if (i1.end == i2.start)
			return text[5];

		// i2.start > i1.start due to starts-with check and sorting
		if (i1.end < i2.end)
			return text[6];
		if (i1.end > i2.end) 
			return text[7];
		
		throw new RuntimeException("Unknown relation between:\n  " + 
				i1.toString() + "\n  " + i2.toString());	
	}
	
	/**
	 * Assumptions made when calling this function....
	 *   Interval i1 ends before or at the same time as Interval i2.
	 * @param i1
	 * @param i2
	 * @return
	 */
	public static String get(Interval i1, Interval i2) {
		if (i1.start == i2.start && i1.end == i2.end)
			return text[0];
		if (i1.start == i2.start)
			return text[1];
		if (i1.start > i2.start)
			return text[8];
		
		if (i1.end < i2.start && i2.start - i1.end < Utils.WINDOW)
			return text[3];
		if (i1.end < i2.start)
			return text[4];
		if (i1.end == i2.start)
			return text[5];
		if (i1.end < i2.end)
			return text[6];
		if (i1.end == i2.end)
			return text[2];
		
		throw new RuntimeException("Unknown relation between:\n  " + 
				i1.toString() + "\n  " + i2.toString());	
	}

	@Override
	public List<String> getProps() {
		List<String> props = new ArrayList<String>();
		props.add(_prop1);
		props.add(_prop2);
		return props;
	}

	@Override
	public List<Interval> getIntervals() {
		List<Interval> intervals = new ArrayList<Interval>();
		intervals.add(_interval1);
		intervals.add(_interval2);
		return intervals;
	}

	@Override
	public String getKey() {
		return _key;
	}	
	
	@Override
	public Symbol copy() { 
		if (_interval1 != null && _interval2 != null) 
			return new AllenRelation(_relation, _interval1, _interval2, _weight);

		return new AllenRelation(_prop1, _relation, _prop2, _weight);
	}
	
	@Override
	public Symbol merge(Symbol B) { 
		return new AllenRelation(_relation, _interval1, _interval2, _weight+B._weight);
	}
	
	public void toXML(Element e) {
		Element allen = e.addElement("symbol")
			.addAttribute("class", "AllenRelation");

		allen.addAttribute("key", this._key);
		allen.addAttribute("relation", this._relation);
		allen.addAttribute("weight", this._weight +"");
		this._interval1.toXML(allen);
		this._interval2.toXML(allen);
	}

	public static AllenRelation fromXML(Element e) {
		String key = e.attributeValue("key");
		String relation = e.attributeValue("relation");
		double weight = Double.parseDouble(e.attributeValue("weight"));

		List list = e.elements("Interval");
		if (list.size() != 2) {
			throw new RuntimeException("The number of intervals is wrong: "
					+ list.size());
		}
		Interval i1 = Interval.fromXML((Element) list.get(0));
		Interval i2 = Interval.fromXML((Element) list.get(1));

		return new AllenRelation(relation, i1, i2, weight);
	}
}