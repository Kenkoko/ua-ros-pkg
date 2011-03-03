package edu.arizona.verbs.loading;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.google.common.base.Function;
import com.google.common.collect.Lists;

import edu.arizona.verbs.shared.OOMDPState;

public class Trace {
	private Map<String, String> bindings;
	private String label;
	private List<SimpleRelationalState> states;
	
	public Map<String, String> getBindings() {
		return bindings;
	}
	
	public void setBindings(Map<String, String> bindings) {
		this.bindings = new HashMap<String, String>();
		for (Map.Entry<String, String> e : bindings.entrySet()) {
			this.bindings.put(e.getValue(), e.getKey());
		}
	}
	
	public String getLabel() {
		return label;
	}
	
	public void setLabel(String label) {
		this.label = label;
	}

	public List<SimpleRelationalState> getStates() {
		return states;
	}

	public void setStates(List<SimpleRelationalState> states) {
		this.states = states;
	}
	
	public List<OOMDPState> convert() {
		return Lists.transform(states, new Function<SimpleRelationalState, OOMDPState>() {
			@Override
			public OOMDPState apply(SimpleRelationalState s) {
				return s.convert(bindings);
			}});
	}
}
