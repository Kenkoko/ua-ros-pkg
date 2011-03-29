package edu.arizona.verbs.experiments;

import java.util.List;
import java.util.Map;

public class State {
	public String name;
	public String className;
	
	private Map<String,String> attributes_;
	
	public void setAttributes(Map<String,String> attributes) {
		this.attributes_ = attributes;
	}

	public void setAttributes(List<String> decoy) {
		
	}
	
	public Map<String, String> getAttributes() {
		return attributes_;
	}
	
}
