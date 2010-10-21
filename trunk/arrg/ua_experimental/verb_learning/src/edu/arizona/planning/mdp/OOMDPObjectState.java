package edu.arizona.planning.mdp;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

import ros.pkg.oomdp_msgs.msg.MDPObjectState;

public class OOMDPObjectState implements Comparable<OOMDPObjectState> {

	public static OOMDPObjectState remapState(OOMDPObjectState object, Map<String,String> nameMap) {
		if (nameMap.containsKey(object.name_)) {
			OOMDPObjectState newObject = new OOMDPObjectState(nameMap.get(object.name_), object.className_);
			for (Map.Entry<String, String> entry : object.attributeMap_.entrySet()) {
				newObject.setAttribute(entry.getKey(), entry.getValue()); // TODO: This could be done faster
			}
			return newObject;
		} else {
			return object; // If the object isn't an argument, don't remap
		}
	}
	
	private String className_;
	private String name_;
	private Map<String,String> attributeMap_; 
	private List<String> attributeList_;
	private String hashString_ = null;
	
	public OOMDPObjectState(MDPObjectState state) {
		className_ = state.class_name;
		name_ = state.name;
		
		// Populate the attributes
		if (state.attributes.length != state.values.length) {
			throw new RuntimeException("ATTRIBUTE NAMES AND VALUES DO NOT MATCH!");
		}
		attributeList_ = new Vector<String>();
		attributeMap_ = new HashMap<String, String>();
		for (int i = 0; i < state.attributes.length; i++) {
			attributeList_.add(state.attributes[i]);
			attributeMap_.put(state.attributes[i], state.values[i]);
		}
		Collections.sort(attributeList_); // To ensure a consistent ordering
	}
	
	// This constructor assumes the attributes will be set using setAttribute 
	public OOMDPObjectState(String name, String className)
	{
		className_ = className;
		name_ = name;
		attributeMap_ = new HashMap<String, String>();
		attributeList_ = new Vector<String>(); 
	}
	
	public MDPObjectState convertToROS() {
		MDPObjectState state = new MDPObjectState();
		
		state.class_name = className_;
		state.name = name_;
		state.attributes = attributeList_.toArray(new String[0]);
		state.values = new String[state.attributes.length];
		for (int i = 0; i < state.attributes.length; i++) {
			state.values[i] = attributeMap_.get(state.attributes[i]);
		}
		
		return state;
	}

	public void setAttribute(String attribute, String value) {
		attributeMap_.put(attribute, value);
		attributeList_ = new Vector<String>(attributeMap_.keySet());
		Collections.sort(attributeList_);
		hashString_ = null; // To regenerate it
	}
	
	public String getName() {
		return name_;
	}
	
	public String getClassName() {
		return className_;
	}
	
	@Override
	public int compareTo(OOMDPObjectState other) {
		return this.toString().compareTo(other.toString());
	}
	
	@Override
	public String toString() {
		if (hashString_ == null) {
			hashString_ = className_ + "<" + name_ + ">";
			for (String attribute : attributeList_) {
				hashString_ += attribute + "={" + attributeMap_.get(attribute) + "}";
			}
		}
		return hashString_;
	}
}
