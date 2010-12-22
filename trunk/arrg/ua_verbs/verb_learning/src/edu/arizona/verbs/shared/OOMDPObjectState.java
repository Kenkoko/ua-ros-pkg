package edu.arizona.verbs.shared;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

// TODO: We may have to remap attribute values if they refer to other objects
public class OOMDPObjectState implements Comparable<OOMDPObjectState>, Remappable<OOMDPObjectState> {
	
	private String className_;
	private String name_;
	private Map<String,String> attributeMap_; 
	private ArrayList<String> attributeList_;
	private String hashString_ = null;
	
	// This constructor assumes the attributes will be set using setAttribute 
	public OOMDPObjectState(String name, String className)
	{
		className_ = className;
		name_ = name;
		attributeMap_ = new HashMap<String, String>();
		attributeList_ = new ArrayList<String>(); 
	}

	public void setAttribute(String attribute, String value) {
		attributeMap_.put(attribute, value);
		attributeList_ = new ArrayList<String>(attributeMap_.keySet());
		Collections.sort(attributeList_);
		hashString_ = null; // To regenerate it
	}
	
	public void setAttributes(ArrayList<String> attributes, ArrayList<String> values) {
		// Populate the attributes
		if (attributes.size() != values.size()) {
			throw new RuntimeException("ATTRIBUTE NAMES AND VALUES DO NOT MATCH!");
		}
		
		attributeList_ = new ArrayList<String>();
		attributeMap_ = new HashMap<String, String>();
		for (int i = 0; i < attributes.size(); i++) {
			attributeList_.add(attributes.get(i));
			attributeMap_.put(attributes.get(i), values.get(i));
		}
		Collections.sort(attributeList_); // To ensure a consistent ordering
	}
	
	public String getName() {
		return name_;
	}
	
	public String getClassName() {
		return className_;
	}
	
	public ArrayList<String> getAttributes() {
		return attributeList_;
	}
	
	/**
	 * Get a single value from the attribute map
	 * @param key
	 * @return
	 */
	public String getValue(String key) { 
		return attributeMap_.get(key);
	}
	
	public ArrayList<String> getValues() {
		ArrayList<String> result = new ArrayList<String>();
		for (String a : attributeList_) {
			result.add(attributeMap_.get(a));
		}
		return result;
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

	@Override
	public OOMDPObjectState remap(Map<String, String> nameMap) {
		if (nameMap.containsKey(name_)) {
			OOMDPObjectState newObject = new OOMDPObjectState(nameMap.get(name_), className_);
			for (Map.Entry<String, String> entry : attributeMap_.entrySet()) {
				newObject.setAttribute(entry.getKey(), entry.getValue()); // TODO: Do this faster
			}
			return newObject;
		} else {
			return this; // If the object isn't an argument, don't remap
		}
	}
}
