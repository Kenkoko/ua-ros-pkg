package edu.arizona.verbs.mdp;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

public class OOMDPObjectState implements Comparable<OOMDPObjectState> {
	
	private String className_;
	private String name_;
	private Map<String,String> attributeMap_; 
	private List<String> attributeList_;
	private String hashString_ = null;
	
	// This constructor assumes the attributes will be set using setAttribute 
	public OOMDPObjectState(String name, String className)
	{
		className_ = className;
		name_ = name;
		attributeMap_ = new HashMap<String, String>();
		attributeList_ = new Vector<String>(); 
	}

	public void setAttribute(String attribute, String value) {
		attributeMap_.put(attribute, value);
		attributeList_ = new Vector<String>(attributeMap_.keySet());
		Collections.sort(attributeList_);
		hashString_ = null; // To regenerate it
	}
	
	public void setAttributes(String[] attributes, String[] values) {
		// Populate the attributes
		if (attributes.length != values.length) {
			throw new RuntimeException("ATTRIBUTE NAMES AND VALUES DO NOT MATCH!");
		}
		
		attributeList_ = new Vector<String>();
		attributeMap_ = new HashMap<String, String>();
		for (int i = 0; i < attributes.length; i++) {
			attributeList_.add(attributes[i]);
			attributeMap_.put(attributes[i], values[i]);
		}
		Collections.sort(attributeList_); // To ensure a consistent ordering
	}
	
	public OOMDPObjectState remapState(Map<String,String> nameMap) {
		if (nameMap.containsKey(name_)) {
			OOMDPObjectState newObject = new OOMDPObjectState(nameMap.get(name_), className_);
			for (Map.Entry<String, String> entry : attributeMap_.entrySet()) {
				newObject.setAttribute(entry.getKey(), entry.getValue()); // TODO: This could be done faster
			}
			return newObject;
		} else {
			return this; // If the object isn't an argument, don't remap
		}
	}
	
	public String getName() {
		return name_;
	}
	
	public String getClassName() {
		return className_;
	}
	
	public List<String> getAttributes() {
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
	
	public List<String> getValues() {
		List<String> result = new Vector<String>();
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
}
