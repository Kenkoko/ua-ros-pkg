package edu.arizona.verbs.shared;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

// TODO: We may have to remap attribute values if they refer to other objects
public class OOMDPObjectState implements Comparable<OOMDPObjectState>, Remappable<OOMDPObjectState> {
	
	// These need to be public for YAML Loading
	public String name;
	public String className;
	
	private TreeMap<String,String> attributes_; 
	private String hashString_ = null;
	
	public OOMDPObjectState() {
	}
	
	// This constructor assumes the attributes will be set using setAttribute 
	public OOMDPObjectState(String name, String className)
	{
		this.className = className;
		this.name = name;
		this.attributes_ = new TreeMap<String, String>();
	}
	
	public void setAttribute(String attribute, String value) {
		attributes_.put(attribute, value);
		
		hashString_ = null; // To regenerate it
	}
	
	public void setAttributes(Map<String,String> attributes) {
		attributes_ = new TreeMap<String, String>();
		attributes_.putAll(attributes);
		
		hashString_ = null; // To regenerate it
	}
	
	public void setAttributes(ArrayList<String> attributeNames, ArrayList<String> attributeValues) {
		// Populate the attributes
		if (attributeNames.size() != attributeValues.size()) {
			throw new RuntimeException("ATTRIBUTE NAMES AND VALUES DO NOT MATCH!");
		}
		
		attributes_ = new TreeMap<String, String>();
		for (int i = 0; i < attributeNames.size(); i++) {
			attributes_.put(attributeNames.get(i), attributeValues.get(i));
		}
		
		hashString_ = null;
	}
	
	public String getName() {
		return name;
	}
	
	public String getClassName() {
		return className;
	}
	
	public ArrayList<String> getAttributeNames() {
		return new ArrayList<String>(attributes_.keySet());
	}
	
	public Map<String,String> getAttributes() {
		return attributes_;
	}
	
	/**
	 * Get a single value from the attribute map
	 * @param key
	 * @return
	 */
	public String getValue(String key) { 
		return attributes_.get(key);
	}
	
	public ArrayList<String> getValues() {
		ArrayList<String> result = new ArrayList<String>();
		for (String a : attributes_.keySet()) {
			result.add(attributes_.get(a));
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
			hashString_ = className + "<" + name + ">";
			for (String attribute : attributes_.keySet()) {
				hashString_ += attribute + "={" + attributes_.get(attribute) + "}";
			}
		}
		return hashString_;
	}

	@Override
	public OOMDPObjectState remap(Map<String, String> nameMap) {
		if (nameMap.containsKey(name)) {
			OOMDPObjectState newObject = new OOMDPObjectState(nameMap.get(name), className);
			for (Map.Entry<String, String> entry : attributes_.entrySet()) {
				newObject.setAttribute(entry.getKey(), entry.getValue()); // TODO: Do this faster
			}
			return newObject;
		} else {
			return this; // If the object isn't an argument, don't remap
		}
	}
	
	
}
