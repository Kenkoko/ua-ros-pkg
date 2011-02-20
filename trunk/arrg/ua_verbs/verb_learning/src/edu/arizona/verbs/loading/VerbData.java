package edu.arizona.verbs.loading;

import java.util.List;

public class VerbData {
	private String verb;
	private List<String> arguments;
	private List<Trace> traces;
	
	public void setVerb(String verb) {
		this.verb = verb;
	}
	
	public String getVerb() {
		return verb;
	}
	
	public void setArguments(List<String> arguments) {
		this.arguments = arguments;
	}
	
	public List<String> getArguments() {
		return arguments;
	}

	public List<Trace> getTraces() {
		return traces;
	}

	public void setTraces(List<Trace> traces) {
		this.traces = traces;
	}

}
