package edu.arizona.verbs.verb;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import ros.pkg.verb_learning.msg.VerbDescription;

import com.google.common.base.Joiner;
import com.google.common.collect.Lists;

import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.timeseries.model.Signature;
import edu.arizona.verbs.fsm.VerbFSM;

public abstract class AbstractVerb implements Verb {
	protected String lexicalForm_;
	protected ArrayList<String> arguments_;
	protected Signature signature_ = null;
	protected Signature negativeSignature_; // This is not really used for pruning, just fits better
	protected VerbFSM fsm_ = null;
	
	AbstractVerb baseVerb_ = null;
	
	/* Updating the signatures */
	
	void initializeSignatures() {
		signature_ = new Signature(lexicalForm_);
		negativeSignature_ = new Signature("non-" + lexicalForm_);
	}
	
	public void setPositiveSignature(Signature s) {
		signature_ = s;
		postInstance();
	}
	
	public void setNegativeSignature(Signature s) {
		negativeSignature_ = s;
		postInstance();
	}
	
	@Override
	public void addPositiveInstances(List<Instance> instances) {
		for (Instance instance : instances) {
			signature_.update(instance.sequence());
		}

		postInstance();
	}
	
	@Override
	public void addPositiveInstance(Instance instance) {
		signature_.update(instance.sequence());
		
		postInstance();
	}
	
	@Override
	public void addNegativeInstance(Instance instance) {
		negativeSignature_.update(instance.sequence());
		
		postInstance();
	}
	
	@Override
	public void addNegativeInstances(List<Instance> instances) {
		for (Instance instance : instances) {
			negativeSignature_.update(instance.sequence());
		}
		
		postInstance();
	}
	
	@Override
	public void forgetInstances() {
		signature_ = new Signature(lexicalForm_);
		negativeSignature_ = new Signature("non-" + lexicalForm_);
	}
	
	abstract void postInstance();

	/* Getters, etc. */
	
	@Override
	public String getLexicalForm() {
		return lexicalForm_;
	}

	@Override
	public Signature getSignature() {
		return signature_;
	}

	@Override
	public VerbFSM getFSM() {
		return fsm_;
	}
	
	@Override
	public String[] getArgumentArray() {
		return arguments_.toArray(new String[0]);
	}
	
	@Override
	public ArrayList<String> getArguments() {
		return arguments_;
	}

	public boolean hasSignature() {
		return signature_ != null;
	}
	
	public boolean hasFSM() {
		return fsm_ != null;
	}
	
	/* Folders and ROS */
	
	void makeVerbFolder() {
		File verbFolder = new File(getVerbFolder());
		if (!verbFolder.exists()) {
			verbFolder.mkdirs();
		}
	}
	
	public String getVerbFolder() {
		return "verbs/" + getIdentifierString() + "/";
	}
	
	// TODO: Do we want to make new Lists?
	public VerbDescription makeVerbDescription() {
		VerbDescription desc = new VerbDescription();
		desc.verb = lexicalForm_;
		desc.arguments = arguments_;
		return desc;
	}

	@Override
	public String getIdentifierString() {
		return Joiner.on(",").join(Lists.asList(lexicalForm_, arguments_.toArray(new String[0])));
	}
	
	public AbstractVerb getBaseVerb() {
		if (baseVerb_ == null) {
			return this;
		} else {
			return baseVerb_;
		}
	}
}
