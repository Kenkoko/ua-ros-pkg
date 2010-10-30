package edu.arizona.verbs.util;

public abstract class Predicate<T> implements com.google.common.base.Predicate<T>, org.apache.commons.collections15.Predicate<T> {

	public abstract boolean value(T arg);
	
	@Override
	public boolean evaluate(T arg) {
		return value(arg);
	}

	@Override
	public boolean apply(T arg) {
		return value(arg);
	}
}
