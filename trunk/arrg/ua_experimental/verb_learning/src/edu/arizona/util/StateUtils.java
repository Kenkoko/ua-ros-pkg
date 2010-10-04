package edu.arizona.util;

import java.text.NumberFormat;

import org.apache.commons.lang.StringUtils;

import ros.pkg.simulator_state.msg.SimpleRelation;

public class StateUtils {
	public static String formatNumber(double number) {
		NumberFormat nf = NumberFormat.getInstance();
		nf.setMinimumFractionDigits(1);
		nf.setMaximumFractionDigits(1);
		return nf.format(number);
	}
	
	public static String formatRelation(SimpleRelation rel) {
		return rel.rel_name + "(" + StringUtils.join(rel.obj_names, ",") + ")";
	}
}
