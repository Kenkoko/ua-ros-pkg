package edu.arizona.verbs.util;

import java.text.NumberFormat;

import org.apache.commons.lang.StringUtils;

import ros.pkg.oomdp_msgs.msg.Relation;

public class StateUtils {
	public static String formatNumber(double number) {
		NumberFormat nf = NumberFormat.getInstance();
		nf.setMinimumFractionDigits(1);
		nf.setMaximumFractionDigits(1);
		return nf.format(number);
	}
	
	public String formatRelation(Relation rel) {
		return rel.relation + "(" + StringUtils.join(rel.obj_names, ",") + ")";
	}
}
