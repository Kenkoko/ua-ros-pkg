package edu.arizona.cs.learn.timeseries.dissertation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import edu.arizona.cs.learn.timeseries.model.Instance;
import edu.arizona.cs.learn.timeseries.model.SequenceType;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;
import edu.arizona.cs.learn.util.Utils;

public class SequenceOverlap {
	public static void main(String[] args) throws Exception {
		String prefix = "ww3d";
		SequenceType type = SequenceType.starts;

		Map<String,List<Instance>> map = Utils.load(prefix, type);
		List<String> classes = new ArrayList<String>(map.keySet());
		Collections.sort(classes);

		BufferedWriter out = new BufferedWriter(new FileWriter("logs/overlap.csv"));
		out.write("name1,id1,name2,id2,type,value\n");
		for (int i = 0; i < classes.size(); i++) {
			List<Instance> list1 = map.get(classes.get(i));

			for (int s = 0; s < list1.size(); s++) {
				Instance i1 = (Instance) list1.get(s);

				for (int j = 0; j < classes.size(); j++) {
					String value = "external";
					if (i == j) {
						value = "internal";
					}
					for (Instance i2 : map.get(classes.get(j))) {
						out.write(i1.name() + "," + i1.id() + "," + i2.name()
								+ "," + i2.id() + "," + value + ","
								+ overlap(i1, i2) + "\n");
					}
				}
			}
		}
		out.close();
	}

	public static int overlap(Instance i1, Instance i2) {
		Set<String> s1 = new TreeSet<String>();
		Set<String> s2 = new TreeSet<String>();

		for (Symbol obj : i1.sequence()) {
			s1.add(obj.toString());
		}
		for (Symbol obj : i2.sequence()) {
			s2.add(obj.toString());
		}
		s1.retainAll(s2);
		return s1.size();
	}
}
