package edu.arizona.cs.learn.timeseries.model;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import org.apache.log4j.Logger;
import org.dom4j.Document;
import org.dom4j.DocumentHelper;
import org.dom4j.Element;
import org.dom4j.io.OutputFormat;
import org.dom4j.io.SAXReader;
import org.dom4j.io.XMLWriter;

import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.SequenceAlignment;
import edu.arizona.cs.learn.algorithm.alignment.model.Instance;
import edu.arizona.cs.learn.algorithm.alignment.model.Symbol;
import edu.arizona.cs.learn.algorithm.alignment.model.WeightedObject;
import edu.arizona.cs.learn.timeseries.distance.Distances;

public class Signature {
	private static Logger logger = Logger.getLogger(Signature.class);

	private String _key;
	private List<WeightedObject> _signature;
	private List<Symbol[]> _rows;

	private int _count;
	private Params _params;

	public Signature(String key) {
		_key = key;
		_count = 0;
		_rows = new ArrayList<Symbol[]>();
		_signature = new ArrayList<WeightedObject>();

		_params = new Params();
		_params.setMin(0, 0);
		_params.setBonus(1.0D, 0.0D);
		_params.setPenalty(-1.0D, 0.0D);
	}

	public String key() {
		return _key;
	}

	public List<WeightedObject> signature() {
		return _signature;
	}

	public int trainingSize() {
		return _count;
	}

	public List<Symbol[]> table() {
		return _rows;
	}

	public void update(List<WeightedObject> seq) {
		_count += 1;

		_params.seq1 = _signature;
		_params.seq2 = seq;
		Report report = SequenceAlignment.align(_params);
		updateTable(report);

		_signature = SequenceAlignment.combineAlignments(report.results1, report.results2);
	}

	private void updateTable(Report report) {
		int newLength = report.results1.size();
		List<Symbol[]> tmp = new ArrayList<Symbol[]>(_rows.size() + 1);
		for (int i = 0; i < this._rows.size() + 1; i++) {
			tmp.add(new Symbol[newLength]);
		}

		int position = 0;
		for (int i = 0; i < report.results1.size(); i++) {
			WeightedObject left = report.results1.get(i);
			WeightedObject right = report.results2.get(i);
			if ((left != null) && (right != null)) {
				for (int j = 0; j < this._rows.size(); j++) {
					Symbol[] oldRow = _rows.get(j);
					Symbol[] newRow = tmp.get(j);
					newRow[i] = oldRow[position];
				}

				tmp.get(_rows.size())[i] = right.key();

				position++;
			} else if (right == null) {
				for (int j = 0; j < this._rows.size(); j++) {
					Symbol[] oldRow = _rows.get(j);
					Symbol[] newRow = tmp.get(j);
					newRow[i] = oldRow[position];
				}
				position++;
			} else {
				if (left != null) {
					continue;
				}

				tmp.get(_rows.size())[i] = right.key();
			}
		}
		_rows = tmp;
	}

	public void train(List<Instance> sequences) {
		for (int i = 0; i < sequences.size(); i++) {
			update(sequences.get(i).sequence());
		}
	}

	public void heuristicTraining(List<Instance> sequences) {
		Params params = new Params();
		params.setBonus(1.0D, 0.0D);
		params.setPenalty(-1.0D, 0.0D);
		params.setMin(0, 0);

		double distance = Double.POSITIVE_INFINITY;
		Instance i1 = null;
		Instance i2 = null;
		for (int i = 0; i < sequences.size(); i++) {
			for (int j = i + 1; j < sequences.size(); j++) {
				params.seq1 = ((Instance) sequences.get(i)).sequence();
				params.seq2 = ((Instance) sequences.get(j)).sequence();
				double d = SequenceAlignment.distance(params);

				if (d < distance) {
					i1 = (Instance) sequences.get(i);
					i2 = (Instance) sequences.get(j);
					distance = d;
				}

			}

		}

		update(i1.sequence());
		update(i2.sequence());

		sequences.remove(i1);
		sequences.remove(i2);

		while (!sequences.isEmpty()) {
			distance = Double.POSITIVE_INFINITY;
			Instance instance = null;

			for (Instance i : sequences) {
				params.seq1 = _signature;
				params.seq2 = i.sequence();

				double d = SequenceAlignment.distance(params);
				if (d < distance) {
					instance = i;
					distance = d;
				}
			}

			update(instance.sequence());
			sequences.remove(instance);
		}
	}

	public void printTable(List<String[]> table) {
		logger.debug("Table: " + table.size());
		StringBuffer buf = new StringBuffer();
		for (String[] row : table) {
			for (String s : row) {
				buf.append(s + "\t");
			}
			buf.append("\n");
		}
		logger.debug("\n" + buf.toString() + "\n");
	}

	/**
	 * Combine this signature with another signature to create
	 * a super signature.
	 * @param s2
	 */
	public void merge(Signature s2) {
		Signature s1 = this;

		Params params = new Params();
		params.setMin(0, 0);
		params.setBonus(1.0D, 1.0D);
		params.setPenalty(-1.0D, -1.0D);
		params.seq1 = s1.signature();
		params.seq2 = s2.signature();

		Report report = SequenceAlignment.align(params);
		List<Symbol[]> table = new ArrayList<Symbol[]>();

		int newLength = report.results1.size();
		for (int i = 0; i < s1._rows.size() + s2._rows.size(); i++) {
			table.add(new Symbol[newLength]);
		}

		int s1pos = 0;
		int s2pos = 0;
		for (int i = 0; i < report.results1.size(); i++) {
			WeightedObject left = report.results1.get(i);
			WeightedObject right = report.results2.get(i);
			if ((left != null) && (right != null)) {
				for (int j = 0; j < s1._rows.size(); j++) {
					Symbol[] oldRow = s1._rows.get(j);
					Symbol[] newRow = table.get(j);
					newRow[i] = oldRow[s1pos];
				}

				for (int j = 0; j < s2._rows.size(); j++) {
					Symbol[] oldRow = s2._rows.get(j);
					Symbol[] newRow = table.get(s1._rows.size() + j);
					newRow[i] = oldRow[s2pos];
				}

				s1pos++;
				s2pos++;
			} else if (right == null) {
				for (int j = 0; j < s1._rows.size(); j++) {
					Symbol[] oldRow = s1._rows.get(j);
					Symbol[] newRow = table.get(j);
					newRow[i] = oldRow[s1pos];
				}
				s1pos++;
			} else if (left == null) {
				for (int j = 0; j < s2._rows.size(); j++) {
					Symbol[] oldRow = s2._rows.get(j);
					Symbol[] newRow = table.get(s1._rows.size() + j);
					newRow[i] = oldRow[s2pos];
				}
				s2pos++;
			}
		}

		_rows = table;
		_count = this._rows.size();
		_signature = SequenceAlignment.combineAlignments(report.results1, report.results2);
	}

	/**
	 * Return the counts for some things...
	 * @return
	 */
	public Map<Symbol, Integer> getCounts() {
		Map<Symbol,Integer> map = new TreeMap<Symbol,Integer>();
		Set<Integer> columns = new HashSet<Integer>();
		for (Symbol[] row : _rows) {
			for (int i = 0; i < row.length; i++) {
				if (columns.contains(Integer.valueOf(i))) {
					continue;
				}
				if (row[i] == null) {
					continue;
				}
				columns.add(Integer.valueOf(i));

				Integer count = (Integer) map.get(row[i]);
				if (count == null) {
					count = Integer.valueOf(0);
				}
				count = Integer.valueOf(count.intValue() + 1);
				map.put(row[i], count);
			}
		}

		return map;
	}

	/**
	 * Write this signature to an XML file so that we don't constantly
	 * have to relearn this signature.
	 * @param file
	 */
	public void toXML(String file) {
		Document document = DocumentHelper.createDocument();
		Element root = document.addElement("Signature ")
				.addAttribute("key", _key)
				.addAttribute("count", _count+"");

		for (WeightedObject obj : this._signature) {
			obj.toXML(root);
		}

		Element table = root.addElement("Table")
				.addAttribute("rows", _rows.size()+"")
				.addAttribute("cols", _rows.get(0).length+"");

		for (int i = 0; i < this._rows.size(); i++) {
			Element rowElement = table.addElement("Row").addAttribute("id", i+"");

			Symbol[] row = (Symbol[]) this._rows.get(i);
			for (int j = 0; j < row.length; j++) {
				if (row[j] != null) {
					Element cellElement = rowElement.addElement("Cell")
							.addAttribute("id", j+"");
					row[j].toXML(cellElement);
				}
			}
		}

		try {
			OutputFormat format = OutputFormat.createPrettyPrint();
			XMLWriter writer = new XMLWriter(new FileWriter(file), format);
			writer.write(document);
			writer.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Prune the items in this signature and return a new signature
	 * that is composed of the frequently occurring items.
	 * @param min
	 * @return
	 */
	public Signature prune(int min) {
		Signature s = new Signature(_key);
		s._count = _count;

		Set<Integer> ignore = new HashSet<Integer>();
		List<WeightedObject> sequence = new ArrayList<WeightedObject>();
		for (int i = 0; i < _signature.size(); i++) {
			WeightedObject obj = _signature.get(i);
			if (obj.weight() > min)
				sequence.add(new WeightedObject(obj.key(), obj.weight()));
			else {
				ignore.add(Integer.valueOf(i));
			}
		}

		s._signature = sequence;
		s._rows = new ArrayList<Symbol[]>();

		int newSize = _rows.get(0).length - ignore.size();
		for (Symbol[] row : _rows) {
			int pos = 0;
			Symbol[] rowCopy = new Symbol[newSize];
			for (int i = 0; i < row.length; i++) {
				if (ignore.contains(Integer.valueOf(i)))
					continue;
				rowCopy[pos] = row[i];
				pos++;
			}
			s._rows.add(rowCopy);
		}
		return s;
	}

	/**
	 * 
	 * @param method
	 * @param instances
	 * @return
	 */
	public static Signature agglomerativeTraining(String method, List<Instance> instances) {
		double[][] matrix = Distances.distances(instances);

		List<AgglomerativeNode> nodes = new ArrayList<AgglomerativeNode>();
		for (int i = 0; i < instances.size(); i++) {
			nodes.add(new AgglomerativeNode(i, instances.get(i)));
		}

		while (nodes.size() > 1) {
			AgglomerativeNode minN1 = null;
			AgglomerativeNode minN2 = null;
			double min = Double.POSITIVE_INFINITY;

			for (int i = 0; i < nodes.size(); i++) {
				AgglomerativeNode n1 = nodes.get(i);
				for (int j = i + 1; j < nodes.size(); j++) {
					AgglomerativeNode n2 = nodes.get(j);
					double distance = n1.distance(n2, matrix, method);
					if (distance < min) {
						min = distance;
						minN1 = n1;
						minN2 = n2;
					}

				}

			}

			nodes.remove(minN1);
			nodes.remove(minN2);

			nodes.add(new AgglomerativeNode(minN1, minN2));
		}

		Signature s = nodes.get(0).signature;
		return s;
	}
	
	/**
	 * Reconstruct a signature from an XML file
	 * @param file
	 * @return
	 */
	public static Signature fromXML(String file) {
		Document document = null;
		try {
			SAXReader reader = new SAXReader();
			document = reader.read(file);
		} catch (Exception e) {
			e.printStackTrace();
		}
		Element root = document.getRootElement();

		String key = root.attributeValue("key");
		Signature s = new Signature(key);
		s._count = Integer.parseInt(root.attributeValue("count"));

		List list = root.elements("WeightedObject");
		for (Object o : list) { 
			Element woe = (Element) o;
			s._signature.add(WeightedObject.fromXML(woe));
		}

		Element table = root.element("Table");
		int nrows = Integer.parseInt(table.attributeValue("rows"));
		int ncols = Integer.parseInt(table.attributeValue("cols"));

		List rowList = table.elements("Row");
		if (rowList.size() != nrows) {
			throw new RuntimeException("Error in the number of rows: "
					+ rowList.size() + " " + nrows);
		}
		for (int i = 0; i < nrows; i++) {
			Element rowElement = (Element) rowList.get(i);
			Symbol[] row = new Symbol[ncols];

			List cellList = rowElement.elements("Cell");
			for (Object o : cellList) { 
				Element cell = (Element) o;
				int id = Integer.parseInt(cell.attributeValue("id"));
				row[id] = Symbol.fromXML(cell.element("Symbol"));
			}
			s._rows.add(row);
		}
		return s;
	}
}