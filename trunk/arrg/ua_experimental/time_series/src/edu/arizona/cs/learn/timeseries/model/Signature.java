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

import edu.arizona.cs.learn.algorithm.alignment.GeneralAlignment;
import edu.arizona.cs.learn.algorithm.alignment.Params;
import edu.arizona.cs.learn.algorithm.alignment.Report;
import edu.arizona.cs.learn.algorithm.alignment.Similarity;
import edu.arizona.cs.learn.timeseries.distance.Distances;
import edu.arizona.cs.learn.timeseries.model.symbols.Symbol;

public class Signature {
	private static Logger logger = Logger.getLogger(Signature.class);

	private String _key;
	private List<Symbol> _signature;
	private List<Symbol[]> _rows;

	private int _count;
	private Params _params;

	public Signature(String key) { 
		this(key, Similarity.strings);
	}
	
	public Signature(String key, Similarity similarity) {
		_key = key;
		_count = 0;
		_rows = new ArrayList<Symbol[]>();
		_signature = new ArrayList<Symbol>();

		_params = new Params();
		_params.setMin(0, 0);
		_params.setBonus(1.0D, 0.0D);
		_params.setPenalty(-1.0D, 0.0D);
		_params.similarity = similarity;
	}

	public String key() {
		return _key;
	}

	public List<Symbol> signature() {
		return _signature;
	}

	public int trainingSize() {
		return _count;
	}

	public List<Symbol[]> table() {
		return _rows;
	}

	public void update(List<Symbol> seq) {
		_count += 1;

		_params.seq1 = _signature;
		_params.seq2 = seq;
		
		Report report = GeneralAlignment.alignCheckp(_params);
		updateTable(report);

		_signature = GeneralAlignment.combineAlignments(report.results1, report.results2);
//		System.out.println("Signature...." + _signature);
	}

	private void updateTable(Report report) {
		int newLength = report.results1.size();
		List<Symbol[]> tmp = new ArrayList<Symbol[]>(_rows.size() + 1);
		for (int i = 0; i < this._rows.size() + 1; i++) {
			tmp.add(new Symbol[newLength]);
		}

		int position = 0;
		for (int i = 0; i < report.results1.size(); i++) {
			Symbol left = report.results1.get(i);
			Symbol right = report.results2.get(i);
			if ((left != null) && (right != null)) {
				for (int j = 0; j < this._rows.size(); j++) {
					Symbol[] oldRow = _rows.get(j);
					Symbol[] newRow = tmp.get(j);
					newRow[i] = oldRow[position];
				}

				tmp.get(_rows.size())[i] = right;

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

				tmp.get(_rows.size())[i] = right;
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
				double d = GeneralAlignment.distance(params);

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

				double d = GeneralAlignment.distance(params);
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

		Report report = GeneralAlignment.align(params);
		List<Symbol[]> table = new ArrayList<Symbol[]>();

		int newLength = report.results1.size();
		for (int i = 0; i < s1._rows.size() + s2._rows.size(); i++) {
			table.add(new Symbol[newLength]);
		}

		int s1pos = 0;
		int s2pos = 0;
		for (int i = 0; i < report.results1.size(); i++) {
			Symbol left = report.results1.get(i);
			Symbol right = report.results2.get(i);
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
		_signature = GeneralAlignment.combineAlignments(report.results1, report.results2);
	}

	/**
	 * Return the counts for some things...
	 * @return
	 */
	public Map<Symbol, Integer> getCounts() {
		if (true)
			throw new RuntimeException("Not yet coverted properly for Symbols!");
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
				.addAttribute("similarity", _params.similarity.toString())
				.addAttribute("count", _count+"");

		for (Symbol obj : _signature) {
			obj.toXML(root);
		}

		Element table = root.addElement("Table")
				.addAttribute("rows", _rows.size()+"")
				.addAttribute("cols", _rows.get(0).length+"");

		for (int i = 0; i < this._rows.size(); i++) {
			Element rowElement = table.addElement("Row").addAttribute("id", i+"");

			Symbol[] row = _rows.get(i);
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
		Signature s = new Signature(_key, _params.similarity);
		s._count = _count;

		Set<Integer> ignore = new HashSet<Integer>();
		List<Symbol> sequence = new ArrayList<Symbol>();
		for (int i = 0; i < _signature.size(); i++) {
			Symbol obj = _signature.get(i);
			if (obj.weight() > min)
				sequence.add(obj.copy());
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

		List<SignatureNode> nodes = new ArrayList<SignatureNode>();
		for (int i = 0; i < instances.size(); i++) {
			nodes.add(new SignatureNode(i, instances.get(i)));
		}

		while (nodes.size() > 1) {
			SignatureNode minN1 = null;
			SignatureNode minN2 = null;
			double min = Double.POSITIVE_INFINITY;

			for (int i = 0; i < nodes.size(); i++) {
				SignatureNode n1 = nodes.get(i);
				for (int j = i + 1; j < nodes.size(); j++) {
					SignatureNode n2 = nodes.get(j);
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

			nodes.add(new SignatureNode(minN1, minN2));
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
		Similarity similarity = Similarity.valueOf(root.attributeValue("similarity"));
		Signature s = new Signature(key, similarity);
		s._count = Integer.parseInt(root.attributeValue("count"));

		List list = root.elements("symbol");
		for (Object o : list) { 
			Element woe = (Element) o;
			s._signature.add(Symbol.fromXML(woe));
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
				row[id] = Symbol.fromXML(cell.element("symbol"));
			}
			s._rows.add(row);
		}
		return s;
	}
}

/**
 * This class allows us to perform Agglomerative clustering as we train
 * up the signature.
 * @author wkerr
 *
 */
class SignatureNode {
	private static Logger logger = Logger.getLogger(SignatureNode.class);

	public Signature signature;
	public List<Integer> sequenceIndexes;

	public SignatureNode(int index, Instance instance) { 
		sequenceIndexes = new ArrayList<Integer>();
		sequenceIndexes.add(index);
		
		signature = new Signature(instance.name());
		signature.update(instance.sequence());
	}

	/**
	 * This constructor is called when you want to combine
	 * two nodes into a single node
	 * @param node1
	 * @param node2
	 */
	public SignatureNode(SignatureNode node1, SignatureNode node2) { 
		logger.debug("Combining: ");
		logger.debug("  " + node1.sequenceIndexes);
		logger.debug("  " + node2.sequenceIndexes);
		
		sequenceIndexes = new ArrayList<Integer>();
		sequenceIndexes.addAll(node1.sequenceIndexes);
		sequenceIndexes.addAll(node2.sequenceIndexes);

		signature = node1.signature;
		signature.merge(node2.signature);
	}
	
	/**
	 * Call the specific method and return the distance between nodes
	 * @param n2
	 * @param matrix
	 * @param method
	 * @return
	 */
	public double distance(SignatureNode n2, double[][] matrix, String method) { 
		if ("single".equals(method))
			return singleLinkage(n2, matrix);
		if ("complete".equals(method))
			return completeLinkage(n2, matrix);
		if ("average".equals(method))
			return averageLinkage(n2, matrix);

		throw new RuntimeException("Unknown method: " + method);
	}

	/**
	 * Return the distance between the closest sequences 
	 * in the two nodes.
	 * @param n
	 * @param matrix
	 * @return
	 */
	public double singleLinkage(SignatureNode n2, double[][] matrix) { 
		double min = Double.POSITIVE_INFINITY;
		for (Integer i : sequenceIndexes) { 
			for (Integer j : n2.sequenceIndexes) { 
//				logger.debug("Distance: " + i + " " + j + " -- " + matrix[i][j]);
				min = Math.min(min, matrix[i][j]);
			}
		}
//		logger.debug("    MIN: " + min);
		return min;
	}

	/**
	 * Return the distance between the two farthest sequences
	 * in the two nodes
	 * @param n2
	 * @param matrix
	 * @return
	 */
	public double completeLinkage(SignatureNode n2, double[][] matrix) { 
		double max = Double.NEGATIVE_INFINITY;
		for (Integer i : sequenceIndexes) { 
			for (Integer j : n2.sequenceIndexes) { 
				max = Math.max(max, matrix[i][j]);
			}
		}
		return max;
	}

	/**
	 * Return the average distance between the cross product of
	 * all the sequences in each node
	 * @param n2
	 * @param matrix
	 * @return
	 */
	public double averageLinkage(SignatureNode n2, double[][] matrix) { 
		double sum = 0;
		double count = 0;
		for (Integer i : sequenceIndexes) { 
			for (Integer j : n2.sequenceIndexes) { 
				sum += matrix[i][j];
				count += 1;
			}
		}
		return sum / count;
	}	
}