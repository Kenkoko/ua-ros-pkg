package edu.arizona.cs.learn.experimental.general;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import org.dom4j.Document;
import org.dom4j.DocumentHelper;
import org.dom4j.Element;
import org.dom4j.io.OutputFormat;
import org.dom4j.io.SAXReader;
import org.dom4j.io.XMLWriter;

public class XMLUtils {

	/**
	 * Load in the data file that is represented as XML for
	 * the time being.
	 * @param fileName
	 * @return
	 */
	public static Pair<String,List<List<Symbol>>> loadXML(String fileName) { 
		return loadXML(new File(fileName));
	}
	
	/**
	 * Load in the data file that is represented as XML for
	 * the time being.
	 * @param file
	 * @return
	 */
	public static Pair<String,List<List<Symbol>>> loadXML(File file) { 
		Document document = null;
		try {
			SAXReader reader = new SAXReader();
			document = reader.read(file);
		} catch (Exception e) {
			e.printStackTrace();
		}
		Element root = document.getRootElement();

		List<List<Symbol>> results = new ArrayList<List<Symbol>>();
		String key = root.attributeValue("key");
		
		List<?> list = root.elements("instance");
		for (int i = 0; i < list.size(); ++i) { 
			Element io = (Element) list.get(i);

			List<Symbol> symbols = new ArrayList<Symbol>();
			List<?> sList = io.elements("symbol");
			for (int j = 0; j < sList.size(); ++j) { 
				Element so = (Element) sList.get(j);
				symbols.add(Symbol.fromXML(so));
			}
			
			results.add(symbols);
		}
		return new Pair<String,List<List<Symbol>>>(key, results);
	}
	
	
	/**
	 * This method takes an instance... represented as a List of 
	 * Symbols and adds an XML instance to the given element
	 * @param root
	 * @param instance
	 * @return
	 */
	public static void toXML(Element root, List<Symbol> instance) {
		Element e = root.addElement("instance");
		for (Symbol s : instance) { 
			s.toXML(e);
		}
	}
	
	/**
	 * Convert the list of Integers into a comma separated
	 * string so that it's easy to load back in.
	 * @param list
	 * @return
	 */
	public static String toString(List<?> list) {
		StringBuffer buf = new StringBuffer();
		for (Object o : list) 
			buf.append(o + ",");
		buf.deleteCharAt(buf.length()-1);
		return buf.toString();
	}
	
	/**
	 * Convert the matrix of doubles into an XML Element
	 * appended to the given element
	 * @param matrix
	 * @return
	 */
	public static void toXML(Element e, double[][] matrix) { 
		Element mElement = e.addElement("matrix")
			.addAttribute("rows", matrix.length+"")
			.addAttribute("cols", matrix[0].length+"");

		for (int i = 0; i < matrix.length; ++i) { 
			StringBuffer buf = new StringBuffer();
			for (int j = 0; j < matrix[i].length; ++j) { 
				buf.append(matrix[i][j] + ",");
			}
			buf.deleteCharAt(buf.length()-1);
			
			mElement.addElement("row")
				.addAttribute("index", i+"")
				.addAttribute("value", buf.toString());
		}
	}
}
