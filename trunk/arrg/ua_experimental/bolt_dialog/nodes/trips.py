#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import urllib
import urllib2
import tempfile
import webbrowser
import xml.dom.minidom

import rdflib
from rdflib.namespace import Namespace, RDF

# we need this to use SPARQL
rdflib.plugin.register('sparql', rdflib.query.Processor,
                       'rdfextras.sparql.processor', 'Processor')
rdflib.plugin.register('sparql', rdflib.query.Result,
                       'rdfextras.sparql.query', 'SPARQLQueryResult')

import pygraphviz as pgv

import roslib; roslib.load_manifest('json_prolog')
import rospy
import json_prolog



# some useful namespaces
LF = Namespace('http://www.cs.rochester.edu/research/trips/LF#')
ROLE = Namespace('http://www.cs.rochester.edu/research/trips/role#')
KNOWROB = Namespace('http://ias.cs.tum.edu/kb/knowrob.owl#')

NAMESPACES = {
    'rdf': RDF,
    'LF': LF,
    'role': ROLE,
    'knowrob': KNOWROB,
}



TRIPS_QUERY_URL = 'http://www.cs.rochester.edu/research/cisd/projects/trips/parser/cgi/web-parser-xml.cgi?input=%(input)s&treecontents=%(treecontents)s&treeformat=%(treeformat)s&lfformat=%(lfformat)s'



def expand_ns(s):
    s = str(s)
    for k,v in NAMESPACES.items():
        s = s.replace('%s:'%k, str(v))
    return s

def collapse_ns(s):
    s = str(s)
    for k,v in NAMESPACES.items():
        s = s.replace(str(v), '%s:'%k)
    return s

def query_trips(text):
    """Sends a sentence to TRIPS and returns the response as RDF."""
    url = TRIPS_QUERY_URL % {
        'input': urllib.quote_plus(text),
        'treecontents': 'full',
        'treeformat': 'lisp',
        'lfformat': 'lisp',
    }
    response = urllib2.urlopen(url)
    dom = xml.dom.minidom.parse(response)
    # I am using only the first RDF graph,
    # but there may be more than one.
    data = dom.getElementsByTagName('rdf:RDF')[0].toxml()
    result = rdflib.Graph().parse(data=data, format='application/rdf+xml')
    return result

def show_graph(text, rdfgraph):
    # build graph
    g = pgv.AGraph(strict=False, directed=True)
    g.graph_attr['label'] = '"%s"' % text
    for triple in rdfgraph:
        g.add_edge(collapse_ns(triple[0]), collapse_ns(triple[2]),
                   label=collapse_ns(triple[1]))

    # save graph as pdf file
    f = file(os.path.join(tempfile.gettempdir(), 'trips_dot.pdf'), 'w+b')
    g.layout(prog='dot')
    g.draw(path=f, format='pdf')
    f.close()

    # display graph
    webbrowser.open('file://%s' % f.name)

def interpret(rdfgraph):
    # what object are we talking about?
    objs = rdfgraph.query("""
        SELECT ?obj
        WHERE {
            ?speechact LF:indicator "SPEECHACT" .
            ?speechact LF:type "SA_REQUEST" .
            ?speechact role:CONTENT ?content .
            ?content LF:type "FIND" .
            ?content role:AGENT ?agent .
            ?agent role:PROFORM "*YOU*" .
            ?content role:THEME ?theme .
            ?theme LF:word ?obj .
        }""", initNs=NAMESPACES)

    objs = list(objs)
    if not objs:
        rospy.logerr('I can only handle find requests. Sorry.')
        return None

    retval = dict(theme=objs[0].capitalize(), mods=[])

    # do we know anything about the object? (color, size, etc.)
    for mod in rdfgraph.query("""
        SELECT ?scale ?type
        WHERE {
            ?theme LF:word ?word .
            ?theme role:MOD ?mod .
            ?mod role:SCALE ?scale .
            ?mod LF:type ?type .
        })""", initNs=NAMESPACES, initBindings={'?word': objs[0]}):

        retval['mods'] += mod

    return retval

def query_knowrob(query):
    prolog = json_prolog.Prolog()
    obj = str(KNOWROB[query['theme']])
    rospy.loginfo('Looking for %s in knowrob knowledge base' % obj)
    query = prolog.query("owl_has(A, '%s', '%s')." % (str(RDF['type']), obj))
    rospy.loginfo('Prolog query complete')
    
    was_empty = True
    for sol in query.solutions():
        was_empty = False
        rospy.loginfo('Found solution.  A = %s' % sol['A'])
        
    query.finish()
    
    if was_empty: rospy.logwarn('Could not find %s' % obj)


if __name__ == '__main__':
    rospy.init_node('test_trips')
    sentence = raw_input('>>> ').strip()
    rdf = query_trips(sentence)
    #show_graph(sentence, rdf)
    query = interpret(rdf)
    
    if query:
        rospy.loginfo('Querying knowrob now...')
        query_knowrob(query)

