import roslib; roslib.load_manifest('point_cloud_classifier')
from numpy import *
from sklearn import decomposition, cluster, neighbors
import matplotlib
import matplotlib.pyplot
import string
import pdb
import pickle

def		run():
#	pdb.set_trace();
	numComponents = 10;
	numCodeWords = 200;
	#data loading and normalization
	print 'loading data'
	data = loadtxt('/home/mohsen/Downloads/shotFiles/shot.txt');
	mean = data.mean(axis=0);
	std = sqrt(data.var(axis=0));
	std [where(std==0)] = 1;
	data = (data - mean ) / std;
	#dimension reduction
	print 'calculating pca'
	pca = decomposition.PCA(n_components=numComponents);
	pca.fit(data);
	projection = pca.transform(data);

#	f = open('pca.bin','wb');	
#	pickle.dump(f,pca);

	#codebook generation
	print 'fnding k-means'
#	kmeans = cluster.KMeans(k=numCodeWords);
#	kmeans.fit(projection);
#	codeBook = kmeans.cluster_centers_;
#	savetxt('codebook.txt',codeBook)
 	codeBook = loadtxt('codebook.txt');
	#building histogram for eahc object
	print 'building histograms'
	root = '/home/mohsen/Downloads/shotFiles/';
	f = open(root + 'listSingleSample.txt','r');
	data = zeros((1,shape(codeBook)[0]));
	target = [0];
	for line in f:
		shotFileName = root + line.replace('\n','');
		objectFeatures = loadtxt(shotFileName);
		objectLabel = labelFromFileName(shotFileName);
		if objectLabel==-1:
			print 'Error: Cannot determine label from file name!!!'
			return;
		objectFeatures = objectFeatures - mean;
		objectFeatures = objectFeatures / std;
		objectPca = pca.transform(objectFeatures);
		objectHist = zeros((1,shape(codeBook)[0]));
		for idx in range(shape(objectPca)[0]):
			dist = sum( (codeBook - objectPca[idx,:])**2 , axis=1 );
			objectHist[0,argmin(dist)] = objectHist[0,argmin(dist)] + 1;
		objectHist = objectHist / sum(objectHist);
		data = concatenate((data,objectHist),axis=0);
		target.append(objectLabel);
#	pdb.set_trace();	
#	dataTrain = concatenate((data,target),axis=1);
#	savetxt('dataTrain.txt',dataTrain);
	train = concatenate( (data[1::3,:],data[2::3,:]),axis=0 );
	test = data[3::3,:];
	traint = concatenate((target[1::3],target[2::3]));
	testt = target[3::3];
	print 'knn'
	knn1 = neighbors.NeighborsClassifier(1)
	knn1.fit(train, traint)
	out = knn1.predict(test);
	correct = sum(out==testt)/float(shape(out)[0])
	print "correct percent is:", correct
	printConfMat(testt,out);


def		printConfMat(target,output):
	tgtUnique = unique(target);
	outUnique = unique(output);
	if len(tgtUnique) > len(outUnique):
		tags = tgtUnique;
	else:
		tags = outUnique;
	numElements = len(tags);
	confMat = zeros((numElements,numElements));
	names = [];
	for i in range(shape(tgtUnique)[0]):
		names.append(nameFromLabel(tgtUnique[i]));
	for i in range(len(target)):
		col = where(tags==output[i]);
		row = where(tags==target[i])[0];
		confMat[row,col] = confMat[row,col] + 1;
	pdb.set_trace();
	for i in range(len(tags)):
		confMat[i,:] = confMat[i,:] / sum(confMat[i,:]);
	row='\t\t';
	for i in range(len(names)):
		row = row + names[i] + '\t'
	print row
	for i in range(shape(confMat)[0]):
		x = names[i] + '\t';
		for j in range(len(confMat[i,:])):
			x = x + '{0}\t\t'.format(confMat[i,j]);
		print x


def		labelFromFileName(fileName):
	lower = string.lower(fileName);
	if (string.find(lower,'1by2_chamfer_lego') > -1) or (string.find(lower,'1by4_lego') > -1) or (string.find(lower,'2by2_lego') > -1):
		return 0;
	elif (string.find(lower,'drill_box') > -1) or (string.find(lower,'foam_cube') > -1) or (string.find(lower,'irwin_box') > -1) or (string.find(lower,'power_supply_box') > -1):
		return 1;
	elif (string.find(lower,'elephant_toy') > -1) or (string.find(lower,'lion_toy') > -1) or (string.find(lower,'monkey_toy') > -1) or (string.find(lower,'piglet_toy') > -1):
		return 2;
	elif (string.find(lower,'foam_ball') > -1) or (string.find(lower,'spikey_ball') > -1) or (string.find(lower,'squeaky_ball') > -1)or(string.find(lower,'yellow_ball') > -1):
		return 3;
	elif (string.find(lower,'blue_cup') > -1) or (string.find(lower,'coffee_mug') > -1)or(string.find(lower,'green_cup') > -1):
		return 4;
	else:
		return -1;



def		nameFromLabel(label):
	if label==0:
		return 'Lego';
	elif label==1:
		return 'Box';
	elif label==2:
		return 'Toy';
	elif label==3:
		return 'Ball';
	elif label==4:
		return 'Cup';
	else:
		return '';
