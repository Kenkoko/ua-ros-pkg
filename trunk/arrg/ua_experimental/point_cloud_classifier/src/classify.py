#!/usr/bin/env python

import roslib; roslib.load_manifest('point_cloud_classifier')
import rospy
from std_msgs.msg import String
from point_cloud_classifier.srv import PclClassifier
from point_cloud_classifier.srv import PclClassifierResponse
from point_cloud_classifier.srv import extractShot
from sklearn import decomposition,neighbors
import numpy as np




class	PclClassifier:
	def __init__(self):
		#init
		nComponents = 10;#PCA components
		nCodeWords  = 200;#knn codewords			
		self.codeBook = np.loadtxt('codebook.txt');
		#PCA
		data = np.loadtxt('shotFeatures.txt');
		mean = data.mean(axis=0);
		std = sqrt(data.var(axis=0));
		std [where(std==0)] = 1;
		data = (data - mean ) / std;
		pca = decomposition.PCA(n_components=10);
		pca.fit(data);
		self.mean = mean;
		self.std  = std;
		self.pca  = pca;
		#KNN
		trainData = loadtxt('trainData.txt');
		knn = neighbors.NeighborsClassifier(1);
		knn.fit(trainData[:,:-1], trainData[:,-1]);
		self.knn = knn;
		#services
		s = rospy.Service('PclClassifier', PclClassifier, self.ServiceHandler);
		rospy.wait_for_service('extractShot')
		self.shotExtractor = rospy.ServiceProxy('extractShot', extractShot)
		
	def ServiceHandler(self,req):
		res = PclClassifierResponse()
		clusters = req.clusters
		for c in clusters:
			shotFeatures = self.ExtractShotFeatures(c);
			label = self.Classify(shotFeatures);
			res.labels.append(label)
		return res;

	def Classify(self,shotFeatures):
		shotFeatures = np.reshape(shotFeatures,(shotFeatures.num_features,shotFeatures.feature_size));
		shotFeatures = shotFeatures - self.mean;
		shotFeatures = shotFeatures / self.std;
		objectPca = self.pca.transform(shotFeatures);
		objectHist = np.zeros((self.codeBook.shape[0]));
		for i in range(objectPca.shape[0]):
			dist = np.sum( (self.codeBook - objectPca[i,:])**2 , axis=1 );
			objectHist[np.argmin(dist)] = objectHist[np.argmin(dist)] + 1;
		objectHist = objectHist / objectHist.sum();
		idx = self.knn.predict(objectHist);
		return self.IdxToLabel(idx);

	def IdxToLabel(self,idx):
		if idx==0:
			return 'Lego';
		elif idx==1:
			return 'Box';
		elif idx==2:
			return 'Toy';
		elif idx==3:
			return 'Ball';
		elif idx==4:
			return 'Cup';
		else:
			return '';
	
	def ExtractShotFeatures(self,point_cloud):
		try:	
			resp1 = self.shotExtractor(point_cloud)
			return resp1.shot_feature
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e



#def handler(msg):
#	print msg.data

if __name__ == '__main__':
	rospy.init_node('point_cloud_classifier');
	pclClf = PclClassifier();
	
	rospy.spin();

#	rospy.init_node('point_cloud_classifier')
#	pub = rospy.Publisher('topic1', ClassifiactionResult)
#	msg = ClassifiactionResult()
#	msg.obejct_id = 2
#	msg.object_name = 'df'
#	pub.publish(msg)
#	rospy.Subscriber('topic2', String, handler)
#
#	
#	srv_client = rospy.ServiceClient('srv_name', DoClassify)
#	res = srv_slient('dd')
#
#	r = rospy.Rate(10)
#	while not rospy.is_shutdown():
#		res = srv_slient('dd')
#		# do something
#		r.sleep() 
#	rospy.spin()
