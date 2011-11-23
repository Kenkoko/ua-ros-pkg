#!/usr/bin/env python

import roslib; roslib.load_manifest('point_cloud_classifier')
import rospy
from point_cloud_classifier.srv import GetClusterLabels
from point_cloud_classifier.srv import GetClusterLabelsResponse
from point_cloud_classifier.srv import ExtractSHOTDescriptor
from sklearn import decomposition,neighbors
from numpy import *


class ClusterClassification:
    def __init__(self):
        #init
        nComponents = 10#PCA components
        nCodeWords  = 200#knn codewords
        rospy.loginfo('Loading codebook...')
        self.codeBook = loadtxt('codebook.txt')
        #PCA
        rospy.loginfo('Loading PCA data...')
	self.pcaMean  = loadtxt('pcamean.txt');
	self.pcaComps = loadtxt('pcacomps.txt');
	self.mean     = loadtxt('mean.txt');
	self.std      = loadtxt('std.txt');
	
#        data = loadtxt('shot.txt')
#        mean = data.mean(axis=0)
#        std = sqrt(data.var(axis=0))
#        std [where(std==0)] = 1
#        data = (data - mean ) / std
#        rospy.loginfo('Calculating PCA')
#        pca = decomposition.PCA(n_components=10)
#        pca.fit(data)
#        del data
#        self.mean = mean
#        self.std  = std
#        self.pca  = pca
        #KNN
        rospy.loginfo('Loading training data...')
        trainData = loadtxt('dataTrain.txt')
        rospy.loginfo('Training KNN...')
        knn = neighbors.NeighborsClassifier(1)
        knn.fit(trainData[:,:-1], trainData[:,-1])
        self.knn = knn
        del trainData
        
        #services
        self.s = rospy.Service('get_cluster_labels', GetClusterLabels, self.get_cluster_labels)
        
        rospy.wait_for_service('extract_shot_descriptor')
        self.shotExtractor = rospy.ServiceProxy('extract_shot_descriptor', ExtractSHOTDescriptor)
        rospy.loginfo('Classification node initialization done')
        
    def get_cluster_labels(self,req):
        rospy.loginfo('got %d clusters' % len(req.clusters))
        res = GetClusterLabelsResponse()
        clusters = req.clusters
        for idx,c in enumerate(clusters):
            num_features, feature_size, shotFeatures = self.ExtractSHOTDescriptorFeatures(c)
            label = self.Classify(num_features, feature_size, shotFeatures)
            res.labels.append(label)
            rospy.loginfo('labeled object %d with %s category' % (idx, label))
        return res

    def Classify(self, num_features, feature_size, shotFeatures):
        shotFeatures = reshape(shotFeatures,(num_features, feature_size))
        shotFeatures = shotFeatures - self.mean
        shotFeatures = shotFeatures / self.std
	#pca transform
        temp = shotFeatures - self.pcaMean
        objectPca = dot(temp, self.pcaComps.T)
#        objectPca = self.pca.transform(shotFeatures)
        objectHist = zeros((self.codeBook.shape[0]))
        for i in range(objectPca.shape[0]):
            dist = sum( (self.codeBook - objectPca[i,:])**2 , axis=1 )
            objectHist[argmin(dist)] = objectHist[argmin(dist)] + 1
        objectHist = objectHist / objectHist.sum()
        idx = self.knn.predict(objectHist)
        return self.IdxToLabel(idx)

    def IdxToLabel(self,idx):
        if idx==0:
            return 'Lego'
        elif idx==1:
            return 'Box'
        elif idx==2:
            return 'Toy'
        elif idx==3:
            return 'Ball'
        elif idx==4:
            return 'Cup'
        else:
            return 'Unknown'
    
    def ExtractSHOTDescriptorFeatures(self,point_cloud):
        try:
            resp1 = self.shotExtractor(point_cloud)
            return resp1.num_features, resp1.feature_size, resp1.shot_feature
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('point_cloud_classifier')
    pclClf = ClusterClassification()
    
    rospy.spin()

