#!/usr/bin/env python

import roslib; roslib.load_manifest('point_cloud_classifier')
import rospy
from point_cloud_classifier.srv import GetClusterLabels
from point_cloud_classifier.srv import GetClusterLabelsResponse
from point_cloud_classifier.srv import ExtractSHOTDescriptor
from sklearn import decomposition,neighbors
from numpy import *
import os


class ClusterClassification:
    def __init__(self):
        self.index_to_label = {
            0: 'Lego',
            1: 'Box',
            2: 'Toy',
            3: 'Ball',
            4: 'Cup',
        }
        
        # PCA components
        num_components = rospy.get_param('num_components', 10)
        
        # knn codewords
        num_code_words  = rospy.get_param('num_code_words', 200)
        
        # path to directory where all files are stored
        default_data_path = os.path.join(roslib.packages.get_pkg_dir('point_cloud_classifier'), 'data')
        data_path = rospy.get_param('data_path', default_data_path)
        
        rospy.loginfo('Loading codebook...')
        self.code_book = loadtxt(os.path.join(data_path, 'codebook.txt'))
        
        # PCA
        rospy.loginfo('Loading PCA data...')
        self.pca_mean  = loadtxt(os.path.join(data_path, 'pcamean.txt'))
        self.pca_comps = loadtxt(os.path.join(data_path, 'pcacomps.txt'))
        self.mean      = loadtxt(os.path.join(data_path, 'mean.txt'))
        self.std       = loadtxt(os.path.join(data_path, 'std.txt'))
        
#        data = loadtxt('shot.txt')
#        mean = data.mean(axis=0)
#        std = sqrt(data.var(axis=0))
#        std [where(std==0)] = 1
#        data = (data - mean ) / std
#        rospy.loginfo('Calculating PCA')
#        pca = decomposition.PCA(n_components=num_components)
#        pca.fit(data)
#        del data
#        self.mean = mean
#        self.std  = std
#        self.pca  = pca
        
        # KNN
        rospy.loginfo('Loading training data...')
        trainData = loadtxt(os.path.join(data_path, 'dataTrain.txt'))
        rospy.loginfo('Training KNN...')
        self.knn = neighbors.NeighborsClassifier(1)
        self.knn.fit(trainData[:,:-1], trainData[:,-1])
        del trainData
        
        rospy.wait_for_service('extract_shot_descriptor')
        self.extract_shot_features_srv = rospy.ServiceProxy('extract_shot_descriptor', ExtractSHOTDescriptor)
        rospy.loginfo('Classification node initialization done')
        
        rospy.Service('get_cluster_labels', GetClusterLabels, self.get_cluster_labels)

    def get_cluster_labels(self, req):
        rospy.loginfo('got %d clusters' % len(req.clusters))
        res = GetClusterLabelsResponse()
        clusters = req.clusters
        
        for idx,point_cloud in enumerate(clusters):
            try:
                label = self.classify(point_cloud)
                res.labels.append(label)
            except Exception as e:
                label = 'Unknown'
                rospy.logerr('classification failed: %s' % e)
                
            rospy.loginfo('labeled object %d with %s category' % (idx, label))
            
        return res

    def classify(self, point_cloud):
        res = self.extract_shot_features_srv(point_cloud)
        num_features = res.num_features
        feature_size = res.feature_size
        shot_features = res.shot_feature
        rospy.loginfo('successfully extarcted shot features')
        
        shot_features = reshape(shot_features, (num_features, feature_size))
        shot_features = shot_features - self.mean
        shot_features = shot_features / self.std
        
        #pca transform
        temp = shot_features - self.pca_mean
        objectPca = dot(temp, self.pca_comps.T)
#        objectPca = self.pca.transform(shot_features)
        objectHist = zeros((self.code_book.shape[0]))
        
        for i in range(objectPca.shape[0]):
            dist = sum( (self.code_book - objectPca[i,:])**2 , axis=1 )
            objectHist[argmin(dist)] = objectHist[argmin(dist)] + 1
            
        objectHist = objectHist / objectHist.sum()
        idx = self.knn.predict(objectHist)[0]
        
        if idx in self.index_to_label: return self.index_to_label[idx]
        else: return 'Unknown'


if __name__ == '__main__':
    rospy.init_node('point_cloud_classifier')
    pclClf = ClusterClassification()
    
    rospy.spin()

