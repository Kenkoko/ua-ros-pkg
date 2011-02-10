package edu.arizona.cs.learn.timeseries.clustering.kmeans;

public enum ClusterType {

	signature {
		@Override
		public Cluster make(int id) {
			return new SignatureCluster(id);
		}

		@Override
		public Cluster make(String name, int id){
			return new SignatureCluster(name, id);
		}
	},
	medoid {
		@Override
		public Cluster make(int id) {
			return new InstanceCluster(id);
		}

		@Override
		public Cluster make(String name, int id){
			return new InstanceCluster(name, id);
		}
	};
	
	public abstract Cluster make(int id);
	public abstract Cluster make(String name, int id);
}
