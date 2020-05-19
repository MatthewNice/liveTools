#This has been transferred into liveRadar.py

# from sklearn.cluster import AgglomerativeClustering
#
# def clusterRadar(radar_batch):
#     cluster = AgglomerativeClustering(n_clusters=None, distance_threshold = 1, affinity='euclidean', linkage='ward')
#     # Dx2 observations in the radar_batch, returns a cluster label for each point
#     return cluster.fit_predict(radar_batch)

# def clusterCenter(radar_batch,labels):
#     #find the median value for each cluster
#     a=[]
#     for i in range((max(labels)+1)):
#         a.append([])
#     for i in range(0,len(labels)):
#         a[labels[i]].append(radar_batch[i])
