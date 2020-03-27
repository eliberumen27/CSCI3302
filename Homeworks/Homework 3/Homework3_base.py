# CSCI 3302: Homework 3 -- Clustering and Classification
# Implementations of K-Means clustering and K-Nearest Neighbor classification
import pickle
import random
import copy
import pdb
import matplotlib.pyplot as plt
from hw3_data import *

# TODO: INSERT YOUR NAME HERE
LAST_NAME = "Berumen"


def visualize_data(data, cluster_centers_file):
  fig = plt.figure(1, figsize=(4,3))
  f = open(cluster_centers_file, 'rb')
  centers = pickle.load(f)
  f.close()

  km = KMeansClassifier()
  km._cluster_centers = centers

  colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

  labels = []
  center_colors = []
  for pt in data:
    labels.append(colors[km.classify(pt) % len(colors)])

  for i in range(len(centers)):
    center_colors.append(colors[i])

  plt.scatter([d[0] for d in data], [d[1] for d in data], c=labels, marker='x')
  plt.scatter([c[0] for c in centers], [c[1] for c in centers], c=center_colors, marker='D')
  plt.title("K-Means Visualization")
  plt.show()


class KMeansClassifier(object):

  def __init__(self):
    self._cluster_centers = [] # List of cluster centers, each of which is a point. ex: [ [10,10], [2,1], [0,-3] ]
    self._data = [] # List of datapoints (list of immutable lists, ex:  [ (0,0), (1.,5.), (2., 3.) ] )

  def add_datapoint(self, datapoint):
    self._data.append(datapoint)

  def fit(self, k):
    # Fit k clusters to the data, by starting with k randomly selected cluster centers.
    self._cluster_centers = [] # Reset cluster centers array

    # TODO: Initialize k cluster centers at random points
    # HINT: To choose reasonable initial cluster centers, you can set them to be in the same spot as random (different) points from the dataset
    for i in range(k):
        self._cluster_centers.append(random.choice(self._data))
    # TODO Follow convergence procedure to find final locations for each center
    while True:
      # TODO: Iterate through each datapoint in self._data and figure out which cluster it belongs to
      # HINT: Use self.classify(p) for each datapoint p

      # for each datapoint see what index of cluster centers array it's closest to
      pInd = [] # stores the index of the center each p is closest to and p itself
      for p in self._data:
          pInd.append((self.classify(p), p))
      #print(pInd) # this is a test
      # TODO: Figure out new positions for each cluster center (should be the average position of all its points)
      avg = avg1 = avg2 = avg3 = [] # each list contains tuples of points that need their positions averaged
      for item in pInd:
          if item[0] == 0:
              avg.append((item[1][0], item[1][1]))
          elif item[0] == 1:
              avg1.append((item[1][0], item[1][1]))
          elif item[0] == 2:
              avg2.append((item[1][0], item[1][1]))
          elif item[0] == 3:
              avg3.append((item[1][0], item[1][1]))

      # Now we get cluster 0 average
      newPos = newPos1 = newPos2 = newPos3 = (0, 0)
      runValx = 0 # running avg value x coord
      runValy = 0 # y coord
      for i in avg:
          runValx = runValx + i[0]
          runValy = runValy + i[1]
      newPos = runValx/len(avg),  runValy/len(avg)

      # Now we get cluster 1 average
      runValx = 0 # running avg value x coord
      runValy = 0 # y coord
      for i in avg1:
          runValx = runValx + i[0]
          runValy = runValy + i[1]
      newPos1 = runValx/len(avg1), runValy/len(avg1)

      # Now we get cluster 2 average
      #newPos = newPos1 = newPos2 = newPos3 = (0, 0)
      runValx = 0 # running avg value x coord
      runValy = 0 # y coord
      for i in avg2:
          runValx = runValx + i[0]
          runValy = runValy + i[1]
      newPos2 = runValx/len(avg2), runValy/len(avg2)

      # Now we get cluster 3 average
      #newPos = newPos1 = newPos2 = newPos3 = (0, 0)
      runValx = 0 # running avg value x coord
      runValy = 0 # y coord
      for i in avg3:
          runValx = runValx + i[0]
          runValy = runValy + i[1]
      newPos3 = runValx/len(avg3), runValy/len(avg3)

      # TODO: Check to see how much the cluster centers have moved (for the stopping condition)
      diffx = abs(newPos[0] - self._cluster_centers[0][0])
      diffy = abs(newPos[1] - self._cluster_centers[0][1])
      #print(diffx, diffy)

      diffx1 = abs(newPos1[0] - self._cluster_centers[1][0])
      diffy1 = abs(newPos1[1] - self._cluster_centers[1][1])
      #print(diffx1, diffy1)

      diffx2 = abs(newPos2[0] - self._cluster_centers[2][0])
      diffy2 = abs(newPos2[1] - self._cluster_centers[2][1])
      #print(diffx2, diffy2)

      diffx3 = abs(newPos3[0] - self._cluster_centers[3][0])
      diffy3 = abs(newPos3[1] - self._cluster_centers[3][1])
      #print(diffx3, diffy3)
      # optimize later to put these variables in a list

      if all([diffx, diffx1, diffx2, diffx3 < 10]): # TODO: If the centers have moved less than some predefined threshold (you choose!) then exit the loop
        break

    # TODO Add each of the 'k' final cluster_centers to the model (self._cluster_centers)
    #self._cluster_centers = []
    #self._cluster_centers.append(newPos)
    #self._cluster_centers.append(newPos1)
    #self._cluster_centers.append(newPos2)
    #self._cluster_centers.append(newPos3)

  def classify(self,p):
    # Given a data point p, figure out which cluster it belongs to and return that cluster's ID (its index in self._cluster_centers)
    closest_cluster_index = 0

    # TODO Find nearest cluster center, then return its index in self._cluster_centers
    # optimize this later
    dist = 1000
    for i in self._cluster_centers:
        ind = self._cluster_centers.index(i)
        if (i[0] - p[0])**2 + (i[1] - p[1]) < dist:
            dist = (i[0] - p[0])**2 + (i[1] - p[1])
            ind = ind = self._cluster_centers.index(i)

    closest_cluster_index = ind

    return closest_cluster_index

class KNNClassifier(object):

  def __init__(self):
    self._data = [] # list of (datapoint, label) tuples

  def clear_data(self):
    # Removes all data stored within the model
    self._data = []

  def add_labeled_datapoint(self, data_point, label):
    # Adds a labeled datapoint tuple onto the object's _data member
    self._data.append((data_point, label))

  def classify_datapoint(self, data_point, k):
    label_counts = {} # Dictionary mapping "label" => vote count
    best_label = None

    # Perform k_nearest_neighbor classification, setting best_label to the majority-vote label for k-nearest points
    #TODO: Find the k nearest points in self._data to data_point
    #TODO: Populate label_counts with the number of votes each label got from the k nearest points
    #TODO: Make sure to scale the weight of the vote each point gets by how far away it is from data_point
    #      Since you're just taking the max at the end of the algorithm, these do not need to be normalized in any way


    return best_label



def print_and_save_cluster_centers(classifier, filename):
  for idx, center in enumerate(classifier._cluster_centers):
    print("  Cluster %d, center at: %s" % (idx, str(center)))


  f = open(filename,'wb')
  pickle.dump(classifier._cluster_centers, f)
  f.close()

def read_data_file(filename):
  f = open(filename)
  data_dict = pickle.load(f)
  f.close()

  return data_dict['data'], data_dict['labels']

def read_hw_data():
  global hw_data
  data_dict = pickle.loads(hw_data)
  return data_dict['data'], data_dict['labels']

def main():
  global LAST_NAME
  # read data file
  #data, labels = read_data_file('hw3_data.pkl')

  # load dataset
  data, labels = read_hw_data()

  # data is an 'N' x 'M' matrix, where N=number of examples and M=number of dimensions per example
  # data[0] retrieves the 0th example, a list with 'M' elements, one for each dimension (xy-points would have M=2)
  # labels is an 'N'-element list, where labels[0] is the label for the datapoint at data[0]


  ########## PART 1 ############
  # perform K-means clustering
  kMeans_classifier = KMeansClassifier()
  for datapoint in data:
    kMeans_classifier.add_datapoint(datapoint) # add data to the model

  kMeans_classifier.fit(4) # Fit 4 clusters to the data

  # plot results
  print('\n'*2)
  print("K-means Classifier Test")
  print('-'*40)
  print("Cluster center locations:")
  print_and_save_cluster_centers(kMeans_classifier, "hw3_kmeans_" + LAST_NAME + ".pkl")

  print('\n'*2)


  ########## PART 2 ############
  print("K-Nearest Neighbor Classifier Test")
  print('-'*40)

  # Create and test K-nearest neighbor classifier
  kNN_classifier = KNNClassifier()
  k = 2

  correct_classifications = 0
  # Perform leave-one-out cross validation (LOOCV) to evaluate KNN performance
  for holdout_idx in range(len(data)):
    # Reset classifier
    kNN_classifier.clear_data()

    for idx in range(len(data)):
      if idx == holdout_idx: continue # Skip held-out data point being classified

      # Add (data point, label) tuples to KNNClassifier
      kNN_classifier.add_labeled_datapoint(data[idx], labels[idx])

    guess = kNN_classifier.classify_datapoint(data[holdout_idx], k) # Perform kNN classification
    if guess == labels[holdout_idx]:
      correct_classifications += 1.0

  print("kNN classifier for k=%d" % k)
  print("Accuracy: %g" % (correct_classifications / len(data)))
  print('\n'*2)

  visualize_data(data, 'hw3_kmeans_' + LAST_NAME + '.pkl')


if __name__ == '__main__':
  main()
