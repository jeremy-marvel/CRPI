// cluster_fncs_v3.cpp

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include "kmeans_2006.h"

// default constructor: assumes NDIM and NUM_TRAINING_PATTERNS
Cluster::Cluster(void)
{
  avg_attribute=0;
  name[0]='\0';
  num_members=0;
  for (int i=0;i<NDIM;i++)
	  features[i]=0.0;
}



void Cluster::add_member(double *val_vec, double attribute)
{
   // include influence of new pattern on cluster centroid, appropriately weighted
   for (int i=0; i<NDIM;i++)
	   features[i] = 
	      (features[i]*num_members+val_vec[i])/((double) (num_members+1));
   avg_attribute = (avg_attribute*num_members+attribute)/((double) (num_members+1));
   num_members++; // note there is a new member

}
	// fnc to remove a member from this cluster
void Cluster::remove_member(double *val_vec,double attribute)
{
     for (int i=0; i<NDIM;i++)
	   features[i] = 
	      (features[i]*num_members-val_vec[i])/((double) (num_members-1));
	 avg_attribute = 
		 (num_members*avg_attribute-attribute)/((double) (num_members-1));
   
   num_members--; // decrease head count
}

// used to fill a vector w/ a copy of the cluster features
void Cluster::get_features(double *val_vec)
{
		for (int i=0;i<NDIM;i++)
			val_vec[i]=features[i];	
}

void Cluster::print_cluster(void)
	{
	    int i;
		cout<<"   number of members = "<<num_members<<endl;
		cout<<"   avg_attribute = "<<avg_attribute<<endl;
		cout<<"   feature values: "<<endl;
		cout<<"     ";
		for (i=0;i<NDIM/2;i++)
			cout<<features[i]<<"  ";
		cout<<endl;
		cout<<"     ";
		for (i=NDIM/2;i<NDIM;i++)
			cout<<features[i]<<"  ";
		cout<<endl<<endl;
	};

// compute Euclidean distance from val_vec to cluster center
double Cluster::distance(double *val_vec)
{
  double dist=0;
  for (int i=0;i<NDIM;i++)
	  dist+= (val_vec[i]-features[i])*(val_vec[i]-features[i]);
  dist = sqrt(dist);
  return dist;
}


// these are unnecessary: remove them
void Clusters::install_member(int kclust,double *val_vec,double attribute)
{
  // factor in the feature values in the selected cluster object:
    cluster_list[kclust].add_member(val_vec,attribute);
}

void Clusters::remove_member(int kclust,double *val_vec,double attribute)
{
  // factor in the feature values in the selected cluster object:
    cluster_list[kclust].remove_member(val_vec,attribute);
}

int Clusters::closest_cluster(double *val_vec) // return index of cluster closest to val_vec
{
    double min_dist;
	double test_dist;
	int k_closest=0;
	min_dist = cluster_list[0].distance(val_vec);
	for (int iclust=1;iclust<k_clusters;iclust++)
	{
      test_dist = cluster_list[iclust].distance(val_vec);
	  if (test_dist<min_dist)
	  {
         min_dist = test_dist;
		 k_closest = iclust;
	  }
	}
	return k_closest;
}

