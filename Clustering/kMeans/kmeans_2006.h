// kmeans.h; wsn
// contains interface definitions: all class definitions
// small member functions are defined in-line
// larger member functions are defined in separate implementation
// files cluster_fncs.cpp, pattern_fncs.cpp and kmeans_fncs.cpp

#include <iostream>
#include <fstream>
#include <time.h> // used to seed "rand()"

using namespace std;

// use NDIM=5 for pixels
const int NDIM=5; // hard-coded dimension of feature vector;
                   // must change this and recompile for different clustering applications

// a pattern object includes data members:
// "cluster_affiliation" (associating this pattern with one of KCLUST clusters),
// a text label for the pattern,
// and an attribute, which is defined for the training set but is
// to be inferred from cluster membership for new patterns
// to which the pattern belongs.  (This is the whole point of clustering)
// The "print_pattern()" member function prints the pattern's 
//  data to the monitor.
// The "set_features()" member function accepts a vector of values
// (which must be of consistent dimensionality) and copies these
// values into the feature vector (for pattern initialization)
// ... more...






// a "cluster" is much like a "pattern"
// it has data members for ndims, a vector of feature values
// (scaled values only); an optional text name, a numerical ID,
// and a member fnc to print data
// additionally, it has a count of how many members are in the cluster,
// and a vector to encode which patterns comprise the members
// The cluster's other attribute follows from a "vote" of the members
// Member functions "add_member()" and "remove_member()" alter
// the cluster's feature values (the cluster centroid), change
// the member count, and change the roster of members (member_key)

// warning: need to set global variable "NUM_TRAINING_PATTERNS"
// before creating clusters.  (ugly)
// needed to allocate memory for membership roster;
// try doing without this?  have patterns be responsible for
// recording their membership
class Cluster
{
private:
	// avg_attribute is the average of "attribute" over all cluster members
	double avg_attribute; // property inferred by virtue of cluster membership
	int num_members; // number of members in this function
	//int cluster_ID;
	//bool member_key[NUM_TRAINING_PATTERNS]; // ugly--uses a global var
	double features[NDIM]; 	// vector of cluster archetype (centroid)
	char name[256];  // text name for this pattern (may not be needed)
 
public:
	Cluster(void); // constructor: do initializations
	void print_cluster(void); // prints info about this cluster
	// fnc to add a member to the cluster; provide pattern ID and feature vec
	void add_member(double *val_vec,double attribute); // val_vec shifts cluster centroid
	// fnc to remove a member from this cluster
	void remove_member(double *val_vec,double attribute); 
    void get_features(double *val_vec); // used to fill a vector w/ a copy of the cluster centroid
    double get_attribute(void) { return avg_attribute; };
	//bool is_member(int pat_id) { return member_key[pat_id];};
	double distance(double *val_vec); // return Eucl. distance from val_vec to cluster ctr
};

// class to hold a list of clusters...
class Clusters
{
private:
	int k_clusters;

public:
	Cluster *cluster_list; // an array of clusters;
	// default constructor: prompt for number of clusters
    Clusters(void)
	{
		cout<<"how many clusters do you want?:";
		cin>>k_clusters;
        cluster_list = new Cluster[k_clusters]; // dyn alloc memory for array of clusters
	}

	// optional constructor w/ arg for number of clusters
	Clusters(int kclusts) // constructor
	{
	  k_clusters = kclusts; 
      cluster_list = new Cluster[kclusts]; // dyn alloc memory for array of clusters
	}
	~Clusters(void) { delete [] cluster_list;}; // destructor

    int closest_cluster(double *val_vec); // return index of cluster closest to val_vec
    // install a member in a cluster; include operation of removing
	// pattern from previous cluster, if formerly categorized
	void install_member(int kclust,double *val_vec,double attribute);
    void remove_member(int kclust,double *val_vec,double attribute);
	int get_num_clusters(void) {return k_clusters;};
};

// class that holds both patterns and clusters:
// also holds matrix of membership of patterns in clusters
class Kmeans
{
private:
  // create a "Patterns" object; 
  // default constructor prompts for file name, parses file,
  // allocates appropriate memory, fills memory from file,
  // finds min and max features, and fills scaled feature vector
  //Patterns training_patterns;  // uses default constructor

  Patterns training_patterns;  // for named image file
  // create a clusters structure 
  // constructor will prompt for number of clusters to use
  Clusters clusters; 

  int num_patterns;
  int num_clusters;
  // the following matrix is used to cross-check pattern-cluster affiliations
  bool **is_mem_clust_pat; // create dimensions for this in constructor
  int *pattern_assignments; // assignment of each pattern to a cluster
public:
  Kmeans(void); // constructor--does LOTS of stuff;
  ~Kmeans(void)
  {
		for (int k=0;k<num_clusters;k++)
				delete [] is_mem_clust_pat[k];
		delete [] is_mem_clust_pat;
		delete [] pattern_assignments;

  }
  //int parse_file(char *fname,int ndims);
  void seed_clusters(void);
  int recluster(void); // returns number of patterns re-assigned
  double inferred_pattern_attribute(double *val_vec); // evaluates 
			// a feature vector, finds which cluster to which it is most similar,
			// and reports back the avg attribute value of that cluster
  // "distance" computes distance from "feature_vec" to cluster kclust
  double distance(double* feature_vec, int kclust); 
  void print_clusters(void);
  double eval_pattern(double *val_vec); // return avg_attribute of closest cluster
  bool ismember(int ipat,int kclust)
	{
      return is_mem_clust_pat[kclust][ipat];
	};
  void add_member(int ipat,int kclust); // add pattern ipat to cluster kclust
  void remove_member(int ipat,int kclust); // remove pattern ipat from cluster kclust
  void get_ranges(double *max_vals, double *min_vals)
  {
	  training_patterns.get_ranges(max_vals,min_vals);
  }; // consult training_patterns object for scaling data
  void create_cluster_image(void); //paint an output image w/ colors
                         //replaced w/ associated cluster colors
  void create_cluster_image(int kclust); //paint an output image
     // with ONLY the pixels belonging to the selected cluster turned on
  void save_clustered_points(void);       
};
