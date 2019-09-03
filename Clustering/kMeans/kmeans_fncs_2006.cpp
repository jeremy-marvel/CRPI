// kmeans_fncs_v3.cpp: member function implementations for class Kmeans

#include <iostream>
#include <fstream>
#include <string>
#include "kmeans_2006.h"
#include "ImageFunc.h"
int ans=0;

Kmeans::Kmeans(void) // default constructor
{
	// Kmeans struct data member num_patterns filled by querying training_patterns
    num_patterns = training_patterns.get_npatterns();
	// Kmeans struct data member num_clusters filled by querying clusters object
	num_clusters = clusters.get_num_clusters();

	// create vector for pattern assignments to clusters:
    pattern_assignments=new int[num_patterns];
	for (int k=0;k<num_patterns;k++)
		pattern_assignments[k]=-1; // init to "unassigned" cluster number

      // create a membership array, kclusts x npats;
	 // can be used for cross-checking which patterns belong to a cluster
      is_mem_clust_pat = new bool *[num_clusters];
	  for (int i=0;i<num_clusters;i++)
		  is_mem_clust_pat[i] = new bool[num_patterns];
	  for (int j=0;j<num_clusters;j++)
		  for (int k=0;k<num_patterns;k++)
			  is_mem_clust_pat[j][k]=false; // clear the membership
}


void Kmeans::seed_clusters(void)
{
	// seed the clusters:
	// pick random patterns and assign them as initial
	// cluster centers:
	int pnum;
	srand( (unsigned)time( NULL ) ); // seed rand w/ time
	int kclust=0;
	while(kclust<num_clusters)
	{
	  int kclust_membership;
	  double val_vec[NDIM];
	  double pat_attrib;
	  // pick a random pattern number:
	  pnum = (rand()*num_patterns/(RAND_MAX+1));
	  //cout<<"selected pattern number "<<pnum<<endl;
	  // see if this number was already used
      kclust_membership = pattern_assignments[pnum];

	  if (kclust_membership<0)
	  {
		cout<<"selected pattern number "<<pnum<<endl;
		// if here, this pattern has not yet been classified
		// increment the cluster  counter and install this
		// pattern as the first member in the new cluster:
		// update the pattern_assignments vector and 
		// pattern-cluster map
		pattern_assignments[pnum]=kclust;
        is_mem_clust_pat[kclust][pnum]=true; 
		// get a copy of the features for this pattern:
		training_patterns.pattern_list[pnum].get_scaled_features(val_vec);

		// get the attribute of this training pattern:
		//pat_attrib = training_patterns->pattern_list[pnum].get_attribute();
		pat_attrib = training_patterns.pattern_list[pnum].get_attribute();

		// install affiliation in Clusters array: 
		clusters.install_member(kclust,val_vec,pat_attrib);

		// debug:
		cout<<"installed pattern "<<pnum<<" in cluster "<<kclust<<endl;
		cout<<"cluster data: "<<endl;
		clusters.cluster_list[kclust].print_cluster();
        kclust++; // note that a new cluster has been created (termination condition)

	  }
	} 
}

int Kmeans::recluster(void)
{
  // step through all patterns and see if membership should change
  int cur_clust;
  int new_clust;
  int num_reassignments=0;
  double val_vec[NDIM];
  cout<<"reclustering: "<<endl;
  for (int ipat=0;ipat<num_patterns;ipat++)
  {
	  //cout<<" checking pattern "<<ipat<<endl;
	  //cout<<" current cluster is ";
	  //cur_clust = training_patterns->pattern_list[ipat].get_cluster_affiliation();
	  //cur_clust = training_patterns.pattern_list[ipat].get_cluster_affiliation();
	  cur_clust = pattern_assignments[ipat];

	  // cout<<cur_clust<<endl;
	  //training_patterns->pattern_list[ipat].get_scaled_features(val_vec);
	  training_patterns.pattern_list[ipat].get_scaled_features(val_vec);
	  new_clust = clusters.closest_cluster(val_vec);
	  //cout<<"closest cluster is "<<new_clust<<endl;

	  if (cur_clust !=new_clust)
	  {
		// found a closer cluster; do reassignment of this pattern
		cout<<"reassigning pattern "<<ipat<<" from cluster ";
		cout<<cur_clust<<" to cluster "<<new_clust<<endl;
		num_reassignments++;
		remove_member(ipat,cur_clust);
		add_member(ipat,new_clust);
	  }
  }
 return num_reassignments;
}

void Kmeans::print_clusters(void)
{
 int num_formers=0;
 int i;
 double attribute;
 for (i=0;i<num_clusters;i++)
 {
	 cout<<"Cluster "<<i<<": "<<endl;
	 clusters.cluster_list[i].print_cluster();
 }
  for (i=0;i<num_clusters;i++)
  {  
	  attribute = clusters.cluster_list[i].get_attribute();
	  cout<<"cluster "<<i<<"; attribute = "<<attribute<<endl;
     if (attribute>0.5)
		 num_formers++;
 }
  cout<<num_formers<<" of "<<num_clusters<<" clusters are formers"<<endl;
}

void Kmeans::create_cluster_image(void)
{
	int Nrows,Ncols;
	int row,col;
	int ipix,kclust;
	int icolor;
	int BGRvec[3];
	double dBGRvec[3];
	double feature_vec[NDIM];
	double max_vals[NDIM];
	double min_vals[NDIM];
	double val_vec[NDIM];
	ImageFunc	imageObj;
	char fname[]="input_image.jpg";
	if(!imageObj.DecodeJPGFileToGeneralBufferColor(fname))
		{
			char dummy;
			cout<<"could not load picture "<<fname<<endl;
			cout<<"Die? (y/y):  "; 	cin>>dummy;
			exit(0);
		}
	get_ranges(max_vals, min_vals);
	// summarize clusters:
	cout<<"cluster summary: "<<endl;
	for (kclust=0;kclust<num_clusters;kclust++)
	{
      cout<<"cluster "<<kclust<<"; ";
	  clusters.cluster_list[kclust].get_features(feature_vec);
	  // unscale features back to raw ranges...
	  for (int ifeat=0;ifeat<NDIM;ifeat++)
		  feature_vec[ifeat]= feature_vec[ifeat]*
					  (max_vals[ifeat]-min_vals[ifeat])+
					  min_vals[ifeat];
	  row=(int) feature_vec[3];
	  col=(int) feature_vec[4];
	  cout<<"row="<<row<<"; col="<<col;
	  cout<<"; B = "<<(int) feature_vec[0];
	  cout<<"; G="<<(int) feature_vec[1]<<"; R="<<(int) feature_vec[2]<<endl;
	}


		// get image size:
		Nrows = imageObj.GetImageHeight();
		Ncols = imageObj.GetImageWidth();
		cout<<"image height: "<<Nrows<<endl;
		cout<<"image width: "<<Ncols<<endl;
		//test color...this is pure blue
		BGRvec[0]=0; // blue component
		BGRvec[1]=0; // green component
		BGRvec[2]=0; // red component
		//for (col=0;col<Ncols;col++)
			//for(row=0;row<Nrows;row++)
		// test: paint a background black
		for (col=0;col<Ncols;col++)
			for(row=0;row<Nrows;row++)
			{
				ipix = row*Ncols+col;
				//get membership for this pixel:
				kclust=pattern_assignments[ipix];
				// get corresponding cluster-avg RGB values:
				if (kclust>-1)
				{
				  //cout<<"col="<<col<<", row="<<row<<" pix assigned to cluster "<<kclust<<endl;
				  training_patterns.pattern_list[ipix].get_raw_features(val_vec);
				  //cout<<"orig pixel values: ";

				  //cout<<"row = "<<val_vec[3];
				  //cout<<"; col = "<<val_vec[4];
				  //cout<<";B = "<<val_vec[0];
				  //cout<<"; G= "<<val_vec[1];
				  //cout<<"; R= "<<val_vec[2]<<endl;

				  clusters.cluster_list[kclust].get_features(feature_vec);
				  // convert these back to ints in full range	
				  for (icolor=0;icolor<3;icolor++)
				  {
				   // undo scaling:
				   //	scaled_features[ifeat] =
		           // (raw_features[ifeat] - min_val_vec[ifeat])/
		           // (max_val_vec[ifeat]-min_val_vec[ifeat]);
				    dBGRvec[icolor]=
					  feature_vec[icolor]*
					  (max_vals[icolor]-min_vals[icolor])+
					  min_vals[icolor];
				    BGRvec[icolor]=(int)dBGRvec[icolor]; 
				  } 
				  //cout<<"cluster data: ";
				  //cout<<"row="<<row<<"; col="<<col<<"; B="<<BGRvec[0];
				  //cout<<"; G="<<BGRvec[1]<<"; R="<<BGRvec[2]<<endl;
					imageObj.SetPixelRGB(col,row,BGRvec);
				}

				//install these values in pixel (row,col)
				//imageObj.SetPixelRGB(col,row,BGRvec);
			}
			// save the resulting image:
	imageObj.EncodeJPGFileFromGeneralBufferColor("cluster_image.jpg");

}


void Kmeans::create_cluster_image(int kDisplay)
{
	int Nrows,Ncols;
	int row,col;
	int ipix,kclust;
	int icolor;
	int BGRvec[3];
	double dBGRvec[3];
	double feature_vec[NDIM];
	double max_vals[NDIM];
	double min_vals[NDIM];

	ImageFunc	imageObj;
	// use the input image to create a consistent imageObj
	char fname[]="input_image.jpg";
	if(!imageObj.DecodeJPGFileToGeneralBufferColor(fname))
		{
			char dummy;
			cout<<"could not load picture "<<fname<<endl;
			cout<<"Die? (y/y):  "; 	cin>>dummy;
			exit(0);
		}

		// get image size:
		Nrows = imageObj.GetImageHeight();
		Ncols = imageObj.GetImageWidth();
		//cout<<"image height: "<<Nrows<<endl;
		//cout<<"image width: "<<Ncols<<endl;
		//vec to set all pixels to black
		BGRvec[0]=0; // blue component
		BGRvec[1]=0; // green component
		BGRvec[2]=0; // red component
		//paint the background black
		for (col=0;col<Ncols;col++)
		   for(row=0;row<Nrows;row++)
			   imageObj.SetPixelRGB(col,row,BGRvec);

		clusters.cluster_list[kDisplay].get_features(feature_vec);
		// get the avg color for this cluster...
		get_ranges(max_vals, min_vals); // need these to un-scale the features
		for (icolor=0;icolor<3;icolor++)
		{
		 // undo scaling:
		 dBGRvec[icolor]=
					  feature_vec[icolor]*
					  (max_vals[icolor]-min_vals[icolor])+
					  min_vals[icolor];
		 BGRvec[icolor]=(int)dBGRvec[icolor]; 
		}
		// use this color to paint all pixels belonging to cluster kDisplay
		for (col=0;col<Ncols;col++)
			for(row=0;row<Nrows;row++)
			{
				ipix = row*Ncols+col;
				//get membership for this pixel:
				kclust=pattern_assignments[ipix];
				// get corresponding cluster-avg RGB values:
				if (kclust==kDisplay)
				{
				  // if here, then we want to paint this pixel
				  imageObj.SetPixelRGB(col,row,BGRvec);
				}
			}
			// save the resulting image:
	imageObj.EncodeJPGFileFromGeneralBufferColor("single_cluster_image.jpg");
}

void Kmeans::save_clustered_points(void)
{
	int Nrows,Ncols;
	int row,col;
	int k;

	int ipix,kclust;
	int BGRvec[3];
	ImageFunc	imageObj;
	ofstream outfile("pixel_cluster_list.dat");
	// use the input image to create a consistent imageObj
	// this is really just an ugly way to get the image dimensions;
	// Could have put these in header file instead.  Oh well.
	char fname[]="input_image.jpg";
	if(!imageObj.DecodeJPGFileToGeneralBufferColor(fname))
	{
		char dummy;
		cout<<"could not load picture "<<fname<<endl;
		cout<<"Die? (y/y):  "; 	cin>>dummy;
		exit(0);
	}

	// get image size:
	Nrows = imageObj.GetImageHeight();
	Ncols = imageObj.GetImageWidth();
	//cout<<"image height: "<<Nrows<<endl;
	//cout<<"image width: "<<Ncols<<endl;

	for (k=0;k<num_clusters;k++)
	{
		cout<<"looking for pixels in cluster "<<k<<endl;
		// use this color to paint all pixels belonging to cluster kDisplay
		for(row=0;row<Nrows;row++)
			for (col=0;col<Ncols;col++)

			{
				ipix = row*Ncols+col;
				//get membership for this pixel:
				kclust=pattern_assignments[ipix];
				// is this a pixel in the current cluster of interest?
				//cout<<"row "<<row<<", col "<<col<<"; ipix "<<ipix<<"; cluster = "<<kclust<<endl;

				if (kclust==k)
				{
					// if here, then we want to record this pixel as
					// belonging to cluster k
					imageObj.GetPixelRGB(col,row,BGRvec);
					cout<<"row "<<row<<", col "<<col<<": cluster assignment= "<<k<<endl;
					outfile<<BGRvec[0]<<" "<<BGRvec[1]<<" "<<BGRvec[2]<<" "<<k<<endl;
					//cout<<"enter 1:"; cin>>ans;
				}
			}
	}
	outfile.close();
}

double Kmeans::eval_pattern(double *val_vec) // return avg_attribute of closest cluster
{
  int knum;
  double a;
  knum = clusters.closest_cluster(val_vec);
  a = clusters.cluster_list[knum].get_attribute();
  return a;
}

void Kmeans::add_member(int ipat,int kclust) // add pattern ipat to cluster kclust
{
 // check first if ipat is already a member of kclust
	//if (!ismember(ipat,kclust))
	if (pattern_assignments[ipat]!=kclust)
	{ // if here, not already a member--so install it
		double val_vec[NDIM];
		training_patterns.pattern_list[ipat].get_scaled_features(val_vec);
		double attribute = training_patterns.pattern_list[ipat].get_attribute();
		is_mem_clust_pat[kclust][ipat]=true;
		pattern_assignments[ipat]=kclust;
        clusters.cluster_list[kclust].add_member(val_vec,attribute);
		//training_patterns.pattern_list[ipat].set_cluster_affiliation(kclust);

	}
}

void Kmeans::remove_member(int ipat,int kclust) // remove pattern ipat from cluster kclust
{ // check first if ipat is currently a member of kclust
	if(kclust>-1) // kclust=-1 means pattern not classified, so don't remove it
		//if (ismember(ipat,kclust)) // here if pattern classified, but does it belong to kclust?
	    if (pattern_assignments[ipat]==kclust)
		{ // if here, ipat is a member--so may remove it
			double val_vec[NDIM];
			training_patterns.pattern_list[ipat].get_scaled_features(val_vec);
			double attribute = training_patterns.pattern_list[ipat].get_attribute();
			is_mem_clust_pat[kclust][ipat]=false;
			clusters.cluster_list[kclust].remove_member(val_vec,attribute);
			
		}
}

