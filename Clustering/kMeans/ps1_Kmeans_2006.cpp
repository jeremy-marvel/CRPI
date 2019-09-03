// ps1_Kmeans_2006.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <fstream>
#include <string>
#include "kmeans_2006.h"

// main program for k-means clustering:
void main(void)
{
	int i;
	int ans=1;
	cout<<"instantiating kmeans object..."<<endl;
	Kmeans kmeans;
	//double max_vals[NDIM];
	//double min_vals[NDIM];
    
	// seed the clusters:
	cout<<"seeding clusters..."<<endl;
	kmeans.seed_clusters();
	cout<<"done seeding clusters; enter 1: ";cin>>ans;
	//cout<<"test...creating image: "<<endl;
	//kmeans.create_cluster_image();
	kmeans.save_clustered_points();
	cout<<"saved seed assignments; enter 1: ";
	cin>>ans;

	// do "reclustering":  assign all patterns to clusters
	// updating cluster centroid with each assignment;
    // continue doing so until no more cluster reassignments,
	// or until max number of reclustering computations reached
	

	//for (i=0;i<100;i++)
	for (i=0;i<100;i++)
	{
	  int reassignments;
	  reassignments=kmeans.recluster();

      if (!reassignments) break; // stop early if no patterns reassigned
      cout<<"pass "<<i<<"; reassigned "<<reassignments<<" patterns"<<endl;
	}
	cout<<"done reclustering after "<<i<<" iterations"<<endl;
	//kmeans.print_clusters();
	cout<<"creating output image..."<<endl;
	kmeans.create_cluster_image();

	kmeans.save_clustered_points();

	int ktest=0;
	while(ktest>-1)
	{
		cout<<"enter cluster number for eval display (<0 to quit): ";
		cin>>ktest;
		// create image with pixels set to black, except for
		// pixels within selected cluster:
		kmeans.create_cluster_image(ktest);
	}

}

// this function is used to count the number of patterns in a datafile, 
// which is useful for allocating sufficient memory for Pattern objects
// The function must be provided with the number of dimensions expected
// and with a data file name 
// The function opens the file, counts how many patterns are in the file
// and returns the number of patterns found.

int parse_file(char *fname, int ndims)
{
	char input_chars[256];
	int npats=0;
	double *vals;
	int attribute;
	bool done=false;
	vals = new double[ndims];
	ifstream newfile(fname);
	if (!newfile)
	{ 
		cout << "cannot open file "<<fname<<endl;
		cout << "I'm outta here!!!"<<endl;
		exit(1);
	}
	cout << "opened file "<<fname<<endl;
	// opened the file: now figure out how many entries...
  /**/
  // nchars=0;
  newfile.getline(input_chars,12,'\t');
       // cout<<"pattern 0"<<endl;
		//cout<<"read label "<<input_chars<<endl;
        //cout<<"enter 1:";
		//cin>>nchars;
	while(!done)
	{

        
		//cout<<"feature values:"<<endl;
		for (int i=0;i<ndims;i++) // feature values
		{
			newfile >> vals[i];
		//	cout<<"   " << vals[i]<<endl;
		}
		// read attribute
		if (newfile >> attribute)
			npats++;
		//cout<<"attribute: "<<attribute<<endl;

		if( newfile.getline(input_chars,12,'\t'))
		{
		 if (strlen(input_chars)<5)
		 { //cout<<"name length too short"<<endl;
			 done=true;
		 }
		 else
		 {
	     //cout<<"pattern "<<npats;
		 //cout<<": label="<<input_chars<<endl;
		 }

		}
		else
		{
			done=true;
			//cout<<"no new label found"<<endl;
		}
	}
/**/
delete [] vals;
cout<<" found "<<npats<<" lines in the file"<<endl;
return npats;

}

