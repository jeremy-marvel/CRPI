///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       OptiTrack Log Merge
//  Workfile:        optitrack_log_merge.cpp
//  Revision:        1.0 9 June, 2021
//  Author:          J. Marvel
//
//  Description
//  ===========
//  
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <math.h>

#pragma warning (disable: 4996)
using namespace std;

//#define NOISY
#define TO_MM
#define ADD_LABELED_MARKERS


#define MINFRAMES 120
#define MAXDIST 3.0f
#define ORIGIN_THRESH 6





//! @brief 3D point structure
//!
struct pt
{
	double x;    //! X axis coordinate
	double y;    //! Y axis coordinate
	double z;    //! Z axis coordinate
	bool valid;  //! Whether this is a valid point (i.e., it is defined)

	//! @brief Display the contents of this point on the screen
	//!
	void print()
	{
		if (valid)
			cout << "(" << x << ", " << y << ", " << z << ")";
		else
			cout << "Not a valid point" << endl;
	}

	//! @brief Calculate the Euclidean distance between this point and another point
	//!
	//! @param no2 The second point to which we are calculating the distance
	//! 
	//! @return The Euclidean distance between these two points if both are valid.  -1 otherwise;
	//!
	double distance(pt no2)
	{
		if (valid && no2.valid)
		{
			return (sqrt( ((x-no2.x)*(x-no2.x)) + ((y-no2.y)*(y-no2.y)) + ((z-no2.z)*(z-no2.z)) ));
		}
		return -1.0f;
	}

	//! @brief Merge two points by taking the average point value.  If one point (or both points) is not valid,
	//!        then take the value of the other point.
	//! 
	//! @param no2 The second point that is being merged with this point
	//!
	void merge (pt no2)
	{
		if (valid)
		{
			if (no2.valid)
			{
				x = (x + no2.x) / 2.0f;
				y = (y + no2.y) / 2.0f;
				z = (z + no2.z) / 2.0f;
			}
			return;
		}

		x = no2.x;
		y = no2.y;
		z = no2.z;
		valid = no2.valid;
		return;
	}
};



//! @brief Individual frame storage structure
//!
struct frame
{
	int number;          //! Frame number
	double timestamp;    //! Time stamp (in seconds since process started)
	vector<pt> markers;  //! Collection of markers
};




void main(int argc, char* argv[])
{
	string buffer;
	double scale = 1.0f;
#ifdef TO_MM
	scale = 1000.0f;
#endif

	if (argc < 4)
	{
		cout << "Usage: OptiTrackLogs <file1> <file2> <outfile> <optional nudgefile>" << endl;
		return;
	}

	ifstream fIn;
	ofstream fout;
	frame data;
	vector<frame> frames1, frames2;
	vector<int> indexes;
	vector<char>::iterator order_iter;
	vector<frame>::iterator frame_iter;
	pt tempoint;
	bool state = true;
	int x = 0;
	int rb = 0;
	int rbm = 0;
	int m = 0;
	vector<char> order;
	int target;
	vector<pt>::iterator pt_iter;  //! Iterator to go through the markers of a single frame

	int frame_threshold = MINFRAMES; //! ~3 seconds
	double dist_threshold = MAXDIST; //! 5mm threshold for declaring markers "the same"
	cout << "dist threshold: " << dist_threshold << endl;

	pt nudge1, nudge2;
	nudge1.x = nudge2.x = nudge1.y = nudge2.y = nudge1.z = nudge2.z = 0.0f;

	//! -------------------------------------------------------------------------------------------------------------
	//! Read data files.  Identify valid and invalid frame data
	//! -------------------------------------------------------------------------------------------------------------

	if (argc == 5)
	{
		ifstream nudgefile;
		nudgefile.open(argv[4]);
		nudgefile >> nudge1.x >> nudge1.y >> nudge1.z;
		nudgefile >> nudge2.x >> nudge2.y >> nudge2.z;
		nudgefile.close();
	}




	cout << "Reading " << argv[1] << "..." << endl;
	fIn.open(argv[1]);

	for (x = 0; x < 22; ++x)
	{
		getline(fIn, buffer, ',');
#ifdef NOISY
		cout << buffer.c_str() << endl;
#endif
	}
	getline(fIn, buffer, ','); // "Type"

	order.clear();
	do
	{
		getline(fIn, buffer, ',');
		if (strcmp(buffer.c_str(), "Name") == 0)
		{
			//! Finished reading in entry designations
			break;
		}
		else if (strcmp(buffer.c_str(), "Rigid Body") == 0)
		{
			++rb;
			order.push_back('r');
			//! Read first rotation value
			getline(fIn, buffer, ','); //! 2nd rotation 
			getline(fIn, buffer, ','); //! 3rd rotation
			getline(fIn, buffer, ','); //! 4th rotation
			getline(fIn, buffer, ','); //! X position
			getline(fIn, buffer, ','); //! Y position
			getline(fIn, buffer, ','); //! Z position
			getline(fIn, buffer, ','); //! Mean marker error
		}
		else if (strcmp(buffer.c_str(), "Rigid Body Marker") == 0)
		{
			++rbm;
			order.push_back('b');
			//! Read X position
			getline(fIn, buffer, ','); //! Y position
			getline(fIn, buffer, ','); //! Z position
			getline(fIn, buffer, ','); //! Marker quality
		}
		else if (strcmp(buffer.c_str(), "Marker") == 0)
		{
			++m;
			order.push_back('m');
			//! Read X position
			getline(fIn, buffer, ','); //! Y position
			getline(fIn, buffer, ','); //! Z position
		}
	} while (true);
	cout << "Rigid bodies: " << rb << endl;
	cout << "Rigid body markers: " << rbm << endl;
	cout << "Unlabeled markers: " << m << endl;

	//! Last entry read is "Name"
	target = (rb * 8) + (rbm * 4) + (m * 3);
	for (x = 0; x < target + 1; ++x)
	{
		getline(fIn, buffer, ',');
	}

	//! Last entry read is "ID"
	for (x = 0; x < target + 1; ++x)
	{
		getline(fIn, buffer, ',');
	}

	//! Last entry read is ""
	for (x = 0; x < target + 1; ++x)
	{
		getline(fIn, buffer, ',');
	}

	//! Last entry read is "Time (Seconds)"
	for (x = 0; x < target - 1; ++x)
	{
		getline(fIn, buffer, ',');
	}
	getline(fIn, buffer, '\n');
	//! Last entry of the label line

	//! Time to read the actual data

	state = true;
	frames1.clear();
	do
	{
		//points.clear();
		data.markers.clear();
		getline(fIn, buffer, ','); //! Frame #
		if (buffer.length() == 0)
		{
			//! End of file
			break;
		}
		data.number = atoi(buffer.c_str());
		getline(fIn, buffer, ','); //! Time
		data.timestamp = atof(buffer.c_str());
		order_iter = order.begin();
		x = 0;
		for (; order_iter != order.end(); ++order_iter, ++x)
		{
			if (*order_iter == 'r')
			{
				//cout << "reading rigid body" << endl;
				//! Rigid body
				getline(fIn, buffer, ','); //! 1st rotation
				getline(fIn, buffer, ','); //! 2nd rotation 
				getline(fIn, buffer, ','); //! 3rd rotation
				getline(fIn, buffer, ','); //! 4th rotation
				getline(fIn, buffer, ','); //! X position
				getline(fIn, buffer, ','); //! Y position
				getline(fIn, buffer, ','); //! Z position
				if (x == (order.size() - 1))
					getline(fIn, buffer, '\n'); //! Mean marker error
				else
					getline(fIn, buffer, ','); //! Mean marker error
			} // if (*order_iter == 'r')
			else if (*order_iter == 'b')
			{
				//! Rigid body markers
				//cout << "Reading rigid body marker" << endl;
				getline(fIn, buffer, ','); //! X position
				if (strcmp(buffer.c_str(), "") == 0)
				{
					state = false; //! Not a valid marker
				}
				tempoint.x = atof(buffer.c_str()) * scale;
				getline(fIn, buffer, ','); //! Y position
				tempoint.y = atof(buffer.c_str()) * scale;
				getline(fIn, buffer, ','); //! Z position
				tempoint.z = atof(buffer.c_str()) * scale;
				if (x == (order.size() - 1))
					getline(fIn, buffer, '\n'); //! Marker quality
				else
					getline(fIn, buffer, ','); //! Marker quality
				tempoint.valid = state;

				tempoint.x += (nudge1.x * scale);
				tempoint.y += (nudge1.y * scale);
				tempoint.z += (nudge1.z * scale);

#ifdef ADD_LABELED_MARKERS
				data.markers.push_back(tempoint);
#endif
				state = true;
			} // else if (*order_iter == 'b')
			else
			{
				//cout << "Adding unlabeled marker" << endl;
				//! Markers
				getline(fIn, buffer, ','); //! X position
				if (strcmp(buffer.c_str(), "") == 0)
				{
					state = false; //! Not a valid marker (no data)
				}
				tempoint.x = atof(buffer.c_str()) * scale;
				getline(fIn, buffer, ','); //! Y position
				tempoint.y = atof(buffer.c_str()) * scale;
				if (x == (order.size() - 1))
					getline(fIn, buffer, '\n'); //! Z position
				else
					getline(fIn, buffer, ','); //! Z position
				tempoint.z = atof(buffer.c_str()) * scale;
				tempoint.valid = state;

				tempoint.x += (nudge1.x * scale);
				tempoint.y += (nudge1.y * scale);
				tempoint.z += (nudge1.z * scale);

				data.markers.push_back(tempoint);
				state = true;
			} // else
		} // for (; order_iter != order.end(); ++order_iter, ++x)
		frames1.push_back(data);

#ifdef NOISY
		cout << "Read frame " << data.number << " (timestamp: " << data.timestamp
			<< ", markers: " << data.markers.size() << ")" << endl;
#endif
	} while (true);

	cout << "Finished reading " << argv[1] << " with " << frames1.size() << " frames of marker data" << endl << endl;
	fIn.close();

	//! --------------------------------------
	//! Read 2nd file
	//! --------------------------------------
	rb = rbm = m = 0;
	cout << "Reading " << argv[2] << "..." << endl;
	fIn.open(argv[2]);

	for (x = 0; x < 22; ++x)
	{
		getline(fIn, buffer, ',');
#ifdef NOISY
		cout << buffer.c_str() << endl;
#endif
	}
	getline(fIn, buffer, ','); // "Type"

	order.clear();
	do
	{
		getline(fIn, buffer, ',');
		if (strcmp(buffer.c_str(), "Name") == 0)
		{
			//! Finished reading in entry designations
			break;
		}
		else if (strcmp(buffer.c_str(), "Rigid Body") == 0)
		{
			++rb;
			order.push_back('r');
			//! Read first rotation value
			getline(fIn, buffer, ','); //! 2nd rotation 
			getline(fIn, buffer, ','); //! 3rd rotation
			getline(fIn, buffer, ','); //! 4th rotation
			getline(fIn, buffer, ','); //! X position
			getline(fIn, buffer, ','); //! Y position
			getline(fIn, buffer, ','); //! Z position
			getline(fIn, buffer, ','); //! Mean marker error
		}
		else if (strcmp(buffer.c_str(), "Rigid Body Marker") == 0)
		{
			++rbm;
			order.push_back('b');
			//! Read X position
			getline(fIn, buffer, ','); //! Y position
			getline(fIn, buffer, ','); //! Z position
			getline(fIn, buffer, ','); //! Marker quality
		}
		else if (strcmp(buffer.c_str(), "Marker") == 0)
		{
			++m;
			order.push_back('m');
			//! Read X position
			getline(fIn, buffer, ','); //! Y position
			getline(fIn, buffer, ','); //! Z position
		}
	} while (true);
	cout << "Rigid bodies: " << rb << endl;
	cout << "Rigid body markers: " << rbm << endl;
	cout << "Unlabeled markers: " << m << endl;

	//! Last entry read is "Name"
	target = (rb * 8) + (rbm * 4) + (m * 3);
	for (x = 0; x < target + 1; ++x)
	{
		getline(fIn, buffer, ',');
	}

	//! Last entry read is "ID"
	for (x = 0; x < target + 1; ++x)
	{
		getline(fIn, buffer, ',');
	}

	//! Last entry read is ""
	for (x = 0; x < target + 1; ++x)
	{
		getline(fIn, buffer, ',');
	}

	//! Last entry read is "Time (Seconds)"
	for (x = 0; x < target - 1; ++x)
	{
		getline(fIn, buffer, ',');
	}
	getline(fIn, buffer, '\n');
	//! Last entry of the label line

	//! Time to read the actual data

	state = true;
	frames2.clear();
	do
	{
		//points.clear();
		data.markers.clear();
		getline(fIn, buffer, ','); //! Frame #
		if (buffer.length() == 0)
		{
			//! End of file
			break;
		}
		data.number = atoi(buffer.c_str());
		getline(fIn, buffer, ','); //! Time
		data.timestamp = atof(buffer.c_str());
		order_iter = order.begin();
		x = 0;
		for (; order_iter != order.end(); ++order_iter, ++x)
		{
			if (*order_iter == 'r')
			{
				//cout << "reading rigid body" << endl;
				//! Rigid body
				getline(fIn, buffer, ','); //! 1st rotation
				getline(fIn, buffer, ','); //! 2nd rotation 
				getline(fIn, buffer, ','); //! 3rd rotation
				getline(fIn, buffer, ','); //! 4th rotation
				getline(fIn, buffer, ','); //! X position
				getline(fIn, buffer, ','); //! Y position
				getline(fIn, buffer, ','); //! Z position
				if (x == (order.size() - 1))
					getline(fIn, buffer, '\n'); //! Mean marker error
				else
					getline(fIn, buffer, ','); //! Mean marker error
			} // if (*order_iter == 'r')
			else if (*order_iter == 'b')
			{
				//! Rigid body markers
				//cout << "Reading rigid body marker" << endl;
				getline(fIn, buffer, ','); //! X position
				if (strcmp(buffer.c_str(), "") == 0)
				{
					state = false; //! Not a valid marker
				}
				tempoint.x = atof(buffer.c_str()) * scale;
				getline(fIn, buffer, ','); //! Y position
				tempoint.y = atof(buffer.c_str()) * scale;
				getline(fIn, buffer, ','); //! Z position
				tempoint.z = atof(buffer.c_str()) * scale;
				if (x == (order.size() - 1))
					getline(fIn, buffer, '\n'); //! Marker quality
				else
					getline(fIn, buffer, ','); //! Marker quality
				tempoint.valid = state;

				tempoint.x += (nudge2.x * scale);
				tempoint.y += (nudge2.y * scale);
				tempoint.z += (nudge2.z * scale);

#ifdef ADD_LABELED_MARKERS
				data.markers.push_back(tempoint);
#endif
				state = true;
			} // else if (*order_iter == 'b')
			else
			{
				//cout << "Adding unlabeled marker" << endl;
				//! Markers
				getline(fIn, buffer, ','); //! X position
				if (strcmp(buffer.c_str(), "") == 0)
				{
					state = false; //! Not a valid marker
				}
				tempoint.x = atof(buffer.c_str()) * scale;
				getline(fIn, buffer, ','); //! Y position
				tempoint.y = atof(buffer.c_str()) * scale;
				if (x == (order.size() - 1))
					getline(fIn, buffer, '\n'); //! Z position
				else
					getline(fIn, buffer, ','); //! Z position
				tempoint.z = atof(buffer.c_str()) * scale;
				tempoint.valid = state;

				tempoint.x += (nudge2.x * scale);
				tempoint.y += (nudge2.y * scale);
				tempoint.z += (nudge2.z * scale);

				data.markers.push_back(tempoint);
				state = true;
			} // else
		} // for (; order_iter != order.end(); ++order_iter, ++x)
		frames2.push_back(data);
#ifdef NOISY
		cout << "Read frame " << data.number << " (timestamp: " << data.timestamp
			<< ", markers: " << data.markers.size() << ")" << endl;
#endif
	} while (true);

	cout << "Finished reading " << argv[2] << " with " << frames2.size() << " frames of marker data" << endl << endl;
	fIn.close();


	//! Keep track of which markers have been processed already so we don't process them again
	vector<bool> processed1, processed2;
	for (x = 0; x < frames1.at(0).markers.size(); ++x)
	{
		processed1.push_back(false);
	}
	for (x = 0; x < frames2.at(0).markers.size(); ++x)
	{
		processed2.push_back(false);
	}

	//! -------------------------------------------------------------------------------------------------------------
	//! Finished reading files and storing data.  Filter out noise markers (< threshold number of valid frames)
	//! -------------------------------------------------------------------------------------------------------------

	int counter;
	int removed = 0;
	for (x = 0; x < frames1.at(0).markers.size(); ++x)
	{
		counter = 0;
		for (frame_iter = frames1.begin(); frame_iter != frames1.end(); ++frame_iter)
		{
			counter += (frame_iter->markers.at(x).valid ? 1 : 0);
		}
		if (counter < frame_threshold)
		{
			//! Not enough frames to be a valid marker.  Likely noise.  Mark as procssed and move on.
			processed1.at(x) = true;
			++removed;
		}
	}

	if (removed > 0)
	{
		cout << "Filtered out " << removed << " markers from " << argv[1] << " due to frame thresholding." << endl;
	}

	removed = 0;
	for (x = 0; x < frames2.at(0).markers.size(); ++x)
	{
		counter = 0;
		for (frame_iter = frames2.begin(); frame_iter != frames2.end(); ++frame_iter)
		{
			counter += (frame_iter->markers.at(x).valid ? 1 : 0);
		}
		if (counter < frame_threshold)
		{
			//! Not enough frames to be a valid marker.  Likely noise.  Mark as procssed and move on.
			processed2.at(x) = true;
			++removed;
		}
	}

	if (removed > 0)
	{
		cout << "Filtered out " << removed << " markers from " << argv[2] << " due to frame thresholding." << endl;
	}

  //! -------------------------------------------------------------------------------------------------------------
	//! Identify origin markers.  Look only at first frame.
	//! -------------------------------------------------------------------------------------------------------------

	cout << endl << "Identifying origin markers... " << endl;

	unsigned int origin_1, origin_2, originx_1, originx_2, originz_1, originz_2;

	unsigned int tmp;
	bool ofound, xfound, zfound;

	tmp = 0;
	ofound = xfound = zfound = false;
	cout << argv[1] << ":" << endl;
	for (tmp = 0; tmp < frames1.at(0).markers.size(); ++tmp)// pt_iter = frames1.at(0).markers.begin(); pt_iter != frames1.at(0).markers.end(); ++pt_iter, ++tmp)
	{
		//cout << endl << tmp << ": " << frames1.at(0).markers.at(tmp).valid << " (" << frames1.at(0).markers.at(tmp).x << ", " << frames1.at(0).markers.at(tmp).y << ", " << frames1.at(0).markers.at(tmp).z << ")";


		if (frames1.at(0).markers.at(tmp).valid)
		{
#ifdef NOISY
			frames1.at(0).markers.at(tmp).print();
#endif
			if (fabs(frames1.at(0).markers.at(tmp).x) < (scale > 2.0f ? 5.0f : 0.005f) &&
				  fabs(frames1.at(0).markers.at(tmp).y) < (scale > 2.0f ? 5.0f : 0.005f) &&
				  fabs(frames1.at(0).markers.at(tmp).z) < (scale > 2.0f ? 5.0f : 0.005f) && !ofound)
			{
				origin_1 = tmp;
				ofound = true;
				continue;
			}
			if (fabs(frames1.at(0).markers.at(tmp).x) > (scale > 2.0f ? 10.0f : 0.01f) &&
				  fabs(frames1.at(0).markers.at(tmp).y) < (scale > 2.0f ? 5.0f : 0.005f) &&
				  fabs(frames1.at(0).markers.at(tmp).z) < (scale > 2.0f ? 5.0f : 0.005f) && !xfound)
			{
				originx_1 = tmp;
				xfound = true;
				continue;
			}
			if (fabs(frames1.at(0).markers.at(tmp).x) < (scale > 2.0f ? 5.0f : 0.005f) &&
				  fabs(frames1.at(0).markers.at(tmp).y) < (scale > 2.0f ? 5.0f : 0.005f) &&
				  fabs(frames1.at(0).markers.at(tmp).z) > (scale > 2.0f ? 10.0f : 0.01f) && !zfound)
			{
				originz_1 = tmp;
				zfound = true;
				continue;
			}
		}
	} //for (; pt_iter != frame_iter->markers.end(); ++pt_iter)

	//! JAM: TODO:  What if we can't find all three origin markers?
	if (!ofound)
	{
		cout << "Cound not find origin" << endl;
	}
	//! JAM: TODO:  What if there are multiple markers found that align with origin markers?

	if (ofound)
	{
		cout << "  Origin:  (" << frames1.at(0).markers.at(origin_1).x << ", " 
			   << frames1.at(0).markers.at(origin_1).y << ", " 
				 << frames1.at(0).markers.at(origin_1).z << ")" << endl;
		processed1.at(origin_1) = true;
	}
	if (xfound)
	{
		cout << "  OriginX:  (" << frames1.at(0).markers.at(originx_1).x << ", "
			   << frames1.at(0).markers.at(originx_1).y << ", "
			   << frames1.at(0).markers.at(originx_1).z << ")" << endl;
		processed1.at(originx_1) = true;
	}
	if (zfound)
	{
		cout << "  OriginZ:  (" << frames1.at(0).markers.at(originz_1).x << ", "
			   << frames1.at(0).markers.at(originz_1).y << ", "
				 << frames1.at(0).markers.at(originz_1).z << ")" << endl;
		processed1.at(originz_1) = true;
	}

	tmp = 0;
	ofound = xfound = zfound = false;
	origin_2 = originx_2 = originz_2 = -1;
	cout << argv[2] << ":" << endl;
	for (tmp = 0; tmp < frames2.at(0).markers.size(); ++tmp)// pt_iter = frames1.at(0).markers.begin(); pt_iter != frames1.at(0).markers.end(); ++pt_iter, ++tmp)
	{
		//cout << endl << tmp << ": " << frames2.at(0).markers.at(tmp).valid << " (" << frames2.at(0).markers.at(tmp).x << ", " << frames2.at(0).markers.at(tmp).y << ", " << frames2.at(0).markers.at(tmp).z << ")";

		if (frames2.at(0).markers.at(tmp).valid)
		{
			if (fabs(frames2.at(0).markers.at(tmp).x) < (scale > 2.0f ? 5.0f : 0.005f) &&
				fabs(frames2.at(0).markers.at(tmp).y) < (scale > 2.0f ? 5.0f : 0.005f) &&
				fabs(frames2.at(0).markers.at(tmp).z) < (scale > 2.0f ? 5.0f : 0.005f) && !ofound)
			{
				origin_2 = tmp;
				ofound = true;
				cout << "Origin Found" << endl;
				continue;
			}
			if (fabs(frames2.at(0).markers.at(tmp).x) > (scale > 2.0f ? 10.0f : 0.01f) &&
				fabs(frames2.at(0).markers.at(tmp).y) < (scale > 2.0f ? 5.0f : 0.005f) &&
				fabs(frames2.at(0).markers.at(tmp).z) < (scale > 2.0f ? 5.0f : 0.005f) && !xfound)
			{
				originx_2 = tmp;
				xfound = true;
				continue;
			}
			if (fabs(frames2.at(0).markers.at(tmp).x) < (scale > 2.0f ? 5.0f : 0.005f) &&
				fabs(frames2.at(0).markers.at(tmp).y) < (scale > 2.0f ? 5.0f : 0.005f) &&
				fabs(frames2.at(0).markers.at(tmp).z) > (scale > 2.0f ? 10.0f : 0.01f) && !zfound)
			{
				originz_2 = tmp;
				zfound = true;
				continue;
			}
		}
	} //for (; pt_iter != frame_iter->markers.end(); ++pt_iter)

	//! JAM: TODO:  What if we can't find all three origin markers?
	//! JAM: TODO:  What if there are multiple markers found that align with origin markers?

	cout << "[o = " << origin_2 << "[" << ofound << "], x = " << originx_2 << "[" << xfound << "], z = " << originz_2 << "[" << zfound << "]]" << endl;
	//cin >> x;

	if (ofound)
	{
		cout << "  Origin:  (" << frames2.at(0).markers.at(origin_2).x << ", "
			<< frames2.at(0).markers.at(origin_2).y << ", "
			<< frames2.at(0).markers.at(origin_2).z << ")" << endl;
		processed2.at(origin_2) = true;
	}
	if (xfound)
	{
		cout << "  OriginX:  (" << frames2.at(0).markers.at(originx_2).x << ", "
			<< frames2.at(0).markers.at(originx_2).y << ", "
			<< frames2.at(0).markers.at(originx_2).z << ")" << endl;
		processed2.at(originx_2) = true;
	}
	if (zfound)
	{
		cout << "  OriginZ:  (" << frames2.at(0).markers.at(originz_2).x << ", "
			<< frames2.at(0).markers.at(originz_2).y << ", "
			<< frames2.at(0).markers.at(originz_2).z << ")" << endl;
		processed2.at(originz_2) = true;
	}

	cout << "Done." << endl;

	//! -------------------------------------------------------------------------------------------------------------
	//! Identify first frames for synchronization
	//! -------------------------------------------------------------------------------------------------------------

	cout << "Finding initial frames for synchronization..." << endl;
	unsigned int frames1_zero, frames2_zero;
	bool originfound;
	
	x = 0;
	frames1_zero = 0;
	for (frame_iter = frames1.begin(); frame_iter != frames1.end(); ++frame_iter, ++x)
	{
		if (frame_iter->markers.at(origin_1).valid)
		{
			if ((x - frames1_zero <= ORIGIN_THRESH))
			{
				frames1_zero = x;
			}
		}
	} //for (frame_iter = frames1.begin(); frame_iter != frames2.end(); ++frame_iter)
	cout << argv[1] << " origin disappears frame #" << frames1_zero << endl;

	x = 0;
	frames2_zero = 0;
	for (frame_iter = frames2.begin(); frame_iter != frames2.end(); ++frame_iter, ++x)
	{
		if (frame_iter->markers.at(origin_2).valid)
		{
			if ((x - frames2_zero <= ORIGIN_THRESH))
			{
				frames2_zero = x;
			}
		}

	} //for (frame_iter = frames1.begin(); frame_iter != frames2.end(); ++frame_iter)
	cout << argv[2] << " origin disappears frame #" << frames2_zero << endl;

	//! -------------------------------------------------------------------------------------------------------------
  //! Identify aligned markers based on overlap.  Aligned markers are merged, and the markers identified as having
	//! been processed.
  //! -------------------------------------------------------------------------------------------------------------

	int c1, c2;
	int m1, m2;
	double dist, mindist, avgdist;

	cout << "Merging " << argv[1] << " and " << argv[2] << "...";
	for (m1 = 0; m1 < frames1.at(0).markers.size(); ++m1)
	{
		//! For all markers in file 1...

		//! If we've already processed this marker in file 1, skip
		if (processed1.at(m1))
		{
			continue;
		}

		for (m2 = 0; m2 < frames2.at(0).markers.size(); ++m2)
		{
			if (processed2.at(m2))
			{
				//! If we've already processed this marker in file 2, skip
				continue;
			}

			mindist = 20000.0f;
			avgdist = 0.0f;
			x = 0;
			c2 = frames2_zero;
			for (c1 = frames1_zero; c1 < frames1.size(); ++c1, ++c2)
			{
				if (c2 >= frames2.size())
				{
					//! If the second stream ends before the first stream, quit this process
					//! If the first stream ends before the second stream, this process quits automatically
					break;
				}

				dist = frames1.at(c1).markers.at(m1).distance(frames2.at(c2).markers.at(m2));
				if (dist >= 0.0f)
				{
					x++;
					//! Valid marker, check to see if distances is smaller than current minimum distance
					mindist = ((dist < mindist) ? dist : mindist);
					avgdist += dist;
				}
			} // for (c1 = frames1_zero; c1 < frames1.size(); ++c1, ++c2)

			if (x > 0)
			{
				avgdist /= (double)x;
				//cout << "Min dist:  " << mindist << endl;
				//cout << "avg dist:  " << avgdist << endl;
				if (avgdist < dist_threshold)
				{
					//! These are likely the same marker.  Merge them. 
					c2 = frames2_zero;
					for (c1 = frames1_zero; c1 < frames1.size(); ++c1, ++c2)
					{
						if (c2 >= frames2.size())
						{
							//! If the second stream ends before the first stream, quit this process
							//! If the first stream ends before the second stream, this process quits automatically
							break;
						}
#ifdef NOISY
						cout << "  Before: ";
						frames1.at(c1).markers.at(m1).print();
						cout << endl;
						cout << "  With: ";
						frames2.at(c2).markers.at(m2).print();
						cout << endl;
#endif
						//! Merge these two streams
						frames1.at(c1).markers.at(m1).merge(frames2.at(c2).markers.at(m2));
#ifdef NOISY
						cout << "  After: ";
						frames1.at(c1).markers.at(m1).print();
						cout << endl;
#endif
					} // for (c1 = frames1_zero; c1 < frames1.size(); ++c1, ++c2)

					//! Mark the second marker as having been processed since it's been merged with a marker in file 1.
					processed2.at(m2) = true;
				} // if (mindist < dist_threshold)
			} // if (mindist >= 0.0f)
		} // for (m2 = 0; m2 < frames2.at(0).markers.size(); ++m2)
	} // for (m1 = 0; m1 < frames1.at(0).markers.size(); ++m1)
	cout << "Done." << endl;

	//! All markers in file 2 have either been merged, or are determined to be unique markers.
	//! Set aside the file 2 data for the time being.  Any markers that are unique are marked as not
	//! having been processed.

	//! Determine if there are any markers within file 1 that should be merged
	//! (After merging with file 2, it is possible that marker streams now have enough data that
	//! orphaned markers can now be identified)
	cout << "Merging processed markers within " << argv[1] << "...";
	bool changed;
	do
	{
		//! Keep doing this process until no more changes are made
		changed = false;

		for (m1 = 0; m1 < frames1.at(0).markers.size(); ++m1)
		{
			//! For all markers in file 1...

			//! If we've already processed this marker in file 1, skip
			if (processed1.at(m1))
			{
				continue;
			}

			for (m2 = m1+1; m2 < frames1.at(0).markers.size(); ++m2)
			{
				if (processed1.at(m2))
				{
					//! If we've already processed this marker in file 2, skip
					continue;
				}

				mindist = 20000.0f;
				avgdist = 0.0f;
				x = 0;
				for (c1 = frames1_zero; c1 < frames1.size(); ++c1)
				{
					dist = frames1.at(c1).markers.at(m1).distance(frames1.at(c1).markers.at(m2));
					if (dist >= 0.0f)
					{
						//! Valid marker, check to see if distances is smaller than current minimum distance
						mindist = ((dist < mindist) ? dist : mindist);
						avgdist += dist;
						x++;
					}
				} // for (c1 = frames1_zero; c1 < frames1.size(); ++c1, ++c2)

				if (x > 0)
				{
					avgdist /= (double)x;
					if (avgdist < dist_threshold)
					{
						//! These are likely the same marker.  Merge them. 
						for (c1 = frames1_zero; c1 < frames1.size(); ++c1)
						{
							//! Merge these two streams

#ifdef NOISY
							cout << "  ";
							frames1.at(c1).markers.at(m1).print();
							cout << endl;
							cout << "  ";
							frames1.at(c1).markers.at(m2).print();
							cout << endl;
#endif
							//! Merge these two streams
							frames1.at(c1).markers.at(m1).merge(frames1.at(c1).markers.at(m2));
#ifdef NOISY
							cout << "  ";
							frames1.at(c1).markers.at(m1).print();
							cout << endl;
#endif
						} // for (c1 = frames1_zero; c1 < frames1.size(); ++c1, ++c2)

						//! Mark the second marker as having been processed since it's been merged with a marker in file 1.
						processed1.at(m2) = true;
						changed = true;
					} // if (mindist < dist_threshold)
				} // if (mindist >= 0.0f)
			} // for (m2 = 0; m2 < frames2.at(0).markers.size(); ++m2)
		} // for (m1 = 0; m1 < frames1.at(0).markers.size(); ++m1)
	} while (changed);
	cout << "Done." << endl;


	//! -------------------------------------------------------------------------------------------------------------
  //! Anything that can be merged has been merged.  Output values that have not been merged or ignored (i.e.,
	//! processed == false).
  //! -------------------------------------------------------------------------------------------------------------
	
	//! It's not uncommon for unprocessed, valid markers to disappear before (or shortly after) the beginning streams
	for (m1 = 0; m1 < frames1.at(0).markers.size(); ++m1)
	{
		x = 0;
		if (processed1.at(m1))
			continue;

		for (c1 = 0; c1 < frames1.size(); ++c1)
		{
			x += ((frames1.at(0).markers.at(m1).valid) ? 1 : 0);
		}
		if (x < 20)
		{
			processed1.at(m1) = true;
		}
	}

	for (m2 = 0; m2 < frames2.at(0).markers.size(); ++m2)
	{
		x = 0;
		if (processed2.at(m2))
			continue;

		for (c2 = 0; c2 < frames2.size(); ++c2)
		{
			x += ((frames2.at(0).markers.at(m2).valid) ? 1 : 0);
		}
		if (x < 20)
		{
			processed2.at(m2) = true;
		}
	}




	cout << endl << "Finished merging and filtering data.  Writing " << argv[3] << "..." << endl;
	fout.open(argv[3]);

	//! Write the column headers
	fout << "Frame, Time";
	c1 = 0;
	for (m1 = 0; m1 < processed1.size(); ++m1)
	{
		if (!(processed1.at(m1)))
		{
			fout << ", Marker" << c1 << "_x, Marker" << c1 << "_y, Marker" << c1 << "_z";
			++c1;
		}
	}
	for (m2 = 0; m2 < processed2.size(); ++m2)
	{
		if (!(processed2.at(m2)))
		{
			fout << ", Marker" << c1 << "_x, Marker" << c1 << "_y, Marker" << c1 << "_z";
			++c1;
		}
	}
	fout << endl;

	c1 = frames1_zero;
	c2 = frames2_zero;
	do
	{
		//! Use the frame and timestamp information for the output
		fout << frames1.at(c1).number << ", " << frames1.at(c1).timestamp;
		//! Add unprocessed markers from file 1
		for (m1 = 0; m1 < processed1.size(); ++m1)
		{
			if (!(processed1.at(m1)))
			{
				if (frames1.at(c1).markers.at(m1).valid)
				{
					fout << ", " << frames1.at(c1).markers.at(m1).x << ", " << frames1.at(c1).markers.at(m1).y << ", " << frames1.at(c1).markers.at(m1).z;
				}
				else
				{
					fout << ", , , ";
				}
			}
		} // for (m1 = 0; m1 < processed1.size(); ++m1)
		//! Add unprocessed markers from file 2
		for (m2 = 0; m2 < processed2.size(); ++m2)
		{
			if (!(processed2.at(m2)))
			{
				if (frames2.at(c2).markers.at(m2).valid)
				{
					fout << ", " << frames2.at(c2).markers.at(m2).x << ", " << frames2.at(c2).markers.at(m2).y << ", " << frames2.at(c2).markers.at(m2).z;
				}
				else
				{
					fout << ", , , ";
				}
			}
		} // for (m2 = 0; m2 < processed2.size(); ++m2)
		fout << endl;
		++c1;
		++c2;
	} while (c1 < frames1.size() && c2 < frames2.size());
	fout.close();

	cout << "Complete" << endl;
}
