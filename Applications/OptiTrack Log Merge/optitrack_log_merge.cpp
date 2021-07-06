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

#pragma warning (disable: 4996)
using namespace std;

//#define NOISY
#define TO_MM

struct pt
{
	double x;
	double y;
	double z;
	bool valid;

	void print()
	{
		if (valid)
			cout << "(" << x << ", " << y << ", " << z << ")";
		else
			cout << "Not a valid point" << endl;
	}
};

struct frame
{
	int number;
	double timestamp;
	vector<pt> markers;
};

void main(int argc, char* argv[])
{
	string buffer;
	double scale = 1.0f;
#ifdef TO_MM
	scale = 1000.0f;
#endif

	if (argc < 3)
	{
		cout << "Usage: OptiTrackLogs <file1> <file2>" << endl;
		return;
	}

	ifstream fIn;
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

	int frame_threshold = 360; //! ~3 seconds

	//! -------------------------------------------------------------------------------------------------------------
	//! Read data files.  Identify valid and invalid frame data
	//! -------------------------------------------------------------------------------------------------------------

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

				data.markers.push_back(tempoint);
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

				data.markers.push_back(tempoint);
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
	for (x = 0; x < frames1.size(); ++x)
	{
		processed1.push_back(false);
	}
	for (x = 0; x < frames2.size(); ++x)
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
	//! Identify first frames for synchronization
	//! -------------------------------------------------------------------------------------------------------------

	unsigned int origin_1, origin_2, originx_1, originx_2, originy_1, originy_2;

	pt_iter = frames1.at(0).markers.begin();
	unsigned int tmp = 0;
	bool ofound, xfound, yfound;
	ofound = xfound = yfound = false;
	for (; pt_iter != frame_iter->markers.end(); ++pt_iter, ++tmp)
	{
		if (pt_iter->valid)
		{
			if (fabs(pt_iter->x) < (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->y) < (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->z) < (scale > 2.0f ? 1.0f : 0.001f))
			{
				origin_1 = tmp;
				ofound = true;
				continue;
			}
			if (fabs(pt_iter->x) > (scale > 2.0f ? 10.0f : 0.01f) &&
				  fabs(pt_iter->y) < (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->z) < (scale > 2.0f ? 1.0f : 0.001f))
			{
				originx_1 = tmp;
				xfound = true;
				continue;
			}
			if (fabs(pt_iter->x) > (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->y) < (scale > 2.0f ? 10.0f : 0.01f) &&
				  fabs(pt_iter->z) < (scale > 2.0f ? 1.0f : 0.001f))
			{
				originy_1 = tmp;
				xfound = true;
				continue;
			}
		}
	} //for (; pt_iter != frame_iter->markers.end(); ++pt_iter)


	//! JAM: TODO:  What if we can't find all three origin markers?
	//! JAM: TODO:  What if there are multiple markers found that align with origin markers?

	processed1.at(origin_1) = true;
	processed1.at(originx_1) = true;
	processed1.at(originy_1) = true;








	processed2.at(origin_2) = true;
	processed2.at(originx_2) = true;
	processed2.at(originy_2) = true;

	//! -------------------------------------------------------------------------------------------------------------
	//! Finished reading files and storing data.  Identify first frames for synchronization
	//! -------------------------------------------------------------------------------------------------------------

	cout << "Finding initial frames for synchronization..." << endl;
	unsigned int frames1_zero, frames2_zero;
	bool originfound;
	
	frames1_zero = 0;
	for (frame_iter = frames1.begin(); frame_iter != frames1.end(); ++frame_iter, ++frames1_zero)
	{
		if (frame_iter->markers.at(origin_1).valid != true)
		{
			break;
		}


		pt_iter = frame_iter->markers.begin();
		originfound = false;
		for (; pt_iter != frame_iter->markers.end(); ++pt_iter)
		{
			if (fabs(pt_iter->x) < (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->y) < (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->z) < (scale > 2.0f ? 1.0f : 0.001f))
			{
				originfound = true;
				break;
			}
		} //for (; pt_iter != frame_iter->markers.end(); ++pt_iter)
		if (!originfound)
		{
			break;
		}
	} //for (frame_iter = frames1.begin(); frame_iter != frames2.end(); ++frame_iter)
	cout << argv[1] << " origin disappears frame #" << frames1_zero << endl;

	frames2_zero = 0;
	for (frame_iter = frames2.begin(); frame_iter != frames2.end(); ++frame_iter, ++frames2_zero)
	{
		pt_iter = frame_iter->markers.begin();
		originfound = false;
		for (; pt_iter != frame_iter->markers.end(); ++pt_iter)
		{
			if (fabs(pt_iter->x) < (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->y) < (scale > 2.0f ? 1.0f : 0.001f) &&
				  fabs(pt_iter->z) < (scale > 2.0f ? 1.0f : 0.001f))
			{
				//! Look for marker at (0, 0, 0) - origin
				originfound = true;
				break;
			}
		} //for (; pt_iter != frame_iter->markers.end(); ++pt_iter)
		if (!originfound)
		{
			break;
		}
	} //for (frame_iter = frames1.begin(); frame_iter != frames2.end(); ++frame_iter)
	cout << argv[2] << " origin disappears frame #" << frames2_zero << endl;

	//! -------------------------------------------------------------------------------------------------------------
  //! Finished reading files and storing data.  Identify first frames for synchronization
  //! -------------------------------------------------------------------------------------------------------------


}
