////////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       System Registration
//  Workfile:        MLRegistration.cpp
//  Revision:        9 October, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Coordinate system registration demo with machine learning.
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "crpi_robot.h"
#include "crpi_universal.h"
#include "crpi_kuka_lwr.h"
#include "crpi_abb.h"
#include "ulapi.h"
#include "CoordFrameReg.h"
#include <algorithm>

//! Clustering
#include "kMeansCluster.h"

//! Neural networks
#include "NeuralNet.h"
#include "Numerical.h"

#pragma warning (disable: 4996)

#define NOISY

using namespace crpi_robot;
using namespace std;
using namespace Registration;
using namespace Clustering;
using namespace NeuralNet;
using namespace Neuron;



bool ToWorldTest(pose in, pose& out, matrix& trans)
{
  robotPose pin, pout;

  matrix r(3, 3);
  matrix mrot(4, 4), mtrans(4, 4), t1(4, 4), t2(4, 4);
  matrix inm(4, 4), outm(4, 4);
  bool flag = true;


  t1 = trans.inv();

  Math::pose ptemp = in;
  flag &= inm.RPYMatrixConvert(ptemp, true);
  inm.at(0, 3) = in.x;
  inm.at(1, 3) = in.y;
  inm.at(2, 3) = in.z;
  inm.at(3, 3) = 1;
  outm = t1 * inm;
  flag &= outm.matrixRPYConvert(ptemp, true);
  out = ptemp;

  if (flag)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void main ()
{
  string str;

  cout << "Enter training file name: ";
  cin >> str;

  char line[256];
  char systembuff[10];

  int c;
  vector<point> raw, registered, truth;
  vector<point> raw2, registered2, truth2;
  vector<pose> poses2;
  vector<double> tempv;
  point tempp;
  pose tempo;
  char *pch;

  ifstream infile;
  infile.open(str.c_str());
  if (infile.is_open())
  {
    c = 0;
    while (!infile.eof())
    {
      infile.getline(line, 256);
      tempv.clear();
      pch = strtok(line, ",\n\0");
      while (pch != NULL)
      {
        tempv.push_back(atof(pch));
        pch = strtok(NULL, ",\n\0");
      }
      if (tempv.size() == 9)
      {
        tempp.x = tempv.at(0);
        tempp.y = tempv.at(1);
        tempp.z = tempv.at(2);
        if (tempp.x == 0 && tempp.y == 0 && tempp.z == 0)
        {
          //! Labels.  Ignore.
        }
        else
        {
          raw.push_back(tempp);
          tempp.x = tempv.at(3);
          tempp.y = tempv.at(4);
          tempp.z = tempv.at(5);
          //! original data set this was the robot-world projection, in new data sets it's raw orientation
          registered.push_back(tempp);
          tempp.x = tempv.at(6);
          tempp.y = tempv.at(7);
          tempp.z = tempv.at(8);
          truth.push_back(tempp);
        }
      }
      else if (tempv.size() == 0)
      {
        //! Blank line or end of file.  Do nothing.
      }
      else
      {
        cout << "incorrect vector length: " << tempv.size() << endl;
      }
    }
  }
  else
  {
    cout << "Could not find \"" << str.c_str() << "\"" << endl;
    return;
  }
  infile.close();


  cout << "Enter testing file name: ";
  cin >> str;

  infile.open(str.c_str());
  if (infile.is_open())
  {
    c = 0;
    while (!infile.eof())
    {
      infile.getline(line, 256);
      tempv.clear();
      pch = strtok(line, ",\n\0");
      while (pch != NULL)
      {
        tempv.push_back(atof(pch));
        pch = strtok(NULL, ",\n\0");
      }
      if (tempv.size() == 9)
      {
        tempp.x = tempv.at(0);
        tempp.y = tempv.at(1);
        tempp.z = tempv.at(2);
        if (tempp.x == 0 && tempp.y == 0 && tempp.z == 0)
        {
          //! Labels.  Ignore.
        }
        else
        {
          raw2.push_back(tempp);
          tempo.x = tempp.x;
          tempo.y = tempp.y;
          tempo.z = tempp.z;
          tempp.x = tempv.at(3);
          tempp.y = tempv.at(4);
          tempp.z = tempv.at(5);
          tempo.xr = tempp.x;
          tempo.yr = tempp.y;
          tempo.zr = tempp.z;
          //! original data set this was the robot-world projection, in new data sets it's raw orientation
          registered2.push_back(tempp);
          poses2.push_back(tempo);
          tempp.x = tempv.at(6);
          tempp.y = tempv.at(7);
          tempp.z = tempv.at(8);
          truth2.push_back(tempp);
        }
      }
      else if (tempv.size() == 0)
      {
        //! Blank line or end of file.  Do nothing.
      }
      else
      {
        cout << "incorrect vector length: " << tempv.size() << endl;
      }
    }
  }
  else
  {
    cout << "Could not find \"" << str.c_str() << "\"" << endl;
    return;
  }
  infile.close();


  point wrd, rob, centerf, centera, p1, p2, p3;
  point plwr[3], pur[3], pworld[3];
  vector<point> tmpr, tmpw;
  vector<vector<point> > wrds, robs;
  matrix r_2_w(4, 4), w_2_r(4, 4), w_2_r_avg[2];
  w_2_r_avg[0].resize(4, 4);
  w_2_r_avg[1].resize(4, 4);
  vector<point> world;
  vector<point> robot;

  //! Clustering

  int numclust = 2;
  kMeans world_clusters(3, 3, numclust, NULL);
  world_clusters.setMinClusterMembers(1);

  vector<double> feat, attrib;
  vector<point>::iterator iter_raw, iter_reg;

  iter_raw = raw.begin();
  iter_reg = truth.begin();
  int count = 0;
  for (; iter_raw != raw.end(); ++iter_raw, ++iter_reg, ++count)
  {
    feat.clear();
    attrib.clear();

    feat.push_back(iter_raw->x);
    feat.push_back(iter_raw->y);
    feat.push_back(iter_raw->z);

    attrib.push_back(iter_reg->x);
    attrib.push_back(iter_reg->y);
    attrib.push_back(iter_reg->z);

    //! Add training pattern to cluster space
    cout << "Add pattern ((" << feat.at(0) << ", " << feat.at(1) << ", " << feat.at(2) << "), ("
         << attrib.at(0) << ", " << attrib.at(1) << ", " << attrib.at(2) << "}}" << endl;
    world_clusters.addTrainingPattern(feat, attrib);
  }
  cout << count << " patterns added to the collection of clusters" << endl;

  cout << "Seeding clusters" << endl;
  world_clusters.seedClusters();
  count = 0;
  do
  {
    count = world_clusters.recluster();
    cout << count << " patterns moved." << endl;
  } while (count > 0);
  cout << "Finished clustering." << endl;

  cout << "Generating robot-world registration data" << endl;
  for (int i = 0; i < numclust; ++i)
  {
    world_clusters.getClusterInfo(i, feat, attrib);
#ifdef NOISY
    cout << "Cluster data: " << i << " ((" << feat.at(0) << ", " << feat.at(1) << ", " << feat.at(2) << "), ("
      << attrib.at(0) << ", " << attrib.at(1) << ", " << attrib.at(2) << ")) with "
      << world_clusters.getClusters().at(i).size() << " members" << endl;
#endif
    centerf.x = feat.at(0);
    centerf.y = feat.at(1);
    centerf.z = feat.at(2);
    centera.x = attrib.at(0);
    centera.y = attrib.at(1);
    centera.z = attrib.at(2);

#ifdef NOISY
    for (int j = 0; j < truth.size(); ++j)
    {
      feat.at(0) = raw.at(j).x;
      feat.at(1) = raw.at(j).y;
      feat.at(2) = raw.at(j).z;
      attrib.at(0) = truth.at(j).x;
      attrib.at(1) = truth.at(j).y;
      attrib.at(2) = truth.at(j).z;
      if (world_clusters.getClusters().closestCluster(feat) == i)
      {
        cout << "  Member : ((" << feat.at(0) << ", " << feat.at(1) << ", " << feat.at(2) << "), ("
          << attrib.at(0) << ", " << attrib.at(1) << ", " << attrib.at(2) << "))" << endl;
      }
    } // for (int j = 0; j < truth.size(); ++j)
    cout << endl;
    cout << "Saving registration #" << (i + 1) << endl;
#endif
    world.clear();
    robot.clear();
    world.resize(3);
    robot.resize(3);
    w_2_r_avg[i].setAll(0.0f);

    world.at(0) = centera;
    robot.at(0) = centerf;
    count = 0;
    for (int j = 0; j < truth.size(); ++j)
    {
      wrd.x = truth.at(j).x;
      wrd.y = truth.at(j).y;
      wrd.z = truth.at(j).z;
      world.at(1) = wrd;
      rob.x = raw.at(j).x;
      rob.y = raw.at(j).y;
      rob.z = raw.at(j).z;
      robot.at(1) = rob;

      for (int k = j + 1; k < truth.size(); ++k)
      {
        wrd.x = truth.at(k).x;
        wrd.y = truth.at(k).y;
        wrd.z = truth.at(k).z;
        world.at(2) = wrd;
        rob.x = raw.at(k).x;
        rob.y = raw.at(k).y;
        rob.z = raw.at(k).z;
        robot.at(2) = rob;

        if (reg2target(robot, world, r_2_w))
        {
          w_2_r = r_2_w.inv();
          w_2_r_avg[i] = w_2_r_avg[i] + w_2_r;
          ++count;
        }
      } // for (int k = j + 1; k < truth.size(); ++k)
    } // for (int j = 0; j < truth.size(); ++j)
    w_2_r_avg[i] = w_2_r_avg[i] / count;
    sprintf(systembuff, "cluster_%d", i);


    matrix tm1(4, 4), tm2(4, 4);
    tm1 = w_2_r_avg[i].trans();
    tm2 = tm1 * w_2_r_avg[i];
    cout << "________" << endl;
    tm2.print();
    cout << "________" << endl;

#ifdef NOISY
    cout << "named coordinate system transformation \"" << systembuff << "\":" << endl;
    w_2_r_avg[i].print();
#endif

    /*
    state = reg2target(rob1, world, r1_2_w);
    w_2_r1 = r1_2_w.inv();

    ur.UpdateWorldTransform(w_2_r1);
    ur.SaveConfig(urfile.c_str());
    */
  } // for (int i = 0; i < numclust; ++i)

  cout << endl << "Finished clustering.  Evaluating testing patterns..." << endl;

  char outname[32];
  sprintf(outname, "..\\Data\\%s_%s.csv", NumDateStr(), TimeStr());
  ofstream outfile(outname);
  cout << "Saving performance data to \"" << outname << "\"" << endl;
  outfile << "rob_x, rob_y, rob_z, rob_xr, rob_yr, rob_zr, wld_x, wld_y, wld_z, wld_xr, wld_yr, wld_zr, truth_x, truth_y, truth_z, pos_err, dist_wld_orig, dist_rob_orig, dist_clust_orig" << endl;

  iter_raw = raw2.begin();
  iter_reg = truth2.begin();
  vector<pose>::iterator iter_pose = poses2.begin();
  count = 0;
  pose tempi, tempvv;
  double dist, dx, dy, dz;
  for (; iter_pose != poses2.end(); ++iter_pose, ++iter_reg, ++count)
  {
    feat.clear();
    attrib.clear();

    tempi = *iter_pose;
    feat.push_back(tempi.x);
    feat.push_back(tempi.y);
    feat.push_back(tempi.z);

    tempvv.x = iter_reg->x;
    tempvv.y = iter_reg->y;
    tempvv.z = iter_reg->z;

    int i = world_clusters.getClusters().closestCluster(feat);
    world_clusters.getClusterInfo(i, feat, attrib);
    ToWorldTest(tempi, tempo, w_2_r_avg[i]);
#ifdef NOISY2
    cout << "----------------------" << endl;
    cout << "in : ";
    tempi.print();
    cout << "out : ";
    tempo.print();
    cout << "truth : (" << tempvv.x << ", " << tempvv.y << ", " << tempvv.z << ")" << endl;
#endif
    dx = tempvv.x - tempo.x;
    dy = tempvv.y - tempo.y;
    dz = tempvv.z - tempo.z;
    //! Distance from ground truth
    dist = sqrt((dx*dx) + (dy*dy) + (dz*dz));
#ifdef NOISY2
    cout << "error : " << dist << endl;
#endif
    outfile << tempi.x << ", " << tempi.y << ", " << tempi.z << ", " << tempi.xr << ", " << tempi.yr << ", " << tempi.zr << ", "
            << tempo.x << ", " << tempo.y << ", " << tempo.z << ", " << tempo.xr << ", " << tempo.yr << ", " << tempo.zr << ", "
            << tempvv.x << ", " << tempvv.y << ", " << tempvv.z << ", " << dist << ", ";
    //! Distance from world
    dist = sqrt((tempvv.x * tempvv.x) + (tempvv.y * tempvv.y) + (tempvv.z * tempvv.z));
    outfile << dist << ", ";
    //! Distance from robot base
    dist = sqrt((tempi.x * tempi.x) + (tempi.y * tempi.y) + (tempi.z * tempi.z));
    outfile << dist << ", ";
    //! Distance from closest cluster
    dx = tempi.x - feat.at(0);
    dy = tempi.y - feat.at(1);
    dz = tempi.z - feat.at(2);
    dist = sqrt((dx*dx) + (dy*dy) + (dz*dz));
    outfile << dist << endl;
  }
  outfile.close();
  

  /*
  double *fffeat = new double[6];
  double *wfeat = new double[3];

  //! Create a 3D surface mapping a R3 world coordinate space to a R3 robot coordinate space
  FeedForward<TanhNeuron> ffnet(3, 500, 3);
  ffnet.initialize();
  */
  /*
  cout << "add training set" << endl;
  ffnet.addTrainingSet(fffeat, wfeat);

  double err = ffnet.trainNetwork(100);
  */
} // main