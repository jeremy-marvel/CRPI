///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Neural Tissue
//  Subsystem:       Genetic Algorithms
//  Workfile:        ABBInterface.cpp
//  Revision:        1.0 - 29 March, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  ABB-specific implementation class definition of genetic algorithm mutation,
//  execution, analysis, and support methods.
///////////////////////////////////////////////////////////////////////////////

#include "ABBInterface.h"
#include "..\Libraries\Math\NumericalMath.h"
#include "..\Libraries\Math\VectorMath.h"

//#define ABBNOISY

using namespace Math;

namespace GeneticAlgorithms
{
  LIBRARY_API ABBInterface::ABBInterface() :
    settings_(NULL),
    logger_(NULL),
    communicator_(NULL),
    populationSize_(defaultPopulationSize),
    subpopulationSize_(defaultSubpopulationSize),
    useRandomOffset_(false),
    deterministic_(false)
  {
  }


  LIBRARY_API ABBInterface::ABBInterface (networkSettings *settings,
                                          Logger *logPtr,
                                          GAComm *comm) :
    settings_(settings),
    logger_(logPtr),
    communicator_(comm),
    populationSize_(defaultPopulationSize),
    useRandomOffset_(false),
    deterministic_(false)
  {
  }


  LIBRARY_API ABBInterface::~ABBInterface()
  {
    settings_ = NULL;
    logger_ = NULL;
    communicator_ = NULL;
  }


  LIBRARY_API void ABBInterface::setPopulationSize (int pop, int subpop)
  {
    populationSize_ = pop;
    subpopulationSize_ = subpop;
  }


  LIBRARY_API void ABBInterface::setAddRandomOffset (bool addOffset)
  {
    useRandomOffset_ = addOffset;
  }


  LIBRARY_API void ABBInterface::setNumberChildIterations (int iter)
  {
    iterations_ = iter;
  }


  LIBRARY_API void ABBInterface::setMaxForce (double val)
  {
    maxForce_ = val;
  }


  LIBRARY_API void ABBInterface::setMaxTime (double val)
  {
    maxTime_ = val;
  }


  LIBRARY_API void ABBInterface::setMaxDistance (double val)
  {
    maxDistance_ = val;
  }


  LIBRARY_API double ABBInterface::fitness (vector<GAResult> &param)
  {
    //! Set these variables to be static since this function is going to be
    //! called several times
    vector<GAResult>::iterator iter;
    static double sum;
    static GAResult temp, total;
#ifdef ABBNOISY
    char str[256];
#endif
//    char str2[256];


    try
    {
      sum = 0.0;
      total.time = 0.0;
      total.force = 0.0;
      total.distance = 0.0;
      for (iter = param.begin(); iter != param.end(); ++iter)
      {
        total.force = (fabs(iter->force) > total.force) ?
                       fabs(iter->force) : total.force;
        //! Occasionally the controller will return invalid (negative) time
        //! stamps in the GSI packets.  Throw those packets away.
#ifdef ABBNOISY
        sprintf (str, "temptime : %f totalTime : %f maxTime : %f", iter->time, total.time, maxTime_);
        exception ("ABBInterface::fitness", str);
#endif
        total.time += (iter->time < 0.0) ? maxTime_ : iter->time;
        total.distance += iter->distance;

        temp.force = (iter->force > maxForce_) ?
                     0.0 : (maxForce_ - iter->force);
        temp.force /= maxForce_;
        temp.time = (iter->time > maxTime_ || iter->time < 0.0) ?
                    0.0 : (maxTime_ - iter->time);
        temp.time /= maxTime_;

        //! Define the fitness as the weighted sum of squares
        sum += (iter->timeWeight * (temp.time * temp.time)) +
               (iter->forceWeight * (temp.force * temp.force));
      }
    }
    catch (...)
    {
      exception ("ABBInterface::fitness", "Unknown error");
    }

    //! If the assembly took longer than the default maximum length of time or
    //! exceeded the maximum allowed force, return a score of 0 (failure)
    if (total.force > maxForce_ || total.time > maxTime_)
    {
//      sprintf (str2, "dist: %f, maxdist: %f", total.distance, maxDistance_);
//      exception ("ABBInterface::fitness", str2);
      if (maxDistance_ <= 0.00000001)
      {
        return 0.0;
      }
      return (total.distance / (10000.0 * maxDistance_));
    }
    return sum;
  }


  LIBRARY_API void ABBInterface::setMutationProbs (vector<GAGene> &probs)
  {
    vector<GAGene>::iterator iter;
    mutationProbs_.clear();
    for (iter = probs.begin(); iter != probs.end(); ++iter)
    {
      mutationProbs_.push_back (*iter);
    }
  }


  LIBRARY_API void ABBInterface::getMutationProbs (vector<GAGene> &probs)
  {
    vector<GAGene>::iterator iter;
    probs.clear();
    for (iter = mutationProbs_.begin(); iter != mutationProbs_.end(); ++iter)
    {
      probs.push_back (*iter);
    }
  }


  LIBRARY_API void ABBInterface::mutation (vector<GAGene>& geneIn,
                                           vector<vector<GAGene> >& genesOut)
  {
    vector<GAGene>::iterator iter, miter;
    static vector<GAGene> tempSeq;
    static GAGene gTemp;
    static int count, x, rval;
    static double rdec;
    static long seed = (long)time(NULL);

    genesOut.clear();

    try
    {
      for (count = 0; count < populationSize_; ++count)
      {
        tempSeq.clear();
        miter = mutationProbs_.begin();
        for (iter = geneIn.begin(); iter != geneIn.end(); ++iter, ++miter)
        {
          //! Insertion?
          //! TODO
          //! REMEMBER TO INSERT MUTATION PROBABILITY

          if (!iter->readOnly)
          {
            //! Deletion
            //! TODO
            //! REMEMBER TO REMOVE MUTATION PROBABILITY

            //! Mutation:
            gTemp = *iter;
            //gTemp.cp(*iter);
            for (x = 0; x < iter->params; ++x)
            {
              //! random # between 1 and 10000
              rval = rand() % 10000;
              rdec = (double)rval / 10000.0;

              //! if rval/10000 <= prob then
              //if (rdec <= mutationProbs_.searchParam[x])
              {
                //! Mutate value by random offset w/ mean 0 and variance 1
                //! Normal Gaussian distribution random #:
                //!   y = ax + b = N(b, a^2)
                //!   x = random # = N(0, 1) (0 mean, 1 variance)
                gTemp.searchParam[x] += (miter->searchParam[x] * gRand(&seed));
              }
            } // for (x = 0; x < iter->params; ++x)
            if (fabs(gTemp.searchParam[MOVEID]) < 0.5)
            {
              //! Special case for spiral search - # of turns must be >= 1
              gTemp.searchParam[ACSprTurns] *=
                sgn(gTemp.searchParam[ACSprTurns]);
              gTemp.searchParam[ACSprTurns] +=
                (gTemp.searchParam[ACSprTurns] < 1.0 ? 1.0 : 0.0);
            }
            else if (fabs(gTemp.searchParam[MOVEID]) < 1.5)
            {
              //! Special case for radial search
              gTemp.searchParam[ACRadRange] *= 
                sgn(gTemp.searchParam[ACRadRange]);
            }
          } // if (!iter->readOnly)
          tempSeq.push_back (gTemp);
        } // for (iter = geneIn.begin(); iter != geneIn.end(); ++iter)
        genesOut.push_back (tempSeq);
      } // for (count = 0; count < populationSize_; ++count)
    } // try
    catch (...)
    {
      exception ("ABBInterface::mutation", "Unknown error");
    }
  }


  LIBRARY_API void ABBInterface::detMutation (vector<GAGene>& geneIn,
                                              vector<vector<GAGene> >& genesOut)
  {
    vector<GAGene>::iterator iter, miter;
    static vector<GAGene> tempSeq;
    static GAGene gTemp;
    static int count, x, rval;
    static double rdec;
    static long seed = (long)time(NULL);

    deterministic_ = true;

    genesOut.clear();

/*
    //! analyze gene mutation sequence 
    //!   take note of all mutable elements, record range
    //!   for range, compute (children)/(# mutable elements)/(range)
*/

    try
    {
      for (count = 0; count < populationSize_; ++count)
      {
        tempSeq.clear();
        miter = mutationProbs_.begin();
        for (iter = geneIn.begin(); iter != geneIn.end(); ++iter, ++miter)
        {
          //! Insertion?
          //! TODO
          //! REMEMBER TO INSERT MUTATION PROBABILITY

          if (!iter->readOnly)
          {
            //! Deletion
            //! TODO
            //! REMEMBER TO REMOVE MUTATION PROBABILITY

            //! Mutation:
            gTemp = *iter;
            //gTemp.cp(*iter);
            for (x = 0; x < iter->params; ++x)
            {
              //! random # between 1 and 10000
              rval = rand() % 10000;
              rdec = (double)rval / 10000.0;

              //! if rval/10000 <= prob then
              //if (rdec <= mutationProbs_.searchParam[x])
              {
                //! Mutate value by random offset w/ mean 0 and variance 1
                //! Normal Gaussian distribution random #:
                //!   y = ax + b = N(b, a^2)
                //!   x = random # = N(0, 1) (0 mean, 1 variance)
                gTemp.searchParam[x] += (miter->searchParam[x] * gRand(&seed));
              }
            } // for (x = 0; x < iter->params; ++x)
            if (fabs(gTemp.searchParam[MOVEID]) < 0.5)
            {
              //! Special case for spiral search - # of turns must be >= 1
              gTemp.searchParam[ACSprTurns] *=
                sgn(gTemp.searchParam[ACSprTurns]);
              gTemp.searchParam[ACSprTurns] +=
                (gTemp.searchParam[ACSprTurns] < 1.0 ? 1.0 : 0.0);
            }
            else if (fabs(gTemp.searchParam[MOVEID]) < 1.5)
            {
              //! Special case for radial search
              gTemp.searchParam[ACRadRange] *= 
                sgn(gTemp.searchParam[ACRadRange]);
            }
          } // if (!iter->readOnly)
          tempSeq.push_back (gTemp);
        } // for (iter = geneIn.begin(); iter != geneIn.end(); ++iter)
        genesOut.push_back (tempSeq);
      } // for (count = 0; count < populationSize_; ++count)
    } // try
    catch (...)
    {
      exception ("ABBInterface::mutation", "Unknown error");
    }
  }


  LIBRARY_API void ABBInterface::execute (vector<GAGene> &geneIn,
                                          vector<GAResult> &resultsOut,
                                          GenData& geneInfo)
  {
    static double rndX[] = {3.0, 3.0, -3.0, -3.0},
                  rndY[] = {3.0, -3.0, -3.0, 3.0};
//    static const int iterations = 3;
    vector<GAGene>::iterator gIter;
    static GAResult rTemp, rCopy;
    static GAGene gTemp;
    static vector <vector<GAResult> > rvTemp;
    static vector <double> times;
    bool flag = false, retry = false, okay = false, allgood = true;
    static double rx, ry, rt, timeTemp, t1, t2;
    int x, xiter, count, q = 0;
    unsigned int y, size;

//    resultsOut.clear();
    rvTemp.clear();
    rvTemp.resize(iterations_);
    //! TODO:  Insert means of preempting execution in event of error
    //!        or intentional program quit
    if (communicator_ != NULL)
    {
      try
      {
        WaitForSingleObject (communicator_->commMutex, INFINITE);
        communicator_->curTimeAvg = communicator_->curTimeStdev = 0.0;
        ReleaseMutex (communicator_->commMutex);
        for (x = 0; x < iterations_; ++x)
        {
		      okay = false;
          while (!okay)
          {
            if (retry)
            {
              exception ("ABBInterface::execute", "retrying assembly");
            }
            retry = false;
            gIter = geneIn.begin();
            WaitForSingleObject (communicator_->commMutex, INFINITE);
            communicator_->curIter = x;
            ReleaseMutex (communicator_->commMutex);

            //! Execute the gene sequence
            timeTemp = 0.0;
            for (; gIter != geneIn.end(); ++gIter)
            {
              WaitForSingleObject (communicator_->commMutex, INFINITE);
              gTemp = *gIter;
//              gTemp.cp(*gIter);
              communicator_->genePtr = &gTemp;
              communicator_->resultPtr = &rTemp;
              communicator_->readyFlag = flag = true;
              ReleaseMutex (communicator_->commMutex);

              //! Wait until the assembly process has completed before returning
              while (flag)
              {
                WaitForSingleObject(communicator_->commMutex, INFINITE);
                flag = communicator_->readyFlag;
			          retry = (retry || (communicator_->connStatus == 0));
                if (retry)
                {
                  exception ("ABBInterface::execute", "connection lost");
                  flag = false;
                }
                ReleaseMutex (communicator_->commMutex);
                Sleep(100); //! Wait for 1/10 of a second before polling again
              }

              if (retry)
              {
                flag = true;
                while (flag)
                {
                  WaitForSingleObject(communicator_->commMutex, INFINITE);
                  flag = (communicator_->connStatus == 0);
                  ReleaseMutex (communicator_->commMutex);
                  Sleep (100);
                }
                exception ("ABBInterface::execute", "connection regained");
                break;
              }
              else
              {
                okay = true;
                WaitForSingleObject (communicator_->commMutex, INFINITE);
                rCopy = rTemp;
                ReleaseMutex (communicator_->commMutex);

                rvTemp[x].push_back (rCopy);
                timeTemp += rCopy.time;
                if (timeTemp > maxTime_ || rCopy.force > maxForce_)
                {
                  //! We've exceeded the maximum time limit or maximum force
                  //! Go ahead and abort, it will still get a score of 0.0
                  break;
                }
              } // if (retry) ... else
            } // for (; gIter != geneIn.end(); ++gIter)

            if (!retry)
            {
              //! Cue Reset & wait until ready
              WaitForSingleObject(communicator_->commMutex, INFINITE);
              communicator_->reset = flag = true;
              communicator_->readyFlag = true;
              communicator_->genePtr->searchParam[MOVEMETHOD] = -1;
              ReleaseMutex (communicator_->commMutex);
              while (flag)
              {
                WaitForSingleObject(communicator_->commMutex, INFINITE);
                flag = communicator_->reset;
                ReleaseMutex (communicator_->commMutex);
                Sleep (100); //! Wait for 1/10 of a second before polling again
              }

              //! Compute the current average assembly time and standard deviation for
              //! the entire assembly attempt
              //! xiter - iteration #
              //! y - stage
              WaitForSingleObject(communicator_->commMutex, INFINITE);

              //! Write all raw gene and result information to disk
              dumpRawRecord (geneIn, rvTemp[x], geneInfo);

              communicator_->curTimeAvg = 0.0f;
              times.clear();
              for (xiter = 0; xiter <= x; ++xiter)
              {
                timeTemp = 0.0;
                size = rvTemp[xiter].size();
                for (y = 0; y < size; ++y)
                {
                  timeTemp += rvTemp[xiter][y].time;
                }

                if (timeTemp > 0.0f)
                {
                  communicator_->curTimeAvg += timeTemp;
                  times.push_back (timeTemp);
                }
              }
              //! xiter == x+1
              //! Current average
              if (times.size() > 0)
              {
                communicator_->curTimeAvg /= (double)(times.size());
                timeTemp = 0.0;
                for (y = 0; y < times.size(); ++y)
                {
                  timeTemp += (times[y] - communicator_->curTimeAvg) *
                              (times[y] - communicator_->curTimeAvg);
                }
                timeTemp /= (double)(times.size());
                communicator_->curTimeStdev = sqrt(timeTemp);
              }

              for (xiter = 0; xiter <= x; ++xiter)
              {
                size = rvTemp[xiter].size();
                for (y = 0; y < size; ++y)
                {
                  rvTemp[xiter][y].timeStdev = communicator_->curTimeStdev;
                }
              }
              ReleaseMutex (communicator_->commMutex);

              //! Add random perturbation & wait until finished
              if (useRandomOffset_)
              {
                WaitForSingleObject(communicator_->commMutex, INFINITE);
                communicator_->readyFlag = flag = true;
                communicator_->genePtr->searchParam[MOVEMETHOD] = 0; // PC move
                communicator_->genePtr->searchParam[MOVEID] = 2; // Linear move
                rx = (double)(1000 - (rand() % 2000)) / 1000.0; // [-1.0, 1.0] X offset
                ry = (double)(1000 - (rand() % 2000)) / 1000.0; // [-1.0, 1.0] Y offset
                rt = (double)(1000 - (rand() % 2000)) / 250.0; // [-4.0, 4.0] Z rotation offset
                communicator_->genePtr->searchParam[PCXOffset] = rx;
                communicator_->genePtr->searchParam[PCYOffset] = ry;
                communicator_->genePtr->searchParam[PCZOffset] = 0.0;
                communicator_->genePtr->searchParam[PCXRot] = 0.0;//rndX[q];
                communicator_->genePtr->searchParam[PCYRot] = 0.0;//rndY[q];
                communicator_->genePtr->searchParam[PCZRot] = rt;
                q = (q + 1 < 4 ? q+1 : 0);
                ReleaseMutex (communicator_->commMutex);
                while (flag)
                {
                  WaitForSingleObject(communicator_->commMutex, INFINITE);
                  flag = communicator_->readyFlag;
                  ReleaseMutex (communicator_->commMutex);
                  Sleep (100); //! Wait for 1/10 of a second before polling again
                }
              } // if (useRandomOffset_)
              communicator_->genePtr = NULL;
              communicator_->resultPtr = NULL;
              okay = true;
            } // if (!retry)
            else
            {
              //! Clear this record and try again
              rvTemp[x].clear();
            }
          } // while (!okay)
        } // (for x = 0; x < iterations_; ++x)
        resultsOut.clear();

        //! Compute the average time and force per stage
        //! x - iteration
        //! y - stage
        count = 0;
        rCopy.time = 0.0f;
        rCopy.force = 0.0f;
        t1 = 0.0f;
        //! Compute average (final)
        for (x = 0; x < iterations_; ++x)
        {
          allgood = true;
          size = rvTemp[x].size();
          for (y = 0; y < size; ++y)
          {
            allgood &= (rvTemp[x][y].time >= 0.0f);
          }
          if (allgood && size > 0)
          {
            ++count;
            for (y = 0; y < size; ++y)
            {
              t1 += rvTemp[x][y].time;
            }
          }
        }
        //! Compute standard deviation (final)
        t1 /= (count > 0) ? (double)count : 1.0f;
        rCopy.time = t1;
        count = 0;
        t2 = 0.0f;
        for (x = 0; x < iterations_; ++x)
        {
          allgood = true;
          size = rvTemp[x].size();
          for (y = 0; y < size; ++y)
          {
            allgood &= (rvTemp[x][y].time >= 0.0f);
          }
          if (allgood && size > 0)
          {
            ++count;
            t1 = 0.0f;
            for (y = 0; y < size; ++y)
            {
              t1 += rvTemp[x][y].time;
            }
            t1 -= rCopy.time;
            t2 += (t1 * t1);
          }
        } // for (x = 0; x < iterations_; ++x)
        t1 = t2 / ((count > 0) ? (double)count : 1.0f);
        t2 = sqrt (t1);
        rCopy.timeStdev = t2;

        size = geneIn.size();
        for (y = 0; y < size; ++y)
        {
          resultsOut.push_back(rCopy);
        }

        /*
        for (y = 0; y < size; ++y)
        {
          rTemp = rvTemp[0][y];
          count = 0;
          for (x = 1; x < iterations_; ++x)
          {
            if (rvTemp[x][y].time >= 0.0)
            {
              rTemp += rvTemp[x][y];
              ++count;
            }
          }
          rTemp /= (double)(count);
          resultsOut.push_back (rTemp);
        }
        */
      }
      catch (...)
      {
        exception ("ABBInterface::execute", "Unknown error");
      }
    }
    else
    {
      logger_->error("Attempted to execute GA test function without a \
                     viable communication pathway");
    }
  }


  LIBRARY_API void ABBInterface::loadGene (const char *fname,
                                           vector<GAGene> &geneIn)
  {
    ifstream in (fname);
    int i, x, y, version;
    GAGene gtemp;

    geneIn.clear();

    if (!in)
    {
      exception ("ABBInterface::loadGene", "File access error");
      return;
    }

    try
    {
      in >> version >> i;
      if (version != gtemp.version)
      {
        exception ("ABBInterface::loadGene", "Attempting to load incompatible gene version");
        //! Error : incorrect gene version
      }
      else
      {
        for (x = 0; x < i; ++x)
        {
          for (y = 0; y < gtemp.params; ++y)
          {
            in >> gtemp.searchParam[y];
          }
          geneIn.push_back (gtemp);
        } // for (x = 0; x < i; ++x)
      }
    }
    catch (...)
    {
      exception ("ABBInterface::loadGene", "Unknown error");
    }
    in.close();
  }


  LIBRARY_API void ABBInterface::saveGene (const char *fname,
                                           const vector<GAGene>& geneOut)
  {
    ofstream out (fname);
    vector<GAGene>::const_iterator iter = geneOut.begin();
    int x;

    if (!out)
    {
      exception ("ABBInterface::saveGene", "File write error");
      return;
    }

    try
    {
      out << iter->version << " " << geneOut.size() << endl;
      while (iter != geneOut.end())
      {
        for (x = 0; x < iter->params; ++x)
        {
          out << iter->searchParam[x] << " ";
        }
        out << endl;
        ++iter;
      }
    }
    catch (...)
    {
      exception ("ABBInterface::saveGene", "Unknown error");
    }
    out.close();
  }


  LIBRARY_API void ABBInterface::logGene (PrivateLogger* pl,
                                          GAGene* gn,
                                          GenData& gd,
                                          bool first)
  {
    static char buffer[256];
    static double stageTemp = 0.0;

    if (first)
    {
      stageTemp = gn->searchParam[MOVEID] + 0.25;
      if ((int)stageTemp == SPIRAL)
      {
        //! Spiral search
        sprintf (buffer, ", generation, parent, child, z_force, z_depth, speed, radius, turns, timeout");
        pl->log (buffer);
      }
      else if ((int)stageTemp == RADIAL)
      {
        //! Radial search
        sprintf (buffer, ", generation, parent, child, z_force, z_depth, speed, range, hop_amp, hop_per, cir_speed, cir_rad, timeout");
        pl->log (buffer);
      }
      else
      {
        //! Linear move
        stageTemp = gn->searchParam[MOVEMETHOD] + 0.25;
        if ((int)stageTemp == PC)
        {
          sprintf (buffer, ", generation, parent, child, x_offset, y_offset, z_offset, x_rot, y_rot, z_rot, timeout");
        }
        else
        {
          sprintf (buffer, ", generation, parent, child, x_offset, y_offset, z_offset, x_force, y_force, z_force, timeout");
        }
        pl->log (buffer);
      }
    } // if (first)
    else
    {
      stageTemp = gn->searchParam[MOVEID] + 0.25;
      if ((int)stageTemp == SPIRAL)
      {
        //! Spiral search
        //! sprintf (buffer, ", generation, parent, child, z_force, z_depth,
        //!          speed, radius, turns, timeout");
        sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f",
                 gd.generation,
                 gd.parent,
                 gd.identity,
                 gn->searchParam[ACSprZForce],
                 gn->searchParam[ACSprZDepth],
                 gn->searchParam[ACSprSpeed],
                 gn->searchParam[ACSprRadius],
                 gn->searchParam[ACSprTurns],
                 gn->searchParam[ACSprTimeout]);
        pl->log (buffer);
      }
      else if ((int)stageTemp == RADIAL)
      {
        //! Radial search
        //! sprintf (buffer, ", generation, parent, child, z_force, z_depth,
        //           speed, range, hop_amp, hop_per, cir_speed, cir_rad");
        sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                 gd.generation,
                 gd.parent,
                 gd.identity,
                 gn->searchParam[ACRadZForce],
                 gn->searchParam[ACRadZDepth],
                 gn->searchParam[ACRadSpeed],
                 gn->searchParam[ACRadRange],
                 gn->searchParam[ACRadHopAmp],
                 gn->searchParam[ACRadHopPer],
                 gn->searchParam[ACRadCirSpeed],
                 gn->searchParam[ACRadCirRad],
                 gn->searchParam[ACRadTimeout]);
        pl->log (buffer);
      }
      else
      {
        stageTemp = gn->searchParam[MOVEMETHOD] + 0.25;
        //! Linear move
        if ((int)stageTemp == PC)
        {
          //sprintf (buffer, ", generation, parent, child, x_offset, y_offset,
          //         z_offset, x_rot, y_rot, z_rot");
          sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f",
                   gd.generation,
                   gd.parent,
                   gd.identity,
                   gn->searchParam[PCXOffset],
                   gn->searchParam[PCYOffset],
                   gn->searchParam[PCZOffset],
                   gn->searchParam[PCXRot],
                   gn->searchParam[PCYRot],
                   gn->searchParam[PCZRot]);
        }
        else if ((int)stageTemp == NAC)
        {
          //! TODO:  Insert code for NAC linear moves
        }
        else
        {
          //sprintf (buffer, ", generation, parent, child, x_offset, y_offset,
          //         z_offset, x_force, y_force, z_force");
          sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f, %f",
                   gd.generation,
                   gd.parent,
                   gd.identity,
                   gn->searchParam[ACLinXOffset],
                   gn->searchParam[ACLinYOffset],
                   gn->searchParam[ACLinZOffset],
                   gn->searchParam[ACLinXForce],
                   gn->searchParam[ACLinYForce],
                   gn->searchParam[ACLinZForce],
                   gn->searchParam[ACLinTimeout]);
        }
        pl->log (buffer);
      } // if ((int)stageTemp == SPIRAL) ... else
    } // if (first) ... else
  }


  LIBRARY_API void ABBInterface::logResult (PrivateLogger* pl,
                                            GAResult* rs,
                                            GenData& gd,
                                            bool first)
  {
    static char buffer[128];

    if (first)
    {
      sprintf (buffer, ", generation, parent, child, force, timeAvg, timeStdev, score");
      pl->log (buffer);
    } // if (first)
    else
    {
      sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f",
               gd.generation,
               gd.parent,
               gd.identity,
               rs->force,
               rs->time,
               rs->timeStdev,
               gd.score);
      pl->log (buffer);
    } // if (first) ... else
  }


  LIBRARY_API void ABBInterface::dumpRawRecord (vector<GAGene> &gn,
                                                vector<GAResult>& rs,
                                                GenData& gd)
  {
    static bool first = true;
    static char tbuf[256], buffer[1024], buffer2[1024];
    static double stageTemp = 0.0;
    static PrivateLogger pl("rawRecordDump");
    static unsigned int x;
    static double timeTemp;

    sprintf (buffer, "");
    sprintf (buffer2, ""); 

    if (first)
    {
      sprintf (buffer, ", generation, parent, child");
      for (x = 0; x < gn.size(); ++x)
      {
        stageTemp = gn.at(x).searchParam[MOVEID] + 0.25;
        if ((int)stageTemp == SPIRAL)
        {
          //! Spiral search
          sprintf (tbuf, ", stage, z_force, z_depth, speed, radius, turns");
        }
        else if ((int)stageTemp == RADIAL)
        {
          //! Radial search
          sprintf (tbuf, ", stage, z_force, z_depth, speed, range, hop_amp, hop_per, cir_speed, cir_rad");
        }
        else
        {
          //! Linear move
          stageTemp = gn.at(x).searchParam[MOVEMETHOD] + 0.25;
          if ((int)stageTemp == PC)
          {
            sprintf (tbuf, ", stage, x_offset, y_offset, z_offset, x_rot, y_rot, z_rot");
          }
          else
          {
            sprintf (tbuf, ", stage, x_offset, y_offset, z_offset, x_force, y_force, z_force");
          }
        }
        sprintf (buffer2, "%s%s", buffer, tbuf);
        sprintf (buffer, "%s", buffer2);
      } // for (x = 0; x < gn.size(); ++x)
      for (x = 0; x < gn.size(); ++x)
      {
        sprintf (tbuf, ", force, time");
        sprintf (buffer2, "%s%s", buffer, tbuf);
        sprintf (buffer, "%s", buffer2);
      }
      sprintf (buffer2, "%s, tot_time", buffer);
      sprintf (buffer, "%s", buffer2);
      pl.log (buffer);
      first = false;
    } // if (first)


    sprintf (buffer2, "");
    sprintf (buffer, ", %d, %d, %d", gd.generation, gd.parent, gd.identity);
    for (x = 0; x < gn.size(); ++x)
    {
      stageTemp = gn.at(x).searchParam[MOVEID] + 0.25;
      if ((int)stageTemp == SPIRAL)
      {
        //! Spiral search
        //! sprintf (tbuf, ", stage, z_force, z_depth, speed, radius, turns");
        sprintf (tbuf, ", %d, %f, %f, %f, %f, %f",
                 x,
                 gn.at(x).searchParam[ACSprZForce],
                 gn.at(x).searchParam[ACSprZDepth],
                 gn.at(x).searchParam[ACSprSpeed],
                 gn.at(x).searchParam[ACSprRadius],
                 gn.at(x).searchParam[ACSprTurns]);
      }
      else if ((int)stageTemp == RADIAL)
      {
        //! Radial search
        //! sprintf (tbuf, ", stage, z_force, z_depth, speed, range, hop_amp, hop_per, cir_speed, cir_rad");
        sprintf (tbuf, ", %d, %f, %f, %f, %f, %f, %f, %f, %f",
                 x,
                 gn.at(x).searchParam[ACRadZForce],
                 gn.at(x).searchParam[ACRadZDepth],
                 gn.at(x).searchParam[ACRadSpeed],
                 gn.at(x).searchParam[ACRadRange],
                 gn.at(x).searchParam[ACRadHopAmp],
                 gn.at(x).searchParam[ACRadHopPer],
                 gn.at(x).searchParam[ACRadCirSpeed],
                 gn.at(x).searchParam[ACRadCirRad]);
      }
      else
      {
        //! Linear move
        stageTemp = gn.at(x).searchParam[MOVEMETHOD] + 0.25;

        if ((int)stageTemp == PC)
        {
          //sprintf (tbuf, ", stage, x_offset, y_offset, z_offset, x_rot, y_rot, z_rot");
          sprintf (tbuf, ", %d, %f, %f, %f, %f, %f, %f",
                   x,
                   gn.at(x).searchParam[PCXOffset],
                   gn.at(x).searchParam[PCYOffset],
                   gn.at(x).searchParam[PCZOffset],
                   gn.at(x).searchParam[PCXRot],
                   gn.at(x).searchParam[PCYRot],
                   gn.at(x).searchParam[PCZRot]);
        }
        else if ((int)stageTemp == NAC)
        {
          //! TODO:  Insert code for NAC linear moves
        }
        else
        {
          //sprintf (tbuf, ", stage, x_offset, y_offset, z_offset, x_force, y_force, z_force");
          sprintf (tbuf, ", %d, %f, %f, %f, %f, %f, %f",
                   x,
                   gn.at(x).searchParam[ACLinXOffset],
                   gn.at(x).searchParam[ACLinYOffset],
                   gn.at(x).searchParam[ACLinZOffset],
                   gn.at(x).searchParam[ACLinXForce],
                   gn.at(x).searchParam[ACLinYForce],
                   gn.at(x).searchParam[ACLinZForce]);
        }
      } // if ((int)stageTemp == SPIRAL) ... else
      sprintf (buffer2, "%s%s", buffer, tbuf);
      sprintf (buffer, "%s", buffer2);
    } // for (x = 0; x < gn.size(); ++x)


    //! Print results
    //! sprintf (tbuf, ", force, time");
    timeTemp = 0.0f;
    for (x = 0; x < rs.size(); ++x)
    {
      sprintf (tbuf, ", %f, %f", rs.at(x).force, rs.at(x).time);
      timeTemp += rs.at(x).time;
      sprintf (buffer2, "%s%s", buffer, tbuf);
      sprintf (buffer, "%s", buffer2);
    }
    //! sprintf (buffer2, "%s, score", buffer);
    sprintf (buffer2, "%s, %f", buffer, timeTemp);
    sprintf (buffer, "%s", buffer2);
    pl.log (buffer);
  }


/*
  LIBRARY_API void ABBInterface::dumpRawRecord (GAGene* gn, GAResult *rs, GenData& gd)
  {
    static bool first = true;
    static char buffer[256];
    static double stageTemp = 0.0;
    static PrivateLogger pl("rawRecordDump");

    if (first)
    {
      stageTemp = gn->searchParam[MOVEID] + 0.25;
      if ((int)stageTemp == SPIRAL)
      {
        //! Spiral search
        sprintf (buffer, ", generation, parent, child, z_force, z_depth, speed, radius, turns, force, time, score");
        pl.log (buffer);
      }
      else if ((int)stageTemp == RADIAL)
      {
        //! Radial search
        sprintf (buffer, ", generation, parent, child, z_force, z_depth, speed, range, hop_amp, hop_per, cir_speed, cir_rad, force, time, score");
        pl.log (buffer);
      }
      else
      {
        //! Linear move
        sprintf (buffer, ", generation, parent, child, x_offset, y_offset, z_offset, x_rot, y_rot, z_rot, force, time, score");
        pl.log (buffer);
      }
      first = false;
    } // if (first)

    {
      stageTemp = gn->searchParam[MOVEID] + 0.25;
      if ((int)stageTemp == SPIRAL)
      {
        //! Spiral search
        //! sprintf (buffer, ", generation, parent, child, z_force, z_depth,
        //!          speed, radius, turns");
        sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f",
                 gd.generation,
                 gd.parent,
                 gd.identity,
                 gn->searchParam[ACSprZForce],
                 gn->searchParam[ACSprZDepth],
                 gn->searchParam[ACSprSpeed],
                 gn->searchParam[ACSprRadius],
                 gn->searchParam[ACSprTurns],
                 rs->force,
                 rs->time,
                 gd.score);
        pl.log (buffer);
      }
      else if ((int)stageTemp == RADIAL)
      {
        //! Radial search
        //! sprintf (buffer, ", generation, parent, child, z_force, z_depth,
        //           speed, range, hop_amp, hop_per, cir_speed, cir_rad");
        sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                 gd.generation,
                 gd.parent,
                 gd.identity,
                 gn->searchParam[ACRadZForce],
                 gn->searchParam[ACRadZDepth],
                 gn->searchParam[ACRadSpeed],
                 gn->searchParam[ACRadRange],
                 gn->searchParam[ACRadHopAmp],
                 gn->searchParam[ACRadHopPer],
                 gn->searchParam[ACRadCirSpeed],
                 gn->searchParam[ACRadCirRad],
                 rs->force,
                 rs->time,
                 gd.score);
        pl.log (buffer);
      }
      else
      {
        //! Linear move
        //sprintf (buffer, ", generation, parent, child, x offset, y offset,
        //         z offset");
        if ((int)stageTemp == PC)
        {
          sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f, %f",
                   gd.generation,
                   gd.parent,
                   gd.identity,
                   gn->searchParam[PCXOffset],
                   gn->searchParam[PCYOffset],
                   gn->searchParam[PCZOffset],
                   rs->force,
                   rs->time,
                   gd.score);
        }
        else if ((int)stageTemp == NAC)
        {
          //! TODO:  Insert code for NAC linear moves
        }
        else
        {
          sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f, %f, %f",
                   gd.generation,
                   gd.parent,
                   gd.identity,
                   gn->searchParam[ACLinXOffset],
                   gn->searchParam[ACLinYOffset],
                   gn->searchParam[ACLinZOffset],
                   rs->force,
                   rs->time,
                   gd.score);
        }
        pl.log (buffer);
      } // if ((int)stageTemp == SPIRAL) ... else
    } // if (first) ... else
  }
  */

  LIBRARY_API void ABBInterface::selectSubpop (vector<vector<GAGene> > &inPop,
                                               vector<vector<GAResult> > &inResult,
                                               vector<vector<GAGene> > &outPop,
                                               int randomChildren)
  {
    double outVal;
    unsigned int p, p1;
    unsigned int c;
    vector<vector<GAResult> >::iterator outerIter;
    vector<double> outs;
    vector<int> indexes;

    if (deterministic_)
    {
      outPop.clear();
      for (c = 0; c < inPop.size(); ++c)
      {
        outPop.push_back (inPop.at (indexes.at(c)));
      }

      return;
    }

    try
    {
      for (outerIter = inResult.begin(); outerIter != inResult.end(); ++outerIter)
      {
        outVal = fitness (*outerIter);
        outs.push_back (outVal);
      }
    }
    catch (...)
    {
      exception ("ABBInterface::selectSubpop", "Error in accessing and computing result vector");
    }
    indexes.resize (outs.size());

    try
    {
      //! Sort the results...
      mergeSort (outs, indexes);
    }
    catch (...)
    {
      exception ("ABBInterface::selectSubpop", "Error in sorting result vector");
    }

    outPop.clear ();
    
    try
    {
      //! ...And take the highest-scoring subpopulationSize_ gene sequences
      p = outs.size()-1;

      p1 = (subpopulationSize_ - randomChildren);
      p1 = (p1 >= 0) ? p1 : 0;
      //p = 0;
      for (c = 0; c < p1; ++c, --p)
      {
        outPop.push_back (inPop.at (indexes.at(p)));
      }
      for (c = 0; c < (unsigned int)randomChildren; ++c)
      {
        p = rand() % inPop.size();
        outPop.push_back (inPop.at (indexes.at(p)));
      }
    }
    catch (...)
    {
      exception ("ABBInterface::selectSubpop", "Error in creating gene sequence subpopulation");
    }

    outs.clear ();
    indexes.clear ();
  }


  LIBRARY_API void ABBInterface::exception (char *where, char *what)
  {
    static char message[1024];
    sprintf (message, "Exception in \" %s \" : %s", where, what);
    logger_->error (message);
  }
}