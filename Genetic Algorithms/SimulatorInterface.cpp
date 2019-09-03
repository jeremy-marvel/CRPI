///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Genetic Algorithms
//  Workfile:        SimulatorInterface.cpp
//  Revision:        1.0 - 13 January, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Simulator-specific implementation class definition of genetic algorithm
//  mutation, execution, analysis, and support methods.
///////////////////////////////////////////////////////////////////////////////

#include "SimulatorInterface.h"
#include "../Libraries/Math/NumericalMath.h"
#include "../Libraries/Math/VectorMath.h"
#include <time.h>

using namespace Math;

namespace GeneticAlgorithms
{
  LIBRARY_API SimInterface::SimInterface() :
    settings_(NULL),
    communicator_(NULL),
    populationSize_(defaultPopulationSize),
    useRandomOffset_(false)
  {
  }


  LIBRARY_API SimInterface::SimInterface (networkSettings *settings,
                                          GAComm *comm) :
    settings_(settings),
    communicator_(comm),
    populationSize_(defaultPopulationSize),
    subpopulationSize_(defaultSubpopulationSize),
    useRandomOffset_(false)
  {
  }


  LIBRARY_API SimInterface::~SimInterface()
  {
    settings_ = NULL;
    communicator_ = NULL;
  }


  LIBRARY_API void SimInterface::setPopulationSize (int pop, int subpop)
  {
    populationSize_ = pop;
    subpopulationSize_ = subpop;
  }


  LIBRARY_API void SimInterface::setAddRandomOffset (bool addOffset)
  {
    useRandomOffset_ = addOffset;
  }


  LIBRARY_API void SimInterface::setNumberChildIterations (int iter)
  {
    iterations_ = iter;
  }


  LIBRARY_API void SimInterface::setMaxOutput (double val)
  {
    maxOutput_ = val;
  }


  LIBRARY_API double SimInterface::fitness (vector<GAResult> &param)
  {
    //! Set these variables to be static since this function is going to be
    //! called several times
    vector<GAResult>::iterator iter;
    static double sum;
    static GAResult temp, total;

    try
    {
      sum = 0.0;
      total.output = 0.0;

      for (iter = param.begin(); iter != param.end(); ++iter)
      {
/*
        total.output += (iter->output < 0.0) ? maxOutput_ : iter->output;

        temp.output = (iter->output > maxOutput_ || iter->output < 0.0) ?
                      0.0 : (maxOutput_ - iter->output);
        temp.output /= maxOutput_;

        //! Define the fitness as the weighted sum of squares
        sum += (iter->outWeight * (temp.output * temp.output));
*/
        sum += iter->output;
      }
    }
    catch (...)
    {
      exception ("SimFitness", "Unknown error");
    }

    //! If the assembly took longer than the default maximum length of time or
    //! exceeded the maximum allowed force, return a score of 0 (failure)
    if (total.output > maxOutput_)
    {
      return 0.0;
    }
    return sum;
  }


  LIBRARY_API void SimInterface::setMutationProbs (vector<GAGene> &probs)
  {
    vector<GAGene>::iterator iter;
    mutationProbs_.clear();
    for (iter = probs.begin(); iter != probs.end(); ++iter)
    {
      mutationProbs_.push_back (*iter);
    }
  }

  LIBRARY_API void SimInterface::getMutationProbs (vector<GAGene> &probs)
  {
    vector<GAGene>::iterator iter;
    probs.clear();
    for (iter = mutationProbs_.begin(); iter != mutationProbs_.end(); ++iter)
    {
      probs.push_back (*iter);
    }
  }

  LIBRARY_API void SimInterface::mutation (vector<GAGene>& geneIn,
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
          if (!iter->readOnly)
          {
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
          } // if (!iter->readOnly)
          tempSeq.push_back (gTemp);
        } // for (iter = geneIn.begin(); iter != geneIn.end(); ++iter)
        genesOut.push_back (tempSeq);
      } // for (count = 0; count < populationSize_; ++count)
    } // try
    catch (...)
    {
      exception ("SimMutation", "Unknown error");
    }
  }


  LIBRARY_API void SimInterface::execute (vector<GAGene> &geneIn,
                                          vector<GAResult> &resultsOut,
                                          GenData& geneInfo)
  {
    static double rndX[] = {3.0, 3.0, -3.0, -3.0},
                  rndY[] = {3.0, -3.0, -3.0, 3.0};
    vector<GAGene>::iterator gIter;
    static GAResult rTemp;
    static GAGene gTemp;
    static vector <vector<GAResult> > rvTemp;
    static vector <double> outs;
    bool flag = false, okay = false;
    static double rx, ry, rt, outTemp;
    int x, xiter, y, size, count, q = 0;

    rvTemp.clear();
    rvTemp.resize(iterations_);

    if (communicator_ != NULL)
    {
      try
      {
        WaitForSingleObject (communicator_->commMutex, INFINITE);
        communicator_->curOutAvg = communicator_->curOutStdev = 0.0;
        ReleaseMutex (communicator_->commMutex);
        for (x = 0; x < iterations_; ++x)
        {
          okay = false;
          while (!okay)
          {
            gIter = geneIn.begin();
            WaitForSingleObject (communicator_->commMutex, INFINITE);
            communicator_->curIter = x;
            ReleaseMutex (communicator_->commMutex);

            //! Execute the gene sequence
            outTemp = 0.0;
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
                ReleaseMutex (communicator_->commMutex);
                Sleep(100); //! Wait for 1/10 of a second before polling again
              }

              {
                okay = true;
                rvTemp[x].push_back (rTemp);
              } // if (retry) ... else
            } // for (; gIter != geneIn.end(); ++gIter)


            {
              //! Compute the current average assembly time and standard deviation for
              //! the entire assembly attempt
              //! xiter - iteration #
              //! y - stage
              WaitForSingleObject(communicator_->commMutex, INFINITE);

              //! Write all raw gene and result information to disk
              dumpRawRecord (geneIn, rvTemp[x], geneInfo);

              communicator_->curOutAvg = 0.0;
              outs.clear();
              for (xiter = 0; xiter <= x; ++xiter)
              {
                outTemp = 0.0;
                size = rvTemp[xiter].size();
                for (y = 0; y < size; ++y)
                {
                  outTemp += rvTemp[xiter][y].output;
                }

//                if (outTemp > 0.0)
                {
                  communicator_->curOutAvg += outTemp;
                  outs.push_back (outTemp);
                }
              }
              //! xiter == x+1
              //! Current average
              communicator_->curOutAvg /= (double)(outs.size());
              outTemp = 0.0;
              for (y = 0; y < xiter; ++y)
              {
                outTemp += (outs[y] - communicator_->curOutAvg) *
                           (outs[y] - communicator_->curOutAvg);
              }
              outTemp /= (double)(outs.size());
              communicator_->curOutStdev = sqrt(outTemp);

              for (xiter = 0; xiter <= x; ++xiter)
              {
                size = rvTemp[xiter].size();
                for (y = 0; y < size; ++y)
                {
                  rvTemp[xiter][y].outStdev = communicator_->curOutStdev;
                }
              }
              ReleaseMutex (communicator_->commMutex);
              communicator_->genePtr = NULL;
              communicator_->resultPtr = NULL;
              okay = true;
            } // reset

          } // while (!okay)
        } // (for x = 0; x < iterations_; ++x)
        resultsOut.clear();
        size = rvTemp[0].size();

        //! curTimeStdev contains the standard deviation of assembly times for all stages
        //! over all training iterations
        rTemp.outStdev = communicator_->curOutStdev;
        //! Compute the average time and force per stage
        //! x - iteration
        //! y - stage
        for (y = 0; y < size; ++y)
        {
          rTemp = rvTemp[0][y];
          count = 1;
          for (x = 1; x < iterations_; ++x)
          {
            //if (rvTemp[x][y].output >= 0.0)
            {
              rTemp += rvTemp[x][y];
              ++count;
            }
          }
          rTemp /= (double)(count);
          resultsOut.push_back (rTemp);
        }
      }
      catch (...)
      {
        exception ("execute", "Unknown error");
      }
    }
    else
    {
      /*
      logger_->error("Attempted to execute GA test function without a \
                     viable communication pathway");
      */
    }
  }


  LIBRARY_API void SimInterface::loadGene (const char *fname,
                                           vector<GAGene> &geneIn)
  {
    ifstream in (fname);
    int i, x, y, version;
    GAGene gtemp;

    geneIn.clear();

    if (!in)
    {
//      exception ("SimLoadGene", "File access error");
      return;
    }

    try
    {
      in >> version >> i;
      if (version != gtemp.version)
      {
//        exception ("SimLoadGene", "Attempting to load incompatible gene version");
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
//      exception ("SimLoadGene", "Unknown error");
    }
    in.close();
  }


  LIBRARY_API void SimInterface::saveGene (const char *fname,
                                           const vector<GAGene>& geneOut)
  {
    ofstream out (fname);
    vector<GAGene>::const_iterator iter = geneOut.begin();
    int x;

    if (!out)
    {
//      exception ("SimSaveGene", "File write error");
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
//      exception ("SimSaveGene", "Unknown error");
    }
    out.close();
  }


  LIBRARY_API void SimInterface::logGene (GAGene* gn,
                                          GenData& gd,
                                          bool first)
  {
    static char buffer[256];
    static double stageTemp = 0.0;

    if (first)
    {
      sprintf (buffer, ", generation, parent, child, P0, P1, P2, P3, P4");
//      pl->log (buffer);
    } // if (first)
    else
    {
      {
        //! Search parameter output
        //! sprintf (buffer, ", generation, parent, child, P0, P1, P2, P3, P4");
        sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f, %f",
                 gd.generation,
                 gd.parent,
                 gd.identity,
                 gn->searchParam[SIMP0],
                 gn->searchParam[SIMP1],
                 gn->searchParam[SIMP2],
                 gn->searchParam[SIMP3],
                 gn->searchParam[SIMP4]);
//        pl->log (buffer);
      }
    } // if (first) ... else
  }


  LIBRARY_API void SimInterface::logResult (GAResult* rs,
                                            GenData& gd,
                                            bool first)
  {
    static char buffer[128];

    if (first)
    {
      sprintf (buffer, ", generation, parent, child, output, stdev, actual, score");
//      pl->log (buffer);
    } // if (first)
    else
    {
      sprintf (buffer, ", %d, %d, %d, %f, %f, %f, %f",
               gd.generation,
               gd.parent,
               gd.identity,
               rs->output,
               rs->outStdev,
               rs->actual,
               gd.score);
//      pl->log (buffer);
    } // if (first) ... else
  }


  LIBRARY_API void SimInterface::dumpRawRecord (vector<GAGene> &gn,
                                                vector<GAResult>& rs,
                                                GenData& gd)
  {
    static bool first = true;
    static char tbuf[256], buffer[1024], buffer2[1024];
    static double stageTemp = 0.0;
    static unsigned int x;
    static double outTemp;

    sprintf (buffer, "");
    sprintf (buffer2, ""); 

    if (first)
    {
      sprintf (buffer, ", generation, parent, child");
      for (x = 0; x < gn.size(); ++x)
      {
        sprintf (tbuf, ", stage, P0, P1, P2, P3, P4");
        sprintf (buffer2, "%s%s", buffer, tbuf);
        sprintf (buffer, "%s", buffer2);
      } // for (x = 0; x < gn.size(); ++x)
      for (x = 0; x < gn.size(); ++x)
      {
        sprintf (tbuf, ", output");
        sprintf (buffer2, "%s%s", buffer, tbuf);
        sprintf (buffer, "%s", buffer2);
      }
      sprintf (buffer2, "%s, tot_output", buffer);
      sprintf (buffer, "%s", buffer2);
      //pl.log (buffer);
      first = false;
    } // if (first)

    sprintf (buffer2, "");
    sprintf (buffer, ", %d, %d, %d", gd.generation, gd.parent, gd.identity);
    for (x = 0; x < gn.size(); ++x)
    {
      //! sprintf (tbuf, ", stage, P0, P1, P2, P3, P4");
      sprintf (tbuf, ", %d, %f, %f, %f, %f, %f",
               x,
               gn.at(x).searchParam[SIMP0],
               gn.at(x).searchParam[SIMP1],
               gn.at(x).searchParam[SIMP2],
               gn.at(x).searchParam[SIMP3],
               gn.at(x).searchParam[SIMP4]);

      sprintf (buffer2, "%s%s", buffer, tbuf);
      sprintf (buffer, "%s", buffer2);
    } // for (x = 0; x < gn.size(); ++x)

    //! Print results
    //! sprintf (tbuf, ", output");
    outTemp = 0.0f;
    for (x = 0; x < rs.size(); ++x)
    {
      sprintf (tbuf, ", %f", rs.at(x).output);
      outTemp += rs.at(x).output;
      sprintf (buffer2, "%s%s", buffer, tbuf);
      sprintf (buffer, "%s", buffer2);
    }
    //! sprintf (buffer2, "%s, score", buffer);
    sprintf (buffer2, "%s, %f", buffer, outTemp);
    sprintf (buffer, "%s", buffer2);
    //pl.log (buffer);
  }


  LIBRARY_API void SimInterface::selectSubpop (vector<vector<GAGene> > &inPop,
                                               vector<vector<GAResult> > &inResult,
                                               vector<vector<GAGene> > &outPop,
                                               int randomChildren)
  {
    double outVal;
    int p, p1, c;
    vector<vector<GAResult> >::iterator outerIter;
    vector<double> outs;
    vector<int> indexes;

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
      exception ("SimInterface::selectSubpop", "Error in accessing and computing result vector");
    }
    indexes.resize (outs.size());

    try
    {
      //! Sort the results...
      mergeSort (outs, indexes);
    }
    catch (...)
    {
      exception ("SimInterface::selectSubpop", "Error in sorting result vector");
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
      for (c = 0; c < randomChildren; ++c)
      {
        p = rand() % inPop.size();
        outPop.push_back (inPop.at (indexes.at(p)));
      }
    }
    catch (...)
    {
      exception ("SimInterface::selectSubpop", "Error in creating gene sequence subpopulation");
    }

    outs.clear ();
    indexes.clear ();
  }


  LIBRARY_API void SimInterface::exception (char *where, char *what)
  {
    static char message[1024];
    sprintf (message, "Exception in \" %s \" : %s", where, what);
    //logger_->error (message);
  }
}