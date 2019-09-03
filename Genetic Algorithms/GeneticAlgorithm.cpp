///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Genetic Algorithms
//  Workfile:        GeneticAlgorithm.cpp
//  Revision:        1.0 - 10 March, 2008
//                   1.1 - 27 January, 2009 : Added dissertation-specific
//                         sections for helper function utilization
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Genetic algorithm class definition.
//
///////////////////////////////////////////////////////////////////////////////


#include "GeneticAlgorithm.h"
#include "SimulatorInterface.h"
//#include "ABBInterface.h"
#include "../Libraries/Math/VectorMath.h"

//! JAM - this is for debugging purposes ONLY!
//! The code within the confines of LOGGENE does not belong in this project
//! file since it is centered around the ABB assembly project
//! COMMENT OUT BEFORE DISTRIBUTING
//#define LOGGENE

//! JAM - This is for dissertation-specific code modifications
//! The code within the confines of DISSERTATION is not part of the genetic
//! algorithm implementation, but is special code integrated for research
//! purposes.
//! COMMENT OUT BEFORE DISTRIBUTING
#define DISSERTATION // runs dissertation-specific code
#define BATCHPROCESS // process training as a batch-mode operation rather than in-line
#define PROBTRUST

//! Explicit instantiations
//template class LIBRARY_API
// GeneticAlgorithms::GeneticAlgorithm<ABBGene, ABBResult>;
//template class LIBRARY_API
// GeneticAlgorithms::GeneticAlgorithm<SimGene, SimResult>;
template class LIBRARY_API
 GeneticAlgorithms::GeneticAlgorithm<GAGene, GAResult>;

using namespace Math;

namespace GeneticAlgorithms
{
  template <class genes, class results>
    GeneticAlgorithm<genes, results>::GeneticAlgorithm () :
      fitnessFnc_(NULL),
      mutationFnc_(NULL),
      testFnc_(NULL),
      helperFnc_(NULL),
      useHelper_(false)
  {
  }
    

  template <class genes, class results>
    GeneticAlgorithm<genes, results>::GeneticAlgorithm (
        double (*fFnc)(vector<results>&),
        void (*mFnc)(vector<genes>&, vector<vector<genes> >&),
        void (*tFnc)(vector<genes>&, vector<results>&, GenData&)) :
      fitnessFnc_(fFnc),
      mutationFnc_(mFnc),
      testFnc_(tFnc),
      useHelper_(false)
  {
  }


  template <class genes, class results>
    GeneticAlgorithm<genes, results>::~GeneticAlgorithm ()
  {
  }

  /*
  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::setLoggers (
      void (*geneFnc)(PrivateLogger* pl, genes* g, GenData& gd, bool first),
      void (*resultFnc)(PrivateLogger* pl, results* r, GenData& gd, bool first))
  {
    logGeneFnc_ = geneFnc;
    logResultFnc_ = resultFnc;
  }
  */

  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::setHelper (void (*fnc)(vector<vector<genes> >&,
                                                                  vector<vector<results> >&,
                                                                  bool))
  {
    helperFnc_ = fnc;
  }

  
  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::help (vector<vector<genes> > &gene,
                                                 vector<vector<results> > &result,
                                                 bool train)
  {
    if (helperFnc_ != NULL)
    {
      helperFnc_ (gene, result, train);
    }
  }


  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::useHelper (bool val)
  {
    useHelper_ = val;
  }


  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::setFitness (double (*fnc)(vector<results>&))
  {
    fitnessFnc_ = fnc;
  }


  template <class genes, class results>
    double GeneticAlgorithm<genes, results>::fitness (vector<results> &result)
  {
    if (fitnessFnc_ != NULL)
    {
      return fitnessFnc_ (result);
    }
    else
    {
      return -1.0;
    }
  }


  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::setMutation (void (*fnc)(vector<genes>&,
                                                                    vector<vector<genes> >&))
  {
    mutationFnc_ = fnc;
  }


  template <class genes, class results> 
    void GeneticAlgorithm<genes, results>::mutate (vector<vector<genes> > &population)
  {
    if (mutationFnc_ != NULL)
    {
      mutationFnc_ (parent_, population);
    }
  }


  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::setSubPop (
      void (*fnc)(vector<vector<genes> >&,
                  vector<vector<results> >&,
                  vector<vector<genes> >&,
                  int))
  {
    subPopFnc_ = fnc;
  }


  template <class genes, class results> 
    void GeneticAlgorithm<genes, results>::subPop (
      vector<vector<genes> > &inPop,
      vector<vector<results> > &inResult,
      vector<vector<genes> > &outPop,
      int randomChildren)
  {
    if (subPopFnc_ != NULL)
    {
      subPopFnc_ (inPop, inResult, outPop, randomChildren);
    }
  }


  template <class genes, class results> 
    void GeneticAlgorithm<genes, results>::setExecution (void (*fnc)(vector<genes>&,
                                                                     vector<results>&,
                                                                     GenData&))
  {
    testFnc_ = fnc;
  }


  template <class genes, class results>
    void GeneticAlgorithm<genes, results>::test (vector<genes>& gene,
                                                 vector<results>& outcome,
                                                 GenData& genInfo)
  {
    if (testFnc_ != NULL)
    {
      testFnc_ (gene, outcome, genInfo);
    }
  }


  template <class genes, class results>
    double GeneticAlgorithm<genes, results>::execute (vector<genes> &inGenes,
                                                      vector<genes> &bestGenes,
                                                      vector<results> &bestResults,
                                                      int id,
                                                      int generation)
  {
    static ofstream lgr("GALog.txt");
    vector<vector<genes> > geneVec, *geneVecPtr;
    vector<vector<genes> >::iterator geneIter;
    vector<results> result;
    static double output = 0, lastout = 0, bestOut = -1;
    static int counter = 0;
    GenData geneInfo;

#ifdef DISSERTATION
    vector<vector<results> > resultInVec, resultOutVec;
    vector<vector<genes> > geneSubVec, geneInVec;

#ifdef PROBTRUST
    static ofstream gdout("GoodOutput.txt");
    static double lastBest = -1.0f;
    static int maxSubPop = 100;
    static int predGood;
    static int actGood;
    static int rndChild = 0;
#endif
#endif

#ifdef LOGGENE
    vector<genes>::iterator gIter;
    vector<genes>::pointer gPtr;
    vector<results>::iterator rIter;
    vector<results>::pointer rPtr;
    static unsigned int x;
    static char buffer[64];
    static bool first = true;
#endif

    //! Set the input Gene as the parent
    parent_.clear();
    parent_.assign (inGenes.begin(), inGenes.end());

    //! Copy the input Gene as the current best gene
    bestGenes.clear();
    bestGenes.assign (inGenes.begin(), inGenes.end());

    geneInfo.generation = generation;
    geneInfo.parent = id;
    geneInfo.identity = -1; //! parent is not a child

#ifdef LOGGENE
    if (first)
    {
      geneLogger_ = new PrivateLogger*[inGenes.size()];
      resultLogger_ = new PrivateLogger*[inGenes.size()];

      for (x = 0; x < inGenes.size(); ++x)
      {
        sprintf (buffer, "GeneLog_%d", x);
        geneLogger_[x] = new PrivateLogger(buffer);
        logGeneFnc_(geneLogger_[x], &inGenes.at(x), geneInfo, true);

        sprintf (buffer, "ResultLog_%d", x);
        resultLogger_[x] = new PrivateLogger(buffer);
        logResultFnc_(resultLogger_[x], NULL, geneInfo, true);
      }
      first = false;
    }
#endif

    //! Rerun the parent gene at every iteration
    test (bestGenes, result, geneInfo);
    bestOut = fitness (result);
    bestResults.clear();
    bestResults.assign (result.begin(), result.end());
    counter = 0;
    geneInfo.score = bestOut;

#ifdef DISSERTATION
#ifdef BESTONLY
    resultInVec.clear();
    resultInVec.push_back (bestResults);
    geneInVec.clear();
    geneInVec.push_back (bestGenes);

    //! Train internal model
    help (geneInVec, resultInVec, true);
#endif
#endif

#ifdef LOGGENE
    //! Log the parent gene
    x = 0;
    //gIter = bestGenes.begin();
    gPtr = &bestGenes[0];
    rIter = result.begin();
    rPtr = &result[0];
    for (; rIter != result.end(); ++rIter, ++gPtr, ++rPtr, ++x)
    {
      //! Log the gene and result information
      //logGeneFnc_(geneLogger_[x], gIter, geneInfo, false);
      logGeneFnc_(geneLogger_[x], gPtr, geneInfo, false);
      //logResultFnc_(resultLogger_[x], rIter, geneInfo, false);
      logResultFnc_(resultLogger_[x], rPtr, geneInfo, false);
    } // for (; rIter != result.end(); ++rIter, ++gIter, ++x)
#endif

    if (testFnc_ != NULL && mutationFnc_ != NULL && fitnessFnc_ != NULL)
    {
      //! Generate a new population of genes for testing
      mutate (geneVec);
      counter = 0;

#ifdef DISSERTATION
      if (useHelper_)
      {
        //! JAM - Dissertation-specific code
        if (helperFnc_ != NULL)
        {
          //! Evaluate the potential gene
          geneIter = geneVec.begin ();

          resultOutVec.clear();

          for (; geneIter != geneVec.end (); ++geneIter)
          {
            resultInVec.clear();
            resultInVec.push_back (result);
            geneInVec.clear();
            geneInVec.push_back (*geneIter);

            //! Run helper function for evaluation on the prospective gene
            try
            {
              help (geneInVec, resultInVec, false);
            }
            catch(...)
            {
              lgr << "Error when assessing potential genes" << endl;
            }
//            help (*geneIter, result, false);
            //! Add results to vector for analysis
            resultOutVec.push_back (resultInVec.at(0));
          }

          //! Sort result vector for analysis and select top-performing
          //! sub-population
#ifdef PROBTRUST
          rndChild = (rndChild >= 0) ? rndChild : 0;
          subPop (geneVec, resultOutVec, geneSubVec, rndChild);
          maxSubPop = geneSubVec.size();
          predGood = (maxSubPop - rndChild);
          actGood = 0;
#else
          subPop (geneVec, resultOutVec, geneSubVec, 0);
#endif
        } // if (helperFnc_ != NULL)
        geneVecPtr = &geneSubVec;
      } // if (testFnc_ != NULL && mutationFnc_ != NULL && fitnessFnc_ != NULL)
      else
      {
        geneVecPtr = &geneVec;
      }
      geneInVec.clear();
      resultInVec.clear();
#else
      geneVecPtr = &geneVec;
#endif // DISSERTATION

      geneIter = geneVecPtr->begin();
      for (; geneIter != geneVecPtr->end(); ++geneIter, ++counter)
      {
        geneInfo.identity = counter;

#ifdef LOGGENE
        x = 0;
        gIter = geneIter->begin();
        for (; gIter != geneIter->end(); ++gIter, ++x)
        {
          gPtr = &gIter[0];
          //logGeneFnc_(geneLogger_[x], gIter, geneInfo, false);
          logGeneFnc_(geneLogger_[x], gPtr, geneInfo, false);
        } // for (; rIter != result.end(); ++rIter, ++gIter, ++x)
#endif
        ///////////////////////////////////////////////////////////////////////
        //      Run the test implementation function defined by the user     //
        ///////////////////////////////////////////////////////////////////////
        test (*geneIter, result, geneInfo);

#ifdef DISSERTATION
#ifndef BESTONLY
#ifdef BATCHPROCESS
        resultInVec.push_back (result);
        geneInVec.push_back (*geneIter);
#else
        resultInVec.clear();
        resultInVec.push_back (result);
        geneInVec.clear();
        geneInVec.push_back (*geneIter);

        //! Train internal model
        help (geneInVec, resultInVec, true);
#endif // ifdef BATCHPROCESS ... else
#endif // ifndef BESTONLY
#endif // ifdef DISSERTATION
        lastout = output;
        output = fitness (result);
        geneInfo.score = output;

#ifdef PROBTRUST
        if (counter < predGood && counter > 0)
        {
          //! resulting output was less than the output of the previous gene
          //! (i.e. model ordered the values correctly)
          actGood += (output < lastout) ? 1 : 0;
        }
#endif

#ifdef LOGGENE
        x = 0;
        rIter = result.begin();
        rPtr = &result[0];
        for (; rIter != result.end(); ++rIter, ++rPtr, ++x)
        {
          //logResultFnc_(resultLogger_[x], rIter, geneInfo, false);
          logResultFnc_(resultLogger_[x], rPtr, geneInfo, false);
        } // for (; rIter != result.end(); ++rIter, ++gIter, ++x)
#endif

        if (output > bestOut)
        {
          bestOut = output;
          bestGenes.clear();
          bestGenes.assign (geneIter->begin(), geneIter->end());

          bestResults.clear();
          bestResults.assign (result.begin(), result.end());
        }
      } // for (; geneIter != geneVecPtr->end(); ++geneIter, ++counter)

#ifdef DISSERTATION
#ifdef BATCHPROCESS
      //! Train internal model
      help (geneInVec, resultInVec, true);
#endif
#ifdef BESTONLY
      resultInVec.clear();
      resultInVec.push_back (bestResults);
      geneInVec.clear();
      geneInVec.push_back (bestGenes);

      //! Train internal model
      help (geneInVec, resultInVec, true);
#endif
#endif

#ifdef PROBTRUST
      ++actGood; //! maximum value = predgood-1, so need to allow for 
      maxSubPop = geneVecPtr->size();
      rndChild += ((predGood - actGood) > 0) ? (predGood - actGood) : -1;//maxSubPop - (int)(((double)actGood / (double)predGood) * maxSubPop);
      rndChild = rndChild > (maxSubPop / 2) ? (maxSubPop / 2) : rndChild;
      lastBest = bestOut;
      gdout << maxSubPop << ", " << predGood << ", " << actGood << ", " << rndChild << endl;
#endif

      return bestOut;
    } // if (testFnc_ != NULL && mutationFnc_ != NULL && fitnessFnc_ != NULL)
    else
    {
      //! One or more requisite functions are not defined, execution of GA
      //! is a failure.
      return 0;
    }
  }

}