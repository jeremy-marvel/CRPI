///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Genetic Algorithms
//  Workfile:        GeneticAlgorithm.h
//  Revision:        1.0 - 10 March, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  A container class for performing genetic algorithm evolution.  Requires the
//  user to provide mutation, fitness, and test functions.  Given these, this
//  class will then automate the gene evolution process given the permitted
//  mutation parameters;
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GENETICALGORITHM_H
#define GENETICALGORITHM_H

#ifdef WIN32
#pragma warning (disable: 4251)
#endif

#include "../portable.h"
#include "GATypes.h"
#include <fstream>
#include <windows.h>
#include <vector>

using namespace std;

namespace GeneticAlgorithms
{

  LIBRARY_API enum {CROSSOVER = 1, INSERTION = 2, DELETION = 4, ADJUSTMENT = 8};

  //! @ingroup GeneticAlgorithms
  //!
  //! @brief Driver class implementation of a GESA-like genetic algorithm.
  //!
  //! @note The classes genes and results are class definitions of genetic
  //!       sequence definitions and test function outputs respectively, and
  //!       must be defined by the user.  It is recommended that structs be
  //!       used in order to ease the test, mutation, and fitness functions.
  //!
  template <class genes, class results> class LIBRARY_API GeneticAlgorithm
  {
  public:
    //! @brief Default constructor
    //!
    GeneticAlgorithm ();

    //! @brief Constructor
    //!
    //! @param fFnc Pointer to the fitness function
    //! @param mFnc Pointer to the mutation function
    //! @param tFnc Pointer to the test function
    //!
    GeneticAlgorithm (double (*fFnc)(vector<results>&),
                      void (*mFnc)(vector<genes>&, vector<vector<genes> >&),
                      void (*tFnc)(vector<genes>&, vector<results>&, GenData&));

    //! @brief Default destructor
    //!
    ~GeneticAlgorithm ();


    //! @brief Define the fitness function for use in the genetic algorithm
    //!        evolution process.
    //!
    //! @param fnc Pointer to the fitness function
    //!
    //! @note The declaration of the function passed into this method MUST
    //!       be in the following format:
    //!          double fncName (vector<struct>& param)
    //!       And must return a double value in the range [0, +INF], where
    //!       results of 0 indicate a gene failure, and non-zero values
    //!       indicate the "goodness" of the resulting success.
    //!
    void setFitness (double (*fnc)(vector<results>&));

    //! @brief Define the mutation function for use in the genetic algorithm assessment
    //!        process
    //!
    //! @param fnc Pointer to the mutation function
    //!
    //! @note The declaration of the function passed into this method MUST
    //!       be in the following format:
    //!          void fncName (vector<struct>& param)
    //!
    void setMutation (void (*fnc)(vector<genes>&, vector<vector<genes> >&));

    //! @brief Define the test/execution function for use in the genetic algorithm
    //!        analysis process
    //!
    //! @param fnc Pointer to the test function
    //!
    //! @note The declaration of the function passed into this method MUST
    //!       be in the following format:
    //!          void fncName (vector<struct1>& param1, vector<struct2>& param2)
    //!
    void setExecution (void (*fnc)(vector<genes>&, vector<results>&, GenData&));

    //! @brief Pass a function to the Genetic Algorithm for pre-execution helper
    //!        processing (JAM Dissertation function)
    //!
    //! @param fnc Pointer to a generic evaluation/simulation function
    //!
    void setHelper (void (*fnc)(vector<vector<genes> >&,
                                vector<vector<results> >&, bool));

    //! @brief Whether or not we should use the helper function for assessment
    //!
    //! @param val Switching value to enable (true) or disable (false) using the
    //!            helper function
    //!
    //! @note If DISSERTATION is not defined, the helper function is disabled
    //!
    void useHelper (bool val);

    //! @brief Handle used by the GA for calling the generalized helper function that
    //!        was provided to the GA
    //!
    //! @param gene   Collection of gene pools for evaluation by the helper function
    //! @param result Collection of gene pool results to be populated by the helper function
    //! @param train  Whether we are training (true) or evaluating (false)
    //!
    void help (vector<vector<genes> > &gene,
               vector<vector<results> > &result,
               bool train);


    //! @brief Pass a function to the Genetic Algorithm for sub-population
    //!        pruning (JAM Dissertation function)
    //!
    //! @param fnc Pointer to the sub-population pruning function
    //!
    void setSubPop (void (*fnc)(vector<vector<genes> >&,
                                vector<vector<results> >&,
                                vector<vector<genes> >&,
                                int));

    //! @brief Prune a genetic population based on an evaluation of its fitness for survival
    //!
    //! @param inPop    The original input population of gene sequences
    //! @param inResult The result vector computed by simulating/executing the
    //!                 input gene sequences
    //! @param outPop   The pruned population of gene sequences
    //!
    void subPop (vector<vector<genes> > &inPop,
                 vector<vector<results> > &inResult,
                 vector<vector<genes> > &outPop,
                 int randomChildren);

    //! @brief Evaluate a gene sequence
    //!
    //! @param inGenes     Population of gene sequences for evaluation
    //! @param bestGenes   Collection of the best-performing gene sequences
    //! @param bestResults Performance results of the best-performing genes
    //! @Param id          Parent strain identification number
    //! @param generation  Generation/brood number being generated
    //!
    //! @return The performance score of the best-performing child genes
    //!
    double execute (vector<genes> &inGenes,
                    vector<genes> &bestGenes,
                    vector<results> &bestResults,
                    int id,
                    int generation);

  private:

    //! @brief Call the fitness function defined by the user
    //!
    //! @param result Vector of raw performance measures to be evaluated against a fitness function
    //!
    //! @return The performance score of a given gene based on the fitness function
    //!
    double fitness (vector<results> &result);

    //! @brief Call the mutation function defined by the user
    //!
    //! @param population A population vector of proginy gene sequences based on the user-supplied mutation function
    //!
    void mutate (vector<vector<genes> > &population);

    //! @brief Call the test function defined by the user
    //!
    //! @param gene     TODO
    //! @param outcome  TODO
    //! @param geneInfo TODO
    //!
    void test (vector<genes>& gene, vector<results>& outcome, GenData& geneInfo);

    //! @brief A fitness function to be defined by the user that
    //!
    double (*fitnessFnc_)(vector<results>&);

    //! @brief A mutation function to be defined by the user that returns a
    //!        population of genes
    //!
    void (*mutationFnc_)(vector<genes>&, vector<vector<genes> >&);

    //! @brief An evaluation function that executes a series of actions that result in performance data
    //!
    void (*testFnc_)(vector<genes>&, vector<results>&, GenData&);

    //! @brief A helper function that acts predicts gene sequence performance
    //!
    void (*helperFnc_)(vector<vector<genes> >&, vector<vector<results> >&, bool);

    //! @brief A helper function that prunes a gene population based on model performance results
    //!
    void (*subPopFnc_)(vector<vector<genes> >&, vector<vector<results> >&, vector<vector<genes> >&, int);

    //! @brief Parent gene sequence
    //!
    vector<genes> parent_;

    //! @brief Whether or not we're using the helper function to selectively run sub-populations
    //!        of the gene pool
    //!
    bool useHelper_;
  };
}

#endif