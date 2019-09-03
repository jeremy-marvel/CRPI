///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Genetic Algorithms
//  Workfile:        SimulatorInterface.h
//  Revision:        1.0 - 13 January, 2009
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Genetic algorithm interface class for a generic nontrivial assembly
//  simulator.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SIMINTERFACE_H
#define SIMINTERFACE_H

#include "../portable.h"
#include "../types.h"
#include "GATypes.h"
#include <vector>

using namespace std;

enum {SIMP0 = 0, SIMP1 = 1, SIMP2 = 2, SIMP3 = 3, SIMP4 = 4};

namespace GeneticAlgorithms
{
  //! @ingroup GeneticAlgorithms
  //!
  //! @brief Simulator-specific genetic algorithm execution code
  //!
  class LIBRARY_API SimInterface
  {
  public:

    //! @brief Default constructor
    //!
    SimInterface();

    //! @brief Constructor
    //!
    //! @param settings TODO
    //! @param comm     TODO
    //!
    SimInterface (networkSettings *settings, GAComm *comm);

    //! @brief Default destructor
    //!
    ~SimInterface();
 
    //! @brief Set the population size
    //!
    //! @param pop
    //! @param subpop
    //!
    void setPopulationSize (int pop, int subpop);

    //! @brief TODO
    //!
    //! @param param Results from a single assembly attempt, each vector item
    //!              should be the time and max force for each stage of
    //!              assembly
    //!
    //! @return The score computed based on the results of the assembly process
    //!
    double fitness (vector<GAResult>& param);

    //! @brief Return a collection of child genes by applying a deterministic mutation
    //!        to an input gene sequence.
    //!
    //! @param geneIn   The parent gene sequence from which the descendent
    //!                 children are derived
    //! @param genesOut The collection of child gene sequences created for
    //!                 testing
    //!
    void detMutation (vector<GAGene>& geneIn,
                      vector<vector<GAGene> >& genesOut);

    //! @brief Return a collection of child genes by applying a random mutation
    //!        to an input gene sequence.
    //!
    //! @param geneIn   The parent gene sequence from which the descendent
    //!                 children are derived
    //! @param genesOut The collection of child gene sequences created for
    //!                 testing
    //!
    void mutation (vector<GAGene>& geneIn,
                   vector<vector<GAGene> >& genesOut);

    //! @brief TODO
    //!
    //! @param inPop          TODO
    //! @param inResult       TODO
    //! @param outPop         TODO
    //! @param randomChildren The number of random children to add to the list
    //!
    void selectSubpop (vector<vector<GAGene> > &inPop,
                       vector<vector<GAResult> > &inResult,
                       vector<vector<GAGene> > &outPop,
                       int randomchildren = 0);

    //! @brief Mutation probability mutator
    //!
    //! @param probs TODO
    //!
    void setMutationProbs (vector<GAGene> &probs);

    //! @brief Mutation probability accessor
    //!
    //! @param probs TODO
    //!
    void getMutationProbs (vector<GAGene> &probs);

    //! @brief Signal the virtual sensor server to run the new gene sequence
    //!
    //! @param geneIn     TODO
    //! @param resultsOut TODO
    //! @param geneInfo   TODO
    //!
    void execute (vector<GAGene>& geneIn,
                  vector<GAResult>& resultsOut,
                  GenData& geneInfo);

    //! @brief Load a gene from disk and populate a gene sequence
    //!
    //! @param fname  The name/path of the file to be read from disk
    //! @param geneIn The gene sequence to be populated by the method
    //!
    void loadGene (const char *fname, vector<GAGene> &geneIn);

    //! @brief Save a gene to disk
    //!
    //! @param fname   The name/path of the file to be saved to disk
    //! @param geneOut The gene sequence to be written to disk
    //!
    void saveGene (const char *fname, const vector<GAGene>& geneOut);

    //! @brief Write a gene to a log (logger is owned by parent GA
    //!        implementation)
    //!
    //! @param gn    Gene data to be written to disk
    //! @param gd    Gene identification chronology and score
    //! @param first Whether or not this is the first time this function is
    //!              being called
    //!
    void logGene (GAGene* gn, GenData& gd, bool first);

    //! @brief Write a gene to a log (logger is owned by parent GA
    //!        implementation)
    //!
    //! @param pl    Private logger used for writing the results to disk
    //! @param rs    Result data to be written to disk
    //! @param gd    Gene identification chronology and score
    //! @param first Whether or not this is the first time this function is
    //!              being called
    //!
    void logResult (GAResult* rs, GenData& gd, bool first);

    //! @brief Write gene and result data (logger is static & owned by method)
    //!        to disk
    //!
    //! @param gn Gene data to be written to disk
    //! @param rs Result data to be written to disk
    //! @param gd Gene identification chronology and score
    //!
    //void dumpRawRecord (GAGene* gn, GAResult *rs, GenData& gd);

    void dumpRawRecord (vector<GAGene> &gn, vector<GAResult>& rs, GenData& gd);


    //! @brief TODO
    //!
    //! @param addOffset TODO
    //!
    void setAddRandomOffset (bool addOffset);

    //! @brief TODO
    //!
    //! @param iter TODO
    //!
    void setNumberChildIterations (int iter);

    //! @brief Set the maximum output value permitted for the simulator
    //!
    //! @param val
    //!
    void setMaxOutput (double val);

  private:

    //! @brief TODO
    //!
    //! @param where Which function generated the error (for debugging purposes)
    //! @param what  The nature of the error
    //!
    void exception (char *where, char *what);

    //! @brief TODO
    //!
    networkSettings *settings_;

    //! @brief The probabilities for mutating any of the individual gene
    //!        properties
    //!
    vector<GAGene> mutationProbs_;

    //! @brief The number of sample gene sequences to create upon mutation
    //!
    int populationSize_;

    //! @brief The subset size of the collection of children to actually run
    //!
    int subpopulationSize_;

    //! @breif Whether or not to add a random offset after each assembly attempt
    //!
    bool useRandomOffset_;

    //! @brief The number of times a specific child should be repeated for testing
    //!
    int iterations_;

    //! @brief TODO
    //!
    GAComm *communicator_;

    //! @brief TODO
    //!
    double maxOutput_;
  };
}

#endif
