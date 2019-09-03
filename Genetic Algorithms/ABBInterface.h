///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Neural Tissue
//  Subsystem:       Genetic Algorithms
//  Workfile:        ABBInterface.h
//  Revision:        1.0 - 29 March, 2008
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Genetic algorithm interface class for the ABB assembly parameter
//  optimization project.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ABBINTERFACE_H
#define ABBINTERFACE_H

#include "..\portable.h"
#include "..\types.h"
#include "..\Libraries\Reporter\reporter.h"
#include <vector>

using namespace std;
using namespace Reporter;

namespace GeneticAlgorithms
{
  //! @ingroup GeneticAlgorithms
  //!
  //! @brief ABB-specific genetic algorithm execution code
  //!
  class LIBRARY_API ABBInterface
  {
  public:

    //! @brief Default constructor
    //!
    ABBInterface();

    //! @brief Constructor
    //!
    //! @param settings TODO
    //! @param logPtr   TODO
    //! @param comm     TODO
    //!
    ABBInterface (networkSettings *settings, Logger *logPtr, GAComm *comm);

    //! @brief Default destructor
    //!
    ~ABBInterface();
 
    //! @brief Set the population size
    //!
    //! @param pop    The size of the new population of children to create
    //! @param subpop The size of the subpopulation to actually run
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
    //! @param pl    Private logger used for writing the genes to disk
    //! @param gn    Gene data to be written to disk
    //! @param gd    Gene identification chronology and score
    //! @param first Whether or not this is the first time this function is
    //!              being called
    //!
    void logGene (PrivateLogger* pl, GAGene* gn, GenData& gd, bool first);

    //! @brief Write a gene to a log (logger is owned by parent GA
    //!        implementation)
    //!
    //! @param pl    Private logger used for writing the results to disk
    //! @param rs    Result data to be written to disk
    //! @param gd    Gene identification chronology and score
    //! @param first Whether or not this is the first time this function is
    //!              being called
    //!
    void logResult (PrivateLogger* pl, GAResult* rs, GenData& gd, bool first);

    //! @brief Write gene and result data (logger is static & owned by method)
    //!        to disk
    //!
    //! @param gn Gene data to be written to disk
    //! @param rs Result data to be written to disk
    //! @param gd Gene identification chronology and score
    //!
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

    //! @brief TODO
    //!
    //! @param val TODO
    //!
    void setMaxForce (double val);

    //! @brief TODO
    //!
    //! @param val TODO
    //!
    void setMaxTime (double val);

    //! @brief TODO
    //!
    //! @param val TODO
    //!
    void setMaxDistance (double val);

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
                       int randomChildren = 0);

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

    //! @brief TODO
    //!
    Logger *logger_;

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
    double maxForce_;

    //! @brief TODO
    //!
    double maxTime_;

    //! @brief TODO
    //!
    double maxDistance_;

    //! @brief Whether or not we're using the deterministic mutation method
    //!
    //! @note If true, selectSubpop is effectively disabled
    //!
    bool deterministic_;
  };
}

#endif
