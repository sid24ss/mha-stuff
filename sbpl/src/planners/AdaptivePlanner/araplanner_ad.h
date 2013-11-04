/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __ARAPLANNER_AD_H_
#define __ARAPLANNER_AD_H_

#define ARA_AD_INCONS_LIST_ID 1

class CMDP;
class CMDPSTATE;
class CMDPACTION;
class CHeap;
class CList;


//-------------------------------------------------------------


/** \brief ARA* planner
  */
class ARAPlanner_AD : public SBPLPlanner
{
  
private:
	bool fixParents(ARAState* state, int Gind);

public:
	void getHeapStates(std::vector<int> *StateIDV, std::vector<int> *KeysV);
	inline int getHeapSize() {
		return pSearchStateSpace_->heap->currentsize;
	};
	
	void fixHeap(std::vector<int> *ModStateIDs, AdaptiveDiscreteSpaceInformation* env_);
	
	/** \brief replan a path within the allocated time, return the solution in the vector
    */
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);
	/** \brief replan a path within the allocated time, return the solution in the vector, also returns solution cost
    */
	int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost);

	/** \brief set the goal state
    */
    int set_goal(int goal_stateID);
	/** \brief set the start state
    */
    int set_start(int start_stateID);

	/** \brief inform the search about the new edge costs
    */
    void costs_changed(StateChangeQuery const & stateChange);

	/** \brief inform the search about the new edge costs - 
	    \note since ARA* is non-incremental, it is sufficient (and more efficient) to just inform ARA* of the fact that some costs changed
  */
    void costs_changed();


   	/** \brief set a flag to get rid of the previous search efforts, release the memory and re-initialize the search, when the next replan is called
      */
	 int force_planning_from_scratch(); 

	/** \brief you can either search forwards or backwards
    */
	int set_search_mode(bool bSearchUntilFirstSolution);

	/** \brief returns the suboptimality bound on the currently found solution
    */
	virtual double get_solution_eps() const {return pSearchStateSpace_->eps_satisfied;};

	/** \brief returns the number of states expanded so far
    */
    virtual int get_n_expands() const { return searchexpands; }

	/** \brief returns the value of the initial epsilon (suboptimality bound) used
    */
	virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps;};

	/** \brief prints out the search path into a file
    */
	void print_searchpath(FILE* fOut);


	/** \brief constructor 
    */
    ARAPlanner_AD(AdaptiveDiscreteSpaceInformation* environment, bool bforwardsearch);
	/** \brief destructor
    */
    ~ARAPlanner_AD();

	/** \brief returns the initial epsilon
    */
  double get_initial_eps(){return finitial_eps;};

	/** \brief returns the time taken to find the first solution
    */
  double get_initial_eps_planning_time(){return finitial_eps_planning_time;}

	/** \brief returns the time taken to get the final solution
    */
  double get_final_eps_planning_time(){return final_eps_planning_time;};

	/** \brief returns the number of expands to find the first solution
    */
  int get_n_expands_init_solution(){return num_of_expands_initial_solution;};

	/** \brief returns the final epsilon achieved during the search
    */
  double get_final_epsilon(){return final_eps;};

private:
	std::vector<ARAState*> *Gind;
	std::vector<ARAState*> *Eind;
	
	AdaptiveDiscreteSpaceInformation* environment_;
	//member variables
  	double finitial_eps, finitial_eps_planning_time, final_eps_planning_time, final_eps;

  	int num_of_expands_initial_solution;

	MDPConfig* MDPCfg_;

	bool bforwardsearch; //if true, then search proceeds forward, otherwise backward

	bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

    	ARASearchStateSpace_t* pSearchStateSpace_;

	unsigned int searchexpands;
	int MaxMemoryCounter;
	clock_t TimeStarted;
	double TimeLimit;
	FILE *fDeb;


	//member functions
	void Initialize_searchinfo(CMDPSTATE* state, ARASearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* CreateState(int stateID, ARASearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* GetState(int stateID, ARASearchStateSpace_t* pSearchStateSpace);

	int ComputeHeuristic(CMDPSTATE* MDPstate, ARASearchStateSpace_t* pSearchStateSpace);

	//initialization of a state
	void InitializeSearchStateInfo(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace);

	//re-initialization of a state
	void ReInitializeSearchStateInfo(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace);

	void DeleteSearchStateData(ARAState* state);

	//used for backward search
	void UpdatePreds(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace);


	//used for forward search
	void UpdateSuccs(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace);

	int GetGVal(int StateID, ARASearchStateSpace_t* pSearchStateSpace);

	//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
	int ImprovePath(ARASearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);

	void BuildNewOPENList(ARASearchStateSpace_t* pSearchStateSpace);

	void Reevaluatefvals(ARASearchStateSpace_t* pSearchStateSpace);

	//creates (allocates memory) search state space
	//does not initialize search statespace
	int CreateSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

	//deallocates memory used by SearchStateSpace
	void DeleteSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

	//debugging 
	void PrintSearchState(ARAState* state, FILE* fOut);


	//reset properly search state space
	//needs to be done before deleting states
	int ResetSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

	//initialization before each search
	void ReInitializeSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

	//very first initialization
	int InitializeSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

	int SetSearchGoalState(int SearchGoalStateID, ARASearchStateSpace_t* pSearchStateSpace);


	int SetSearchStartState(int SearchStartStateID, ARASearchStateSpace_t* pSearchStateSpace);

	//reconstruct path functions are only relevant for forward search
	int ReconstructPath(ARASearchStateSpace_t* pSearchStateSpace, CMDPSTATE* beststate=NULL);


	void PrintSearchPath(ARASearchStateSpace_t* pSearchStateSpace, FILE* fOut);

	int getHeurValue(ARASearchStateSpace_t* pSearchStateSpace, int StateID);

	//get path 
	vector<int> GetSearchPath(ARASearchStateSpace_t* pSearchStateSpace, int& solcost, CMDPSTATE* beststate=NULL);

	bool Search(ARASearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs);


};


#endif



