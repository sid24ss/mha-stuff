#ifndef __ADAPTIVEPLANNER_H_
#define __ADAPTIVEPLANNER_H_

//#define ADP_DEBUG
//#define ADP_VERBOSE
//#define ADP_GRAPHICAL
//#define ADP_LOGGING
#define FORCETRACKING true

//#define DEBUG_INCREMENTAL
//#define DEBUG_MODIFIED

class AdaptivePlanner : public SBPLPlanner
{

public:

	/** \brief replan a path within the allocated time, return the solution in the vector
    	*/
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);
	/** \brief replan a path within the allocated time, return the solution in the vector, also returns solution cost
    	*/
	int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost);
	
	int replan(std::vector<int>* solution_stateIDs_V, ReplanParams params, int* solcost);
	
	int dynamically_replan(double allocated_time_secs, void (*Callback)(std::vector<std::vector<double> >*, void*), void* obj);

	/** \brief set the goal state
    	*/
    	int set_goal(int goal_stateID);
	/** \brief set the start state
    	*/
    	int set_start(int start_stateID);
    	
    	/** \brief set the new sphere radius parameter
    	*/
    	int set_new_sphere_radius(double rad);
    	
    	/** \brief set the tunnel radius parameter
    	*/
    	int set_tunnel_width(double w);

   	/** \brief set a flag to get rid of the previous search efforts, release the memory and re-initialize the search, when the next replan is called
      	*/
	int force_planning_from_scratch(); 

	/** \brief you can either search forwards or backwards
    	*/
	int set_search_mode(bool bSearchUntilFirstSolution);

	/** \brief returns the suboptimality bound on the currently found solution
    	*/
	double get_solution_eps() const {
		SBPL_WARN("get_solution_eps() not implemented for this planner!");
		throw new SBPL_Exception();
		return -1.0;
	};

	/** \brief returns the number of states expanded so far
    	*/
    	int get_n_expands() const { return searchexpands; }

	/** \brief returns the value of the initial epsilon (suboptimality bound) used
    	*/
	void set_initialsolution_eps(double initialsolution_eps);

	/** \brief prints out the search path into a file
    	*/
	void print_searchpath(FILE* fOut);

	/** \brief constructor 
    	*/
    	AdaptivePlanner(AdaptiveDiscreteSpaceInformation* environment, bool bforwardsearch);
	/** \brief destructor
    	*/
    	~AdaptivePlanner();

	/** \brief returns the time taken to get the final solution
    	*/
  	double get_final_eps_planning_time(){return final_eps_planning_time;};

	/** \brief returns the final epsilon achieved during the search
    	*/
  	double get_final_epsilon(){return final_eps;};
  	
  	void costs_changed(StateChangeQuery const & stateChange){
  		SBPL_WARN("costs_changed(...) NOT IMPLEMENTED FOR THIS PLANNER");
  	}
  	
  	void pause(){
  		printf("Enter to continue...");
  		char inp;
  		do {
  			inp = getchar();
  		} while (inp != '\n');
  	}
  	
  	int nIterations;
	double repair_time;
	double track_tot;
	double plan_tot;
protected:
	AdaptiveDiscreteSpaceInformation* adaptive_environment_;
private:

	SBPLPlanner *planner;
	SBPLPlanner *tracker;
	
	int StartStateID;
	int GoalStateID;
	
	double newSphereRad;
	double tunnelWidth;
	
	double planningEPS;
	double trackingEPS;

	//member variables
  	double final_eps_planning_time, final_eps;

	bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
	bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

	unsigned int searchexpands;
	//member functions
};

#endif
