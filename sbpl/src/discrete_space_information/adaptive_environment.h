#ifndef __ADAPTIVE_ENVIRONMENT_H_
#define __ADAPTIVE_ENVIRONMENT_H_

/** \brief base class for environments defining planning graphs
It is independent of the graph search used
The main means of communication between environment and graph search is through stateID. 
Each state is uniquely defined by stateID and graph search is ONLY aware of stateIDs. It doesn't know anything about the actual state variables.
Environment, on the other hand, maintains a mapping from stateID to actual state variables (coordinates) using StateID2IndexMapping array
*/
class AdaptiveDiscreteSpaceInformation : public DiscreteSpaceInformation
{

public:
	std::vector<int> modifiedStates;
	std::vector<int> invalidStates;
	int LVSID;
	
	virtual bool isHighDPath(std::vector<int> *stateIDV){
		SBPL_ERROR("isHighDPath not implemented for this environment");
		return false;
	}
	
	virtual bool isGoalReached() {
		SBPL_ERROR("isGoalReached not implemented for this environment");
		return false;
	}
	
	virtual int getFutureStateEstimate(std::vector<std::vector<double> > *traj) {
		SBPL_ERROR("getFutureStateEstimate not implemented for this environment");
		return -1;
	}

	virtual void resetExpandedStates() = 0;/*{
		SBPL_ERROR("resetExpandedStates not implemented for this environment");
	}*/

	virtual void expandingState(int StateID) = 0;/*{
		SBPL_ERROR("expandingState(int) not implemented for this environment");
	}*/
	
	virtual bool stopTracking(int StateID) {
		return false;
	}
	
	virtual void sendStateData(int StateID, int fVal, int gVal, int hVal) = 0;/*{
		SBPL_ERROR("sendStateData(int, int, int, int) not implemented for this environment");
	}*/
	
	virtual int getBestSeenState() {
		SBPL_ERROR("getBestSeenState() not implemented for this environment! Cannot construct a partial path!");
		return -1;
	};
	
	virtual void SearchTimeoutCallback(){
		SBPL_ERROR("SearchTimeoutCallback() not implemented for this environment");
	}
	/** \brief sets the environment in adaptive planning mode
	*/
	virtual void setPlanMode() = 0;
	
	virtual void visualizeStatePath(std::vector<int> *path, int scolor, int ecolor, std::string name) {
		SBPL_ERROR("visualizeStatePath not implemented!");
	};
	
	/** \brief constructs a tunnel of width tunnelWidth around the path specified by stateIDs_V 
	and sets the environment in tracking mode
	*/
	virtual void setTrackMode(std::vector<int> *stateIDs_V, double tunnelWidth, double TIMEOUT=1000.0, std::vector<int> *ModStates = NULL) =0;
	
	/** \brief adds a new sphere of radius rad at the state coordinates specified by StateID
	*/
	virtual void addSphere(int StateID, double rad, std::vector<int> *modifiedStates=NULL) =0;
	virtual void getTrackingModStates(std::vector<int>* planModStates, std::vector<int>* trkModStates){ SBPL_ERROR("getTrackingModStates not implemented for this environment!"); };
	virtual void moveStartTo(int StateID, std::vector<int> *modStates=NULL) { SBPL_ERROR("moveStartTo(int stateID) not implemented for this environment!"); };
	virtual int getTrackingFailStateID()= 0;
	virtual void addSphereWhereTrackingFailed(double rad) = 0;
	/** \brief destructor
    	*/
	virtual ~AdaptiveDiscreteSpaceInformation() {
		spheres_.clear();
	}
  	
  	virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) = 0;
  	
	virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) = 0;

	/** \brief constructor - specify the high-dim size and low-dim size
    	*/
	AdaptiveDiscreteSpaceInformation(int HDIM, int LDIM)
	{
		SBPL_ERROR("Trying to use default AdaptiveDiscreteSpaceInformation constructor!");
		throw new SBPL_Exception();
	}
	
	AdaptiveDiscreteSpaceInformation(){

	}
	
	virtual void logStats() {
		SBPL_ERROR("showStats not implemented for this environment!");	
	}
	
	virtual void logStat(char* info) {
		SBPL_ERROR("logStat not implemented for this environment!");
	}
	
	virtual void visualizePath(std::vector<int>* path){
		SBPL_ERROR("visualizePath not implemented for this environment!");
	}
	
	virtual void showSummary(FILE* fptr=NULL){
		SBPL_ERROR("showSummary not implemented for this environment!");
	}
	
	virtual void visualizeStates(std::vector<int> *stateIDs, int lcolor, int hcolor, std::string ns){
	
	}
	
	virtual void visualizeStates(std::vector<int> *stateIDs, std::vector<int> *colorsV, std::string ns){
	
	}
	
	virtual void visualizeState(std::string name, std::vector<double> &coords, int col){
	
	}
	
	virtual void visualizeEnvironment(){
	
	}
	
	virtual bool trackedWell() {
		return true;
	}
	
	virtual int getExecutablePiece(std::vector<int>* path, double time_limit, std::vector<std::vector<double> > *traj, double* horizon=NULL){
		SBPL_ERROR("getExecutablePiece not implemented for this environment!");
		return 0;
	}
	
	virtual void processCostlyPath(std::vector<int>* planning_path, std::vector<int>* tracking_path) {
		SBPL_ERROR("processCostlyPath not implemented for this environment!");
		throw new SBPL_Exception();
	}
	
	/** resets the environment to its original state - no spheres, etc. */
	virtual void reset() = 0;
	
	void pause(){
  		printf("Enter to continue...");
  		char inp;
  		do {
  			inp = getchar();
  		} while (inp != '\n');
  	}
	
	int StartStateID;
	int GoalStateID;

protected:
	/** NOTES: 
	the environment is always in tracking or planning mode. use setTrackMode and setPlanMode to change the mode of the environment
	the SBPL AdaptivePlanner will use setTrackMode and setPlanMode to switch between environment modes
	getSuccs and getPreds functions should take into account the environment mode when generating successor or predecessor states for the planner
	*/
	bool trackMode; // true - tracking, false - planning
	double getSuccTime; //time spent generating successors - for debugging purposes
	double getPredTime; //time spent generating predecessors - for debugging purposes
	int LowDim;	    //specifies the dimensionality of the low dim. manifold
	int HighDim;	    //specifies the dimensionality of the full dim. space
	std::vector<std::vector<double> > spheres_; //the list of spheres added to the environment, 
						    //each sphere is a vector of doubles of size LowDim + 1
						    //LowDim components specify the sphere position coordinates
						    //1 component specifies the sphere radius
						    
	
	
	/** \brief gets successors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetSuccs_Track(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
		SBPL_ERROR("GetSuccs_Track not implemented for this environment");
		throw new SBPL_Exception();
		return;
	}
	
	/** \brief gets successors for planning mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetSuccs_Plan(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
		SBPL_ERROR("GetSuccs_Plan not implemented for this environment");
		throw new SBPL_Exception();
		return;
	}
	
	/** \brief gets predecessors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetPreds_Track(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) {
		SBPL_ERROR("GetPreds_Track not implemented for this environment");
		throw new SBPL_Exception();
		return;
	}
	
	/** \brief gets predecessors for planning mode -- this should be specified by the user when creating their adaptive_environment file
	*/
	virtual void GetPreds_Plan(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) {
		SBPL_ERROR("GetPreds_Plan not implemented for this environment");
		throw new SBPL_Exception();
		return;
	}
};

#endif

