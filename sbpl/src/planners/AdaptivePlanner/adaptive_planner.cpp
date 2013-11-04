//using namespace std;
//#define FORCE3DPLANNING
#define PLANFROMSCRATCH
//#define VISUALIZEHEAPFIX
#include <sbpl/sbpl/headers.h>

using namespace std;

//====================================== PUBLIC ====================================================
  
AdaptivePlanner::AdaptivePlanner(AdaptiveDiscreteSpaceInformation* environment, bool bSearchForward)
{
	printf("Creating adaptive planner...");
	bforwardsearch = bSearchForward;
    	adaptive_environment_ = environment;

		StartStateID = environment->StartStateID;
		GoalStateID = environment->GoalStateID;
        
    	final_eps_planning_time = -1.0;
    	final_eps = -1.0;
    	newSphereRad = 1.0;
    	tunnelWidth = 1.0;
    	printf("done!\n");
    	fflush(stdout);
}


AdaptivePlanner::~AdaptivePlanner()
{
	SBPL_WARN("~AdaptivePlanner() NOT IMPLEMENTED YET!");
	return;
}

//-----------------------------Interface function-----------------------------------------------------

int AdaptivePlanner::dynamically_replan(double allocated_time_secs, void (*Callback)(std::vector<std::vector<double> >*, void*), void* obj){
	#ifdef ADP_VERBOSE
	SBPL_INFO("Dynamic adaptive planning... %.3f sec", allocated_time_secs);
	#endif
	//DO THE ADAPTIVE PLANNING LOOP HERE
	std::vector<int> planning_stateV;
	std::vector<int> tracking_stateV;
	std::vector<int> *use_stateV;
	
	std::vector<std::vector<double> > *executable_pieceV = new std::vector<std::vector<double> >();
	
	double start_t = clock();
	double track_tot = 0;
	double plan_tot = 0;
	
	int round = 0;
	
	double planning_time = 0.0f;
	
	int planning_bRet;
	int tracking_bRet;
	
	printf("Initializing tracker..."); fflush(stdout);
	tracker = new ARAPlanner_AD(adaptive_environment_, true);
	printf("OK\n"); fflush(stdout);
	
	printf("Initializing planner..."); fflush(stdout);
	planner = new ARAPlanner(adaptive_environment_, false);
	printf("OK\n"); fflush(stdout);
	
	printf("Setting start (%d) ang goal (%d)", StartStateID, GoalStateID); fflush(stdout);
	if(planner->set_start(StartStateID) == 0)
        {
        	SBPL_ERROR("ERROR: failed to set start state");
		throw new SBPL_Exception();
        }
        printf("OK\n"); fflush(stdout);
        if(tracker->set_start(StartStateID) == 0){
	        SBPL_ERROR("ERROR: failed to set start state");
		throw new SBPL_Exception();
        }
        printf("OK\n"); fflush(stdout);
	if(planner->set_goal(GoalStateID) == 0)
        {
        	SBPL_ERROR("ERROR: failed to set goal state");
		throw new SBPL_Exception();
        }
        printf("OK\n"); fflush(stdout);
        if(tracker->set_goal(GoalStateID) == 0){
        	SBPL_ERROR("ERROR: failed to set goal state");
		throw new SBPL_Exception();
        }
        printf("OK\n"); fflush(stdout);
        
	double time_buffer = allocated_time_secs;
	double horizon = 0;
	
	//set search mode
	planner->set_search_mode(bsearchuntilfirstsolution);
	tracker->set_search_mode(bsearchuntilfirstsolution);
	
	//set current start state 
	int currStartID = StartStateID;
	
	int elapsed_time = 0;//ros::Time::now().toSec() - start_t;

	adaptive_environment_->setPlanMode();
	//no need for start and goal spheres
	//adaptive_environment_->addSphere(StartStateID, newSphereRad);
	adaptive_environment_->addSphere(GoalStateID, 1.0);
	double buffered_time = 0;
	int viz_prog = 0;
	char log[256];
	std::vector<int> ModifiedStates;
	std::vector<int> startStateIDsV;
	while(!adaptive_environment_->isGoalReached()){
		double t_available = allocated_time_secs;
		int next_startID = 0;
		int episode_iters = 0;
		double episode_start;
		bool replanning_needed = true;
		#ifdef ROS
			episode_start = ros::Time::now().toSec();
		#else
			episode_start = clock();
		#endif
		while(t_available > 0.5 * allocated_time_secs && replanning_needed) {
			//iteration variables
			#ifdef ADP_VERBOSE
			SBPL_INFO("T_aval = %.4f", t_available);
			#endif
			episode_iters++;
			double iter_start;
			#ifdef ROS
				iter_start = ros::Time::now().toSec();
			#else
				iter_start = clock();
			#endif
			double plan_start;
			double track_start;	  //tracking phase start time
			int p_Cost;		  //planning solution cost
			int t_Cost;		  //tracking solution cost
			double plan_time;	  //planning phase time
			double track_time;	  //tracking phase time
			
			//==================================== PLANNING ====================================		
			#ifdef ADP_VERBOSE
			SBPL_ERROR("<===================== NEW ITERATION =====================>");
			SBPL_INFO("<===== Planning =======>");
			#endif
			
			planner->set_initialsolution_eps(planningEPS);
			planner->set_search_mode(bsearchuntilfirstsolution);
			adaptive_environment_->setPlanMode();
			planning_stateV.clear();
			planning_bRet = 0;
			
			#ifdef ROS
				plan_start = ros::Time::now().toSec();
			#else
				plan_start = clock();
			#endif
			planning_bRet = planner->replan(t_available, &planning_stateV, &p_Cost);
			
			#ifdef ROS
				plan_time = ros::Time::now().toSec() - plan_start;
			#else
				plan_time = (clock() - plan_start) / (double) CLOCKS_PER_SEC;
			#endif
			plan_tot += plan_time;
			
			#ifdef ADP_VERBOSE
			SBPL_WARN("Planning time: %.4f", plan_time);
			SBPL_WARN("Solution size: %d , cost: %d", (int) planning_stateV.size(), p_Cost);
			#endif
			
			if(!planning_bRet){
				SBPL_ERROR("Solution does not exist");
				return planning_bRet;
			}
			
			if(!planning_bRet || planning_stateV.size() == 0){
				#ifdef ADP_VERBOSE
				SBPL_ERROR("Solution could not be found within the allowed time (%.3fs.)", t_available);
				#endif
				replanning_needed = true;
				break;
			}
			
			replanning_needed = false;
			
			adaptive_environment_->visualizeStatePath(&planning_stateV, 1, 241, "planning_path");
			
			if(adaptive_environment_->isHighDPath(&planning_stateV)){
				//no need to track
				//AD planning path is high-d
				t_Cost = p_Cost;
				tracking_stateV.clear();
				for(unsigned int i = 0; i < planning_stateV.size(); i++){
					tracking_stateV.push_back(planning_stateV[i]);
					use_stateV = &planning_stateV;
				}
				replanning_needed = false;
				break;
			} else {
				#ifdef ROS
					t_available = allocated_time_secs - (ros::Time::now().toSec() - episode_start);
				#else
					t_available = allocated_time_secs - ((clock() - episode_start) / (double) CLOCKS_PER_SEC);
				#endif
				
				if(t_available <= 0) {
					//no time for tracking
					#ifdef ADP_VERBOSE
					SBPL_INFO("No time for tracking!");
					#endif
					tracking_bRet = false;
					tracking_stateV.clear();
					use_stateV = &planning_stateV;
					replanning_needed = true;
				} else {
					#ifdef ROS
						track_start = ros::Time::now().toSec();
					#else
						track_start = clock();
					#endif
					//==================================== TRACKING ====================================
					#ifdef ADP_VERBOSE
					SBPL_INFO("<===== Tracking =======>");
					#endif
					adaptive_environment_->setTrackMode(&planning_stateV, tunnelWidth, 0.5);
					tracker->set_initialsolution_eps(5.0f);	
					tracker->set_search_mode(bsearchuntilfirstsolution);
					//always do tracking from scratch
					tracker->force_planning_from_scratch();
					//set the allowed time to the time remaining after planning
					tracking_stateV.clear();
					tracking_bRet = 0;
					#ifdef ADP_VERBOSE
					printf("Tracking now..."); fflush(stdout);
					#endif
					tracking_bRet = tracker->replan(max(t_available,3.0), &tracking_stateV, &t_Cost);
					#ifdef ADP_VERBOSE
					printf("done!\n"); fflush(stdout);
					#endif
					
					#ifdef ROS
						track_time = ros::Time::now().toSec() - track_start;
					#else
						track_time = (clock() - track_start) / (double) CLOCKS_PER_SEC;
					#endif
					
					#ifdef ADP_VERBOSE
					SBPL_WARN("Tracking time: %.4f", track_time);
					#endif
					adaptive_environment_->visualizeStatePath(&tracking_stateV, 240, 300, "tracking_path");
					
					track_tot += track_time;
					
					bool track_timeout = false;
					
					if(tracking_bRet && (t_Cost / (1.0f * p_Cost)) > trackingEPS * planningEPS){
						//tracking found a costly path
						#ifdef ADP_LOGGING
						sprintf(log, "[Tracking] Succeeded - costly path found");
						adaptive_environment_->logStat(log);
						#endif
						#ifdef ADP_VERBOSE
						SBPL_INFO("[NO GUARANTEES!]\n\tFound costly path! tCost %d / pCost %d", t_Cost, p_Cost);
						#endif
						use_stateV = &tracking_stateV;
						replanning_needed = true;
					} else if (tracking_bRet && (t_Cost / (1.0f * p_Cost)) <= trackingEPS * planningEPS ) {
						#ifdef ADP_VERBOSE
						SBPL_INFO("[GOOD PATH!]");
						#endif
						#ifdef ADP_LOGGING
						sprintf(log, "[Tracking] Succeeded!");
						adaptive_environment_->logStat(log);
						#endif
						use_stateV = &tracking_stateV;
						replanning_needed = false;
					} else {
						//tracking failed!
						#ifdef ADP_LOGGING
						sprintf(log, "[Tracking] Failed!");
						adaptive_environment_->logStat(log);
						#endif
						#ifdef ADP_VERBOSE
						SBPL_WARN("[NO GUARANTEES!]\n\tTracking Failed!");
						#endif
						if(tracking_stateV.size() > 1){
							if(!track_timeout){
								int TrackFail_StateID = tracking_stateV[tracking_stateV.size()-1];
								adaptive_environment_->setPlanMode();
								adaptive_environment_->addSphere(TrackFail_StateID, newSphereRad, &ModifiedStates);
							}
							use_stateV = &tracking_stateV;
						} else {
							//tracking did not produce a trajectory
							#ifdef ADP_VERBOSE
							SBPL_ERROR("No new spheres added during this planning episode");
							#endif
							use_stateV = &planning_stateV;
						}
						replanning_needed = true;
					}
				}
			}
			#ifdef ADP_LOGGING
			sprintf(log, "[Planning] Time: %.3f sec (%.1f%% of iter time)", plan_time, 100*plan_time / (plan_time + track_time));
			adaptive_environment_->logStat(log);
			sprintf(log, "[Tracking] Time: %.3f sec (%.1f%% of iter time)", track_time, 100*track_time / (plan_time + track_time));
			adaptive_environment_->logStat(log);
			sprintf(log, "[Planning vs Tracking] Time ratio so far: (%.1f%% vs %.1f%%)", 100*plan_tot / (plan_tot + track_tot), 100*track_tot / (plan_tot + track_tot));
			adaptive_environment_->logStat(log);
			adaptive_environment_->logStats();
			#endif
			round++;
			
			#ifdef ROS
				t_available = allocated_time_secs - (ros::Time::now().toSec() - episode_start);
			#else
				t_available = allocated_time_secs - ((clock() - episode_start) / (double) CLOCKS_PER_SEC);
			#endif
		} //end while 
		//time for planning elapsed!
		//
		next_startID = adaptive_environment_->getExecutablePiece(use_stateV, allocated_time_secs, executable_pieceV, &horizon);
		//next_startID = adaptive_environment_->getFutureStateEstimate(executable_pieceV);
		
		if(next_startID < 0){
			SBPL_ERROR("Something went wrong!");
			return false;
		}
		#ifdef ADP_VERBOSE
		SBPL_WARN("Got %d waypoints for execution", (int)executable_pieceV->size());
		#endif
		if((int)executable_pieceV->size() > 0){
			#ifdef ROS
				t_available = allocated_time_secs - (ros::Time::now().toSec() - episode_start);
			#else
				t_available = allocated_time_secs - ((clock() - episode_start) / (double) CLOCKS_PER_SEC);
			#endif
			
			if(t_available > 0.1) {
				usleep((int)(1000000 * t_available));
			}
			
			SBPL_INFO("Callback!");
			SBPL_WARN("Traj. sent for execution!");
			Callback(executable_pieceV, obj);
			
			bool exists = false;
			for(unsigned int s = 0; s < startStateIDsV.size(); s++){
				if(next_startID == startStateIDsV[s]){
					SBPL_WARN("Start used before!");
					adaptive_environment_->setPlanMode();
					adaptive_environment_->addSphere(next_startID, newSphereRad, &ModifiedStates);
					exists = true;
				}
			}
			if(!exists) {
				startStateIDsV.push_back(next_startID);
			}
		  
			adaptive_environment_->moveStartTo(next_startID, &ModifiedStates);
			planner->force_planning_from_scratch();
			if(planner->set_start(next_startID) == 0)
			{
				SBPL_ERROR("ERROR: planner failed to set start state");
				throw new SBPL_Exception();
			}
			tracker->force_planning_from_scratch();
			if(tracker->set_start(next_startID) == 0){
				SBPL_ERROR("ERROR: tracker failed to set start state");
				throw new SBPL_Exception();
			}
			replanning_needed = true;
		} else {
			#ifdef ROS
				t_available = allocated_time_secs - (ros::Time::now().toSec() - episode_start);
			#else
				t_available = allocated_time_secs - ((clock() - episode_start) / (double) CLOCKS_PER_SEC);
			#endif
			if(t_available > 0.1) {
				usleep((int)(1000000 * t_available));
			}
			SBPL_INFO("Callback!");
			SBPL_WARN("No progress this time!");
			replanning_needed = true;
		}
		//adaptive_environment_->pause();
	}
	elapsed_time = 0;//ros::Time::now().toSec() - start_t;
	
	#ifdef ADP_VERBOSE
	//SBPL_INFO("Search completed in: %.4f (sec.)", elapsed_time / (float) CLOCKS_PER_SEC);
	SBPL_INFO("Total planning time: %.4f (sec.)", plan_tot);
	SBPL_INFO("Total tracking time: %.4f (sec.)", track_tot);
	#endif
	
	return tracking_bRet;

}





//returns 1 if found a solution, and 0 otherwise
int AdaptivePlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
	int solcost;
	return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
}

//returns 1 if found a solution, and 0 otherwise
int AdaptivePlanner::replan(double allocated_time_secs, std::vector<int>* solution_stateIDs_V, int* psolcost)
{
	#ifdef ADP_VERBOSE
	SBPL_INFO("Adaptive planning...");
	#endif
	//DO THE ADAPTIVE PLANNING LOOP HERE
	std::vector<int> planning_stateV;
	std::vector<int> tracking_stateV;
	
	std::vector<int> *executable_pieceV = new std::vector<int>();
	
	double start_t = 0;//ros::Time::now().toSec();
	track_tot = 0;
	plan_tot = 0;
	
	int round = 0;
	
	double ad_plan_time_alloc = allocated_time_secs;
	double ad_track_time_alloc = allocated_time_secs;
	
	#ifdef ADP_VERBOSE
	SBPL_INFO("Time limits: (Planning: %.4f) (Tracking: %.4f)", ad_plan_time_alloc, ad_track_time_alloc);
	#endif
	
	int planning_bRet;
	int tracking_bRet;

	bool bsearchuntilfirstsolution = true;
	bool bforwardsearch = true; 
	
	#ifdef ADP_VERBOSE
	printf("Initializing planners..."); fflush(stdout);
	#endif
	
	planner = new ARAPlanner(adaptive_environment_, true);
	tracker = new ARAPlanner_AD(adaptive_environment_, true);
	
	#ifdef ADP_VERBOSE
	printf("done\nSetting start and goal..."); fflush(stdout);
	#endif
	
	if(planner->set_start(StartStateID) == 0)
        {
        	SBPL_ERROR("ERROR: failed to set start state");
		throw new SBPL_Exception();
        }
        #ifdef ADP_VERBOSE
        printf("planner start set!\n"); fflush(stdout);
	#endif
	
        if(tracker->set_start(StartStateID) == 0){
	        SBPL_ERROR("ERROR: failed to set start state");
		throw new SBPL_Exception();
        }
        #ifdef ADP_VERBOSE
	printf("tracker start set!\n"); fflush(stdout);
	#endif
	if(planner->set_goal(GoalStateID) == 0)
        {
        	SBPL_ERROR("ERROR: failed to set goal state");
		throw new SBPL_Exception();
        }
        #ifdef ADP_VERBOSE
        printf("planner goal set!\n"); fflush(stdout);
	#endif
        if(tracker->set_goal(GoalStateID) == 0){
        	SBPL_ERROR("ERROR: failed to set goal state");
		throw new SBPL_Exception();
        }
        #ifdef ADP_VERBOSE
        printf("tracker goal set!\n"); fflush(stdout);
        printf("done\n"); fflush(stdout);
	#endif
        
	double time_buffer = allocated_time_secs;
	double horizon = 0;
	//set search mode
	planner->set_search_mode(bsearchuntilfirstsolution);
	tracker->set_search_mode(bsearchuntilfirstsolution);
	
	double elapsed_time = 0;//ros::Time::now().toSec() - start_t;

	#ifdef ADP_VERBOSE
	printf("Setting planning mode...");
	fflush(stdout);
	#endif
	adaptive_environment_->setPlanMode();
	#ifdef ADP_VERBOSE
	printf("done!\n");
	fflush(stdout);
	#endif
	//adaptive_environment_->addSphere(StartStateID, newSphereRad);
	#ifdef ADP_VERBOSE
	printf("Adding goal sphere...");
	fflush(stdout);
	#endif
	adaptive_environment_->addSphere(GoalStateID, 0.1);
	#ifdef ADP_VERBOSE
	printf("done!\n");
	fflush(stdout);
	printf("Adding start sphere...");
	fflush(stdout);
	#endif
	adaptive_environment_->addSphere(StartStateID, 0.05);
	#ifdef ADP_VERBOSE
	printf("done!\n");
	fflush(stdout);
	#endif
	time_buffer = allocated_time_secs;
	std::vector<int> ModifiedStates;
	std::vector<int> TrkModifiedStates;
	repair_time = 0.0;
	char log[256];
	#ifdef ADP_VERBOSE
	printf("Forcing planning from scratch...");
	fflush(stdout);
	#endif
	planner->force_planning_from_scratch();
	tracker->force_planning_from_scratch();
	#ifdef ADP_VERBOSE
	printf("done!\n");
	fflush(stdout);
	#endif
	do {
		sprintf(log, "=======================================\nIteration %d", round);
		adaptive_environment_->logStat(log);
		//iteration variables
		double iter_start = 0;//ros::Time::now().toSec();
		double plan_start = 0;//ros::Time::now().toSec();
		//int iter_start = clock(); //iteration start time
		//int plan_start = clock(); //planning phase start time
		int track_start;	  //tracking phase start time
		int p_Cost;		  //planning solution cost
		int t_Cost;		  //tracking solution cost
		double plan_time;	  //planning phase time
		double track_time;	  //tracking phase time

		//==================================== PLANNING ====================================		
		#ifdef ADP_VERBOSE
		SBPL_ERROR("<===================== NEW ITERATION =====================>");
		SBPL_INFO("<===== Planning =======>");
		#endif

		#ifdef PLANFROMSCRATCH
			planner->force_planning_from_scratch();
		#else
			double repair_start_t = 0;//ros::Time::now().toSec();
			//((ADPlanner*)planner)->update_succs_of_changededges(&ModifiedStates);
			#ifdef DEBUG_INCREMENTAL
			std::vector<int> HeapStates;
			std::vector<int> HeapKeys;
			
			((ARAPlanner*) planner)->getHeapStates(&HeapStates, &HeapKeys);
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("heap"));
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("heap"));
			printf("Heap before...");
			adaptive_environment_->pause();
			#endif
			
			((ARAPlanner*) planner)->fixHeap(&ModifiedStates, adaptive_environment_);
			repair_time += 0;//(ros::Time::now().toSec() - repair_start_t);
			
			#ifdef DEBUG_INCREMENTAL
			((ARAPlanner*) planner)->getHeapStates(&HeapStates, &HeapKeys);
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("heap"));		
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("heap"));
			printf("Heap after...");
			adaptive_environment_->pause();
			#endif
			
		#endif
		planner->set_initialsolution_eps(planningEPS);
		planner->set_search_mode(bsearchuntilfirstsolution);
		adaptive_environment_->setPlanMode();
		planning_stateV.clear();
		planning_bRet = 0;
		
		//PLANNING HERE!
		planning_bRet = planner->replan(time_buffer, &planning_stateV, &p_Cost);
		plan_time = 0;//(ros::Time::now().toSec() - plan_start);
		plan_tot += plan_time;
		
		if(!planning_bRet || planning_stateV.size() == 0){
			SBPL_ERROR("Solution could not be found within the allowed time (%.3fs.)", time_buffer);
			#ifdef ADP_GRAPHICAL
			adaptive_environment_->visualizeEnvironment();
			#endif
			nIterations = round;
			return planning_bRet;
		}
		#ifdef ADP_GRAPHICAL
		adaptive_environment_->visualizeEnvironment();
		adaptive_environment_->visualizeStatePath(&planning_stateV, 0, 120, "planning_path");
		adaptive_environment_->visualizeStatePath(&planning_stateV, 0, 120, "planning_path");
		#endif
		
		#ifdef ADP_VERBOSE
		printf("Done planning...");
		#endif
		
		//==================================== TRACKING ====================================
		#ifdef ADP_VERBOSE
		SBPL_INFO("<===== Tracking =======>");
		#endif
		track_start = 0;//ros::Time::now().toSec();
		//always do tracking from scratch
		adaptive_environment_->setTrackMode(&planning_stateV, tunnelWidth, 1.0, &TrkModifiedStates);
		
		#ifdef DEBUG_INCREMENTAL
		adaptive_environment_->visualizeStates(&TrkModifiedStates, 300, 240, std::string("trk_modified"));
		adaptive_environment_->visualizeStates(&TrkModifiedStates, 300, 240, std::string("trk_modified"));
		SBPL_INFO("Tracking modified states!");
		adaptive_environment_->pause();
		#endif
		
		#ifdef PLANFROMSCRATCH
			tracker->force_planning_from_scratch();
		#else
			#ifdef DEBUG_INCREMENTAL
			((ARAPlanner_AD*) tracker)->getHeapStates(&HeapStates, &HeapKeys);
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("trk_heap"));
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("trk_heap"));
			printf("Track Heap before...");
			adaptive_environment_->pause();
			#endif
			
			repair_start_t = 0;//ros::Time::now().toSec();
			//adaptive_environment_->getTrackingModStates(&ModifiedStates, &TrkModifiedStates);
			((ARAPlanner_AD*) tracker)->fixHeap(&TrkModifiedStates, adaptive_environment_);
			repair_time += 0;//(ros::Time::now().toSec() - repair_start_t);
			
			#ifdef DEBUG_INCREMENTAL
			((ARAPlanner_AD*) tracker)->getHeapStates(&HeapStates, &HeapKeys);
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("trk_heap"));		
			adaptive_environment_->visualizeStates(&HeapStates, 0, 0, std::string("trk_heap"));
			printf("Track Heap after...");
			adaptive_environment_->pause();
			#endif
			
		#endif
		tracker->set_initialsolution_eps(20.0);
		tracker->set_search_mode(true);
		tracking_stateV.clear();
		tracking_bRet = 0;
		//set the allowed time to the time remaining after planning
		//time_buffer -= (clock() - iter_start) / (1.0f * CLOCKS_PER_SEC); 
		
		//TRACKING HERE!!!
		tracking_bRet = tracker->replan(1.0, &tracking_stateV, &t_Cost);
		track_time = 0;//(ros::Time::now().toSec() - track_start);
		track_tot += track_time;
		ModifiedStates.clear();
		#ifdef ADP_LOGGING
		sprintf(log, "[Planning] Time: %.3f sec (%.1f%% of iter time)", plan_time, 100*plan_time / (plan_time + track_time));
		adaptive_environment_->logStat(log);
		sprintf(log, "[Tracking] Time: %.3f sec (%.1f%% of iter time)", track_time, 100*track_time / (plan_time + track_time));
		adaptive_environment_->logStat(log);
		sprintf(log, "[Planning vs Tracking] Time ratio so far: (%.1f%% vs %.1f%%)", 100*plan_tot / (plan_tot + track_tot), 100*track_tot / (plan_tot + track_tot));
		adaptive_environment_->logStat(log);
		adaptive_environment_->logStats();
		#endif
		
		#ifdef ADP_GRAPHICAL
		adaptive_environment_->visualizeEnvironment();
		adaptive_environment_->visualizeStatePath(&tracking_stateV, 240, 300, "tracking_path");
		adaptive_environment_->visualizeStatePath(&tracking_stateV, 240, 300, "tracking_path");
		#endif
		
		#ifdef APD_VERBOSE
		printf("Done tracking...");
		#endif
		
		if(tracking_bRet && (t_Cost / (1.0f * p_Cost)) > trackingEPS * planningEPS){
			#ifdef ADP_LOGGING
			sprintf(log, "[Tracking] Succeeded - costly path found");
			adaptive_environment_->logStat(log);
			#endif
			//tracking found a costly path 
			SBPL_INFO("[NO GUARANTEES!]\n\tFound costly path! tCost %d / pCost %d", t_Cost, p_Cost);
			//extract an executable piece from the tracking path
			//set the new start state to the end of the extracted executable piece
			//process the costly path adding spheres where necessary
			//adaptive_environment_->processCostlyPath(&planning_stateV, &tracking_stateV);
		} else if (tracking_bRet && (t_Cost / (1.0f * p_Cost)) <= trackingEPS * planningEPS ) {
			#ifdef ADP_LOGGING
			sprintf(log, "[Tracking] Succeeded!");
			adaptive_environment_->logStat(log);
			#endif
			SBPL_INFO("[GOOD PATH!]");
			adaptive_environment_->visualizeStatePath(&tracking_stateV, 0, 240, "tracking_path");
			solution_stateIDs_V->clear();
			for(unsigned int i = 0; i < tracking_stateV.size(); i++){
				solution_stateIDs_V->push_back(tracking_stateV[i]);
			}
			#ifdef ADP_LOGGING
			sprintf(log, "Iteration Time: %.3f sec (avg: %.3f)", ros::Time::now().toSec() - iter_start, (ros::Time::now().toSec() - start_t) / (round+1.0));
			adaptive_environment_->logStat(log);
			sprintf(log, "Done in: %.3f sec", ros::Time::now().toSec() - start_t);
			adaptive_environment_->logStat(log);
			#endif
			nIterations = round;
			return true;
		} else {
			#ifdef ADP_LOGGING
			sprintf(log, "[Tracking] Failed!");
			adaptive_environment_->logStat(log);
			#endif
			SBPL_WARN("[NO GUARANTEES!]\n\tTracking Failed!");
			
			//since tracking failed -- introduce new spheres
			if(tracking_stateV.size() > 1){
				//get the point of failure				
				int TrackFail_StateID = tracking_stateV[tracking_stateV.size()-1];
				adaptive_environment_->setPlanMode();
				adaptive_environment_->addSphere(TrackFail_StateID, newSphereRad, &ModifiedStates);
				/*SBPL_WARN("Got %d modified states", ModifiedStates.size());
				planner->update_preds_of_changededges(&ModifiedStates);
				*/
			} else {
				SBPL_ERROR("No new spheres added during this planning episode");
			}
		}
		#ifdef ADP_LOGGING
		sprintf(log, "Iteration Time: %.3f sec (avg: %.3f)", ros::Time::now().toSec() - iter_start, (ros::Time::now().toSec() - start_t) / (round+1.0));
		adaptive_environment_->logStat(log);
		sprintf(log, "Total Time so far: %.3f sec", ros::Time::now().toSec() - start_t);
		adaptive_environment_->logStat(log);
		#endif
		round++;
	} while (true);
	elapsed_time = 0;//ros::Time::now().toSec() - start_t;
	
	#ifdef ADP_VERBOSE
	SBPL_INFO("Search done in %.4f (sec.)", elapsed_time);
	#endif
	nIterations = round;
	return tracking_bRet;
}


int AdaptivePlanner::set_goal(int goal_stateID)
{
	GoalStateID = goal_stateID;
	SBPL_INFO("ADP: goal set (%d)", goal_stateID);
	return 1;
}


int AdaptivePlanner::set_start(int start_stateID)
{
	StartStateID = start_stateID;
	SBPL_INFO("ADP: start set (%d)", start_stateID);
	return 1;
}


int AdaptivePlanner::force_planning_from_scratch()
{
	if(planner != NULL && tracker != NULL){
		SBPL_INFO("Resetting planner and tracker!");
		planner->force_planning_from_scratch();
		tracker->force_planning_from_scratch();
	}
	return 1;
}


int AdaptivePlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
	bsearchuntilfirstsolution = bSearchUntilFirstSolution;
	return 1;
}


void AdaptivePlanner::print_searchpath(FILE* fOut)
{
	SBPL_WARN("print_searchpath() NOT IMPLEMENTED YET!");
	return;
}

int AdaptivePlanner::set_new_sphere_radius(double rad){
	newSphereRad = rad;
	return 1;
}
    	
int AdaptivePlanner::set_tunnel_width(double w){
	tunnelWidth = w;
	return 1;
}

void AdaptivePlanner::set_initialsolution_eps(double initialsolution_eps){
	planningEPS = sqrt(initialsolution_eps);
	trackingEPS = sqrt(initialsolution_eps);
}

int AdaptivePlanner::replan(vector<int>* solution_stateIDs_V, ReplanParams params, int* solcost){
  set_initialsolution_eps(params.initial_eps);
  bsearchuntilfirstsolution = params.return_first_solution;
  return replan(params.max_time, solution_stateIDs_V, solcost);
}
