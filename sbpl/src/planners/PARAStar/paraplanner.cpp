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
#include <iostream>
using namespace std;


#include <sbpl/sbpl/headers.h>

//-----------------------------------------------------------------------------------------------------

PARAPlanner::PARAPlanner(DiscreteSpaceInformation* environment, bool bSearchForward)
{
	printf("PARAPlanner:: new PARAPlanner!\n"); fflush(stdout);
	bforwardsearch = bSearchForward;
	environment_ = environment;
	bsearchuntilfirstsolution = false;
	finitial_eps = ARA_DEFAULT_INITIAL_EPS;
	searchexpands = 0;
	MaxMemoryCounter = 0;
	eps_prob_max = 1.0;
    
#ifndef ROS
	const char* debug = "debug.txt";
#endif
	fDeb = SBPL_FOPEN(debug, "w");
	if(fDeb == NULL){
	    SBPL_ERROR("ERROR: could not open planner debug file\n");
	    throw new SBPL_Exception();
	}
    
	pSearchStateSpace_ = new PARASearchStateSpace_t;
	Gind = new std::vector<PARAState*>();
	Eind = new std::vector<PARAState*>();
    
	//create the ARA planner
	printf("Creating search state space..."); fflush(stdout);
	if(CreateSearchStateSpace(pSearchStateSpace_) != 1)
	{
	    SBPL_ERROR("ERROR: failed to create statespace\n");
	    return;
	}
	printf("OK\n"); fflush(stdout);
    
	//set the start and goal states
	printf("Init search state space..."); fflush(stdout);
	if(InitializeSearchStateSpace(pSearchStateSpace_) != 1)
	{
	    SBPL_ERROR("ERROR: failed to create statespace\n");
	    return;
	} 
	printf("OK\n"); fflush(stdout);
	finitial_eps_planning_time = -1.0;
	final_eps_planning_time = -1.0;
	num_of_expands_initial_solution = 0;
	final_eps = -1.0;
}


PARAPlanner::~PARAPlanner()
{
	if(pSearchStateSpace_ != NULL){
	    //delete the statespace
	    DeleteSearchStateSpace(pSearchStateSpace_);
	    delete pSearchStateSpace_;
	}
	Gind->clear();
	Eind->clear();
	delete Gind;
	delete Eind;
	SBPL_FCLOSE(fDeb);
}


void PARAPlanner::Initialize_searchinfo(CMDPSTATE* state, PARASearchStateSpace_t* pSearchStateSpace)
{
	PARAState* searchstateinfo = (PARAState*)state->PlannerSpecificData;
	searchstateinfo->MDPstate = state;
	InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace); 
}


CMDPSTATE* PARAPlanner::CreateState(int stateID, PARASearchStateSpace_t* pSearchStateSpace)
{
	CMDPSTATE* state = NULL;
#if DEBUG
	if(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] != -1)
	{
		SBPL_ERROR("ERROR in CreateState: state already created\n");
		throw new SBPL_Exception();
	}
#endif

	//adds to the tail a state
	state = pSearchStateSpace->searchMDP.AddState(stateID);

	//remember the index of the state
	environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;

#if DEBUG
	if(state != pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND]])
	{
		SBPL_ERROR("ERROR in CreateState: invalid state index\n");
		throw new SBPL_Exception();
	}
#endif

	//create search specific info
	state->PlannerSpecificData = (PARAState*)malloc(sizeof(PARAState));
	Initialize_searchinfo(state, pSearchStateSpace);
	MaxMemoryCounter += sizeof(PARAState);
	return state;
}

CMDPSTATE* PARAPlanner::GetState(int stateID, PARASearchStateSpace_t* pSearchStateSpace)
{
	if(stateID >= (int)environment_->StateID2IndexMapping.size())
	{
		SBPL_ERROR("ERROR int GetState: stateID %d is invalid\n", stateID);
		throw new SBPL_Exception();
	}

	if(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] == -1){
		return CreateState(stateID, pSearchStateSpace);
	} else {
	
		CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND]];
		if(state->PlannerSpecificData == NULL){
			state->PlannerSpecificData = (PARAState*)malloc(sizeof(PARAState));
			Initialize_searchinfo(state, pSearchStateSpace);
			MaxMemoryCounter += sizeof(PARAState);
		}
		return state;
	}
}

void PARAPlanner::DestroyState(int stateID, PARASearchStateSpace_t* pSearchStateSpace){
	if(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND] == -1) return;
	
	CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND]];
	PARAState* stateinfo = (PARAState*)state->PlannerSpecificData;
	
	if(state == NULL || stateinfo == NULL) return;
	
	DeleteSearchStateData(stateinfo);
	free(stateinfo);
	state->PlannerSpecificData = NULL;
	MaxMemoryCounter -= sizeof(PARAState);
}

//-----------------------------------------------------------------------------------------------------

int PARAPlanner::ComputeHeuristic(CMDPSTATE* MDPstate, PARASearchStateSpace_t* pSearchStateSpace)
{
	//compute heuristic for search
	if(bforwardsearch)
	{
		//forward search: heur = distance from state to searchgoal which is Goal ARAState
		int retv =  environment_->GetGoalHeuristic(MDPstate->StateID);
		return retv;
	}
	else
	{
		//backward search: heur = distance from searchgoal to state
		return environment_->GetStartHeuristic(MDPstate->StateID);
	}
}


//initialization of a state
void PARAPlanner::InitializeSearchStateInfo(PARAState* state, PARASearchStateSpace_t* pSearchStateSpace)
{
	state->g = INFINITECOST;
	state->v = INFINITECOST;
	state->iterationclosed = 0;
	state->callnumberaccessed = pSearchStateSpace->callnumber;
	state->bestnextstate = NULL;
	state->costtobestnextstate = INFINITECOST;
	state->heapindex = 0;
	state->listelem[PARA_INCONS_LIST_ID] = NULL;
	state->numofexpands = 0;
	state->Gind = INFINITECOST;
	state->Eind = INFINITECOST;
	state->P_good = 1.0;
	state->bestpredstate = NULL;
	state->parent_hist = new std::vector<PARAState*>();
	state->Gval_hist = new std::vector<unsigned int>();
	//compute heuristics
#if USE_HEUR
	if(pSearchStateSpace->searchgoalstate != NULL)
		state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
	else 
		state->h = 0;
#else
	state->h = 0;
#endif
}

//re-initialization of a state
void PARAPlanner::ReInitializeSearchStateInfo(PARAState* state, PARASearchStateSpace_t* pSearchStateSpace)
{
	state->g = INFINITECOST;
	state->v = INFINITECOST;
	state->iterationclosed = 0;
	state->callnumberaccessed = pSearchStateSpace->callnumber;
	state->bestnextstate = NULL;
	state->costtobestnextstate = INFINITECOST;
	state->heapindex = 0;
	state->listelem[PARA_INCONS_LIST_ID] = NULL;
	state->numofexpands = 0;
	state->Gind = INFINITECOST;
	state->Eind = INFINITECOST;
	state->P_good = 1.0;
	state->bestpredstate = NULL;
	state->parent_hist->clear();
	state->Gval_hist->clear();
	//compute heuristics
#if USE_HEUR
	if(pSearchStateSpace->searchgoalstate != NULL)
	{
		state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
	}
	else 
		state->h = 0;
#else
	state->h = 0;
#endif
}

void PARAPlanner::DeleteSearchStateData(PARAState* state)
{
	//no memory was allocated
	state->parent_hist->clear();
	state->Gval_hist->clear();
	delete state->parent_hist;
	delete state->Gval_hist;
	//MaxMemoryCounter = 0;
	return;
}

//used for backward search
void PARAPlanner::UpdatePreds(PARAState* state, PARASearchStateSpace_t* pSearchStateSpace)
{
	vector<int> PredIDV;
	vector<int> CostV;
	CKey key;
	PARAState *predstate;
	
	environment_->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);
	//iterate through predecessors of s
	for(int pind = 0; pind < (int)PredIDV.size(); pind++)
	{
		CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
		predstate = (PARAState*)(PredMDPState->PlannerSpecificData);
		if(predstate->callnumberaccessed != pSearchStateSpace->callnumber)
			ReInitializeSearchStateInfo(predstate, pSearchStateSpace);
		//see if we can improve the value of predstate
		if(predstate->g > state->v + CostV[pind])
		{
			predstate->g = state->v + CostV[pind];
			predstate->bestnextstate = state->MDPstate;
			predstate->costtobestnextstate = CostV[pind];
			
			//update parent history
			predstate->parent_hist->push_back(state);
			predstate->Gval_hist->push_back(predstate->g);
			
			//re-insert into heap if not closed yet
			if(predstate->iterationclosed != pSearchStateSpace->searchiteration)
			{
				Gind->push_back(predstate);
				predstate->Gind = Gind->size() - 1; //generated
				predstate->Eind = INFINITECOST; //not yet expanded
			  
				key.key[0] = predstate->g + (int)(pSearchStateSpace->eps*predstate->h);
				//key.key[1] = predstate->h;
				if(predstate->heapindex != 0)
					pSearchStateSpace->heap->updateheap(predstate,key);
				else
					pSearchStateSpace->heap->insertheap(predstate,key);
			}
			//take care of incons list
			else if(predstate->listelem[PARA_INCONS_LIST_ID] == NULL)
			{
				pSearchStateSpace->inconslist->insert(predstate, PARA_INCONS_LIST_ID);
			}
		}
	} //for predecessors
}

//used for forward search
void PARAPlanner::UpdateSuccs(PARAState* state, PARASearchStateSpace_t* pSearchStateSpace)
{
	vector<int> SuccIDV;
	vector<int> CostV;
	CKey key;
	PARAState *succstate;

	environment_->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

	//iterate through predecessors of s
	for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
	{
		CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
		int cost = CostV[sind];

		succstate = (PARAState*)(SuccMDPState->PlannerSpecificData);
		if(succstate->callnumberaccessed != pSearchStateSpace->callnumber)
			ReInitializeSearchStateInfo(succstate, pSearchStateSpace);
		//update generated index

		//see if we can improve the value of succstate
		//taking into account the cost of action
		if(succstate->g > state->v + cost)
		{
			succstate->g = state->v + cost;
			succstate->bestpredstate = state->MDPstate;
			//update parent history
			succstate->parent_hist->push_back(state);
			succstate->Gval_hist->push_back(succstate->g);
			//re-insert into heap if not closed yet
			if(succstate->iterationclosed != pSearchStateSpace->searchiteration)
			{
				Gind->push_back(succstate);
				succstate->Gind = Gind->size() - 1; //generated
				succstate->Eind = INFINITECOST; //not yet expanded
				
				double P_good = environment_->GetStateProb(SuccIDV[sind]);
				if(P_good == 0) P_good += 0.01;
				double P_eps = min(eps_prob, 1.0 / P_good);
				
				if(P_eps > eps_prob_max) eps_prob_max = P_eps;
				
				key.key[0] = (succstate->g + (int)(P_eps*pSearchStateSpace->eps*succstate->h));
				//key.key[1] = (int) (100.0*P_eps);//succstate->h;
				if(succstate->heapindex != 0)
					pSearchStateSpace->heap->updateheap(succstate,key);
				else
					pSearchStateSpace->heap->insertheap(succstate,key);
			}
			//take care of incons list
			else if(succstate->listelem[PARA_INCONS_LIST_ID] == NULL)
			{
				pSearchStateSpace->inconslist->insert(succstate, PARA_INCONS_LIST_ID);
			}
		} //check for cost improvement 
	} //for actions
}

int PARAPlanner::GetGVal(int StateID, PARASearchStateSpace_t* pSearchStateSpace)
{
	 CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
	 PARAState* state = (PARAState*)cmdp_state->PlannerSpecificData;
	 return state->g;
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int PARAPlanner::ImprovePath(PARASearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs)
{
	SBPL_ERROR("Improving path!");
	int expands;
	PARAState *state, *searchgoalstate;
	CKey key, minkey;
	CKey goalkey;
	expands = 0;

	if(pSearchStateSpace->searchgoalstate == NULL)
	{
		SBPL_ERROR("ERROR searching: no goal state is set\n");
		throw new SBPL_Exception();
	}

	//goal state
	searchgoalstate = (PARAState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
	if(searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber)
		ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace);

	//set goal key
	goalkey.key[0] = searchgoalstate->g;
	//goalkey.key[1] = searchgoalstate->h;

	//expand states until done
	minkey = pSearchStateSpace->heap->getminkeyheap();
	CKey oldkey = minkey;
	while(!pSearchStateSpace->heap->emptyheap() && minkey.key[0] < INFINITECOST && goalkey > minkey &&
		(clock()-TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC) 
	{
		//get the state		
		state = (PARAState*)pSearchStateSpace->heap->deleteminheap();
		//update expanded index
		//Eind->push_back(state);
		state->Eind = Gind->size() - 1;
		//printf(":"); fflush(stdout);
		environment_->PlannerExpandingState(state->MDPstate->StateID, state);
#if DEBUG
		//SBPL_FPRINTF(fDeb, "expanding state(%d): h=%d g=%u key=%u v=%u iterc=%d callnuma=%d expands=%d (g(goal)=%u)\n",
		//	state->MDPstate->StateID, state->h, state->g, state->g+(int)(pSearchStateSpace->eps*state->h), state->v, 
		//	state->iterationclosed, state->callnumberaccessed, state->numofexpands, searchgoalstate->g);
		//SBPL_FPRINTF(fDeb, "expanding: ");
		//PrintSearchState(state, fDeb);
		if(state->listelem[PARA_INCONS_LIST_ID]  != NULL)
		{
			//SBPL_FPRINTF(fDeb, "ERROR: expanding a state from inconslist\n");
			SBPL_WARN("ERROR: expanding a state from inconslist\n");
			//throw new SBPL_Exception();
		}
		//SBPL_FFLUSH(fDeb);
#endif

#if DEBUG
		if(minkey.key[0] < oldkey.key[0] && fabs(this->finitial_eps - 1.0) < ERR_EPS)
		{
			//SBPL_PRINTF("WARN in search: the sequence of keys decreases\n");
			//throw new SBPL_Exception();
		}
		oldkey = minkey;
#endif

		/*if(state->v == state->g)
		{
			SBPL_ERROR("ERROR: consistent state is being expanded\n");
#if DEBUG
			SBPL_FPRINTF(fDeb, "ERROR: consistent state is being expanded\n");
			throw new SBPL_Exception();
#endif
		}*/

		//recompute state value      
		state->v = state->g;
		state->iterationclosed = pSearchStateSpace->searchiteration;

		//new expand      
		expands++;
		state->numofexpands++;

		if(bforwardsearch == false)
			UpdatePreds(state, pSearchStateSpace);
		else
			UpdateSuccs(state, pSearchStateSpace);
		
		//recompute minkey
		minkey = pSearchStateSpace->heap->getminkeyheap();

		//recompute goalkey if necessary
		if(goalkey.key[0] != (int)searchgoalstate->g)
		{
			//SBPL_PRINTF("re-computing goal key\n");
			//recompute the goal key (heuristics should be zero)
			goalkey.key[0] = searchgoalstate->g;
			//goalkey.key[1] = searchgoalstate->h;
		}
		
		if(expands%100000 == 0 && expands > 0)
		{
			SBPL_PRINTF("expands so far=%u\n", expands);
		}
	}
	
	int retv = 1;
	if(searchgoalstate->g == INFINITECOST && pSearchStateSpace->heap->emptyheap())
	{
		printf("solution does not exist: search exited because heap is empty\n");
		retv = 0;
	}
	else if(!pSearchStateSpace->heap->emptyheap() && goalkey > minkey)
	{
		printf("search exited because it ran out of time\n");
		retv = 2;
	}
	else if(searchgoalstate->g == INFINITECOST && !pSearchStateSpace->heap->emptyheap())
	{
		printf("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
		retv = 0;
	}
	else
	{
		printf("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
		retv = 1;
	}
	//SBPL_FPRINTF(fDeb, "expanded=%d\n", expands);
	searchexpands += expands;
	return retv;
}


void PARAPlanner::BuildNewOPENList(PARASearchStateSpace_t* pSearchStateSpace)
{
	PARAState *state;
	CKey key;
	CHeap* pheap = pSearchStateSpace->heap;
	CList* pinconslist = pSearchStateSpace->inconslist; 
		
	//move incons into open
	while(pinconslist->firstelement != NULL)
	{
		state = (PARAState*)pinconslist->firstelement->liststate;
	    
		//compute f-value
		key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h);
		//key.key[1] = state->h;
	    
		//insert into OPEN
		pheap->insertheap(state, key);
		//remove from INCONS
		pinconslist->remove(state, PARA_INCONS_LIST_ID);
	}
}


void PARAPlanner::Reevaluatefvals(PARASearchStateSpace_t* pSearchStateSpace)
{
	CKey key;
	int i;
	CHeap* pheap = pSearchStateSpace->heap;
	
	//recompute priorities for states in OPEN and reorder it
	for (i = 1; i <= pheap->currentsize; ++i)
	{
		ARAState* state = (ARAState*)pheap->heap[i].heapstate;
		pheap->heap[i].key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h); 
		//pheap->heap[i].key.key[1] = state->h;
	}
	pheap->makeheap();
	pSearchStateSpace->bReevaluatefvals = false;
}


//creates (allocates memory) search state space
//does not initialize search statespace
int PARAPlanner::CreateSearchStateSpace(PARASearchStateSpace_t* pSearchStateSpace)
{
	//create a heap
	pSearchStateSpace->heap = new CHeap;
	pSearchStateSpace->inconslist = new CList;
	MaxMemoryCounter += sizeof(CHeap);
	MaxMemoryCounter += sizeof(CList);

	pSearchStateSpace->searchgoalstate = NULL;
	pSearchStateSpace->searchstartstate = NULL;

	searchexpands = 0;

	pSearchStateSpace->bReinitializeSearchStateSpace = false;
	
	return 1;
}

//deallocates memory used by SearchStateSpace
void PARAPlanner::DeleteSearchStateSpace(PARASearchStateSpace_t* pSearchStateSpace)
{
	if(pSearchStateSpace->heap != NULL)
	{
		pSearchStateSpace->heap->makeemptyheap();
		delete pSearchStateSpace->heap;
		pSearchStateSpace->heap = NULL;
	}

	if(pSearchStateSpace->inconslist != NULL)
	{
		pSearchStateSpace->inconslist->makeemptylist(PARA_INCONS_LIST_ID);
		delete pSearchStateSpace->inconslist;
		pSearchStateSpace->inconslist = NULL;
	}

	//delete the states themselves
	int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
	for(int i=0; i < iend; i++)
	{
		CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
		if(state != NULL && state->PlannerSpecificData != NULL){
			DeleteSearchStateData((PARAState*)state->PlannerSpecificData);
			free((ARAState*)state->PlannerSpecificData);
			state->PlannerSpecificData = NULL;
		}
	}
	pSearchStateSpace->searchMDP.Delete();
}

//reset properly search state space
//needs to be done before deleting states
int PARAPlanner::ResetSearchStateSpace(PARASearchStateSpace_t* pSearchStateSpace)
{
	pSearchStateSpace->heap->makeemptyheap();
	pSearchStateSpace->inconslist->makeemptylist(PARA_INCONS_LIST_ID);
	return 1;
}

//initialization before each search
void PARAPlanner::ReInitializeSearchStateSpace(PARASearchStateSpace_t* pSearchStateSpace)
{
	CKey key;
	//increase callnumber
	pSearchStateSpace->callnumber++;
	//reset iteration
	pSearchStateSpace->searchiteration = 0;
	pSearchStateSpace->bNewSearchIteration = true;

#if DEBUG
	SBPL_FPRINTF(fDeb, "reinitializing search state-space (new call number=%d search iter=%d)\n", 
	pSearchStateSpace->callnumber,pSearchStateSpace->searchiteration );
#endif

	pSearchStateSpace->heap->makeemptyheap();
	pSearchStateSpace->inconslist->makeemptylist(PARA_INCONS_LIST_ID);

    //reset 
	pSearchStateSpace->eps = this->finitial_eps;
	pSearchStateSpace->eps_prob = this->eps_prob;
	pSearchStateSpace->eps_satisfied = INFINITECOST;

	//initialize start state
	PARAState* startstateinfo = (PARAState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
	if(startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber)
		ReInitializeSearchStateInfo(startstateinfo, pSearchStateSpace);

	startstateinfo->g = 0;

	//insert start state into the heap
	key.key[0] = (long int)(pSearchStateSpace->eps*startstateinfo->h);
	//key.key[1] = startstateinfo->h;
	Gind->clear();
	Eind->clear();
	Gind->push_back(startstateinfo);
	startstateinfo->Gind = Gind->size() - 1;
	startstateinfo->Eind = INFINITECOST;
	pSearchStateSpace->heap->insertheap(startstateinfo, key);
	pSearchStateSpace->bReinitializeSearchStateSpace = false;
	pSearchStateSpace->bReevaluatefvals = false;
}

//very first initialization
int PARAPlanner::InitializeSearchStateSpace(PARASearchStateSpace_t* pSearchStateSpace)
{
	if(pSearchStateSpace->heap->currentsize != 0 || pSearchStateSpace->inconslist->currentsize != 0)
	{
		SBPL_ERROR("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
		throw new SBPL_Exception();
	}

	pSearchStateSpace->eps = this->finitial_eps;
	pSearchStateSpace->eps_satisfied = INFINITECOST;
	pSearchStateSpace->eps_prob = this->eps_prob;
	pSearchStateSpace->searchiteration = 0;
	pSearchStateSpace->bNewSearchIteration = true;
	pSearchStateSpace->callnumber = 0;
	pSearchStateSpace->bReevaluatefvals = false;

	//create and set the search start state
	pSearchStateSpace->searchgoalstate = NULL;
	//pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);
	pSearchStateSpace->searchstartstate = NULL;
	
	pSearchStateSpace->bReinitializeSearchStateSpace = true;
	return 1;
}


int PARAPlanner::SetSearchGoalState(int SearchGoalStateID, PARASearchStateSpace_t* pSearchStateSpace)
{
	if(pSearchStateSpace->searchgoalstate == NULL || pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
	{
		pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);
		//should be new search iteration
		
		pSearchStateSpace->eps_satisfied = INFINITECOST;
		pSearchStateSpace->bNewSearchIteration = true;
		pSearchStateSpace_->eps = this->finitial_eps;
		//recompute heuristic for the heap if heuristics is used
#if USE_HEUR
		printf("Recomputing heuristics..."); fflush(stdout);
		for(int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++)
		{
			CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
			PARAState* state = (PARAState*) (GetState(MDPstate->StateID, pSearchStateSpace)->PlannerSpecificData);
			
			state->h = ComputeHeuristic(MDPstate, pSearchStateSpace);
		}
		
		pSearchStateSpace->bReevaluatefvals = true;
		printf("done!\n"); fflush(stdout);
#endif
	}
	return 1;
}


int PARAPlanner::SetSearchStartState(int SearchStartStateID, PARASearchStateSpace_t* pSearchStateSpace)
{
	CMDPSTATE* MDPstate = GetState(SearchStartStateID, pSearchStateSpace);
	if(MDPstate !=  pSearchStateSpace->searchstartstate)
	{
		pSearchStateSpace->searchstartstate = MDPstate;
		pSearchStateSpace->bReinitializeSearchStateSpace = true;
	}
	return 1;
}



int PARAPlanner::ReconstructPath(PARASearchStateSpace_t* pSearchStateSpace)
{
	if(bforwardsearch) //nothing to do, if search is backward
	{
		CMDPSTATE* MDPstate = pSearchStateSpace->searchgoalstate;
		CMDPSTATE* PredMDPstate;
		PARAState *predstateinfo, *stateinfo;
#if DEBUG
		SBPL_FPRINTF(fDeb, "reconstructing a path:\n");
#endif
		while(MDPstate != pSearchStateSpace->searchstartstate)
		{
			stateinfo = (PARAState*)MDPstate->PlannerSpecificData;
#if DEBUG
			PrintSearchState(stateinfo, fDeb);
#endif
			//SBPL_WARN("StateID %d -> g = %d", MDPstate->StateID, stateinfo->g);
			if(stateinfo->g == INFINITECOST)
			{
				//SBPL_ERROR("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
				//throw new SBPL_Exception();
				return -1;
			}
			if(stateinfo->bestpredstate == NULL)
			{
				SBPL_ERROR("ERROR in ReconstructPath: bestpred is NULL\n");
				throw new SBPL_Exception();
			}
			//get the parent state
			PredMDPstate = stateinfo->bestpredstate;
			predstateinfo = (PARAState*)PredMDPstate->PlannerSpecificData;
			//set its best next info
			predstateinfo->bestnextstate = MDPstate;
			//check the decrease of g-values along the path
			if(predstateinfo->v >= stateinfo->g)
			{
				//SBPL_WARN("PredStateID %d -> v = %d", predstateinfo->MDPstate->StateID, predstateinfo->v);
				SBPL_ERROR("ERROR in ReconstructPath: g-values are non-decreasing\n");			
				PrintSearchState(predstateinfo, fDeb);
				throw new SBPL_Exception();
			}
			//transition back
			MDPstate = PredMDPstate;
		}
	}
	return 1;
}


void PARAPlanner::PrintSearchPath(PARASearchStateSpace_t* pSearchStateSpace, FILE* fOut)
{
	PARAState* searchstateinfo;
	CMDPSTATE* state;
	int goalID;
	int PathCost;
	if(bforwardsearch)
	{
		state  = pSearchStateSpace->searchstartstate;
		goalID = pSearchStateSpace->searchgoalstate->StateID;
	}
	else
	{
		state = pSearchStateSpace->searchgoalstate;
		goalID = pSearchStateSpace->searchstartstate->StateID;
	}
	if(fOut == NULL)
		fOut = stdout; 

	PathCost = ((PARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;

	SBPL_FPRINTF(fOut, "Printing a path from state %d to the goal state %d\n", 
			state->StateID, pSearchStateSpace->searchgoalstate->StateID);
	SBPL_FPRINTF(fOut, "Path cost = %d:\n", PathCost);
	environment_->PrintState(state->StateID, false, fOut);

	int costFromStart = 0;
	while(state->StateID != goalID)
	{
		SBPL_FPRINTF(fOut, "state %d ", state->StateID);
		if(state->PlannerSpecificData == NULL)
		{
			SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
			break;
		}

		searchstateinfo = (PARAState*)state->PlannerSpecificData;

		if(searchstateinfo->bestnextstate == NULL)
		{
			SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}

		if(searchstateinfo->g == INFINITECOST)
		{
			SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}

		int costToGoal = PathCost - costFromStart;
		int transcost = searchstateinfo->g - ((ARAState*)(searchstateinfo->bestnextstate->PlannerSpecificData))->v;
		if(bforwardsearch)
			transcost = -transcost;

		costFromStart += transcost;

		SBPL_FPRINTF(fOut, "g=%d-->state %d, h = %d ctg = %d  ", searchstateinfo->g, 			
			searchstateinfo->bestnextstate->StateID, searchstateinfo->h, costToGoal);

		state = searchstateinfo->bestnextstate;

		environment_->PrintState(state->StateID, false, fOut);
	}
}

void PARAPlanner::PrintSearchState(PARAState* state, FILE* fOut)
{
	SBPL_FPRINTF(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d expands=%d heapind=%d inconslist=%d\n",
		state->MDPstate->StateID, state->h, state->g, state->v, 
		state->iterationclosed, state->callnumberaccessed, state->numofexpands, state->heapindex, state->listelem[PARA_INCONS_LIST_ID]?1:0);
	environment_->PrintState(state->MDPstate->StateID, true, fOut);
}


int PARAPlanner::getHeurValue(PARASearchStateSpace_t* pSearchStateSpace, int StateID)
{
	CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
	PARAState* searchstateinfo = (PARAState*)MDPstate->PlannerSpecificData;
	return searchstateinfo->h;
}


void PARAPlanner::getHeapStates(std::vector<int> *StateIDV, std::vector<int> *KeysV){
	StateIDV->clear();
	KeysV->clear();
	for(int i = 1; i <= pSearchStateSpace_->heap->currentsize; i++){
		PARAState* state = (PARAState*)pSearchStateSpace_->heap->heap[i].heapstate;
		if((int)(pSearchStateSpace_->heap->heap[i].key[0]) < INFINITECOST){
			StateIDV->push_back((int)state->MDPstate->StateID);
			KeysV->push_back((int)state->g);
		}
	}
}

int PARAPlanner::getHeapSize(){
	return pSearchStateSpace_->heap->currentsize;
}

bool PARAPlanner::inHeap(int min_g){
	unsigned int max_g = 0;
	for(int i = 1; i <= pSearchStateSpace_->heap->currentsize; i++){
		PARAState* state = (PARAState*)pSearchStateSpace_->heap->heap[i].heapstate;
		if(state->g > max_g) max_g = state->g;
		if((int)state->g >= min_g){
			printf("Heap max G: %d (%d)\n", max_g, min_g);
			return true;
		}
	}
	printf("Heap max G: %d (%d)\n", max_g, min_g);
	return false;
}

PARAState* PARAPlanner::getMaxGFromHeap(int min_g, AdaptiveDiscreteSpaceInformation* env_){
	unsigned int max_g = (unsigned int)min_g;
	PARAState* s = NULL;
	for(int i = 1; i <= pSearchStateSpace_->heap->currentsize; i++){
		PARAState* state = (PARAState*)pSearchStateSpace_->heap->heap[i].heapstate;
		if(state->g >= max_g){
			max_g = state->g;
			s = state;
		}
	}
	if(s != NULL){
		pSearchStateSpace_->heap->deleteheap(s);
	}
	return s;
}

bool PARAPlanner::fixParents(PARAState* state, int Gind){
	if(state->Gind >= Gind){
		//the state has not yet been created! -- reset parents history
		state->g = INFINITECOST;
		state->bestpredstate = NULL;
		state->parent_hist->clear();
		state->Gval_hist->clear();
		return true;
	}
	if(state->parent_hist->size() == 0) {
		if(state->MDPstate->StateID != pSearchStateSpace_->searchstartstate->StateID){
			SBPL_WARN("The state has no parents! (StateID: %d)", state->MDPstate->StateID);
			SBPL_WARN("How/why is it in the CREATED list???");
			exit(0);
			state->g = INFINITECOST;
			state->bestpredstate = NULL;
			return false;
		}
		return true;
	}

	state->bestpredstate = NULL;
	state->g = INFINITECOST;
	
	for(int i = (int)state->parent_hist->size() - 1; i >= 0; i--){
		//check if the parent is valid
		//expanded before Gind
		if(state->parent_hist->at(i)->Eind < Gind){
			//first valid parent
			state->bestpredstate = state->parent_hist->at(i)->MDPstate;
			state->g = state->Gval_hist->at(i);
			//SBPL_WARN("Valid parent found!\nHistory len before %d", (int)state->parent_hist->size());
			state->parent_hist->resize(i+1);
			state->Gval_hist->resize(i+1);
			//SBPL_WARN("History len after %d\n-----", (int)state->parent_hist->size());
			return true;
		} else {
			//SBPL_WARN("Invalid parent found!!! Keep looking...");
		}
	}
	//SBPL_ERROR("No valid parents found for state -- resetting state!");
	state->g = INFINITECOST;
	state->bestpredstate = NULL;
	state->parent_hist->clear();
	state->Gval_hist->clear();
	return false;
}

void PARAPlanner::fixHeap(std::vector<int> *ModStateIDs, DiscreteSpaceInformation* env_){
	double start_t = 0;//ros::Time::now().toSec();
	unsigned int min_g = Gind->size();
	CMDPSTATE* startMDPState = pSearchStateSpace_->searchstartstate;
	PARAState* startState = (PARAState*)(startMDPState->PlannerSpecificData);
	for(unsigned int i = 0; i < ModStateIDs->size(); i++){
		CMDPSTATE* myMDPState = GetState(ModStateIDs->at(i), pSearchStateSpace_);
		PARAState* mystate = (PARAState*)(myMDPState->PlannerSpecificData);
		if(mystate->Gind < (int) min_g){
			min_g = mystate->Gind;
		}
	}

	printf("Fixing heap!!!\nMin Gind: %d\n", min_g); fflush(stdout);
	if(min_g >= Gind->size()) return;
	
	if(min_g == 0){
		//start from scratch
		force_planning_from_scratch();
		return;
	}
	pSearchStateSpace_->heap->makeemptyheap();
	pSearchStateSpace_->inconslist->makeemptylist(PARA_INCONS_LIST_ID);
	std::vector<int> heapStates;
	std::vector<int> heapKeys;
	for(unsigned int i = 0; i < min_g; i++){
		if(!fixParents(Gind->at(i), min_g)){
			SBPL_ERROR("Could not fix parents for i = %d! Something went wrong!!!", i);
			min_g = i;
			if(min_g == 0) {
				force_planning_from_scratch();
				return;
			}
			i = 0;
			continue;
		} else {
			if((i > 1 && 
				(Gind->at(i)->bestnextstate == NULL || Gind->at(i)->g == INFINITECOST) && 
				!bforwardsearch) ||
			   (i > 1 && 
			   	(Gind->at(i)->bestpredstate == NULL || Gind->at(i)->g == INFINITECOST) &&
			   	bforwardsearch))
			{   	
   			   	SBPL_ERROR("Parent or g-val mix-up!");
			   	min_g = i;
			   	if(min_g == 0) {
					force_planning_from_scratch();
					return;
			   	}
			   	i = 0;
			   	continue;
			}
			//all states generated before min_g
			if(Gind->at(i)->Eind >= (int) min_g){
				//printf("State expanded after!");
				//and expanded after min_g
				//should be in the open list
				//1. remove from closed
				Gind->at(i)->iterationclosed = 0;
				//2. insert into heap
				CKey key;
				int new_hval = ComputeHeuristic(Gind->at(i)->MDPstate, pSearchStateSpace_);
				Gind->at(i)->h = new_hval;
				key.key[0] = Gind->at(i)->g + (int)(pSearchStateSpace_->eps*Gind->at(i)->h);
				if(Gind->at(i)->heapindex != 0){
					pSearchStateSpace_->heap->updateheap(Gind->at(i), key);
				} else {
					pSearchStateSpace_->heap->insertheap(Gind->at(i), key);
				}
				//3. remove from incons list
				if(pSearchStateSpace_->inconslist->in(Gind->at(i), PARA_INCONS_LIST_ID)){
					pSearchStateSpace_->inconslist->remove(Gind->at(i), PARA_INCONS_LIST_ID);
				}
				//4. mark as not yet expanded
				Gind->at(i)->Eind = INFINITECOST;
			} else {
				//printf("State expanded before -- already closed!\n");
				//the state has been generated and closed at this point
				//should not be on the heap
				if(Gind->at(i)->heapindex != 0){
					pSearchStateSpace_->heap->deleteheap(Gind->at(i));
				}
				if(pSearchStateSpace_->inconslist->in(Gind->at(i), PARA_INCONS_LIST_ID)){
					pSearchStateSpace_->inconslist->remove(Gind->at(i), PARA_INCONS_LIST_ID);
				}
				Gind->at(i)->iterationclosed = pSearchStateSpace_->searchiteration;
				//new_Gind->push_back(Gind->at(i));
			}
		}
	}
	for(unsigned int i = min_g + 1; i < Gind->size(); i++){
		//states generated after min_g should be marked as never seen
		if(Gind->at(i) == NULL) {
			printf("Skipping NULL!\n"); continue;
		}
		//printf("Resetting..."); fflush(stdout);
		Gind->at(i)->iterationclosed = 0;
		Gind->at(i)->g = INFINITECOST;
		Gind->at(i)->Gind = INFINITECOST;
		Gind->at(i)->Eind = INFINITECOST;
		Gind->at(i)->parent_hist->clear();
		Gind->at(i)->Gval_hist->clear();
		Gind->at(i)->listelem[PARA_INCONS_LIST_ID] = NULL;
		Gind->at(i)->heapindex = 0;
		//printf("done!\n"); fflush(stdout);
		if(pSearchStateSpace_->inconslist->in(Gind->at(i), PARA_INCONS_LIST_ID)){
			//printf("Removing from inconslist..."); fflush(stdout);
			pSearchStateSpace_->inconslist->remove(Gind->at(i), PARA_INCONS_LIST_ID);
			//printf("done!\n"); fflush(stdout);
		}
		if(Gind->at(i)->heapindex != 0){
			//printf("Removing from heap..."); fflush(stdout);
			pSearchStateSpace_->heap->deleteheap(Gind->at(i));
			//printf("done!\n"); fflush(stdout);
		}
		#ifdef ALLOW_DESTROY_STATES
		if(Gind->at(i)->MDPstate->StateID != pSearchStateSpace_->searchgoalstate->StateID){
			//printf("Destroying state %d (G-ind: %d / %d)...", Gind->at(i)->MDPstate->StateID, i, (int)Gind->size());
			//fflush(stdout);
			DestroyState(Gind->at(i)->MDPstate->StateID, pSearchStateSpace_);
			Gind->at(i) = NULL;
			//printf("OK!\n"); fflush(stdout);
		} else {
			//printf("NOT Destroying state %d (G-ind: %d / %d)!!!\n", Gind->at(i)->MDPstate->StateID, i, (int)Gind->size());
			//fflush(stdout);
		}
		#endif
	}
	//printf("fixed!!!\n");
	fflush(stdout);
	Gind->resize(min_g);
	pSearchStateSpace_->eps_satisfied = INFINITECOST;
	PARAState* searchgoalstate = (PARAState*)(pSearchStateSpace_->searchgoalstate->PlannerSpecificData);
	searchgoalstate->g = INFINITECOST;
	
	char log[256];
	double t = 0;//ros::Time::now().toSec() - start_t;
	SBPL_ERROR("!!!! Fixed heap in %.3f sec.", t);
	if(t > 0.05){
		sprintf(log, "[Planning] !!!! Fixed heap in %.3f sec.", t);
	} else {
		sprintf(log, "[Planning] Fixed heap in %.3f sec.", t);
	}
	//env_->logStat(log);
	//getHeapStates(&heapStates, &heapKeys);
	//env_->visualizeStates(&heapStates, &heapKeys, std::string("heap"));
}

void PARAPlanner::rollbackHeap(std::vector<int> *ModStateIDs, AdaptiveDiscreteSpaceInformation* env_){
	unsigned int min_g = INFINITECOST;
	for(unsigned int i = 0; i < ModStateIDs->size(); i++){
		CMDPSTATE* myMDPState = GetState(ModStateIDs->at(i), pSearchStateSpace_);
		PARAState* mystate = (PARAState*)(myMDPState->PlannerSpecificData);
		if(mystate->g < INFINITECOST && mystate->g < min_g){
			min_g = mystate->g;
		}
	}
	printf("Min G: %d\n", min_g);
	std::vector<int> iList;
	//getInconsListStates(&iList);
	
	double start_t = 0;//ros::Time::now().toSec();
	int fixed = 0;
	CKey key;
	PARAState* state = getMaxGFromHeap(min_g, env_);
	while(state!=NULL && !pSearchStateSpace_->heap->emptyheap()){
		//1. get a state from the heap
		//ARAState* state = (ARAState*)pSearchStateSpace_->heap->deleteminheap();
		//2. push its parent on the heap if it is not there already
		/*
		if(state->bestpredstate != NULL){
			ARAState* parent = (ARAState*) (state->bestpredstate->PlannerSpecificData);
			parent->iterationclosed = 0;//pSearchStateSpace_->searchiteration-1;
			key.key[0] = parent->g + (int)(pSearchStateSpace_->eps*parent->h);
			if(key.key[0] < INFINITECOST){
				if(parent->heapindex != 0){
					pSearchStateSpace_->heap->updateheap(parent, key);
				} else {
					pSearchStateSpace_->heap->insertheap(parent, key);
				}
			}
			//2a. fix parent stuff (set closed to false)
			parent->iterationclosed = 0;//pSearchStateSpace_->searchiteration-1;
			//state->bestpredstate = NULL;
		}*/
		for(unsigned int i = 0; i < state->parent_hist->size(); i++){
			PARAState* parent = (PARAState*)(state->parent_hist->at(i));
			if(parent->heapindex != 0){
				//parent already in heap!
				continue;
			}
			parent->iterationclosed = 0;//pSearchStateSpace_->searchiteration-1;
			key.key[0] = parent->g + (int)(pSearchStateSpace_->eps*parent->h);
			if(key.key[0] < INFINITECOST){
				pSearchStateSpace_->heap->insertheap(parent, key);
			}
			//2a. fix parent stuff (set closed to false)
			parent->iterationclosed = 0;//pSearchStateSpace_->searchiteration-1;
			//state->bestpredstate = NULL;
		}
		state->g = INFINITECOST;
		state->parent_hist->clear();
		state->bestpredstate = NULL;
		state->iterationclosed = 0;//pSearchStateSpace_->searchiteration-1;
		if(pSearchStateSpace_->inconslist->in(state, PARA_INCONS_LIST_ID)){
			pSearchStateSpace_->inconslist->remove(state, PARA_INCONS_LIST_ID);
		}
		
		//3. check if state is modified -- if yes fixed++
		/*for(unsigned int i = 0; i < ModStateIDs->size(); i++){
			if(state->MDPstate->StateID == ModStateIDs->at(i)){
				fixed++;
				if(fixed == ModStateIDs->size()){
					break;
				}
			}	
		}*/
		state = getMaxGFromHeap(min_g, env_);
	}
	pSearchStateSpace_->heap->makeheap();
	pSearchStateSpace_->eps_satisfied = INFINITECOST;
	PARAState* searchgoalstate = (PARAState*)(pSearchStateSpace_->searchgoalstate->PlannerSpecificData);
	searchgoalstate->g = INFINITECOST;
	char log[256];
	double t = 0;//ros::Time::now().toSec() - start_t;
	if(t > 0.05){
		sprintf(log, "[Planning] !!!! Fixed heap in %.3f sec.", t);
	} else {
		sprintf(log, "[Planning] Fixed heap in %.3f sec.", t);
	}
	env_->logStat(log);
}


void PARAPlanner::getInconsListStates(std::vector<int> *StateIDV){
	StateIDV->clear();
	listelement* elt = pSearchStateSpace_->inconslist->firstelement;
	while(elt != NULL){
		PARAState* state = (PARAState*)elt->liststate;
		StateIDV->push_back((int)state->MDPstate->StateID);
		elt = elt->next;
	}
	//SBPL_ERROR("Got %d states from incons list", StateIDV->size());
}

vector<int> PARAPlanner::GetSearchPath(PARASearchStateSpace_t* pSearchStateSpace, int& solcost)
{
	vector<int> SuccIDV;
	vector<int> CostV;
	vector<int> wholePathIds;
	PARAState* searchstateinfo;
	CMDPSTATE* state = NULL; 
	CMDPSTATE* goalstate = NULL;
	CMDPSTATE* startstate=NULL;
  	//SBPL_WARN("reconstructing path...");
	if(bforwardsearch)
	{
		startstate = pSearchStateSpace->searchstartstate;
		goalstate = pSearchStateSpace->searchgoalstate;
		//reconstruct the path by setting bestnextstate pointers appropriately
		ReconstructPath(pSearchStateSpace);
	}
	else
	{
		startstate = pSearchStateSpace->searchgoalstate;
		goalstate = pSearchStateSpace->searchstartstate;
	}
	state = startstate;
  
	wholePathIds.push_back(state->StateID);
	solcost = 0;
  
	FILE* fOut = stdout;
	if(fOut == NULL){
		SBPL_ERROR("ERROR: could not open file\n");
		throw new SBPL_Exception();
	}
	int start_t = clock();
	while(state->StateID != goalstate->StateID)
	{
		double elapsed_t = (double)(clock() - start_t) / (double) CLOCKS_PER_SEC;
		if(elapsed_t > 5.0) {
			SBPL_ERROR("Something went wrong when reconstructing the path!");
			//prevent infinite loops
			break;
		}
		if(state->PlannerSpecificData == NULL)
		{
			SBPL_ERROR("path does not exist since search data does not exist\n");
			break;
		}
      
		searchstateinfo = (PARAState*)state->PlannerSpecificData;
      
		if(searchstateinfo->bestnextstate == NULL)
		{
			SBPL_WARN("path does not exist since bestnextstate == NULL\n");
			break;
		}
		if(searchstateinfo->g == INFINITECOST)
		{
			SBPL_WARN("path does not exist since state's g-val is INFINITE\n");
			break;
		}
      
		environment_->GetSuccs(state->StateID, &SuccIDV, &CostV);
		int actioncost = INFINITECOST;
		for(int i = 0; i < (int)SuccIDV.size(); i++)
		{   
			if(SuccIDV.at(i) == searchstateinfo->bestnextstate->StateID && CostV.at(i)<actioncost)
				actioncost = CostV.at(i);
		}
		if(actioncost == INFINITECOST) {
			SBPL_WARN("WARNING: actioncost = %d\n", actioncost);
		}
		solcost += actioncost;
		//SBPL_FPRINTF(fDeb, "actioncost=%d between states %d and %d\n", 
		//        actioncost, state->StateID, searchstateinfo->bestnextstate->StateID);
		//environment_->PrintState(state->StateID, false, fDeb);
		//environment_->PrintState(searchstateinfo->bestnextstate->StateID, false, fDeb);
#if DEBUG
		/* PARAState* nextstateinfo = (PARAState*)(searchstateinfo->bestnextstate->PlannerSpecificData);
		if(actioncost != abs((int)(searchstateinfo->g - nextstateinfo->g)) && pSearchStateSpace->eps_satisfied <= 1.001)
		{
			SBPL_FPRINTF(fDeb, "ERROR: actioncost=%d is not matching the difference in g-values of %d\n", 
			actioncost, abs((int)(searchstateinfo->g - nextstateinfo->g)));
			SBPL_ERROR("ERROR: actioncost=%d is not matching the difference in g-values of %d\n", 
			actioncost,abs((int)(searchstateinfo->g - nextstateinfo->g)));
			PrintSearchState(searchstateinfo, fDeb);
			PrintSearchState(nextstateinfo, fDeb);
		}*/
#endif
		state = searchstateinfo->bestnextstate;
		wholePathIds.push_back(state->StateID);
	}
	return wholePathIds;
}


bool PARAPlanner::Search(PARASearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs)
{
	CKey key;
	TimeStarted = clock();
	searchexpands = 0;
#if DEBUG
	SBPL_FPRINTF(fDeb, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif
	if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
		//re-initialize state space 
		ReInitializeSearchStateSpace(pSearchStateSpace);
	}

	if(bOptimalSolution)
	{
		pSearchStateSpace->eps = 1;
		MaxNumofSecs = INFINITECOST;
	}
	else if(bFirstSolution)
	{
		//MaxNumofSecs = INFINITECOST;
	}

	//ensure heuristics are up-to-date
	environment_->EnsureHeuristicsUpdated((bforwardsearch==true));
	//the main loop of ARA*
	int prevexpands = 0;
	clock_t loop_time;
	while(pSearchStateSpace->eps_satisfied > ARA_FINAL_EPS && (clock()- TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC)
	{
		loop_time = clock();
		//decrease eps for all subsequent iterations
		if(fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps) < ERR_EPS && !bFirstSolution)
		{
			pSearchStateSpace->eps = pSearchStateSpace->eps - ARA_DECREASE_EPS;
			if(pSearchStateSpace->eps < ARA_FINAL_EPS)
				pSearchStateSpace->eps = ARA_FINAL_EPS;

			//the priorities need to be updated
			pSearchStateSpace->bReevaluatefvals = true; 

			//it will be a new search
			pSearchStateSpace->bNewSearchIteration = true;

			//build a new open list by merging it with incons one
			BuildNewOPENList(pSearchStateSpace); 
		}

		if(pSearchStateSpace->bNewSearchIteration)
		{
			pSearchStateSpace->searchiteration++;
			pSearchStateSpace->bNewSearchIteration = false;
		}

		//re-compute f-values if necessary and reorder the heap
		if(pSearchStateSpace->bReevaluatefvals) 
			Reevaluatefvals(pSearchStateSpace);

		//improve or compute path
		if(ImprovePath(pSearchStateSpace, MaxNumofSecs) == 1){
			pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps;
		}
		//print the solution cost and eps bound
		SBPL_INFO("eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,
			((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);

		if(pSearchStateSpace->eps_satisfied == finitial_eps && pSearchStateSpace->eps == finitial_eps)
		{
			finitial_eps_planning_time = double(clock()-loop_time)/CLOCKS_PER_SEC;
			num_of_expands_initial_solution = searchexpands - prevexpands;
		}

#if DEBUG
		//SBPL_FPRINTF(fDeb, "eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,
		//					((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);
		//PrintSearchState((PARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData, fDeb);
#endif
		prevexpands = searchexpands;
		//if just the first solution then we are done
		if(bFirstSolution){
			SBPL_INFO("done!");
			break;
		}

		//no solution exists
		if(((PARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST)
			break;
	}

#if DEBUG
	//SBPL_FFLUSH(fDeb);
#endif

	PathCost = ((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
	MaxMemoryCounter += environment_->StateID2IndexMapping.size()*sizeof(int);
	
	SBPL_INFO("MaxMemoryCounter = %d\n", MaxMemoryCounter);

	int solcost = INFINITECOST;
	bool ret = false;
	if(PathCost == INFINITECOST)
	{
		SBPL_WARN("could not find a solution\n");
		ret = false;
	}
	else
	{
		SBPL_INFO("solution is found\n");      
		pathIds = GetSearchPath(pSearchStateSpace, solcost);
		ret = true;
	}

	SBPL_WARN("total expands this call = %d, planning time = %.3f secs, solution cost=%d\n", 
           searchexpands, (clock()-TimeStarted)/((double)CLOCKS_PER_SEC), solcost);
	final_eps_planning_time = (clock()-TimeStarted)/((double)CLOCKS_PER_SEC);
	final_eps = pSearchStateSpace->eps_satisfied;
	//SBPL_FPRINTF(fStat, "%d %d\n", searchexpands, solcost);
	return ret;
}


//-----------------------------Interface function-----------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int PARAPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
	int solcost;
	return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
	
}

//returns 1 if found a solution, and 0 otherwise
int PARAPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost)
{
	vector<int> pathIds; 
	bool bFound = false;
	int PathCost;
	bool bFirstSolution = this->bsearchuntilfirstsolution;
	bool bOptimalSolution = false;
	*psolcost = 0;
  
	SBPL_PRINTF("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);
  
	//plan
	if((bFound = Search(pSearchStateSpace_, pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false) 
	{
		SBPL_PRINTF("failed to find a solution\n");
	}
	
	SBPL_ERROR("eps_prob_max = %.3f", this->eps_prob_max);
  
	//copy the solution
	*solution_stateIDs_V = pathIds;
	*psolcost = PathCost;
	return (int)bFound;
}


int PARAPlanner::set_goal(int goal_stateID)
{
	//SBPL_PRINTF("planner: setting goal to %d\n", goal_stateID);
	//environment_->PrintState(goal_stateID, true, stdout);

	if(bforwardsearch)
	{
		if(SetSearchGoalState(goal_stateID, pSearchStateSpace_) != 1)
		{
			SBPL_ERROR("ERROR: failed to set search goal state\n");
			return 0;
		}
	}
	else
	{
		if(SetSearchStartState(goal_stateID, pSearchStateSpace_) != 1)
		{
			SBPL_ERROR("ERROR: failed to set search start state\n");
			return 0;
		}
	}
	return 1;
}


int PARAPlanner::set_start(int start_stateID)
{
	printf("PARAPlanner:: setting start to %d\n", start_stateID); fflush(stdout);
	SBPL_PRINTF("planner: setting start to %d\n", start_stateID);
	environment_->PrintState(start_stateID, true, stdout);

	if(bforwardsearch)
	{
		if(SetSearchStartState(start_stateID, pSearchStateSpace_) != 1)
		{
			SBPL_ERROR("ERROR: failed to set search start state\n");
			return 0;
		}
	}
	else
	{
		if(SetSearchGoalState(start_stateID, pSearchStateSpace_) != 1)
		{
			SBPL_ERROR("ERROR: failed to set search goal state\n");
			return 0;
		}
	}
	printf("PARAPlanner:: start set!"); fflush(stdout);
	return 1;
}


void PARAPlanner::costs_changed(StateChangeQuery const & stateChange)
{
	pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}


void PARAPlanner::costs_changed()
{
	pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}


int PARAPlanner::force_planning_from_scratch()
{
	SBPL_PRINTF("planner: forceplanfromscratch set\n");
	pSearchStateSpace_->bReinitializeSearchStateSpace = true;
	return 1;
}


int PARAPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
	SBPL_PRINTF("planner: search mode set to %d\n", bSearchUntilFirstSolution);
	bsearchuntilfirstsolution = bSearchUntilFirstSolution;
	return 1;
}


void PARAPlanner::print_searchpath(FILE* fOut)
{
	PrintSearchPath(pSearchStateSpace_, fOut);
}

//---------------------------------------------------------------------------------------------------------
