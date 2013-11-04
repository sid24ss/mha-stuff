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
/*
#include <cmath>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/planners/MHA/mplanner.h>
#include <sbpl/utils/heap.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/list.h>
#include <assert.h>
*/
#include <sbpl/sbpl/headers.h>

bool _mstate_print = false;
int conv = -1;
int expand_s[11];

MPlanner::MPlanner(DiscreteSpaceInformation* environment, int kk, bool bSearchForward)
{
    bforwardsearch = bSearchForward;
    environment_ = environment;
    env_num =kk;
    bsearchuntilfirstsolution = false;
    finitial_eps = M_DEFAULT_INITIAL_EPS;
    finitial_eps1 = M_DEFAULT_INITIAL_EPS;
    finitial_eps2 = M_DEFAULT_INITIAL_EPS;
    searchexpands = 0;
    MaxMemoryCounter = 0;
    
#ifndef ROS
  const char* debug = "debug.txt";
#endif
  fDeb = SBPL_FOPEN(debug, "w");
  if(fDeb == NULL){
    SBPL_ERROR("ERROR: could not open planner debug file\n");
    throw new SBPL_Exception();
  }
    
    pSearchStateSpace_ = new MSearchStateSpace_t;
    
    
    //create the M planner
    if(CreateSearchStateSpace(pSearchStateSpace_) != 1)
        {
            SBPL_ERROR("ERROR: failed to create statespace\n");
            return;
        }
    
    //set the start and goal states
    if(InitializeSearchStateSpace(pSearchStateSpace_) != 1)
        {
            SBPL_ERROR("ERROR: failed to create statespace\n");
            return;
        }    
    finitial_eps_planning_time = -1.0;
    final_eps_planning_time = -1.0;
    num_of_expands_initial_solution = 0;
    final_eps = -1.0;
}

MPlanner::~MPlanner()
{
  if(pSearchStateSpace_ != NULL){
    //delete the statespace
    DeleteSearchStateSpace(pSearchStateSpace_);
    delete pSearchStateSpace_;
  }
  SBPL_FCLOSE(fDeb);
}


void MPlanner::Initialize_searchinfo(CMDPSTATE* state, MSearchStateSpace_t* pSearchStateSpace)
{

	MState* searchstateinfo = (MState*)state->PlannerSpecificData;
	searchstateinfo->MDPstate = state;
	InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace); 
}

CMDPSTATE* MPlanner::CreateState(int stateID, MSearchStateSpace_t* pSearchStateSpace)
{	
	CMDPSTATE* state = NULL;

#if DEBUG
	if(environment_->StateID2IndexMapping[stateID][MMDP_STATEID2IND] != -1)
	{
		SBPL_ERROR("ERROR in CreateState: state already created\n");
		throw new SBPL_Exception();
	}
#endif

	//adds to the tail a state
	state = pSearchStateSpace->searchMDP.AddState(stateID);

	//remember the index of the state
	environment_->StateID2IndexMapping[stateID][MMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;
#if DEBUG
	if(state != pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][MMDP_STATEID2IND]])
	{
		SBPL_ERROR("ERROR in CreateState: invalid state index\n");
		throw new SBPL_Exception();
	}
#endif
	//create search specific info
	state->PlannerSpecificData = (MState*)malloc(sizeof(MState));	
	MaxMemoryCounter += sizeof(MState);
	Initialize_searchinfo(state, pSearchStateSpace);
	return state;

}

CMDPSTATE* MPlanner::GetState(int stateID, MSearchStateSpace_t* pSearchStateSpace)
{	

	if(stateID >= (int)environment_->StateID2IndexMapping.size()) // && stateID >= (int)env_d->StateID2IndexMapping.size())
	{
          SBPL_ERROR("ERROR int GetState: stateID %d is invalid , size - %d\n", stateID, (int) environment_->StateID2IndexMapping.size());
		throw new SBPL_Exception();
	}

	if (environment_->StateID2IndexMapping[stateID][MMDP_STATEID2IND] == -1) { 
		return CreateState(stateID, pSearchStateSpace);
        } else {
		CMDPSTATE* pp = pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][MMDP_STATEID2IND]];
		assert(pp);
		return pp;
        }
}



bool MPlanner::IsExpanded (int stateID) {
	//first check that the state exists (to avoid creation of additional states)
	if(environment_->StateID2IndexMapping[stateID][MMDP_STATEID2IND] == -1)
		return false; 
	CMDPSTATE* state = GetState(stateID, pSearchStateSpace_);
	MState* searchstateinfo = (MState*)state->PlannerSpecificData;
	for (int i =0; i < env_num; i++) {
	 if (searchstateinfo->iterationclosed[i] == pSearchStateSpace_->searchiteration)
		return true;
	}
	return false; 
}

int MPlanner::ComputeHeuristic(CMDPSTATE* MDPstate, MSearchStateSpace_t* pSearchStateSpace, int i)
{
	int x, y;
	//compute heuristic for search
	if(bforwardsearch)
	{
		//forward search: heur = distance from state to searchgoal which is Goal MState
		if (i == 0) 
		   return environment_->GetGoalHeuristic(MDPstate->StateID);
		else 
		   return environment_->GetGoalHeuristic(MDPstate->StateID, i); //Venkat

	}
	else
	{
		if (i == 0) 
			return environment_->GetStartHeuristic(MDPstate->StateID);
		else 
			return environment_->GetStartHeuristic(MDPstate->StateID, i); //Venkat
	}
}

//initialization of a state
void MPlanner::InitializeSearchStateInfo(MState* state, MSearchStateSpace_t* pSearchStateSpace)
{
	for (int i = 0; i < env_num; i++) {
		state->g[i] = INFINITECOST;
		state->v[i] = INFINITECOST;
		state->iterationclosed[i] = 0;
		state->callnumberaccessed[i] = pSearchStateSpace->callnumber;
		state->bestnextstate[i] = NULL;
		state->bestpredstate[i] = NULL;
		state->costtobestnextstate[i] = INFINITECOST;
		state->heapind[i] = 0;
#if USE_HEUR
		if(pSearchStateSpace->searchgoalstate != NULL) {
			//state->h[i] = ComputeHeuristic1(state->MDPstate, pSearchStateSpace, i);
			state->h[i] = ComputeHeuristic(state->MDPstate, pSearchStateSpace, i);
		} 
		else { 
			state->h[i] = 0;
		}
#else
		state->h[i] = 0;
#endif
	}
	state->listelem[M_INCONS_LIST_ID1] = 0;
	state->listelem[M_INCONS_LIST_ID2] = 0;
	state->numofexpands = 0;
	//compute heuristics
}


//re-initialization of a state
void MPlanner::ReInitializeSearchStateInfo(MState* state, MSearchStateSpace_t* pSearchStateSpace)
{
	assert(state->MDPstate != NULL);
	for (int i = 0; i < env_num; i++) {
		state->g[i] = INFINITECOST;
		state->v[i] = INFINITECOST;
		state->iterationclosed[i] = 0;
		state->callnumberaccessed[i] = pSearchStateSpace->callnumber;
		state->bestnextstate[i] = NULL;
		state->bestpredstate[i] = NULL;
		state->costtobestnextstate[i] = INFINITECOST;
		state->heapind[i] = 0;
#if USE_HEUR
		if(pSearchStateSpace->searchgoalstate != NULL) {
			state->h[i] = ComputeHeuristic(state->MDPstate, pSearchStateSpace, i);
		} 
		else { 
			state->h[i] = 0;
		}
#else
		state->h[i] = 0;
#endif
	}
}
void MPlanner::ReInitializeSearchStateInfo(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i)
{
	assert(state->MDPstate != NULL);
	state->g[i] = INFINITECOST;
	state->v[i] = INFINITECOST;
	state->iterationclosed[i] = 0;
	state->callnumberaccessed[i] = pSearchStateSpace->callnumber;
	state->bestnextstate[i] = NULL;
		state->bestpredstate[i] = NULL;
	state->costtobestnextstate[i] = INFINITECOST;
	state->heapind[i] = 0;
#if USE_HEUR
	if(pSearchStateSpace->searchgoalstate != NULL) {
		state->h[i] = ComputeHeuristic(state->MDPstate, pSearchStateSpace, i);
	} 
	else { 
		state->h[i] = 0;
	}
#else
	state->h[i] = 0;
#endif
}
void MPlanner::DeleteSearchStateData(MState* state)
{
	//no memory was allocated
	MaxMemoryCounter = 0;
	return;
}

//used for backward search
//UpdatePreds routine for IMHA*, ith search. 
void MPlanner::UpdatePreds(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i)
{
    vector<int> PredIDV;
    vector<int> CostV;
    CKey key;
    MState *predstate;
    environment_->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

	//iterate through predecessors of s
	for(int pind = 0; pind < (int)PredIDV.size(); pind++)
	{
		CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
		predstate = (MState*)(PredMDPState->PlannerSpecificData);
		if(predstate->callnumberaccessed[i] != pSearchStateSpace->callnumber) {
			ReInitializeSearchStateInfo(predstate, pSearchStateSpace, i);
                }
		//see if we can improve the value of predstate
		if(predstate->g[i] > state->g[i] + CostV[pind])
		{
			predstate->g[i] = state->g[i] + CostV[pind];
			predstate->bestnextstate[i] = state->MDPstate;
			predstate->costtobestnextstate[i] = CostV[pind];
			//re-insert into heap if not closed yet
			if ((predstate->iterationclosed[i] != pSearchStateSpace->searchiteration) && (predstate->h[i] < INFINITECOST))
			{
				int kk = (int)(pSearchStateSpace->eps1*predstate->h[i]);
				key.key[0] = predstate->g[i] + kk; 
				if(predstate->heapind[i] != 0)
					pSearchStateSpace->heap->updateheap(predstate,key,i);
				else
					pSearchStateSpace->heap->insertheap(predstate,key,i);

			} /*else if(predstate->listel[i] == NULL) {
				pSearchStateSpace->inconslist[i]->insert_mult(predstate, i);
			}*/
		  }
		//take care of incons list
		
	} //for predecessors
}
/* UpdateSuccs routine for IMHA* */

void MPlanner::UpdateSuccs(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i)
{
	vector<int> PredIDV;
	vector<int> CostV;
	CKey key;
	MState *predstate;
	environment_->GetSuccs(state->MDPstate->StateID, &PredIDV, &CostV);
	for(int pind = 0; pind < (int)PredIDV.size(); pind++)
	{
		CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
		predstate = (MState*)(PredMDPState->PlannerSpecificData);
		if(predstate->callnumberaccessed[i] != pSearchStateSpace->callnumber) {
			ReInitializeSearchStateInfo(predstate, pSearchStateSpace, i);
		}
		//environment_->PrintState(predstate->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
		//see if we can improve the value of predstate

                if(predstate->g[i] > state->g[i] + CostV[pind])
		{
			//print("State %d -- 
			predstate->g[i] = state->g[i] + CostV[pind];
			//predstate->bestnextstate[i] = state->MDPstate;
			predstate->bestpredstate[i] = state->MDPstate;
			//predstate->costtobestnextstate[i] = CostV[pind];
			//re-insert into heap if not closed yet
			if ((predstate->iterationclosed[i] != pSearchStateSpace->searchiteration) && (predstate->h[i] < INFINITECOST))
			{
				int kk = (int)(pSearchStateSpace->eps1*predstate->h[i]);
				key.key[0] = predstate->g[i] + kk; 
				if(predstate->heapind[i] != 0)
					pSearchStateSpace->heap->updateheap(predstate,key,i);
				else
					pSearchStateSpace->heap->insertheap(predstate,key,i);

			} /*else if(predstate->listel[i] == NULL) {
			    pSearchStateSpace->inconslist[i]->insert_mult(predstate, i);
			    }*/
		}
		//take care of incons list

	} //for predecessors
}

//Put in all the queue, dont update if already expaded in anchor, SMHA*

void MPlanner::UpdatePredsShared(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i)
{
    vector<int> PredIDV;
    vector<int> CostV;
    CKey key;
    MState *predstate;
    environment_->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);
	//iterate through predecessors of s
	for(int pind = 0; pind < (int)PredIDV.size(); pind++)
	{
		CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
		predstate = (MState*)(PredMDPState->PlannerSpecificData);
		if(predstate->callnumberaccessed[0] != pSearchStateSpace->callnumber) {
			ReInitializeSearchStateInfo(predstate, pSearchStateSpace);
                }
		//printf("State [%d] = %d\n",pind, PredMDPState->StateID); 
		//see if we can improve the value of predstate
		if(predstate->g[0] > state->g[0] + CostV[pind])
		{
			predstate->g[0] = state->g[0] + CostV[pind];
			predstate->bestnextstate[0] = state->MDPstate;
			predstate->costtobestnextstate[0] = CostV[pind];
			//re-insert into heap if not closed yet
			if ((predstate->iterationclosed[1] == pSearchStateSpace->searchiteration)) {
				//DELETE, EXPANDED IN ANCHOR
				for (int ii = 0; ii < env_num; ii++) {
					if(predstate->heapind[ii] != 0)
						pSearchStateSpace->heap->deleteheap(predstate,ii); //,ii);
				}
				
			} else if ((predstate->iterationclosed[1] != pSearchStateSpace->searchiteration)&& (predstate->iterationclosed[0] == pSearchStateSpace->searchiteration) && (predstate->h[0] < INFINITECOST)) {
                                //NOT EXPANDED IN ANCHOR, BUT EXPANDED IN INAD, ONLY PUT IN ANCHOR
				int anchor = predstate->g[0] + (int)(pSearchStateSpace->eps1*predstate->h[0]);
				key.key[0] = anchor;
				if(predstate->heapind[0] != 0)
					pSearchStateSpace->heap->updateheap(predstate,key,0);
				else
					pSearchStateSpace->heap->insertheap(predstate,key,0);
			} else if ((predstate->iterationclosed[0] != pSearchStateSpace->searchiteration) && (predstate->h[0] < INFINITECOST)) {
				//NOT EXPANDED IN ANCHOR OR ANY OTHER 
				int anchor = predstate->g[0] + (int)(pSearchStateSpace->eps1*predstate->h[0]);
				//int anchor = predstate->g[0] + (int)(eps_1[0]*predstate->h[0]);
				for (int ii = 0; ii < env_num; ii++) {
					if ( predstate->h[ii] < INFINITECOST) {
						int kk = predstate->g[0] + (int)(pSearchStateSpace->eps1*predstate->h[ii]);
						if (kk < pSearchStateSpace->eps2*anchor) {
							key.key[0] = kk;
							if(predstate->heapind[ii] != 0)
								pSearchStateSpace->heap->updateheap(predstate,key,ii);
							else
								pSearchStateSpace->heap->insertheap(predstate,key,ii);
						} 
					}
				}
			} 
			//take care of incons list
		}	
	} //for predecessors
}

void MPlanner::UpdateSuccsShared(MState* state, MSearchStateSpace_t* pSearchStateSpace, int i)
{
    vector<int> PredIDV;
    vector<int> CostV;
    CKey key;
    MState *predstate;
    environment_->GetSuccs(state->MDPstate->StateID, &PredIDV, &CostV);
	//iterate through predecessors of s
	for(int pind = 0; pind < (int)PredIDV.size(); pind++)
	{
		CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
		predstate = (MState*)(PredMDPState->PlannerSpecificData);
		if(predstate->callnumberaccessed[0] != pSearchStateSpace->callnumber) {
			ReInitializeSearchStateInfo(predstate, pSearchStateSpace);
                }
		//printf("State [%d] = %d\n",pind, PredMDPState->StateID); 
		//see if we can improve the value of predstate
		if(predstate->g[0] > state->g[0] + CostV[pind])
		{
			predstate->g[0] = state->g[0] + CostV[pind];
			//predstate->bestnextstate[0] = state->MDPstate;
			predstate->bestpredstate[0] = state->MDPstate;
			//predstate->costtobestnextstate[0] = CostV[pind];
			//re-insert into heap if not closed yet
			if ((predstate->iterationclosed[1] == pSearchStateSpace->searchiteration)) {
				//DELETE, EXPANDED IN ANCHOR
				for (int ii = 0; ii < env_num; ii++) {
					if(predstate->heapind[ii] != 0)
						pSearchStateSpace->heap->deleteheap(predstate,ii); //,ii);
				}
				
			} else if ((predstate->iterationclosed[1] != pSearchStateSpace->searchiteration)&& (predstate->iterationclosed[0] == pSearchStateSpace->searchiteration) && (predstate->h[0] < INFINITECOST)) {
                                //NOT EXPANDED IN ANCHOR, BUT EXPANDED IN INAD, ONLY PUT IN ANCHOR
				int anchor = predstate->g[0] + (int)(pSearchStateSpace->eps1*predstate->h[0]);
				key.key[0] = anchor;
				if(predstate->heapind[0] != 0)
					pSearchStateSpace->heap->updateheap(predstate,key,0);
				else
					pSearchStateSpace->heap->insertheap(predstate,key,0);
			} else if ((predstate->iterationclosed[0] != pSearchStateSpace->searchiteration) && (predstate->h[0] < INFINITECOST)) {
				//NOT EXPANDED IN ANCHOR OR ANY OTHER 
				int anchor = predstate->g[0] + (int)(pSearchStateSpace->eps1*predstate->h[0]);
				//int anchor = predstate->g[0] + (int)(eps_1[0]*predstate->h[0]);
                                bool nono = false;
				for (int ii = 0; ii < env_num; ii++) {
					if ( predstate->h[ii] < INFINITECOST) {
						int kk = predstate->g[0] + (int)(pSearchStateSpace->eps1*predstate->h[ii]);
						if (kk < pSearchStateSpace->eps2*anchor) {
							key.key[0] = kk;
							if(predstate->heapind[ii] != 0)
								pSearchStateSpace->heap->updateheap(predstate,key,ii);
							else
								pSearchStateSpace->heap->insertheap(predstate,key,ii);
						} 
					}
				}
			} 
			//take care of incons list
		}	
	} //for successors
}


//TODO-debugmax - add obsthresh and other thresholds to other environments in 3dkin
int MPlanner::GetGVal(int StateID, MSearchStateSpace_t* pSearchStateSpace)
{
	 CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
	 MState* state = (MState*)cmdp_state->PlannerSpecificData;
	 return state->g[0];
}

void MPlanner::PrintOpenList(MSearchStateSpace_t* pSearchStateSpace, int i)
{
	MState *state;
	CKey key;
	CHeapArr* pheap = pSearchStateSpace->heap;
	vector<MState*> general;
	printf("Printing The Remaining Queue %d\n", pheap->currentsize[i]);
	while(!pheap->emptyheap(i)) {
		//get the state		
		state = (MState*)pheap->deleteminheap(i);
		PrintSearchState(state, stdout);
		general.push_back(state);
		//printf("what \n");
	}		//assert(pheap->emptyheap());
	//move incons into open
	for (int pp =0; pp < (int) general.size(); pp++)  
	{
		state = general[pp];
		//compute f-value
		if (i > 0)  
			key.key[0] = state->g[i] + (int)(pSearchStateSpace->eps1*state->h[i]);
		else 
			key.key[0] = state->g[i] + (int)(pSearchStateSpace->eps1*state->h[i]);
		//insert into OPEN
		if(state->heapind[i] == 0)
			pheap->insertheap(state, key, i);
		else
			pheap->updateheap(state, key, i);
		//should never happen, but sometimes it does - somewhere there is a bug TODO
		//remove from INCONS
	}

}

//IMHA* 
int MPlanner::ImprovePathRoundRobin(MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs)
{
	int expands;
	MState *state, *searchgoalstate;
	CKey key, mink, minkey2;
	int minin = -1;
	CKey goalkey[MAX_NUM],minkey[MAX_NUM];
	expands = 0;
	printf("IN IMPROVEPATH -- %0.3f, %0.3f, %0.3f\n",pSearchStateSpace->eps1, pSearchStateSpace->eps2,pSearchStateSpace->eps1*pSearchStateSpace->eps2 );
	if (pSearchStateSpace->searchgoalstate == NULL) {
		SBPL_ERROR("ERROR searching: no goal state is set\n");
		throw new SBPL_Exception();
	}
	//goal state
	searchgoalstate = (MState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
	for (int ii =0; ii < env_num; ii++) {
		if (searchgoalstate->callnumberaccessed[ii] != pSearchStateSpace->callnumber) {
			ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace, ii);
		}
        }
	//set goal key
        mink = pSearchStateSpace->heap->getminkeyheap(0);
        for (int ii =0; ii < env_num; ii++) {
		goalkey[ii].key[0] = searchgoalstate->g[ii];
        }
        bool hush = false;
	if (env_num == 1)  { //No multiple heuristics. Mainly for debug purpose.
		int ii = 0;
		minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
		while (!pSearchStateSpace->heap->emptyheap(ii) && minkey[ii].key[0] < INFINITECOST) {
			if ((minkey[ii] >= goalkey[ii]) && (minkey[ii].key[0] < INFINITECOST)) {
				conv = ii;
				hush = true; 
				break;
			} 
			state = (MState*) pSearchStateSpace->heap->deleteminheap(ii);
			if (state->iterationclosed[ii] != pSearchStateSpace->searchiteration) { 
				state->v[ii] = state->g[ii];
				state->iterationclosed[ii] = pSearchStateSpace->searchiteration;
				expands++;
				state->numofexpands++;
				if (bforwardsearch == false)
					UpdatePreds(state, pSearchStateSpace, ii);
				else
					UpdateSuccs(state, pSearchStateSpace, ii);
				if (_mstate_print) { 
					SBPL_FPRINTF(stdout, "%d:: expanding state(%d): h=%d, g=%u (g(goal)=%u) :: Coord ", ii, state->MDPstate->StateID, state->h[ii], state->g[ii], searchgoalstate->g[ii]);
					environment_->PrintState(state->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
				}
				expand_s[ii]++;
			}
			minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
			goalkey[ii].key[0] = searchgoalstate->g[ii];
			if (hush) break;
			if(expands%100000 == 0 && expands > 0)
			{
				SBPL_PRINTF("expands so far=%u\n", expands);
			}
		}
        } else {
		assert(env_num > 1); 
	        mink = pSearchStateSpace->heap->getminkeyheap(0); 	
		while (!pSearchStateSpace->heap->emptyheap(0) && mink.key[0] < INFINITECOST) {
			bool subopt = false;
			//Number of environments 
			for (int ii = 1; ii < env_num; ii++) {  
				//First expansion
				minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
				if ((minkey[ii] >= goalkey[ii]) && (minkey[ii].key[0] < INFINITECOST)) {
					conv = ii;
					hush = true; 
					break;
				} 
				if ((minkey[ii].key[0] <= pSearchStateSpace->eps2*mink.key[0]) && (minkey[ii].key[0] < INFINITECOST)) {
					state = (MState*) pSearchStateSpace->heap->deleteminheap(ii);
					if (state->iterationclosed[ii] != pSearchStateSpace->searchiteration) { 
						state->v[ii] = state->g[ii];
						state->iterationclosed[ii] = pSearchStateSpace->searchiteration;
						expands++;
						state->numofexpands++;
						if (bforwardsearch == false)
							UpdatePreds(state, pSearchStateSpace, ii);
						else
							UpdateSuccs(state, pSearchStateSpace, ii);
						if (_mstate_print) { 
							SBPL_FPRINTF(stdout, "%d:: expanding state(%d): h=%d, g=%u (g(goal)=%u) :: Coord ", ii, state->MDPstate->StateID, state->h[ii], state->g[ii], searchgoalstate->g[ii]);
							environment_->PrintState(state->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
						}
						expand_s[ii]++;
					} else {
						printf("State -- %d, search -- %d\n", pSearchStateSpace->searchiteration, state->iterationclosed[ii]);
						printf("DO I EVER GET THIS\n");
						assert(0);
					}
					goalkey[ii].key[0] = searchgoalstate->g[ii];
					minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii); //minkey changes
				} else {
					int ii = 0;
					minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
					if ((minkey[ii] >= goalkey[ii]) && (minkey[ii].key[0] < INFINITECOST)) {
						conv = ii;
						hush = true; 
						break;
					} 
					state = (MState*) pSearchStateSpace->heap->deleteminheap(ii);
					if (state->iterationclosed[ii] != pSearchStateSpace->searchiteration) { 
						state->v[ii] = state->g[ii];
						state->iterationclosed[ii] = pSearchStateSpace->searchiteration;
						expands++;
						state->numofexpands++;
						if (bforwardsearch == false)
							UpdatePreds(state, pSearchStateSpace, ii);
						else
							UpdateSuccs(state, pSearchStateSpace, ii);
						if (_mstate_print) { 
							SBPL_FPRINTF(stdout, "%d:: expanding state(%d): h=%d, g=%u (g(goal)=%u) :: Coord ", ii, state->MDPstate->StateID, state->h[ii], state->g[ii], searchgoalstate->g[ii]);
							environment_->PrintState(state->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
						}
						expand_s[ii]++;
					}
					goalkey[ii].key[0] = searchgoalstate->g[ii];
					mink = pSearchStateSpace->heap->getminkeyheap(ii); // anchor heap changes
				}
			}
			if (hush) 
				break;
			if(expands%100000 == 0 && expands > 0)
			{
				SBPL_PRINTF("expands so far=%u\n", expands);
			}


		}
	}

	int retv = 1;
	if (hush) {
		printf("Got a solution in search --%d, multfactor -- %0.1f, margin -- %0.1f\n", conv, pSearchStateSpace->eps1, pSearchStateSpace->eps2);
		for (int ii = 0; ii < env_num; ii++) {
			printf("search[%d] expands= %d\n", ii, expand_s[ii]); 
		}
	} else {
		printf("Got no solution\n");
		retv = 0;

	}

	//SBPL_FPRINTF(fDeb, "expanded=%d\n", expands);

	searchexpands += expands;

	return retv;		
}
//SMHA* 
int MPlanner::ImprovePathRoundRobinShared (MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs)
{
	int expands;
	MState *state, *searchgoalstate;
	CKey key, mink, minkey2;
	int minin = -1;
	CKey goalkey[MAX_NUM],minkey[MAX_NUM];
	expands = 0;
	printf("IN IMPROVEPATH SHARED  -- %0.3f, %0.3f, %0.3f\n",pSearchStateSpace->eps1, pSearchStateSpace->eps2,pSearchStateSpace->eps1*pSearchStateSpace->eps2 );
	
	if (pSearchStateSpace->searchgoalstate == NULL) {
		SBPL_ERROR("ERROR searching: no goal state is set\n");
		throw new SBPL_Exception();
	}
	//goal state
	searchgoalstate = (MState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
	for (int ii =0; ii < env_num; ii++) {
		if (searchgoalstate->callnumberaccessed[ii] != pSearchStateSpace->callnumber) {
			ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace, ii);
		}
        }

	//set goal key
	mink = pSearchStateSpace->heap->getminkeyheap(0);
	for (int ii =0; ii < env_num; ii++) {
		goalkey[ii].key[0] = searchgoalstate->g[0];
	}
	bool hush = false;
	if (env_num == 1)  { //No multiple heuristics. Mainly for debug purpose.
		int ii = 0;
		minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
		while (!pSearchStateSpace->heap->emptyheap(ii) && minkey[ii].key[0] < INFINITECOST) {
			if ((minkey[ii] >= goalkey[ii]) && (minkey[ii].key[0] < INFINITECOST)) {
				conv = ii;
				hush = true; 
				break;
			} 
			state = (MState*) pSearchStateSpace->heap->deleteminheap(ii);
			if (state->iterationclosed[ii] != pSearchStateSpace->searchiteration) { 
				state->v[ii] = state->g[ii];
				state->iterationclosed[ii] = pSearchStateSpace->searchiteration;
				expands++;
				state->numofexpands++;
				if (bforwardsearch == false)
					UpdatePreds(state, pSearchStateSpace, ii);
				else
					UpdateSuccs(state, pSearchStateSpace, ii);
				if (_mstate_print) { 
					SBPL_FPRINTF(stdout, "%d:: expanding state(%d): h=%d, g=%u (g(goal)=%u) :: Coord ", ii, state->MDPstate->StateID, state->h[ii], state->g[ii], searchgoalstate->g[ii]);
					environment_->PrintState(state->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
				}
				expand_s[ii]++;
			}
			minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
			goalkey[ii].key[0] = searchgoalstate->g[ii];
			if (hush) break;
			if(expands%100000 == 0 && expands > 0)
			{
				SBPL_PRINTF("expands so far=%u\n", expands);
			}
		}
        } else {
		assert(env_num > 1); 
		mink = pSearchStateSpace->heap->getminkeyheap(0); 	
		while (!pSearchStateSpace->heap->emptyheap(0) && mink.key[0] < INFINITECOST) {
			bool subopt = false;
			//Number of environments 
			for (int ii = 1; ii < env_num; ii++) {  
				minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
				if (minkey[ii] >= goalkey[ii] && goalkey[ii].key[0] < INFINITECOST) {
					conv = ii;
					hush = true; 
					break;
				} 
				if ((minkey[ii].key[0] <= pSearchStateSpace->eps2*mink.key[0]) && (minkey[ii].key[0] < INFINITECOST)) {
					//printf("From ii = %d\n", ii);
					state = (MState*) pSearchStateSpace->heap->deleteminheap(ii);
					assert (state->iterationclosed[0] != pSearchStateSpace->searchiteration);  
					if (state->iterationclosed[0] != pSearchStateSpace->searchiteration) { 
						state->v[ii] = state->g[0];
						state->iterationclosed[0] = pSearchStateSpace->searchiteration;
						expands++;
						state->numofexpands++;
						if (bforwardsearch == false)
							UpdatePredsShared (state, pSearchStateSpace, ii);
						else
							UpdateSuccsShared (state, pSearchStateSpace, ii);
						if (_mstate_print) { 
							SBPL_FPRINTF(stdout, "%d:: expanding state(%d): h=%d, h_ad =%d, g=%u (g(goal)=%u) :: Coord ", ii, state->MDPstate->StateID, state->h[ii], state->h[0], state->g[0], searchgoalstate->g[0]);
							environment_->PrintState(state->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
						}
						expand_s[ii]++;
						for (int iii = 0; iii < env_num; iii++) {
							if(state->heapind[iii] != 0)
								pSearchStateSpace->heap->deleteheap(state,iii); //,ii);
							minkey[iii] = pSearchStateSpace->heap->getminkeyheap(iii);
						}	
					}
					goalkey[ii].key[0] = searchgoalstate->g[0];
					mink = pSearchStateSpace->heap->getminkeyheap(0);	
				} else {
					int ii = 0;
					minkey[ii] = pSearchStateSpace->heap->getminkeyheap(ii);
					if (minkey[ii] >= goalkey[ii] && goalkey[ii].key[0] < INFINITECOST) {
						conv = ii;
						hush = true; 
						break;
					} 
					state = (MState*) pSearchStateSpace->heap->deleteminheap(ii);
					assert (state->iterationclosed[1] != pSearchStateSpace->searchiteration);  
					if (state->iterationclosed[1] != pSearchStateSpace->searchiteration) { 
						state->v[ii] = state->g[ii];
						state->iterationclosed[ii] = pSearchStateSpace->searchiteration;
						state->iterationclosed[1] = pSearchStateSpace->searchiteration; 
						expands++;
						state->numofexpands++;
						if (bforwardsearch == false)
							UpdatePredsShared (state, pSearchStateSpace, ii);
						else
							UpdateSuccsShared (state, pSearchStateSpace, ii);

						if (_mstate_print) { 
							SBPL_FPRINTF(stdout, "Admissible %d:: expanding state(%d): h=%d, g=%u (g(goal)=%u) :: Coord ", ii, state->MDPstate->StateID, state->h[ii], state->g[ii], searchgoalstate->g[ii]);
							environment_->PrintState(state->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
						}
						expand_s[ii]++;
						for (int iii = 0; iii < env_num; iii++) {
							if(state->heapind[iii] != 0)
								pSearchStateSpace->heap->deleteheap(state,iii); //,ii);
							minkey[iii] = pSearchStateSpace->heap->getminkeyheap(iii);
						} 
					}
					goalkey[ii].key[0] = searchgoalstate->g[0];
					mink = pSearchStateSpace->heap->getminkeyheap(0);	
				}
			}
			if (hush) 
				break;
			if(expands%100000 == 0 && expands > 0)
				SBPL_PRINTF("expands so far=%u\n", expands);

		}

	}
        //printf("Min key -- %ld, index -- %d\n", mink.key[0], 0);
	int retv = 1;
	if (hush) {
		printf("Got a solution in search --%d, multfactor -- %0.1f, margin -- %0.1f\n", conv, pSearchStateSpace->eps1, pSearchStateSpace->eps2);
		for (int ii = 0; ii < env_num; ii++) {
			printf("search[%d] expands= %d\n", ii, expand_s[ii]); 
		}
		conv = 0;
	} else {
		printf("Got no solution\n");
		retv = 0;

	}
	//SBPL_FPRINTF(fDeb, "expanded=%d\n", expands);
	searchexpands += expands;
	return retv;		

}
//Single Search
int MPlanner::ImprovePath(MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs)
{
        int expands;
	MState *state, *searchgoalstate; 
	CKey key, mink, minkey2;
	int minin = -1;
	CKey goalkey,minkey;
	expands = 0;

	if (pSearchStateSpace->searchgoalstate == NULL) {
		SBPL_ERROR("ERROR searching: no goal state is set\n");
		throw new SBPL_Exception();
	}
	//goal state
	searchgoalstate = (MState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
	if (searchgoalstate->callnumberaccessed[0] != pSearchStateSpace->callnumber) {
		ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace); //, ii);
	}
	//set goal key



	//set goal key
	goalkey.key[0] = searchgoalstate->h[0];
	//expand states until done
	minkey = pSearchStateSpace->heap->getminkeyheap(0);
	bool hush = false;
	conv = 0;
	while(!pSearchStateSpace->heap->emptyheap(0) && minkey.key[0] < INFINITECOST && goalkey[0] > minkey[0] &&
		(clock()-TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC) 
    {

		//get the state		
		state = (MState*)pSearchStateSpace->heap->deleteminheap(0);
		//environment_->PrintState(state->MDPstate->StateID, true, stdout); //, xx, yy, tt); 
		
		if(state->v[0] == state->g[0])
		{
			SBPL_ERROR("ERROR: consistent state is being expanded\n");
			SBPL_FPRINTF(fDeb, "ERROR: consistent state is being expanded\n");
			throw new SBPL_Exception();
		}

		//recompute state value      
		state->v[0] = state->g[0];
		state->iterationclosed[0] = pSearchStateSpace->searchiteration;

		expands++;
		state->numofexpands++;
		if (bforwardsearch == false)
			UpdatePredsShared (state, pSearchStateSpace, 0);
		else
			UpdateSuccsShared (state, pSearchStateSpace, 0);


		//recompute minkey
		minkey = pSearchStateSpace->heap->getminkeyheap(0);

		//recompute goalkey if necessary
		goalkey.key[0] = searchgoalstate->g[0];

	}

	int retv = 1;
	if(searchgoalstate->g[0] == INFINITECOST && pSearchStateSpace->heap->emptyheap(0))
	{
		SBPL_PRINTF("solution does not exist: search exited because heap is empty\n");
		retv = 0;
	}
	else if(!pSearchStateSpace->heap->emptyheap(0) && goalkey > minkey )
	{
		SBPL_PRINTF("search exited because it ran out of time\n");
		retv = 2;
	}
	else if(searchgoalstate->g[0] == INFINITECOST && !pSearchStateSpace->heap->emptyheap(0))
	{
		SBPL_PRINTF("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
		retv = 0;
	}
	else
	{
		SBPL_PRINTF("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
		retv = 1;
	}

	//SBPL_FPRINTF(fDeb, "expanded=%d\n", expands);

	searchexpands += expands;
	return retv;	
}

//Simple Search for Ith env
int MPlanner::ImprovePath(MSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs, int i)
{
}
void MPlanner::BuildNewOPENList(MSearchStateSpace_t* pSearchStateSpace)
{
}
void MPlanner::Reevaluatefvals(MSearchStateSpace_t* pSearchStateSpace)
{
}

//creates (allocates memory) search state space
//does not initialize search statespace
int MPlanner::CreateSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace)
{

	//create a heap
	pSearchStateSpace->heap = new CHeapArr(env_num);
	for (int i=0; i < env_num; i++) {
		pSearchStateSpace->inconslist[i] = new CList;
	}
	MaxMemoryCounter += sizeof(CHeapArr);
	MaxMemoryCounter += env_num*sizeof(CList);

	pSearchStateSpace->searchgoalstate = NULL;
	pSearchStateSpace->searchstartstate = NULL;

	searchexpands = 0;


    pSearchStateSpace->bReinitializeSearchStateSpace = false;
	
	return 1;
}

//deallocates memory used by SearchStateSpace
void MPlanner::DeleteSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace)
{
	if(pSearchStateSpace->heap != NULL) {
		for (int oo=0; oo < env_num; oo++) 
			pSearchStateSpace->heap->makeemptyheap(oo);
        }
	for (int oo=0; oo < env_num; oo++) { 
		if(pSearchStateSpace->inconslist[oo] != NULL)
		{
			//pSearchStateSpace->inconslist[oo]->makeemptylist(oo);
			delete pSearchStateSpace->inconslist[oo];
			pSearchStateSpace->inconslist[oo] = NULL;
		}
	}
	//delete the states themselves
	int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
	for(int i=0; i < iend; i++)
	{
		CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
		if(state != NULL && state->PlannerSpecificData != NULL){
			DeleteSearchStateData((MState*)state->PlannerSpecificData);
			free((MState*)state->PlannerSpecificData);
			state->PlannerSpecificData = NULL;
		}
	}
	pSearchStateSpace->searchMDP.Delete();
}



//reset properly search state space
//needs to be done before deleting states
int MPlanner::ResetSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace)
{
	return 1;
}

//initialization before each search
void MPlanner::ReInitializeSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace)
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

    for (int oo = 0; oo < env_num; oo++) {
	    pSearchStateSpace->heap->makeemptyheap(oo);
	    //pSearchStateSpace->inconslist[oo]->makeemptylist(oo); 
    }
    //reset 
	pSearchStateSpace->eps = this->finitial_eps;
	pSearchStateSpace->eps1 = this->finitial_eps1;
	pSearchStateSpace->eps2 = this->finitial_eps2;
    	pSearchStateSpace->eps_satisfied = INFINITECOST;

	//initialize start state
	assert(pSearchStateSpace->searchstartstate);
	MState* startstateinfo = (MState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
	if(startstateinfo->callnumberaccessed[0] != pSearchStateSpace->callnumber)
		ReInitializeSearchStateInfo(startstateinfo, pSearchStateSpace);

        for (int oo = 0; oo < env_num; oo++) {
		startstateinfo->g[oo] = 0;
		key.key[0] = (long int)(pSearchStateSpace->eps1*startstateinfo->h[oo]);
		if (startstateinfo->heapind[oo] == 0) 
			pSearchStateSpace->heap->insertheap(startstateinfo, key, oo);
		else 
			pSearchStateSpace->heap->updateheap(startstateinfo, key, oo);
	}
        pSearchStateSpace->bReinitializeSearchStateSpace = false;
	pSearchStateSpace->bReevaluatefvals = false;
}

//very first initialization
int MPlanner::InitializeSearchStateSpace(MSearchStateSpace_t* pSearchStateSpace)
{

	for (int oo = 0; oo < env_num; oo++) {
		if(pSearchStateSpace->heap->currentsize[oo] != 0 || pSearchStateSpace->inconslist[oo]->currentsize !=0) {
			SBPL_ERROR("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
			throw new SBPL_Exception();
		} 
	}
	pSearchStateSpace->eps = this->finitial_eps;
	pSearchStateSpace->eps1 = this->finitial_eps1;
	pSearchStateSpace->eps2 = this->finitial_eps2;
    	pSearchStateSpace->eps_satisfied = INFINITECOST;
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
int MPlanner::SetSearchGoalState(int SearchGoalStateID, MSearchStateSpace_t* pSearchStateSpace, int j)
{
  //deprecated
  assert(0); 
  return -1;
}

int MPlanner::SetSearchGoalState(int SearchGoalStateID, MSearchStateSpace_t* pSearchStateSpace)
{
	if(pSearchStateSpace->searchgoalstate == NULL || 
		pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
	{
		pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);
		assert(pSearchStateSpace->searchgoalstate);
		//should be new search iteration
		pSearchStateSpace->eps_satisfied = INFINITECOST;
		pSearchStateSpace->bNewSearchIteration = true;
		pSearchStateSpace_->eps = this->finitial_eps;
		pSearchStateSpace_->eps1 = this->finitial_eps1;
		pSearchStateSpace_->eps2 = this->finitial_eps2;


		//recompute heuristic for the heap if heuristics is used
#if USE_HEUR
		for(int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++)
		{
			CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
			MState* state = (MState*)MDPstate->PlannerSpecificData;
			for (int oo = 0; oo < env_num; oo++) { 
				state->h[oo] = ComputeHeuristic(MDPstate, pSearchStateSpace, oo);
			}
		}
		pSearchStateSpace->bReevaluatefvals = true;
#endif
	}
	return 1;

}


int MPlanner::SetSearchStartState(int SearchStartStateID, MSearchStateSpace_t* pSearchStateSpace)
{
	CMDPSTATE* MDPstate = GetState(SearchStartStateID, pSearchStateSpace);
        assert (MDPstate != NULL);
	if(MDPstate !=  pSearchStateSpace->searchstartstate)
	{	
		pSearchStateSpace->searchstartstate = MDPstate;
		pSearchStateSpace->bReinitializeSearchStateSpace = true;
	}
	
	return 1;
}



int MPlanner::ReconstructPath(MSearchStateSpace_t* pSearchStateSpace)
{	
	if(bforwardsearch) //nothing to do, if search is backward
	{
		CMDPSTATE* MDPstate = pSearchStateSpace->searchgoalstate;
		CMDPSTATE* PredMDPstate;
		MState *predstateinfo, *stateinfo;
#if DEBUG
		SBPL_FPRINTF(fDeb, "reconstructing a path:\n");
#endif

		while(MDPstate != pSearchStateSpace->searchstartstate)
		{
			stateinfo = (MState*)MDPstate->PlannerSpecificData;

#if DEBUG
			PrintSearchState(stateinfo, fDeb);
#endif
			if(stateinfo->g[conv] == INFINITECOST)
			{	
				SBPL_ERROR("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
				//throw new SBPL_Exception();
				return -1;
			}

			if(stateinfo->bestpredstate[conv] == NULL)
			{
				SBPL_ERROR("ERROR in ReconstructPath: bestpred is NULL\n");
				throw new SBPL_Exception();
			}
			//get the parent state
			PredMDPstate = stateinfo->bestpredstate[conv];
			predstateinfo = (MState*)PredMDPstate->PlannerSpecificData;

			//set its best next info
			predstateinfo->bestnextstate[conv] = MDPstate;

			//check the decrease of g-values along the path
			if(predstateinfo->g[conv] >= stateinfo->g[conv])
			{
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



void MPlanner::PrintSearchPath(MSearchStateSpace_t* pSearchStateSpace, FILE* fOut)
{
	MState* searchstateinfo;
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

	PathCost = ((MState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g[conv];

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

		searchstateinfo = (MState*)state->PlannerSpecificData;

		if(searchstateinfo->bestnextstate[conv] == NULL)
		{
			SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}
		if(searchstateinfo->g[conv] == INFINITECOST)
		{
			SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
			break;
		}

		int costToGoal = PathCost - costFromStart;
		int transcost = searchstateinfo->g[conv] - ((MState*)(searchstateinfo->bestnextstate[conv]->PlannerSpecificData))->v[conv];
		if(bforwardsearch)
			transcost = -transcost;

		costFromStart += transcost;

		SBPL_FPRINTF(fOut, "g=%d-->state %d, h = %d, ctg = %d  ", searchstateinfo->g[conv], 			
			searchstateinfo->bestnextstate[conv]->StateID, searchstateinfo->h[conv], costToGoal);

		state = searchstateinfo->bestnextstate[conv];

		environment_->PrintState(state->StateID, false, fOut);



	}
}

void MPlanner::PrintSearchState(MState* state, FILE* fOut)
{
	/*SBPL_FPRINTF(fOut, "state %d: h1=%d h2=%d g=%u v=%u iterc=%d callnuma=%d expands=%d heapind=%d inconslist=%d\n",
		state->MDPstate->StateID, state->h1, state->h2, state->g, state->v, 
		state->iterationclosed, state->callnumberaccessed1, state->numofexpands, state->heapindex, state->listelem[M_INCONS_LIST_ID1]?1:0);*/
	environment_->PrintState(state->MDPstate->StateID, true, fOut);

}



int MPlanner::getHeurValue(MSearchStateSpace_t* pSearchStateSpace, int StateID)
{
	CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
	MState* searchstateinfo = (MState*)MDPstate->PlannerSpecificData;
	return searchstateinfo->h[0];
}


vector<int> MPlanner::GetSearchPath(MSearchStateSpace_t* pSearchStateSpace, int& solcost)
{
  vector<int> SuccIDV;
  vector<int> CostV;
  vector<int> wholePathIds;
  MState* searchstateinfo;
  CMDPSTATE* state = NULL; 
  CMDPSTATE* goalstate = NULL;
  CMDPSTATE* startstate=NULL;
  
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
  while(state->StateID != goalstate->StateID)
    {
      if(state->PlannerSpecificData == NULL)
	{
	  SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
	  break;
	}
      
      searchstateinfo = (MState*)state->PlannerSpecificData;
      
      if(searchstateinfo->bestnextstate[conv] == NULL)
	{
	  SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
	  break;
	}

	if(searchstateinfo->g[conv] == INFINITECOST)
	{
	  SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
	  break;
	}
      
      environment_->GetSuccs(state->StateID, &SuccIDV, &CostV);
      int actioncost = INFINITECOST;
      for(int i = 0; i < (int)SuccIDV.size(); i++)
        {   
	  
    if(SuccIDV.at(i) == searchstateinfo->bestnextstate[conv]->StateID && CostV.at(i)<actioncost)
	    actioncost = CostV.at(i);
	  
        }
      if(actioncost == INFINITECOST)
	SBPL_PRINTF("WARNING: actioncost = %d\n", actioncost);
      
      solcost += actioncost;
      
    /*  SBPL_FPRINTF(stdout, "actioncost=%d between states %d and %d\n", 
              actioncost, state->StateID, searchstateinfo->bestnextstate[conv]->StateID);
      environment_->PrintState(state->StateID, false, stdout);
      environment_->PrintState(searchstateinfo->bestnextstate[conv]->StateID, false, stdout);
     */ 
      
#if DEBUG
      MState* nextstateinfo = (MState*)(searchstateinfo->bestnextstate[conv]->PlannerSpecificData);
      if(actioncost != abs((int)(searchstateinfo->g[conv] - nextstateinfo->g[conv])) && pSearchStateSpace->eps_satisfied <= 1.001)
	{
	  SBPL_FPRINTF(fDeb, "ERROR: actioncost=%d is not matching the difference in g-values of %d\n", 
		  actioncost, abs((int)(searchstateinfo->g[conv] - nextstateinfo->g[conv])));
	  SBPL_ERROR("ERROR: actioncost=%d is not matching the difference in g-values of %d\n", 
		 actioncost,abs((int)(searchstateinfo->g[conv] - nextstateinfo->g[conv])));
	  PrintSearchState(searchstateinfo, fDeb);
	  PrintSearchState(nextstateinfo, fDeb);
	}
#endif
      
      
      state = searchstateinfo->bestnextstate[conv];
      
      wholePathIds.push_back(state->StateID);
    }


  return wholePathIds;
}
	

bool MPlanner::Search(MSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs)
{
	CKey key;
	TimeStarted = clock();
    searchexpands = 0;
	SBPL_FPRINTF(stdout, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);

	pSearchStateSpace->callnumber++;
    if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
        ReInitializeSearchStateSpace(pSearchStateSpace);
    }

	if(bOptimalSolution)
	{
		pSearchStateSpace->eps1 = 1;
		MaxNumofSecs = INFINITECOST;
	}
	else if(bFirstSolution)
	{
		MaxNumofSecs = INFINITECOST;
	}

	//ensure heuristics are up-to-date
	clock_t loop_time;
        loop_time = clock();
	environment_->EnsureHeuristicsUpdated((bforwardsearch==true)); //Venkat : Please make sure all the heuristics are precaculated
	int prevexpands = 0;
	while(pSearchStateSpace->eps_satisfied > M_FINAL_EPS && 
		(clock()- TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC)
	{
        	loop_time = clock();
		for (int ii = 0; ii < env_num; ii++) {
		  expand_s[ii] = 0; 
		}
                pSearchStateSpace->searchiteration++;// = pSearchStateSpace->eps1;
	        if(ImprovePathRoundRobin(pSearchStateSpace, MaxNumofSecs) == 1){
			pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps1*pSearchStateSpace->eps2;
		}
		int mincost =  ((MState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g[conv]; 
		/*if(ImprovePathRoundRobinShared (pSearchStateSpace, MaxNumofSecs) == 1){
			pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps1*pSearchStateSpace->eps2;
		}
		int mincost = ((MState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g[0];*/
		SBPL_PRINTF("eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,
				mincost,double(clock()-loop_time)/CLOCKS_PER_SEC);
		//if just the first solution then we are done
		if(bFirstSolution)
			break;
		//no solution exists
		if(((MState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g[conv] == INFINITECOST) 
			break;

	}

#if DEBUG
	SBPL_FFLUSH(fDeb);
#endif

	clock_t final_time = (clock()-TimeStarted);
	PathCost = ((MState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g[conv];
	/*float MM = 0;
	for (int oo =0; oo < env_num; oo++){ 
		MaxMemoryCounter += env_[oo]->StateID2IndexMapping.size()*sizeof(int);
		MaxMemoryCounter += sizeof(pSearchStateSpace->inconslist[oo]);
		MaxMemoryCounter += pSearchStateSpace->heap->allocated[oo]*sizeof(int);
		MM += (pSearchStateSpace->heap->allocated[oo]*sizeof(MState)/1000000.0);
		MM += (sizeof(pSearchStateSpace->inconslist[oo])*sizeof(MState))/1000000.0;
                //printf("q [%d] => m1=%d, m2=%d, m3=%d\n", oo, int (env_[oo]->StateID2IndexMapping.size()*sizeof(int)),int (sizeof(pSearchStateSpace->inconslist[oo])), pSearchStateSpace->heap->allocated[oo]);   
		//printf("sizeof heap -- %ld\n", sizeof(pSearchStateSpace->heap));
	}*/	
	//MaxMemoryCounter += sizeof(pSearchStateSpace->heap); //->allocated[oo];
	SBPL_PRINTF("MaxMemoryCounter = %ld\n", MaxMemoryCounter);

	int solcost = INFINITECOST;
	bool ret = false;
	if(PathCost == INFINITECOST)
	{
		SBPL_PRINTF("could not find a solution\n");
		ret = false;
	}
	else
	{
		SBPL_PRINTF("solution is found\n");      
		pathIds = GetSearchPath(pSearchStateSpace, solcost);
        	ret = true;
	}

	SBPL_PRINTF("total expands this call = %d, planning time = %.3f secs, memory = %0.2f, solution cost=%d\n", 
           searchexpands, final_time/((double)CLOCKS_PER_SEC), MaxMemoryCounter/1000000.0, solcost);
        final_eps_planning_time = (clock()-TimeStarted)/((double)CLOCKS_PER_SEC);
        final_eps = pSearchStateSpace->eps_satisfied;
	return ret;

}


//-----------------------------Interface function-----------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int MPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
	int solcost;

	return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
	
}

//returns 1 if found a solution, and 0 otherwise
int MPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost)
{
  vector<int> pathIds; 
  bool bFound = false;
  int PathCost;
  bool bFirstSolution = true; //this->bsearchuntilfirstsolution;
  bool bOptimalSolution = false;
  *psolcost = 0;
  
  SBPL_PRINTF("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);
  
  //plan
  if((bFound = Search(pSearchStateSpace_, pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false) 
    {
      SBPL_PRINTF("failed to find a solution\n");
    }
  
  //copy the solution
  *solution_stateIDs_V = pathIds;
  *psolcost = PathCost;
  
  return (int)bFound;

}


int MPlanner::set_goal(int goal_stateID, int i)
{
	//deprecated
	return 1;
}
int MPlanner::set_goal(int goal_stateID)
{

	SBPL_PRINTF("planner: setting goal to %d\n", goal_stateID);
	environment_->PrintState(goal_stateID, true, stdout);
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

int MPlanner::set_start(int start_stateID)
{

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
    return 1;
}



void MPlanner::costs_changed(StateChangeQuery const & stateChange)
{


    pSearchStateSpace_->bReinitializeSearchStateSpace = true;


}

void MPlanner::costs_changed()
{

    pSearchStateSpace_->bReinitializeSearchStateSpace = true;

}



int MPlanner::force_planning_from_scratch()
{
	SBPL_PRINTF("planner: forceplanfromscratch set\n");

    pSearchStateSpace_->bReinitializeSearchStateSpace = true;

    return 1;
}


int MPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{

	SBPL_PRINTF("planner: search mode set to %d\n", bSearchUntilFirstSolution);

	bsearchuntilfirstsolution = bSearchUntilFirstSolution;

	return 1;
}


void MPlanner::print_searchpath(FILE* fOut)
{
	PrintSearchPath(pSearchStateSpace_, fOut);
}
