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
#ifndef __SBPL_HEADERS_H_
#define __SBPL_HEADERS_H_

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>

#include <sbpl/sbpl/sbpl_exception.h>
#include <sbpl/sbpl/config.h>


#if MEM_CHECK == 1
#define _CRTDBG_MAP_ALLOC 
#define CRTDBG_MAP_ALLOC
#endif

#include <stdlib.h> //have to go after the defines above

#if MEM_CHECK == 1
#include <crtdbg.h>
#endif

#include <sbpl/utils/key.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/utils.h>
#include <sbpl/planners/planner.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/discrete_space_information/adaptive_environment.h>
#include <sbpl/discrete_space_information/generic_adaptive_environment.h>
#include <sbpl/discrete_space_information/template/environment_XXX.h>
#include <sbpl/discrete_space_information/nav2d/environment_nav2D.h>
#include <sbpl/discrete_space_information/navxythetalat/environment_navxythetalat.h>
#include <sbpl/discrete_space_information/navxythetalat/environment_dynnavxythetalat.h>
#include <sbpl/discrete_space_information/navxythetalat/environment_navxythetalat_adaptive.h>
#include <sbpl/discrete_space_information/navxythetalat/environment_navxythetamlevlat.h>
#include <sbpl/discrete_space_information/robarm/environment_robarm.h>
#include <sbpl/discrete_space_information/nav2d_uu/environment_nav2Duu.h>
#include <sbpl/discrete_space_information/adaptive_nav/adaptive_env3D2D.h>
#include <sbpl/utils/list.h>
#include <sbpl/utils/heap.h>
#include <sbpl/planners/VI/viplanner.h>
#include <sbpl/planners/ARAStar/araplanner.h>
#include <sbpl/planners/TRAStar/traplanner.h>
#include <sbpl/planners/ADStar/adplanner.h>
#include <sbpl/planners/MHA/mplanner.h>
#include <sbpl/planners/AdaptivePlanner/araplanner_ad.h>
#include <sbpl/planners/AdaptivePlanner/adaptive_planner.h>
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/planners/PPCP/ppcpplanner.h>
#include <sbpl/planners/RStar/rstarplanner.h>
#include <sbpl/planners/ANA/ANAplanner.h>


#endif

