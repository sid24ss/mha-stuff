#include <iostream>
using namespace std;

#include <sbpl/sbpl/headers.h>

void DynamicEnvironmentNAVXYTHETALATTICE::ReadMapData(FILE* fCfg)
{
	int dTemp = 0;
	char sTemp[100];
	//environment:
	if(fscanf(fCfg, "%s", sTemp) != 1){
		SBPL_ERROR("ERROR: ran out of env file early\n");
		throw new SBPL_Exception();
	}

	for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++)
	{
		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				SBPL_ERROR("ERROR: reading in cell (%d / %d , %d / %d)\n", x, y, EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c);
				
				SBPL_ERROR("ERROR: incorrect format of config file\n");
				throw new SBPL_Exception();
			}
			TrueGrid2D[x][y] = dTemp;
		}
	}
}

void DynamicEnvironmentNAVXYTHETALATTICE::StoreMapData(const char* fname){
	FILE* fOut = fopen(fname, "w");
	fprintf(fOut, "map:\n");
	for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++)
	{
		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
		{
			fprintf(fOut, "%d ", TrueGrid2D[x][y]);
		}
		fprintf(fOut, "\n");
	}
	fflush(fOut);
	fclose(fOut);
}

void DynamicEnvironmentNAVXYTHETALATTICE::ComputeModifiedCells(int x, int y, std::vector<nav2dcell_t> *modCells){
	/*if(bDetected2D[x][y]){
		//already detected!
		return;
	}*/
	bool BeliefObstacle = (EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh);
	bool DetectObstacle = (TrueGrid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh);
	
	bool bModified = false;
	if(BeliefObstacle && !DetectObstacle){
		//modified!
		//cost decrease!!! (effectively removing an obstacle) -- recompute heuristic values
		bModified = true;
	}
	if(!BeliefObstacle && DetectObstacle){
		//modified!
		//cost increase!!! (effectively adding an obstacle)
		bModified = true;
	}
	if(bModified){
		//printf("belief/detect disparity at %d,%d!\n", x, y); fflush(stdout);
		nav2dcell_t cell;
		cell.x = x;
		cell.y = y;
		modCells->push_back(cell);
	}
}

void DynamicEnvironmentNAVXYTHETALATTICE::GenerateRandomMap(double prob_free, int size1, double prob_obs, int size2)
{
	for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++)
	{
		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
		{
			if(EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh)
			{
				//the cell is an obstacle
				//flip to non-obstacle with small probability
				double p = ((double)(rand() % 10000)) / 10000.0;
				if(p <= prob_obs){
					int min_x = max(0, x - size2 / 2);
					int max_x = min(EnvNAVXYTHETALATCfg.EnvWidth_c - 1, x + size2 / 2);
					int min_y = max(0, y - size2 / 2);
					int max_y = min(EnvNAVXYTHETALATCfg.EnvHeight_c - 1, y + size2 / 2);
					for(int x_ = min_x; x_ < max_x; x_++){
						for(int y_ = min_y; y_ < max_y; y_++){
							TrueGrid2D[x_][y_] = 0;			
						}		
					}
				}
			} else {
				//the cell is not an obstacle
				//flip to obstacle with small probability
				int dxS = x - EnvNAVXYTHETALATCfg.StartX_c;
				int dyS = y - EnvNAVXYTHETALATCfg.StartY_c;
				int dxG = x - EnvNAVXYTHETALATCfg.EndX_c;
				int dyG = y - EnvNAVXYTHETALATCfg.EndY_c;
				int distsqS = (dxS * dxS) + (dyS * dyS);
				int distsqG = (dxG * dxG) + (dyG * dyG);
				int distsq = min(distsqS, distsqG);
				double p = ((double)(rand() % 10000)) / 10000.0;
				if(p <= prob_free && distsq > 25){
					int min_x = max(0, x - size1 / 2);
					int max_x = min(EnvNAVXYTHETALATCfg.EnvWidth_c - 1, x + size1 / 2);
					int min_y = max(0, y - size1 / 2);
					int max_y = min(EnvNAVXYTHETALATCfg.EnvHeight_c - 1, y + size1 / 2);
					for(int x_ = min_x; x_ < max_x; x_++){
						for(int y_ = min_y; y_ < max_y; y_++){
							TrueGrid2D[x_][y_] = EnvNAVXYTHETALATCfg.obsthresh;
						}		
					}
				}
			}
		}
	}
}

bool DynamicEnvironmentNAVXYTHETALATTICE::DynInitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile, const char* sMapFile)
{
	EnvNAVXYTHETALATCfg.FootprintPolygon = perimeterptsV;

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
		throw new SBPL_Exception();
	}
	printf("Reading config file: %s...", sEnvFile); fflush(stdout);
	
	ReadConfiguration(fCfg);
  	fclose(fCfg);
  	
	printf("done!"); fflush(stdout);
  	
  	if(TrueGrid2D == NULL){
		//allocate the 2D environment
		TrueGrid2D = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
  		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
			TrueGrid2D[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
		}
	}
	
	if(expandsGrid == NULL){
		//allocate the 2D environment
		expandsGrid = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
  		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
			expandsGrid[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
			for(int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
				expandsGrid[x][y] = 0;
			}
		}
	}
	
	if(bDetected2D == NULL){
		//allocate the 2D environment
		bDetected2D = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
  		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
			bDetected2D[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
		}
	}
	
	//init TrueGrid2D the same as in config file
	for(int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
		for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
			TrueGrid2D[x][y] = EnvNAVXYTHETALATCfg.Grid2D[x][y];
		}
	}
  	
  	//read in map data from file (which actual "simulated sensor map" and is slightly different than the belief map (blueprint) in the config file
  	FILE* fMap = fopen(sMapFile, "r");
  	if(fMap == NULL)
  	{
  		SBPL_WARN("WARNING: unable to open map file %s\n", sMapFile);
  		GenerateRandomMap(0.0005, 10, 0.002, 8);
  		StoreMapData(sMapFile);
  		SBPL_WARN("WARNING: Generated random-ish map in file %s\n", sMapFile);
  		fMap = fopen(sMapFile, "r");
  		if(fMap == NULL) {
  			SBPL_ERROR("ERROR: could not get a map!");
  			exit(1);
  		}
  	}
  	ReadMapData(fMap);
  	fclose(fMap);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
			throw new SBPL_Exception();
		}
		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
			throw new SBPL_Exception();
		}
		InitGeneral(&EnvNAVXYTHETALATCfg.mprimV);
    		fclose(fMotPrim);
	}
	else
	{
		InitGeneral(NULL);
	}

	SBPL_PRINTF("size of env: %d by %d\n", EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c);

	return true;
}

DynamicEnvironmentNAVXYTHETALATTICE::DynamicEnvironmentNAVXYTHETALATTICE(){

}
