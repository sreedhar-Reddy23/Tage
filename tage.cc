#include <bitset>
#include <cstdlib>
#include "ooo_cpu.h"

/*Result of Three bit predictor of TAGE*/
enum class TaGePredResult : uint8_t {
    tStronglyNotTaken = 0,
    tModeratelyNotTaken2,
    tModeratelyNotTaken1,
    tWeaklyNotTaken,
    tWeaklyTaken,
    tModeratelyTaken1,
    tModeratelyTaken2,
    tStronglyTaken = 7
};

/*Result of Two bit predictor of Bi-Modal*/
enum class BiModPredResult : uint8_t {
    StronglyNotTaken = 0,
    WeaklyNotTaken,
    WeaklyTaken,
    StronglyTaken = 3
};

/* Macros */
#define TAKEN 			          1 	  /*Branch Taken */
#define NOT_TAKEN 		          0		  /*Branch not Taken */
#define BIMODAL_TABLE_ROWS        13      /*No: of rows in Bimodal pred table*/
#define TAGE_USEFULBIT_MAX        3       /* 2-bit useful bit max value*/
#define NUM_TAGE_TABLES           12       /*No: of TAGE Pred tables*/
#define USE_ALT_ON_NA_CTR_MAX     15      /* USE_ALT_ON_NA is 4 bit*/
#define USE_ALT_ON_NA_CTR_INIT    8       /*Intialisation of ctr as weakly taken*/
#define PATH_HIST_REG_LEN         16      /*Length of Path History Register*/
#define CYCLE_PERIOD              1<<20   /*No: of cycles required for reset or flush of Usefulbit*/
#define INIT_BIMODAL_PRED         (uint32_t)BiModPredResult::WeaklyTaken

/* Constants for TAGE table 12 Component configuration */
const uint32_t ghistLen[NUM_TAGE_TABLES] = {4, 6, 10, 16, 25, 40, 64, 101, 160, 254, 403, 640};
const uint32_t TAGE_TABLE_SIZE[NUM_TAGE_TABLES] = {10, 10, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9};
const uint32_t TAGE_TAG_SIZE[NUM_TAGE_TABLES] = {7, 7, 8, 8, 9, 10, 11, 12, 12, 13, 14, 15};


/******************* STORAGE BUDGET JUSTIFICATION ********************************/
/**  Binomial table : 2 ^ 13 2 - bit counters = 2 ^ 14 bits = 16Kb              **/
/**  ghistLen :  4+6+....+640 =  1723 bits   = 1.68Kb                           **/
/**  Tage tables : 16                                                           **/
/**  Each entry:  tag size + 3 bit prediction counter + 2 bit useful counter    **/
/**  TableSize * Each entry =  12Kb + 12Kb + 26Kb + .....+ 10Kb = 221.5Kb       **/
/**  Total Size = sum of Tage table sizes + Binom table + ghistLen =  239.18Kb  **/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

/* GHRfold class for folding the Global History Register */
class GHRfold
{
public:
  uint32_t val;
  uint32_t ComprLen;
  uint32_t OrgLen;
  uint32_t OutPoint;
	
  /* Constructor for GHRfold*/
  GHRfold (){}
  
  /* Initialize the GHR*/
  void init (uint32_t original_length, uint32_t compressed_length)
  {
    val = 0;
    OrgLen = original_length;
    ComprLen = compressed_length;
    OutPoint = OrgLen % ComprLen;
  }
  
  /* update the GHR (Hashing)*/
  void update (std::bitset<1024> ghr)
  {
    val = (val << 1) | ghr[0];
    val ^= ghr[OrgLen] << OutPoint;
    val ^= (val >> ComprLen);
    val &= (1 << ComprLen) - 1;
  }
};

/* bimodPred class for Bi-Modal base branch predictor */
class bimodPred{
public:
    struct bimodalentry
    {
        uint32_t pred;
    };
	
    uint32_t numBimodalEntries;         /*Number of Entries in Bimodal Table*/
    bimodalentry *bimodalTable;
	
	/* Constructor for bimodPred*/
    bimodPred() {}
	
	/* Initialize the Bi-Modal predictor */
    void init() {
        
        numBimodalEntries = (1 << BIMODAL_TABLE_ROWS); //2^13
        bimodalTable = new bimodalentry[numBimodalEntries];
        for (uint32_t i = 0; i < numBimodalEntries; i++)
        {
            bimodalTable[i].pred = INIT_BIMODAL_PRED; //2^13*2 = 2^14
        }

    }

};

/* TAGEpred class for Tage tables*/
class TAGEpred{
public:
    struct tageentry
    {
        uint8_t pred;
        uint16_t tag;
        uint8_t u;
    };
    
    tageentry *tagTables;               
    uint32_t TageTableSize;            
    uint32_t TageTagSize;              
    uint32_t TageHistory;
    uint32_t TageTableIndex;                
    uint32_t TageTag; 
		
	/* Constructor for TAGEpred*/
    TAGEpred() {}
	
	/*To reset the entries in the Table*/
    void reset(tageentry *Entry)
    {
        Entry->pred = 0;
        Entry->tag = 0;
        Entry->u = 0;
    }
};


std::bitset<1024> ghr;                /*Global History Register */      
uint32_t PHR;                         /*Path History Register */
bimodPred BiModPred;                  /* Bi-Modal branch predictor */
TAGEpred  tagePred[NUM_TAGE_TABLES];  /* TAGE branch predictors */
GHRfold CSRindex[NUM_TAGE_TABLES];    /* Compressed GHR for table indices */
GHRfold CSRTag[2][NUM_TAGE_TABLES];   /* Compressed GHR for tags */


bool final_pred;             /* Final branch prediction */
bool alt_pred;               /* Alternative branch prediction */
int main_table;              /* Main TAGE table used for prediction */
int alt_table;               /* Alternative TAGE table used for prediction */
uint32_t main_idx;           /* Index of main TAGE table entry */
uint32_t alt_idx;            /* Index of alternative TAGE table entry */
uint32_t clockCycle_cnt;     /* Clock cycle counter */                   
bool conf_state;             /* Confidence state */       
int32_t alt_conf;            /* Alternative confidence counter */  
uint8_t pred_dir;            /* Predicted branch direction */

/****** Function prototypes **********/

/*predict functions*/
void calcualteTageTags(uint64_t& pc);
void calculateTageTableIndices(uint64_t& pc);
void findMatchingAndAltPred();
uint8_t predictBranchDir( uint32_t& bimodalEntryIndex);

/*update functions*/
void updatePredictionCounters(uint8_t& taken, uint32_t& bimodalEntryIndex, int& predictionVal, int& altPredVal);
void updateAltBetterCount(uint8_t& taken);
void updateTageTablesOnMissPred(uint8_t& taken);
void updateUsefulBits(uint8_t& taken);
void updateClockState();
void updateGHR(uint8_t& taken);
void updatePHR(uint64_t PC);
/**************************************/

/* Initialize the branch predictor */
void O3_CPU::initialize_branch_predictor()
{
    uint32_t i,j;
    
	// Initialize TAGE predictor tables
    for(i = 0; i <NUM_TAGE_TABLES; i++ )
    {
        tagePred[i].TageTableSize =  TAGE_TABLE_SIZE[i];
        tagePred[i].TageTagSize = TAGE_TAG_SIZE[i];
        uint32_t tableSize = (1<<tagePred[i].TageTableSize);
        tagePred[i].tagTables = new TAGEpred::tageentry[tableSize];
        for(j = 0; j <tableSize; j++ )
        {
            tagePred[i].reset(&tagePred[i].tagTables[j]);   // Reset each TAGE table entry
        }
        tagePred[i].TageHistory = ghistLen[NUM_TAGE_TABLES - i - 1];
        tagePred[i].TageTableIndex = 0;
        tagePred[i].TageTag = 0;  
    }
	
	// Initialize Bi-Modal predictor
    BiModPred.init();

    for(i = 0; i<NUM_TAGE_TABLES; i++){
        CSRindex[i].init(tagePred[i].TageHistory, tagePred[i].TageTagSize);
        CSRTag[0][i].init(tagePred[i].TageHistory, tagePred[i].TageTagSize);
        CSRTag[1][i].init(tagePred[i].TageHistory, (tagePred[i].TageTagSize-1));
    }

	// Initialize other variables
	
    final_pred = -1;
    alt_pred = -1;
    main_table = NUM_TAGE_TABLES;
    alt_table = NUM_TAGE_TABLES;

    clockCycle_cnt = 0;
    conf_state = 0;

    PHR = 0;
    ghr.reset();
    alt_conf = USE_ALT_ON_NA_CTR_INIT;

}

/* Predict a branch */
uint8_t O3_CPU::predict_branch(uint64_t pc)
{
	/* Calculate Bimodal Entry Index = F(PC)*/
    uint32_t bimodalEntryIndex = (pc) % (BiModPred.numBimodalEntries); 
	
	// Resetting prediction variables
    final_pred = -1;
    alt_pred = -1;
    main_table = NUM_TAGE_TABLES;
    alt_table = NUM_TAGE_TABLES;
	
	// Calculate TAGE tags
    calcualteTageTags(pc);
	
	// Calculate TAGE table indices
    calculateTageTableIndices(pc);
	
	// Find matching and alternative predictions
    findMatchingAndAltPred();
	
	// Predict branch direction based on TAGE and Bi-Modal predictors
    return predictBranchDir(bimodalEntryIndex);
}

/* Update the branch predictor with the last branch result */
void O3_CPU::last_branch_result(uint64_t PC, uint64_t branch_target, uint8_t taken, uint8_t branch_type)
{
	/* Calculate Bimodal Entry Index = F(PC)*/
    uint32_t bimodalEntryIndex = (PC) % (BiModPred.numBimodalEntries);  

    int predictionVal = -1;
    int altPredVal = -1;

    updatePredictionCounters(taken, bimodalEntryIndex, predictionVal, altPredVal);
    updateAltBetterCount(taken);
    updateTageTablesOnMissPred(taken);
    updateUsefulBits(taken);
    updateClockState();
    updateGHR(taken);
    updatePHR(PC);
}

/************************ Helper functions **********************************/

/* Hashing PHR = F(PC) */
void updatePHR(uint64_t PC) {
    PHR = (PHR << 1);
    if (PC & 1)
        PHR = PHR + 1;
    PHR = (PHR & ((1 << PATH_HIST_REG_LEN) - 1));
}

/*Update GHR*/
void updateGHR(uint8_t& taken) {
    ghr = ghr << 1;
    if (taken == TAKEN)
        ghr.set(0, 1);
	
	 // Update compressed GHR for table indices and tags
    for (int i = 0; i < NUM_TAGE_TABLES; i++) {
        CSRindex[i].update(ghr);
        CSRTag[0][i].update(ghr);
        CSRTag[1][i].update(ghr);
    }
}


void updateClockState() {
	
	// Increment clock cycle counter
    clockCycle_cnt++;
	
	// Reset useful bits periodically based on CYCLE_PERIOD
    if (clockCycle_cnt == (1 << CYCLE_PERIOD)) {
        clockCycle_cnt = 0;
        conf_state = 1 - conf_state;

        for (uint32_t i = 0; i < NUM_TAGE_TABLES; i++)
            for (uint32_t j = 0; j < (1 << tagePred[i].TageTableSize); j++)
                tagePred[i].tagTables[j].u &= (conf_state + 1);
    }
}

// Update useful bits for the main TAGE table entry for confidence
void updateUsefulBits(uint8_t& taken) {
	
	// Update useful bits only if a main table is used
    if (main_table < NUM_TAGE_TABLES) {
		
		// If the final and alternative predictions differ
        if ((pred_dir != alt_pred)) {
			
			// If the final prediction matches the taken branch
            if (pred_dir == taken && tagePred[main_table].tagTables[main_idx].u < TAGE_USEFULBIT_MAX)
				
				// Increment the useful bit counter (up to the maximum value)
                tagePred[main_table].tagTables[main_idx].u += 1;
			
			// If the final prediction does not match the taken branch
            else if (pred_dir != taken && tagePred[main_table].tagTables[main_idx].u > 0)
				
				// Decrement the useful bit counter (down to zero)
                tagePred[main_table].tagTables[main_idx].u -= 1;
        }
    }
}

// Update TAGE tables on misprediction
void updateTageTablesOnMissPred(uint8_t& taken) {

	// Update TAGE tables on misprediction only if a main table is used
    if (((pred_dir != taken) & (main_table > 0))) {
        bool alloc = false;
		
		// Check if any lower table entry has a zero useful bit counter
        for (int i = 0; i < main_table; i++)
            if (tagePred[i].tagTables[tagePred[i].TageTableIndex].u == 0)
                alloc = true;
		
		// If no lower table entry has a zero useful bit counter
        if (!alloc) {
			
			// Decrement the useful bit counter for all lower tables
            for (int i = main_table - 1; i >= 0; i--)
                tagePred[i].tagTables[tagePred[i].TageTableIndex].u--;
        }
        else {
			// Allocate a new entry in the lower tables
            for (int i = main_table - 1; i >= 0; i--) {
				
				// If the entry has a zero useful bit counter and a random condition is met
                if ((tagePred[i].tagTables[tagePred[i].TageTableIndex].u == 0 && !(rand() % 10))) {
					
					// Set the prediction counter based on the taken branch
                    if (taken) {
                        tagePred[i].tagTables[tagePred[i].TageTableIndex].pred = (uint8_t)TaGePredResult::tWeaklyTaken;
                    }
                    else {
                        tagePred[i].tagTables[tagePred[i].TageTableIndex].pred = (uint8_t)TaGePredResult::tWeaklyNotTaken;
                    }
					
					// Update the tag and reset the useful bit counter
                    tagePred[i].tagTables[tagePred[i].TageTableIndex].tag = tagePred[i].TageTag;
                    tagePred[i].tagTables[tagePred[i].TageTableIndex].u = 0;
                    break;
                }
            }
        }
    }

}

// Update alternative confidence counter
void updateAltBetterCount(uint8_t& taken) {
	
	// If a main table is used
    if (main_table < NUM_TAGE_TABLES) {
        if ((tagePred[main_table].tagTables[main_idx].u == 0) &&
            ((tagePred[main_table].tagTables[main_idx].pred == (uint8_t)TaGePredResult::tWeaklyNotTaken) ||
                (tagePred[main_table].tagTables[main_idx].pred == (uint8_t)TaGePredResult::tWeaklyTaken))) {


            if (final_pred != alt_pred) {
                if (alt_pred == taken) {
                    if (alt_conf < USE_ALT_ON_NA_CTR_MAX)
                        alt_conf++;
                }
                else if (alt_conf > 0)
                    alt_conf--;
            }
        }
    }

}

// Update Prediction Counters
void updatePredictionCounters(uint8_t& taken, uint32_t& bimodalEntryIndex, int& predictionVal, int& altPredVal) {
	
	// If a main table is used
    if (main_table < NUM_TAGE_TABLES) {
		
		// Get the prediction value from the main table
        predictionVal = tagePred[main_table].tagTables[main_idx].pred;
		
		// Update the prediction counter based on the taken branch
		
        if (taken && predictionVal < (uint8_t)TaGePredResult::tStronglyTaken)
            ++(tagePred[main_table].tagTables[main_idx].pred);
        else if (!taken && predictionVal > 0)
            --(tagePred[main_table].tagTables[main_idx].pred);
		
		// Get the alternative prediction value if an alternative table is used
        altPredVal = -1;
        if (alt_table != NUM_TAGE_TABLES)
            altPredVal = tagePred[alt_table].tagTables[alt_idx].pred;
		
		// If the main table entry has a zero useful bit counter
        if (tagePred[main_table].tagTables[main_idx].u == 0 && altPredVal != -1) {
			
			// Update the alternative prediction counter based on the taken branch
            if (taken && altPredVal < (uint8_t)TaGePredResult::tStronglyTaken)
                ++(tagePred[alt_table].tagTables[alt_idx].pred);
            else if (!taken && altPredVal > 0)
                --(tagePred[alt_table].tagTables[alt_idx].pred);
        }
    }
    else {
		
		// If no main table is used, get the prediction value from the Bi-Modal predictor
        predictionVal = BiModPred.bimodalTable[bimodalEntryIndex].pred;
		
		// Update the Bi-Modal prediction counter based on the taken branch
        if (taken && predictionVal < (uint8_t)BiModPredResult::StronglyTaken)
            ++(BiModPred.bimodalTable[bimodalEntryIndex].pred);
        else if (!taken && predictionVal > 0)
            --(BiModPred.bimodalTable[bimodalEntryIndex].pred);
    }

}

//calcualte Tage Tags by hashing F(PC, GHR, CSR)
void calcualteTageTags(uint64_t& pc) {

    for (int i = 0; i < NUM_TAGE_TABLES; i++)
        tagePred[i].TageTag = (pc ^ CSRTag[0][i].val ^ (CSRTag[1][i].val << 1)) & ((1 << tagePred[i].TageTagSize) - 1);

}

//calcualte Tage Table Indices by hashing F(PC, GHR, PHR, CSR)
void calculateTageTableIndices(uint64_t& pc) {

    uint32_t offset[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < NUM_TAGE_TABLES; i++)
        tagePred[i].TageTableIndex = (pc ^ (pc >> tagePred[i].TageTableSize) ^ CSRindex[i].val ^ PHR ^ (PHR & ((1 << offset[i]) - 1))) & ((1 << tagePred[i].TageTableSize) - 1);

}

// Find matching and alternative predictions
void findMatchingAndAltPred() {
	
	// Find the main table that matches the tag
    for (uint32_t i = 0; i < NUM_TAGE_TABLES; i++)
        if (tagePred[i].tagTables[tagePred[i].TageTableIndex].tag == tagePred[i].TageTag) {
            main_table = i;
            main_idx = tagePred[i].TageTableIndex;
            break;
        }
	
	// Find the alternative table that matches the tag (if any)
    for (uint32_t i = main_table + 1; i < NUM_TAGE_TABLES; i++)
        if (tagePred[i].tagTables[tagePred[i].TageTableIndex].tag == tagePred[i].TageTag) {
            alt_table = i;
            alt_idx = tagePred[i].TageTableIndex;
            break;
        }

}

// Predict branch direction using Bi-Modal predictor or TAGE predictor
uint8_t predictBranchDir(uint32_t& bimodalEntryIndex) {
	
	// If a main table is used
    if (main_table < NUM_TAGE_TABLES) {
		
		// If no alternative table is used, get the alternative prediction from the Bi-Modal predictor
        if (alt_table == NUM_TAGE_TABLES) {
            alt_pred = (BiModPred.bimodalTable[bimodalEntryIndex].pred > (uint8_t)BiModPredResult::WeaklyNotTaken);
        }
        else {
			
			// Get the alternative prediction from the alternative table
            if (tagePred[alt_table].tagTables[alt_idx].pred >= (uint8_t)TaGePredResult::tWeaklyNotTaken)
                alt_pred = TAKEN;
            else
                alt_pred = NOT_TAKEN;
        }
		
		// Determine the final prediction based on the main table prediction, useful bit counter, and alternative confidence counter
        if ((tagePred[main_table].tagTables[main_idx].pred != (uint8_t)TaGePredResult::tWeaklyNotTaken) ||
            (tagePred[main_table].tagTables[main_idx].pred != (uint8_t)TaGePredResult::tWeaklyTaken) ||
            (tagePred[main_table].tagTables[main_idx].u != 0) ||
            (alt_conf < USE_ALT_ON_NA_CTR_INIT)) {

            final_pred = tagePred[main_table].tagTables[main_idx].pred >= (uint8_t)TaGePredResult::tWeaklyNotTaken;
            return pred_dir = final_pred;
        }
        else {
            return pred_dir = alt_pred;
        }
    }
    else {
        alt_pred = (BiModPred.bimodalTable[bimodalEntryIndex].pred > (uint8_t)(BiModPredResult::WeaklyNotTaken));
        return pred_dir = alt_pred;
    }

}
