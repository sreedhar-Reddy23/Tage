# TAGE Branch Predictor

A high-performance branch predictor implementation using the **TAgged GEometric (TAGE)** algorithm, designed for modern out-of-order processors.

## Overview

This project implements a TAGE branch predictor that combines:
- **Bi-modal predictor** as the base predictor
- **12 TAGE component tables** with different history lengths
- **Global History Register (GHR)** for pattern recognition
- **Path History Register (PHR)** for additional context

The TAGE predictor achieves superior accuracy by leveraging multiple prediction tables with varying history lengths, allowing it to capture both short-term and long-term branch patterns.

## Features

- **Hybrid Design**: Combines bi-modal and TAGE predictors for optimal performance
- **Storage Efficient**: ~239KB total storage budget with optimized table sizes
- **Configurable**: 12 TAGE tables with history lengths from 4 to 640 bits
- **Adaptive**: Uses useful bits and confidence counters for dynamic prediction
- **High Accuracy**: Designed for modern workloads with complex branch patterns

## Architecture

### TAGE Tables Configuration
| Table | History Length | Table Size (bits) | Tag Size (bits) |
|-------|---------------|-------------------|-----------------|
| T0    | 4             | 10                | 7               |
| T1    | 6             | 10                | 7               |
| T2    | 10            | 11                | 8               |
| ...   | ...           | ...               | ...             |
| T11   | 640           | 9                 | 15              |

### Storage Budget Breakdown
- **Bi-modal Table**: 16KB (2^13 × 2-bit counters)
- **Global History**: 1.68KB (sum of all history lengths)
- **TAGE Tables**: 221.5KB (12 tables with tags + prediction + useful counters)
- **Total**: 239.18KB

## Usage

### Integration with ChampSim

1. **Create TAGE directory**:
   ```bash
   mkdir -p ./branch/tage
   ```

2. **Copy the predictor file**:
   ```bash
   cp tage.cc ./branch/tage/
   ```

3. **Build ChampSim with TAGE**:
  

4. **Run simulation**:
   

## Implementation Details

### Key Components

- **`O3_CPU::initialize_branch_predictor()`**: Initializes all predictor tables and registers
- **`O3_CPU::predict_branch(uint64_t pc)`**: Main prediction function returning branch direction
- **`O3_CPU::last_branch_result(...)`**: Updates predictor state based on actual branch outcome

### Prediction Algorithm

1. **Index Calculation**: Hash PC with GHR and PHR for table indices
2. **Tag Matching**: Find the longest matching history table
3. **Prediction Selection**: Choose between main and alternative predictions
4. **Confidence Assessment**: Use useful bits and alternative confidence counters

### Update Mechanism

- **Prediction Counters**: 3-bit saturating counters for TAGE, 2-bit for bi-modal
- **Useful Bits**: 2-bit counters tracking prediction utility
- **Alternative Confidence**: 4-bit counter for alternative prediction quality
- **History Updates**: Shift GHR and update compressed representations

## Files

- **`tage.cc`**: Complete TAGE predictor implementation
- **`CSCE-614 Report.pdf`**: Detailed analysis and performance evaluation
- **`README.md`**: This documentation file

## Performance

The TAGE predictor demonstrates superior performance compared to traditional predictors:
- Higher accuracy on complex branch patterns
- Adaptive learning from mispredictions
- Efficient storage utilization
- Suitable for modern high-performance processors

## References

- Original TAGE paper: "A case for (partially) TAgged GEometric history length branch prediction" by André Seznec and Pierre Michaud
- ChampSim framework: https://github.com/ChampSim/ChampSim

## Author
Sreedhar Reddy
Developed for CSCE-614 Computer Architecture course.