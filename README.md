# Multi-Cycle RISC Processor

## Project Overview
This project involves the design and verification of a multi-cycle RISC processor using Verilog. The processor uses a 16-bit Instruction Set Architecture (ISA) and supports three instruction types: R-type, I-type, and J-type. It handles basic operations like arithmetic, logical, memory load/store, and branching instructions.

## Features
- **16-bit ISA** with R-type, I-type, and J-type instructions
- Support for basic arithmetic and logical operations
- Load/store instructions for memory access
- Branching instructions for conditional execution
- Multi-cycle datapath to optimize resource utilization
- MIPS-style instruction encoding
- ALU with zero detection
-   

## Project Structure
- **Verilog code**: Contains the processor design files, including datapath and control path modules
- **Testbenches**: Includes testbenches to verify the functionality of the processor
- **Simulation**: A set of test programs to verify the correctness of each instruction

## How to Run the Project
1. Ensure you have a Verilog simulation tool such as Active-HDL,or another Verilog simulator.
2. Compile the Verilog files using the simulator.
3. Run the simulation with the provided testbenches to verify the functionality of the processor.

## Instructions Set
- **R-type**: Register operations (e.g., ADD, SUB, AND)
- **I-type**: Immediate operations (e.g., ADDI, BEQ, BNE)
- **J-type**: Jump operations (e.g., JMP, CALL, RET)

## Simulation
Simulation files and test cases are provided to test each instruction and validate the processor's operation. Results can be viewed in the simulator window.

## Report
The full report on the design and verification process is available, covering:
- Datapath and control path design
- Instruction set and operation details
- Simulation and test results
