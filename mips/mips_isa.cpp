/**
 * @file      mips_isa.cpp
 * @author    Sandro Rigo
 *            Marcus Bartholomeu
 *            Alexandro Baldassin (acasm information)
 *
 *            The ArchC Team
 *            http://www.archc.org/
 *
 *            Computer Systems Laboratory (LSC)
 *            IC-UNICAMP
 *            http://www.lsc.ic.unicamp.br/
 *
 * @version   1.0
 * @date      Mon, 19 Jun 2006 15:50:52 -0300
 *
 * @brief     The ArchC i8051 functional model.
 *
 * @attention Copyright (C) 2002-2006 --- The ArchC Team
 *
 */

#include  "mips_isa.H"
#include  "mips_isa_init.cpp"
#include  "mips_bhv_macros.H"

#include <deque>
#include <set>
#include <vector>
#include <algorithm>

//If you want debug information for this model, uncomment next line
// #define DEBUG_MODEL
#include "ac_debug_model.H"


//!User defined macros to reference registers.
#define Ra 31
#define Sp 29

// 'using namespace' statement to allow access to all
// mips-specific datatypes
using namespace mips_parms;

static int processors_started = 0;
#define DEFAULT_STACK_SIZE (256*1024)

// *****************************************************

struct mips_instruction {
  enum i_type { kR, kI, kJ };
  i_type type;
  unsigned int op;
  unsigned int rs;
  unsigned int rt;
  unsigned int rd;
  unsigned int shamt;
  unsigned int func;
  unsigned int addr;
  int imm;
};

std::ostream& operator<<(std::ostream& os, const mips_instruction& inst) {
  switch (inst.type) {
  case mips_instruction::kR:
    os << "R-instruction: " <<
       "op: " << inst.op << " " <<
       "rs: " << inst.rs << ", " <<
       "rt: " << inst.rt << ", " <<
       "rd: " << inst.rd << ", " <<
       "shamt: " << inst.shamt << ", " <<
       "func: " << inst.func;
    break;
  case mips_instruction::kI:
    os << "I-instruction: " <<
       "op: " << inst.op << " " <<
       "rs: " << inst.rs << ", " <<
       "rt: " << inst.rt << ", " <<
       "imm: " << inst.imm;
    break;
  case mips_instruction::kJ:
    os << "J-instruction: " <<
       "op: " << inst.op << " " <<
       "addr: " << inst.addr;
    break;
  default:
    os << "Invalid instruction!";
    break;
  }
  return os;
}

struct variables {
  unsigned int number_of_instructions; // Include NOP instructions
  unsigned int number_of_nops;
  int pc_addr; // Current pc value
  int static_wrong_predictions;
  int saturating_wrong_predictions;
  int two_level_wrong_predictions;
  unsigned int total_number_of_branches;
  int two_level_history;
  int saturating_stage;
  std::vector<int> two_level_stages;
  static constexpr int kNumberOfStoredInstructions = 10;
  static constexpr int kNumberOfStages = 2; // The total number of stages is twice that (taken + not taken)
  static constexpr int kHistoryDepth = 2;
  static constexpr bool is_fowarding = true;
  enum pipeline_stages { k5, k7, k13, SIZE };
  // Wait for previous instruction to complete its data read/write
  std::vector<unsigned int> number_of_data_hazards;
  // Deciding on control action depends on previous instruction
  std::vector<unsigned int> number_of_control_hazards;
  std::deque<mips_instruction> latest_instructions;
  const static std::set<std::pair<int, int>> instructions_dont_write;
  const static std::set<std::pair<int, int>> branch_instructions;
  const static std::set<std::pair<int, int>> ld_instructions;
  std::vector<std::vector<int>> hazard_table;

  std::vector<int> last_write;

  // superscalar
  struct _ss {
    bool ssLoaded = false;
    int ssInstCount = 0;
  } ss;

  static constexpr int Rd=1, Rs=2, Rt=4, Rm=8;
  enum InstGroups {ArithLog, DivMult, Shift, ShiftV, JumpR, MoveFrom, MoveTo,
    ArithLogI, LoadI, Branch, BranchZ, LoadStore, Jump, Trap};
  struct IGroup {
    InstGroups igroup;
    int readFrom, writeTo;
    std::set<pair<int,int>> instOp; // <opc, func>
  };
  const static std::vector<IGroup> groups {
    {ArithLog, Rs|Rt, Rd, {{0,0x20},{0,0x21},{0,0x24},{0,0x27},{0,0x25},
        {0,0x22},{0,0x23},{0,0x26},{0,0x2a},{0,0x29}}},
    {DivMult, Rs|Rt, Rm, {{0,0x1a},{0,0x1b},{0,0x18},{0,0x19}}},
    {Shift, Rt, Rd, {{0,0x0},{0,0x3},{0,0x2}}},
    {ShiftV, Rs|Rt, Rd, {{0,0x4},{0,0x7},{0,0x6}}},
    {JumpR, Rs, 0, {{0,0x9},{0,0x8}}},
    {MoveFrom, Rm, Rd, {{0,0x10},{0,0x12}}},
    {MoveTo, Rs, Rm, {{0,0x11},{0,0x13}}},
    {ArithLogI, Rs, Rt, {{0x8,0},{0x9,0},{0xc,0},{0xd,0},{0xe,0},{0xa,0},
        {0x9,0}}},
    {LoadI, 0, Rt, {{0x19,0},{0x18,0}}},
    {Branch, Rs|Rt, 0, {{0x4,0},{0x5,0}}},
    {BranchZ, Rs, 0, {{0x7,0},{0x6,0}}},
    {LoadStore, Rs|Rt, Rs|Rt, {{0x20,0},{0x24,0},{0x21,0},{0x25,0},{0x23,0},
        {0x28,0},{0x29,0},{0x2b,0}}},
    {Jump, 0, 0, {{0x2,0},{0x3,0}}},
    {Trap, 0, 0, {{0x1a,0}}}
  };

  std::set<int> getRegs(const mips_instruction &i, int regs) {
    std::set<int> S;
    if (regs&Rs)
      S.insert(i.rs);
    if (regs&Rt)
      S.insert(i.rt);
    if (regs&Rd)
      S.insert(i.rd);
    return S;
  }
  
  bool setHasIntersec(const std::set<int> &A, const std::set<int> &B) {
    for (const int &a : A)
      for (const int &b : B)
        if (a==b)
          return true;
    return false;
  }

  void testSuperscalar() { // must be called after push
    if (latest_instructions.size() < 2)
      return;
    if (!ss.ssLoaded) {
      mips_instruction &i_prev = latest_instructions[1];
      mips_instruction &i_cur = latest_instructions[0];

      const IGroup *g_prev=nullptr, *g_cur=nullptr;
      for (const IGroup &g : groups) { // search prev and cur inst groups
        if (g.instOp.count(make_pair(i_prev.op, i_prev.func)))
          g_prev = &g;
        if (g.instOp.count(make_pair(i_cur.op, i_cur.func)))
          g_cur = &g;
      }
      if (!g_prev || !g_cur)
        return; // error, shouldnt happen. just in case.
      if (g_prev->igroup == g_cur->igroup && g_cur->igroup != ArithLog &&
            g_cur->igroup != ArithLogI) // same group, abort, except arith ops
        return;
      if ((g_prev->readFrom & g_cur->writeTo & Rm) ||
            (g_prev->writeTo & g_cur->readFrom & Rm) ||
             (g_prev->writeTo & g_cur->writeTo & Rm)) 
        return ; // conflict in special multiplier registers

      // get register values for cur and prev instruction, read and write registers
      std::set<int> rd_prev = getRegs(i_prev, g_prev->readFrom);
      std::set<int> wr_prev = getRegs(i_prev, g_prev->writeTo);
      std::set<int> rd_cur = getRegs(i_cur, g_cur->readFrom);
      std::set<int> wr_cur = getRegs(i_cur, g_cur->writeTo);
      // if any conflict, abort: r-w, r-w, w-w => return
      if (setHasIntersec(rd_prev, wr_cur)||
          setHasIntersec(rd_cur, wr_prev)||
          setHasIntersec(wr_prev, wr_cur))
        return;
      
      // if no conflict so far, set bool and increment
      ss.ssLoaded = true;
      ss.ssInstCount++;
    }
    else
      ss.ssLoaded = false;
  }

  variables() :
    number_of_instructions(0),
    number_of_nops(0),
    static_wrong_predictions(0),
    saturating_wrong_predictions(0),
    total_number_of_branches(0),
    two_level_history(0),
    saturating_stage(kNumberOfStages) {
    // kNumberOfStages is the first 'taken' value, as the stage range
    // is [0, 2 * kNumberOfStages). This initial value is arbitrary.
    two_level_stages.resize(1 << kHistoryDepth, (int) kNumberOfStages);
    last_write.resize(34);
    number_of_data_hazards.resize(3, 0);
    number_of_control_hazards.resize(3, 0);
    // Processors
    // 5 Stages -> MIPS R2000 -> branch misprediction penalty = 1 cycle
    // 7 Stages -> MIPS R10000 -> branch misprediction penalty = 5 cycles
    // 13 Stages -> ARM Cortex A8 -> branch misprediction penalty = 13 cycles
    hazard_table = { {2, 1, 1}, {1, 2, 3} };
  }

  void push(mips_instruction inst) {
    // Check for hazards
    read_hazard(inst, k5);
    read_hazard(inst, k7);
    read_hazard(inst, k13);
    write_hazard(inst);
    int taken = actual_branch_taken(inst);
    // Verifies that 'inst' is a branch instruction and maps 'taken' into a bool
    if (taken--) {
      static_branch_prediction(taken, inst);
      saturating_branch_prediction(taken);
      two_level_branch_predictor(taken);
    }
    // std::cout << inst << std::endl;
    latest_instructions.push_front(inst);
    // Remove NOP from latest_instructions
    if (inst.op == 0 && inst.rs == 0 && inst.rt == 0 && inst.rd == 0 && inst.func == 0 && inst.imm == 0) {
      latest_instructions.pop_front();
    }
    if (latest_instructions.size() > kNumberOfStoredInstructions) {
      latest_instructions.pop_back();
    }
  }

  void write_hazard(mips_instruction inst) {
    if (inst.type == mips_instruction::kJ ||
        instructions_dont_write.find(std::make_pair(inst.op, inst.func)) != instructions_dont_write.end() ||
        (inst.op == 0 && inst.rs == 0 && inst.rt == 0 && inst.func == 0 && inst.imm == 0)) {
      return;
    }
    // mult, multu, div, divu
    if (inst.func == 0x18 || inst.func == 0x19 || inst.func == 0x1A || inst.func == 0x1B) {
      last_write[32] = last_write[33] = number_of_instructions;
    } else if (inst.func == 0x11) { // mthi
      last_write[32] = number_of_instructions;
    } else if (inst.func == 0x13) { // mtlo
      last_write[33] = number_of_instructions;
    } else if (inst.type == mips_instruction::kR) { // R-type
      last_write[inst.rd] = number_of_instructions;
    } else { // I-type
      last_write[inst.rt] = number_of_instructions;
    }
  }

  void read_hazard(mips_instruction inst, pipeline_stages pipeline_stage) {
    bool load = false;
    // NOP
    if (inst.op == 0 && inst.rs == 0 && inst.rt == 0 && inst.rd == 0 && inst.func == 0 && inst.imm == 0) {
      if (pipeline_stage == k5) {
        number_of_nops++;
      }
      // Update time stamp to ignore any NOP inserted by the simulator
      for (int i = 0; i < last_write.size(); i++) {
        last_write[i]++;
      }
      return;
    }
    // Check if the last instruction was a load
    if (latest_instructions.size() > 0 &&
         ld_instructions.find(std::make_pair(latest_instructions[0].op, latest_instructions[0].func)) != ld_instructions.end()) {
      // When we consider fowarding, the only possibility of hazard is in the instruction that comes right after a load
      // std::cout << latest_instructions[0] << std::endl;
      // std::cout << inst << std::endl;
      load = true;
    } else if (latest_instructions.size() > 1 && pipeline_stage != k5 &&
         ld_instructions.find(std::make_pair(latest_instructions[1].op, latest_instructions[1].func)) != ld_instructions.end()) {
      load = true;
    } else if (latest_instructions.size() > 2 && pipeline_stage == k13 &&
         ld_instructions.find(std::make_pair(latest_instructions[2].op, latest_instructions[2].func)) != ld_instructions.end()) {
      load = true;
    }
    if (is_fowarding == true && load == false) { // The last instruction was not a load
      // There are no hazards associated with R-type instructions when we consider fowarding
      return;
    }
    if (inst.type == mips_instruction::kR) {
      if (inst.func == 0x0D || inst.func == 0x0C) { // break, syscall
        return;
      }
      if (inst.func == 0x10) { // mfhi
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[32], pipeline_stage);
      } else if (inst.func == 0x12) { // mflo
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[33], pipeline_stage);
      } else if (inst.func == 0x11 || inst.func == 0x13) { // mthi, mtlo
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage);
      } else if (inst.func == 0x08 || inst.func == 0x09) { // jr, jalr
        number_of_control_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage);
      } else if (inst.shamt != 0) { // sll, sra, srl
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rt], pipeline_stage);
      } else if (inst.rs != 0 && inst.rt != 0) {
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage) |
            isHazard(number_of_instructions - last_write[inst.rt], pipeline_stage);
      } else if (inst.rs != 0) {
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage);
      } else if (inst.rt != 0) {
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rt], pipeline_stage);
      }
    } else if (inst.type == mips_instruction::kI) {
      if (inst.op == 0x0F) { // lui
        return;
      } else if ((inst.op == 0x04 || inst.op == 0x05) && (inst.rs != 0 || inst.rt != 0)) { // beq, bne
        // A branch that depends on the result of the previous instruction is a control hazard
        number_of_control_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage) |
            isHazard(number_of_instructions - last_write[inst.rt], pipeline_stage);
      } else if (branch_instructions.find(std::make_pair(inst.op, inst.func)) != branch_instructions.end()) {
        // A branch that depends on the result of the previous instruction is a control hazard
        number_of_control_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage);
      } else if ((inst.op == 0x28 || inst.op == 0x29 || inst.op == 0x2B) && (inst.rs != 0 && inst.rt != 0)) {
        // sb, sh, sw
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage) |
            isHazard(number_of_instructions - last_write[inst.rt], pipeline_stage);
      } else if ((inst.op == 0x28 || inst.op == 0x29 || inst.op == 0x2B) && inst.rs != 0) {
        // sb, sh, sw
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage);
      } else if ((inst.op == 0x28 || inst.op == 0x29 || inst.op == 0x2B) && inst.rt != 0) {
        // sb, sh, sw
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rt], pipeline_stage);
      } else if (inst.rs != 0) {
        number_of_data_hazards[pipeline_stage] += isHazard(number_of_instructions - last_write[inst.rs], pipeline_stage);
      }
    }
  }

  int isHazard(int distance, pipeline_stages pipeline_stage) {
    return hazard_table[is_fowarding][pipeline_stage] >= distance;
  }

  /**
   * Returns 0 if 'inst' isn't a branch instruction, 1 if it is but branch
   * is not taken, 2 if it is and branch is taken.
   **/
  int actual_branch_taken(mips_instruction inst) {
    int taken = 0;
    if (inst.type == mips_instruction::kI &&
        branch_instructions.find(std::make_pair(inst.op, inst.func)) != branch_instructions.end()) {
      ++taken;
      total_number_of_branches++;
      switch (inst.op) {
      case 0x01:
        taken += inst.rt ? inst.rs >= 0 : inst.rs < 0;
        break;
      case 0x04:
        taken += inst.rs == inst.rt;
        break;
      case 0x05:
        taken += inst.rs != inst.rt;
        break;
      case 0x06:
        taken += inst.rs <= 0;
        break;
      case 0x07:
        taken += inst.rs > 0;
        break;
      }
    }
    return taken;
  }

  void static_branch_prediction(bool taken, mips_instruction inst) {
    static_wrong_predictions += taken != inst.imm < pc_addr;
  }

  void read_saturating_counter(bool taken, int& wrong_predictions, int stage) {
    wrong_predictions += taken != (stage >= kNumberOfStages);
  }

  void update_saturating_counter(bool taken, int& stage) {
    stage += 2 * int(taken) - 1; // Adds 1 if taken, -1 otherwise
    // 0 <= stage < 2 * numberOfStages
    stage = std::max(std::min(stage, 2 * kNumberOfStages - 1), 0);
  }

  void saturating_branch_prediction(bool taken) {
    read_saturating_counter(taken, saturating_wrong_predictions, saturating_stage);
    update_saturating_counter(taken, saturating_stage);
  }

 /*
  * Two_level_history contains the history of the last 'kHistoryDepth' actual
  * branch decisions. This method updates that history, adding 'taken' and
  * removing the oldest entry recorded on the history.
  */
  void update_two_level_history(bool taken) {
    two_level_history = (two_level_history << 1 | taken) & ((1 << kHistoryDepth) - 1);
  }

  void two_level_branch_predictor(bool taken) {
    read_saturating_counter(taken, two_level_wrong_predictions, two_level_stages.at(two_level_history));
    update_saturating_counter(taken, two_level_stages.at(two_level_history));
    update_two_level_history(taken);
  }

  // void generate read_and_write_log(mips instruction) {
  //   FILE *f;
  //   if (number_ofinstructions <= 1) {
  //     f = fopen("read_write.txt", "w");
  //   } else {
  //     f = fopen("read_write.txt", "a");
  //   }
  //   if (!f) {
  //     return;
  //   }
  //
  //   if (ld_instructions.find(std::make_pair(inst.op, inst.func)) != ld_instructions.end()) {
  //     fprintf(f, "r %d"); // w endereço tamanho
  //   }
  //
  //   fclose(f);
  // }

} global;

const std::set<std::pair<int, int>> variables::instructions_dont_write {
  { 0, 0x8 },  // jr
  { 0, 0x0C }, // syscall
  { 0, 0x0D }, // break
  { 0x04, 0 }, // beq
  { 0x05, 0 }, // bne
  { 0x06, 0 }, // blez
  { 0x07, 0 }, // bgtz
  { 0x01, 0 }, // bltz, bgez
  { 0x28, 0 }, // sb
  { 0x29, 0 }, // sh
  { 0x2B, 0 }, // sw
  { 0x39, 0 }  // swc1
  // bltzal, bgezal
};

const std::set<std::pair<int, int>> variables::branch_instructions {
  { 0x04, 0 }, // beq
  { 0x05, 0 }, // bne
  { 0x06, 0 }, // blez
  { 0x07, 0 }, // bgtz
  { 0x01, 0 }  // bltz, bgez
  // bltzal, bgezal
};

const std::set<std::pair<int, int>> variables::ld_instructions {
  { 0x20, 0 }, // lb
  { 0x24, 0 }, // lbu
  { 0x21, 0 }, // lh
  { 0x25, 0 }, // lhu
  { 0x23, 0 }  // lw
  // { 0x31, 0 }  // lwc1
};

// *****************************************************

//!Generic instruction behavior method.
void ac_behavior(instruction) {

  global.number_of_instructions++;
  global.pc_addr = npc;

  dbg_printf("----- PC=%#x ----- %lld\n", (int)ac_pc, ac_instr_counter);
  //  dbg_printf("----- PC=%#x NPC=%#x ----- %lld\n", (int) ac_pc, (int)npc, ac_instr_counter);
#ifndef NO_NEED_PC_UPDATE
  ac_pc = npc;
  npc = ac_pc + 4;
#endif
};

//! Instruction Format behavior methods.
void ac_behavior(Type_R) {
  global.push({
    mips_instruction::kR,
    op,
    rs,
    rt,
    rd,
    shamt,
    func,
    0,
    0
  });
  global.testSuperscalar();
}

void ac_behavior(Type_I) {
  global.push({
    mips_instruction::kI,
    op,
    rs,
    rt,
    0,
    0,
    0,
    0,
    imm
  });
  global.testSuperscalar();
}

void ac_behavior(Type_J) {
  global.push({
    mips_instruction::kJ,
    op,
    0,
    0,
    0,
    0,
    0,
    addr,
    0
  });
  global.testSuperscalar();
}

//!Behavior called before starting simulation
void ac_behavior(begin) {
  dbg_printf("@@@ begin behavior @@@\n");
  RB[0] = 0;
  npc = ac_pc + 4;

  // It is not required by the architecture, but makes debug really easier
  for (int regNum = 0; regNum < 32; regNum++)
    RB[regNum] = 0;
  hi = 0;
  lo = 0;

  RB[29] = AC_RAM_END - 1024 - processors_started++ * DEFAULT_STACK_SIZE;
}

//!Behavior called after finishing simulation
void ac_behavior(end) {
  dbg_printf("@@@ end behavior @@@\n");

  printf("\n");
  printf("*******************************************************\n\n");
  printf("Number of NOPS: %d\n", global.number_of_nops);
  printf("Number of Instructions: %d\n\n", global.number_of_instructions);
  printf("Number of data hazards    (5 stages):  %d\n", global.number_of_data_hazards[0]);
  printf("Number of control hazards (5 stages):  %d\n", global.number_of_control_hazards[0]);
  printf("Number of data hazards    (7 stages):  %d\n", global.number_of_data_hazards[1]);
  printf("Number of control hazards (7 stages):  %d\n", global.number_of_control_hazards[1]);
  printf("Number of data hazards    (13 stages): %d\n", global.number_of_data_hazards[2]);
  printf("Number of control hazards (13 stages): %d\n\n", global.number_of_control_hazards[2]);
  printf("Total number of branches:  %d\n\n", global.total_number_of_branches);
  printf("Wrong branch predictions (static):     %d (%.2f \%)\n", global.static_wrong_predictions, ((float) global.static_wrong_predictions / global.total_number_of_branches) * 100);
  printf("Wrong branch predictions (saturating): %d (%.2f \%)\n", global.saturating_wrong_predictions, ((float) global.saturating_wrong_predictions / global.total_number_of_branches) * 100);
  printf("Wrong branch predictions (two level):  %d (%.2f \%)\n\n", global.two_level_wrong_predictions, ((float) global.two_level_wrong_predictions / global.total_number_of_branches) * 100);
  // Processors
  // 5 Stages -> MIPS R2000 -> branch misprediction penalty = 1 cycle
  // 7 Stages -> MIPS R10000 -> branch misprediction penalty = 5 cycles
  // 13 Stages -> ARM Cortex A8 -> branch misprediction penalty = 13 cycles
  printf("Number of stall cycles (5 stages + static):      %d\n", global.static_wrong_predictions);
  printf("Number of stall cycles (5 stages + saturating):  %d\n", global.saturating_wrong_predictions);
  printf("Number of stall cycles (5 stages + two level):   %d\n", global.two_level_wrong_predictions);
  printf("Number of stall cycles (7 stages + static):      %d\n", global.static_wrong_predictions * 5);
  printf("Number of stall cycles (7 stages + saturating):  %d\n", global.saturating_wrong_predictions * 5);
  printf("Number of stall cycles (7 stages + two level):   %d\n", global.two_level_wrong_predictions * 5);
  printf("Number of stall cycles (13 stages + static):     %d\n", global.static_wrong_predictions * 13);
  printf("Number of stall cycles (13 stages + saturating): %d\n", global.saturating_wrong_predictions * 13);
  printf("Number of stall cycles (13 stages + two level):  %d\n", global.two_level_wrong_predictions * 13);
  printf("Superscaled instr count: %d\n", global.ss.ssInstCount);
  printf("\n*******************************************************\n");
}

//!Instruction lb behavior method.
void ac_behavior(lb) {
  char byte;
  unsigned address, offset;

  dbg_printf("lb r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);

  address = RB[rs] + imm;
  offset = address & 3;
  byte = (DM.read(address & ~3) >> ((3 - offset) * 8)) & 0xFF;
  RB[rt] = (ac_Sword)byte;

  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lbu behavior method.
void ac_behavior(lbu) {
  unsigned char byte;
  unsigned address, offset;

  dbg_printf("lbu r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  address = RB[rs] + imm;
  offset = address & 3;
  byte = (DM.read(address & ~3) >> ((3 - offset) * 8)) & 0xFF;

  RB[rt] = byte;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lh behavior method.
void ac_behavior(lh) {
  short int half;
  unsigned address, offset;

  dbg_printf("lh r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  address = RB[rs] + imm;
  offset = (address & 3) >> 1;
  half = (DM.read(address & ~3) >> (1 - offset) * 16) & 0xFFFF;

  RB[rt] = (ac_Sword)half;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lhu behavior method.
void ac_behavior(lhu) {
  unsigned short int  half;
  unsigned address, offset;

  dbg_printf("lhu r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  address = RB[rs] + imm;
  offset = (address & 3) >> 1;
  half = (DM.read(address & ~3) >> (1 - offset) * 16) & 0xFFFF;

  RB[rt] = half;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lw behavior method.
void ac_behavior(lw) {
  dbg_printf("lw r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  RB[rt] = DM.read(RB[rs] + imm);
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lwl behavior method.
void ac_behavior(lwl) {
  dbg_printf("lwl r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (addr & 0x3) * 8;
  data = DM.read(addr & 0xFFFFFFFC);
  data <<= offset;
  data |= RB[rt] & ((1 << offset) - 1);
  RB[rt] = data;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lwr behavior method.
void ac_behavior(lwr) {
  dbg_printf("lwr r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (3 - (addr & 0x3)) * 8;
  data = DM.read(addr & 0xFFFFFFFC);
  data >>= offset;
  data |= RB[rt] & (0xFFFFFFFF << (32 - offset));
  RB[rt] = data;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction sb behavior method.
void ac_behavior(sb) {
  unsigned char byte;
  unsigned address, offset_ammount;
  ac_word data;

  dbg_printf("sb r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);

  address = RB[rs] + imm;
  offset_ammount = (3 - (address & 3)) * 8;
  byte = RB[rt] & 0xFF;
  data = DM.read(address & ~3) & ~(0xFF << offset_ammount) | (byte << offset_ammount);
  DM.write(address & ~3, data);

  dbg_printf("Result = %#x\n", (int)byte);
};

//!Instruction sh behavior method.
void ac_behavior(sh) {
  unsigned short int half;
  unsigned address, offset_ammount;
  ac_word data;

  dbg_printf("sh r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);

  address = RB[rs] + imm;
  offset_ammount = (1 - ((address & 3) >> 1)) * 16;
  half = RB[rt] & 0xFFFF;
  data = DM.read(address & ~3) & ~(0xFFFF << offset_ammount) | (half << offset_ammount);
  DM.write(address & ~3, data);

  dbg_printf("Result = %#x\n", (int)half);
};

//!Instruction sw behavior method.
void ac_behavior(sw) {
  dbg_printf("sw r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  DM.write(RB[rs] + imm, RB[rt]);
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction swl behavior method.
void ac_behavior(swl) {
  dbg_printf("swl r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (addr & 0x3) * 8;
  data = RB[rt];
  data >>= offset;
  data |= DM.read(addr & 0xFFFFFFFC) & (0xFFFFFFFF << (32 - offset));
  DM.write(addr & 0xFFFFFFFC, data);
  dbg_printf("Result = %#x\n", data);
};

//!Instruction swr behavior method.
void ac_behavior(swr) {
  dbg_printf("swr r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (3 - (addr & 0x3)) * 8;
  data = RB[rt];
  data <<= offset;
  data |= DM.read(addr & 0xFFFFFFFC) & ((1 << offset) - 1);
  DM.write(addr & 0xFFFFFFFC, data);
  dbg_printf("Result = %#x\n", data);
};

//!Instruction addi behavior method.
void ac_behavior(addi) {
  dbg_printf("addi r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] + imm;
  dbg_printf("Result = %#x\n", RB[rt]);
  //Test overflow
  if (((RB[rs] & 0x80000000) == (imm & 0x80000000)) &&
      ((imm & 0x80000000) != (RB[rt] & 0x80000000))) {
    fprintf(stderr, "EXCEPTION(addi): integer overflow.\n");
    exit(EXIT_FAILURE);
  }
};

//!Instruction addiu behavior method.
void ac_behavior(addiu) {
  dbg_printf("addiu r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] + imm;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction slti behavior method.
void ac_behavior(slti) {
  dbg_printf("slti r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  // Set the RD if RS< IMM
  if ((ac_Sword)RB[rs] < (ac_Sword)imm)
    RB[rt] = 1;
  // Else reset RD
  else
    RB[rt] = 0;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction sltiu behavior method.
void ac_behavior(sltiu) {
  dbg_printf("sltiu r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  // Set the RD if RS< IMM
  if ((ac_Uword)RB[rs] < (ac_Uword)imm)
    RB[rt] = 1;
  // Else reset RD
  else
    RB[rt] = 0;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction andi behavior method.
void ac_behavior(andi) {
  dbg_printf("andi r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] & (imm & 0xFFFF);
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction ori behavior method.
void ac_behavior(ori) {
  dbg_printf("ori r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] | (imm & 0xFFFF);
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction xori behavior method.
void ac_behavior(xori) {
  dbg_printf("xori r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] ^ (imm & 0xFFFF);
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lui behavior method.
void ac_behavior(lui) {
  dbg_printf("lui r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  // Load a constant in the upper 16 bits of a register
  // To achieve the desired behaviour, the constant was shifted 16 bits left
  // and moved to the target register ( rt )
  RB[rt] = imm << 16;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction add behavior method.
void ac_behavior(add) {
  dbg_printf("add r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] + RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
  //Test overflow
  if (((RB[rs] & 0x80000000) == (RB[rd] & 0x80000000)) &&
      ((RB[rd] & 0x80000000) != (RB[rt] & 0x80000000))) {
    fprintf(stderr, "EXCEPTION(add): integer overflow.\n");
    exit(EXIT_FAILURE);
  }
};

//!Instruction addu behavior method.
void ac_behavior(addu) {
  dbg_printf("addu r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] + RB[rt];
  //cout << "  RS: " << (unsigned int)RB[rs] << " RT: " << (unsigned int)RB[rt] << endl;
  //cout << "  Result =  " <<  (unsigned int)RB[rd] <<endl;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sub behavior method.
void ac_behavior(sub) {
  dbg_printf("sub r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] - RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
  //TODO: test integer overflow exception for sub
};

//!Instruction subu behavior method.
void ac_behavior(subu) {
  dbg_printf("subu r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] - RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction slt behavior method.
void ac_behavior(slt) {
  dbg_printf("slt r%d, r%d, r%d\n", rd, rs, rt);
  // Set the RD if RS< RT
  if ((ac_Sword)RB[rs] < (ac_Sword)RB[rt])
    RB[rd] = 1;
  // Else reset RD
  else
    RB[rd] = 0;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sltu behavior method.
void ac_behavior(sltu) {
  dbg_printf("sltu r%d, r%d, r%d\n", rd, rs, rt);
  // Set the RD if RS < RT
  if (RB[rs] < RB[rt])
    RB[rd] = 1;
  // Else reset RD
  else
    RB[rd] = 0;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_and behavior method.
void ac_behavior(instr_and) {
  dbg_printf("instr_and r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] & RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_or behavior method.
void ac_behavior(instr_or) {
  dbg_printf("instr_or r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] | RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_xor behavior method.
void ac_behavior(instr_xor) {
  dbg_printf("instr_xor r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] ^ RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_nor behavior method.
void ac_behavior(instr_nor) {
  dbg_printf("nor r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = ~(RB[rs] | RB[rt]);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction nop behavior method.
//void ac_behavior( nop )
//{
//  dbg_printf("nop\n");
//};

//!Instruction sll behavior method.
void ac_behavior(sll) {
  dbg_printf("sll r%d, r%d, %d\n", rd, rs, shamt);
  RB[rd] = RB[rt] << shamt;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction srl behavior method.
void ac_behavior(srl) {
  dbg_printf("srl r%d, r%d, %d\n", rd, rs, shamt);
  RB[rd] = RB[rt] >> shamt;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sra behavior method.
void ac_behavior(sra) {
  dbg_printf("sra r%d, r%d, %d\n", rd, rs, shamt);
  RB[rd] = (ac_Sword)RB[rt] >> shamt;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sllv behavior method.
void ac_behavior(sllv) {
  dbg_printf("sllv r%d, r%d, r%d\n", rd, rt, rs);
  RB[rd] = RB[rt] << (RB[rs] & 0x1F);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction srlv behavior method.
void ac_behavior(srlv) {
  dbg_printf("srlv r%d, r%d, r%d\n", rd, rt, rs);
  RB[rd] = RB[rt] >> (RB[rs] & 0x1F);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction srav behavior method.
void ac_behavior(srav) {
  dbg_printf("srav r%d, r%d, r%d\n", rd, rt, rs);
  RB[rd] = (ac_Sword)RB[rt] >> (RB[rs] & 0x1F);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction mult behavior method.
void ac_behavior(mult) {
  dbg_printf("mult r%d, r%d\n", rs, rt);

  long long result;
  int half_result;

  result = (ac_Sword)RB[rs];
  result *= (ac_Sword)RB[rt];

  half_result = (result & 0xFFFFFFFF);
  // Register LO receives 32 less significant bits
  lo = half_result;

  half_result = ((result >> 32) & 0xFFFFFFFF);
  // Register HI receives 32 most significant bits
  hi = half_result;

  dbg_printf("Result = %#llx\n", result);
};

//!Instruction multu behavior method.
void ac_behavior(multu) {
  dbg_printf("multu r%d, r%d\n", rs, rt);

  unsigned long long result;
  unsigned int half_result;

  result = RB[rs];
  result *= RB[rt];

  half_result = (result & 0xFFFFFFFF);
  // Register LO receives 32 less significant bits
  lo = half_result;

  half_result = ((result >> 32) & 0xFFFFFFFF);
  // Register HI receives 32 most significant bits
  hi = half_result;

  dbg_printf("Result = %#llx\n", result);
};

//!Instruction div behavior method.
void ac_behavior(div) {
  dbg_printf("div r%d, r%d\n", rs, rt);
  // Register LO receives quotient
  lo = (ac_Sword)RB[rs] / (ac_Sword)RB[rt];
  // Register HI receives remainder
  hi = (ac_Sword)RB[rs] % (ac_Sword)RB[rt];
};

//!Instruction divu behavior method.
void ac_behavior(divu) {
  dbg_printf("divu r%d, r%d\n", rs, rt);
  // Register LO receives quotient
  lo = RB[rs] / RB[rt];
  // Register HI receives remainder
  hi = RB[rs] % RB[rt];
};

//!Instruction mfhi behavior method.
void ac_behavior(mfhi) {
  dbg_printf("mfhi r%d\n", rd);
  RB[rd] = hi;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction mthi behavior method.
void ac_behavior(mthi) {
  dbg_printf("mthi r%d\n", rs);
  hi = RB[rs];
  dbg_printf("Result = %#x\n", (unsigned int)hi);
};

//!Instruction mflo behavior method.
void ac_behavior(mflo) {
  dbg_printf("mflo r%d\n", rd);
  RB[rd] = lo;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction mtlo behavior method.
void ac_behavior(mtlo) {
  dbg_printf("mtlo r%d\n", rs);
  lo = RB[rs];
  dbg_printf("Result = %#x\n", (unsigned int)lo);
};

//!Instruction j behavior method.
void ac_behavior(j) {
  dbg_printf("j %d\n", addr);
  addr = addr << 2;
#ifndef NO_NEED_PC_UPDATE
  npc = (ac_pc & 0xF0000000) | addr;
#endif
  dbg_printf("Target = %#x\n", (ac_pc & 0xF0000000) | addr);
};

//!Instruction jal behavior method.
void ac_behavior(jal) {
  dbg_printf("jal %d\n", addr);
  // Save the value of PC + 8 (return address) in $ra ($31) and
  // jump to the address given by PC(31...28)||(addr<<2)
  // It must also flush the instructions that were loaded into the pipeline
  RB[Ra] = ac_pc + 4; //ac_pc is pc+4, we need pc+8

  addr = addr << 2;
#ifndef NO_NEED_PC_UPDATE
  npc = (ac_pc & 0xF0000000) | addr;
#endif

  dbg_printf("Target = %#x\n", (ac_pc & 0xF0000000) | addr);
  dbg_printf("Return = %#x\n", ac_pc + 4);
};

//!Instruction jr behavior method.
void ac_behavior(jr) {
  dbg_printf("jr r%d\n", rs);
  // Jump to the address stored on the register reg[RS]
  // It must also flush the instructions that were loaded into the pipeline
#ifndef NO_NEED_PC_UPDATE
  npc = RB[rs], 1;
#endif
  dbg_printf("Target = %#x\n", RB[rs]);
};

//!Instruction jalr behavior method.
void ac_behavior(jalr) {
  dbg_printf("jalr r%d, r%d\n", rd, rs);
  // Save the value of PC + 8(return address) in rd and
  // jump to the address given by [rs]

#ifndef NO_NEED_PC_UPDATE
  npc = RB[rs], 1;
#endif
  dbg_printf("Target = %#x\n", RB[rs]);

  if (rd == 0) //If rd is not defined use default
    rd = Ra;
  RB[rd] = ac_pc + 4;
  dbg_printf("Return = %#x\n", ac_pc + 4);
};

//!Instruction beq behavior method.
void ac_behavior(beq) {
  dbg_printf("beq r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  if (RB[rs] == RB[rt]) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
};

//!Instruction bne behavior method.
void ac_behavior(bne) {
  dbg_printf("bne r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  if (RB[rs] != RB[rt]) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
};

//!Instruction blez behavior method.
void ac_behavior(blez) {
  dbg_printf("blez r%d, %d\n", rs, imm & 0xFFFF);
  if ((RB[rs] == 0) || (RB[rs] & 0x80000000)) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2), 1;
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
};

//!Instruction bgtz behavior method.
void ac_behavior(bgtz) {
  dbg_printf("bgtz r%d, %d\n", rs, imm & 0xFFFF);
  if (!(RB[rs] & 0x80000000) && (RB[rs] != 0)) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
};

//!Instruction bltz behavior method.
void ac_behavior(bltz) {
  dbg_printf("bltz r%d, %d\n", rs, imm & 0xFFFF);
  if (RB[rs] & 0x80000000) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
};

//!Instruction bgez behavior method.
void ac_behavior(bgez) {
  dbg_printf("bgez r%d, %d\n", rs, imm & 0xFFFF);
  if (!(RB[rs] & 0x80000000)) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
};

//!Instruction bltzal behavior method.
void ac_behavior(bltzal) {
  dbg_printf("bltzal r%d, %d\n", rs, imm & 0xFFFF);
  RB[Ra] = ac_pc + 4; //ac_pc is pc+4, we need pc+8
  if (RB[rs] & 0x80000000) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
  dbg_printf("Return = %#x\n", ac_pc + 4);
};

//!Instruction bgezal behavior method.
void ac_behavior(bgezal) {
  dbg_printf("bgezal r%d, %d\n", rs, imm & 0xFFFF);
  RB[Ra] = ac_pc + 4; //ac_pc is pc+4, we need pc+8
  if (!(RB[rs] & 0x80000000)) {
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm << 2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm << 2));
  }
  dbg_printf("Return = %#x\n", ac_pc + 4);
};

//!Instruction sys_call behavior method.
void ac_behavior(sys_call) {
  dbg_printf("syscall\n");
  stop();
}

//!Instruction instr_break behavior method.
void ac_behavior(instr_break) {
  fprintf(stderr, "instr_break behavior not implemented.\n");
  exit(EXIT_FAILURE);
}

