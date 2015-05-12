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
        inst.op << " " <<
        inst.rs << ", " <<
        inst.rt << ", " <<
        inst.rd << ", " <<
        inst.shamt << ", " <<
        inst.func;
      break;
    case mips_instruction::kI:
      os << "I-instruction: " <<
        inst.op << " " <<
        inst.rs << ", " <<
        inst.rt << ", " <<
        inst.imm;
      break;
    case mips_instruction::kJ:
      os << "J-instruction: " <<
        inst.op << " " <<
        inst.addr;
      break;
    default:
      os << "Invalid instruction!";
      break;
  }
  return os;
}

struct variables {
  unsigned int number_of_instructions;
  unsigned int number_of_hazards;
  int pc_addr;
  int static_wrong_predictions;
  int saturating_wrong_predictions;
  int two_level_wrong_predictions;
  int total_number_of_branches;
  int two_level_history;
  int saturating_stage;
  std::vector<int> two_level_stages;
  static constexpr int kNumberOfStoredInstructions = 10;
  static constexpr int kNumberOfStages = 2; // The total number of stages is twice that (taken + not taken)
  static constexpr int kHistoryDepth = 2;
  static constexpr bool is_forwarding = false;
  enum pipeline_stages { k5, k7, k13, SIZE };
  pipeline_stages pipeline_stage = k5;
  std::deque<mips_instruction> latest_instructions;
  const static std::set<std::pair<int, int>> instructions_dont_write;
  const static std::set<std::pair<int, int>> branch_instructions;
  std::vector<std::vector<int>> hazard_table;

  std::vector<int> last_write;

  variables() :
    number_of_instructions(0),
    number_of_hazards(0),
    static_wrong_predictions(0),
    saturating_wrong_predictions(0),
    total_number_of_branches(0),
    two_level_history(0),
    saturating_stage(kNumberOfStages) {
      // kNumberOfStages is the first 'taken' value, as the stage range
      // is [0, 2 * kNumberOfStages). This initial value is arbitrary.
      two_level_stages.resize(1 << kHistoryDepth, (int) kNumberOfStages);
      last_write.resize(34);
      hazard_table = { { 2, 1, 1 }, { 1, 1, 1 } };
    }

  void push(mips_instruction inst) {
    read_hazard(inst);
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
    if (latest_instructions.size() > kNumberOfStoredInstructions) {
      latest_instructions.pop_back();
    }
  }

  void write_hazard(mips_instruction inst) {
    if (inst.type == mips_instruction::kJ || instructions_dont_write.find(std::make_pair(inst.op, inst.func)) != instructions_dont_write.end()) {
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

  void read_hazard(mips_instruction inst) {
    if (inst.type == mips_instruction::kR) {
      if (inst.rs != 0) {
        number_of_hazards += isHazard(number_of_instructions - last_write[inst.rs]);
      } else if (inst.rt != 0) {
        number_of_hazards += isHazard(number_of_instructions - last_write[inst.rt]);
      }
    } else if (inst.type == mips_instruction::kI) {
      if (inst.op == 0x0F) {
        return;
      } else if ((inst.op == 0x04 || inst.op == 0x05) && (inst.rs != 0 || inst.rt != 0)) {
        number_of_hazards += isHazard(number_of_instructions - last_write[inst.rs]) |
          isHazard(number_of_instructions - last_write[inst.rt]);
      } else if (inst.rs != 0) {
        number_of_hazards += isHazard(number_of_instructions - last_write[inst.rs]);
      }
    }
  }

  int isHazard(int distance) {
    return hazard_table[is_forwarding][pipeline_stage] >= distance;
  }

  /**
   * Returns 0 if 'inst' isn't a branch instruction, 1 if it is but branch
   * is not taken, 2 if it is and branch is taken.
   */
  int actual_branch_taken(mips_instruction inst) {
    int taken = 0;
    if (inst.type == mips_instruction::kI && branch_instructions.find(std::make_pair(inst.op, inst.func)) != branch_instructions.end()) {
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
    stage = std::min(stage, 3);
    stage = std::max(stage, 0);
    // 0 <= stage < 2 * numberOfStages
    // stage = std::min(std::max(stage, 2 * kNumberOfStages - 1), 0);
  }

  void saturating_branch_prediction(bool taken) {
    read_saturating_counter(taken, saturating_wrong_predictions, saturating_stage);
    update_saturating_counter(taken, saturating_stage);
  }

  void update_two_level_history(bool taken) {
    two_level_history = (two_level_history << 1 | taken) & ((1 << kHistoryDepth) - 1);
  }

  void two_level_branch_predictor(bool taken) {
    read_saturating_counter(taken, two_level_wrong_predictions, two_level_stages.at(two_level_history));
    update_saturating_counter(taken, two_level_stages.at(two_level_history));
    update_two_level_history(taken);
  }

} global;

const std::set<std::pair<int, int>> variables::instructions_dont_write{
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

const std::set<std::pair<int, int>> variables::branch_instructions{
  { 0x04, 0 }, // beq
    { 0x05, 0 }, // bne
    { 0x06, 0 }, // blez
    { 0x07, 0 }, // bgtz
    { 0x01, 0 }  // bltz, bgez
  // bltzal, bgezal
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
}

//!Behavior called before starting simulation
void ac_behavior(begin) {
  dbg_printf("@@@ begin behavior @@@\n");
  RB[0] = 0;
  npc = ac_pc + 4;

  // Is is not required by the architecture, but makes debug really easier
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
  printf("******************************\n");
  printf("Number of Instructions: %d\n", global.number_of_instructions);
  printf("Number of data hazards: %d\n", global.number_of_hazards);
  printf("Total number of branches: %d\n", global.total_number_of_branches);
  printf("Wrong branch predictions (static): %d\n", global.static_wrong_predictions);
  printf("Wrong branch predictions (saturating): %d\n", global.saturating_wrong_predictions);
  printf("Wrong branch predictions (two level): %d\n", global.two_level_wrong_predictions);
  printf("******************************\n");
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
