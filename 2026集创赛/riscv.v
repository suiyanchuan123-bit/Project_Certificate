//替换了sram
// NPC control signal
`define NPC_PC          2'b00
`define NPC_Offset12    2'b01
`define NPC_rs          2'b10
`define NPC_Offset20    2'b11

// A control signal
`define ALUSrcA_A       1'b0
`define ALUSrcA_sa      1'b1

// B control signal
`define ALUSrcB_B       2'b00
`define ALUSrcB_Imm     2'b01
`define ALUSrcB_Offset  2'b10
`define ALUSrcB_else    2'b11

// EXT control signal
`define ExtSel_ZERO     1'b0
`define ExtSel_SIGNED   1'b1

// ALU control signal
`define ALUOp_ADD      4'b0000
`define ALUOp_SUB      4'b0001
`define ALUOp_AND      4'b0010
`define ALUOp_OR       4'b0011
`define ALUOp_XOR      4'b0100
`define ALUOp_SRA      4'b0111
`define ALUOp_SLL      4'b1000
`define ALUOp_SRL      4'b1001
`define ALUOp_BR       4'b1010

// RF control signal
`define RegSel_rd       2'b00
`define RegSel_rt       2'b01
`define RegSel_31       2'b10
`define RegSel_else     2'b11

`define WDSel_FromALU   2'b00
`define WDSel_FromMEM   2'b01
`define WDSel_FromPC    2'b10
`define WDSel_Else      2'b11

// DM control signal
`define DMCtrl_RD       1'b0
`define DMCtrl_WR       1'b1

//INSTRUCTION DEF
`define INSTR_RTYPE_OP  7'b0110011
`define INSTR_ITYPE_OP  7'b0010011
`define INSTR_BTYPE_OP  7'b1100011
`define INSTR_LW_OP     7'b0000011
`define INSTR_SW_OP     7'b0100011
`define INSTR_JAL_OP    7'b1101111
`define INSTR_JALR_OP   7'b1100111

`define INSTR_ADD_FUNCT 10'b0000000_000
`define INSTR_SUB_FUNCT 10'b0100000_000
`define INSTR_SUBU_FUNCT 6'b100011
`define INSTR_AND_FUNCT 10'b0000000_111
`define INSTR_OR_FUNCT  10'b0000000_110
`define INSTR_XOR_FUNCT 10'b0000000_100
`define INSTR_NOR_FUNCT  6'b100111
`define INSTR_SLL_FUNCT 10'b0000000_001
`define INSTR_SRL_FUNCT 10'b0000000_101
`define INSTR_SRA_FUNCT 10'b0100000_101
`define INSTR_SRLV FUNCT 6'b000110
`define INSTR_SRAV FUNCT 6'b000111
`define INSTR_SLLV FUNCT 6'b000100
`define INSTR_JR FUNCT   6'b001000

`define INSTR_BEQ_FUNCT     3'b000
`define INSTR_BNE_FUNCT     3'b001

`define INSTR_ADDI_FUNCT    3'b000
`define INSTR_ORI_FUNCT     3'b110

`define EXT 1
`define INSTR_LUI_OP    7'b0110111//新增
`define INSTR_AUIPC_OP  7'b0010111//新增
`define INSTR_BLT_FUNCT 3'b100
`define INSTR_BGE_FUNCT 3'b101

//top
module riscv(
    input clk,
`ifdef EXT
    output [31:0] x3,
    output [31:0] x26,
    output [31:0] x27,
`endif
    input rst
);
`ifdef EXT
    wire imm_sel;
    wire ctrl;
    wire [19:0] Imm20;
`endif

// 控制信号
wire RFWrite, stall,
DMCtrl,DMCE,
 PCWrite, IRWrite, InsMemRW, ExtSel, zero, ALUSrcA;
wire [1:0] ALUSrcB;
wire [1:0] NPCOp, WDSel, RegSel;
wire [3:0] ALUOp;

// 指令字段
wire [6:0] opcode;
wire [2:0] Funct3;
wire [6:0] Funct7;

// 地址和数据信号
wire [31:0] NPC, PC ,PC_r ,PC_r2,
PCA4,PCA4_r;
wire [31:0] in_ins, out_ins, RD, DR_out;
wire [4:0] rs1, rs2, 
rd,rd_r;
wire [11:0] Imm12;
wire [31:0] Imm32;
wire [20:1] Offset20;
wire [11:0] Offset;
wire [4:0] WR;
wire [31:0] WD, RD1 ,RD1_r, RD2, RD2_r;
wire [31:0] A, B, ALU_result, ALU_result_r ,jalr_rs;
wire forward_en_A;
wire forward_en_B;
wire [31:0] data_forwardA;
wire [31:0] data_forwardB;
// 指令字段提取
assign opcode = out_ins[6:0];
assign Funct3 = out_ins[14:12];
assign Funct7 = out_ins[31:25];  
assign rs1 = out_ins[19:15];
assign rs2 = out_ins[24:20];
assign rd = out_ins[11:7];//2--3
assign Imm12 = out_ins[31:20];

// 跳转偏移量生成
`ifdef EXT

assign Offset20 = (opcode == `INSTR_AUIPC_OP)?Imm20:{out_ins[31], out_ins[19:12], out_ins[20], out_ins[30:21]};

`else

assign Offset20 = {out_ins[31], out_ins[19:12], out_ins[20], out_ins[30:21]};

`endif
// 偏移量选择逻辑
assign Offset = (opcode == `INSTR_BTYPE_OP) ? 
                  {out_ins[31], out_ins[7], out_ins[30:25], out_ins[11:8]} : 
                  (opcode == `INSTR_SW_OP) ? 
                  {out_ins[31:25], out_ins[11:7]} : 
                  Imm12;

// 实例化 ControlUnit
ControlUnit U_ControlUnit(
`ifdef EXT
    .imm_sel    (imm_sel),//3--4
    .ctrl       (ctrl),//3--4
    .out_ins    (out_ins),//3--4
    .Imm20      (Imm20),//3--4
    .ALU_result(ALU_result),
`endif
    .clk(clk),
    .rst(rst),
    .zero(zero),//3--4
    .opcode(opcode),
    .Funct7(Funct7),
    .Funct3(Funct3),
    //跳转指令相关
    .NPCOp      (NPCOp),//0--1
    .PCWrite    (PCWrite),//0--1
    .PC         (PC),//1--2
    .PCA4       (PCA4),//3--4
    .PC_r       (PC_r),//3--4
    .PC_r2      (PC_r2),//3--4
    //流水线停顿
    .stall      (stall),
    //数据前递
    .ALU_result_r   (ALU_result_r),
    .rs1            (rs1),//寄存器1地址   
    .rs2            (rs2),//寄存器2地址 
    .rd             (rd),//写回寄存器地址
    .rd_r           (rd_r),
    .forward_en_A   (forward_en_A),
    .forward_en_B   (forward_en_B),
    .data_forwardA  (data_forwardA),
    .data_forwardB  (data_forwardB),
    //ALU相关
    .ExtSel(ExtSel),//3--4
    .ALUOp(ALUOp),//3--4
    .ALUSrcA(ALUSrcA),//3--4
    .ALUSrcB(ALUSrcB),//3--4
    //访存相关
    .DMCE(DMCE),//3--4
    .DMCtrl(DMCtrl),//3--4
    //取指相关
    .InsMemRW(InsMemRW),//1--2
    .IRWrite(IRWrite),//2--3
    //写回寄存器相关
    .PCA4_r(PCA4_r),//4--5
    .RFWrite(RFWrite),//4--5
    .WDSel(WDSel),//4--5
    .RegSel(RegSel)//4--5
);

// 实例化 PC
PC U_PC(//1
    .clk(clk),
    .rst(rst),
    .PCWrite(PCWrite),
    .PC_r(PC_r),
    .NPC(NPC),
    .PC(PC)
);

// 实例化 NPC
NPC U_NPC(//0--1
    .PC(PC),
    .PC_r2(PC_r2),
    .NPCOp(NPCOp),
    .Offset12(Offset),
    .Offset20(Offset20),
    .rs(jalr_rs),
    .PCA4(PCA4),
    .NPC(NPC)
);

// 实例化 IM
IM U_IM(//2
    .clk(clk),
    .addr(PC[11:2]),
    .InsMemRW(InsMemRW),
    .Ins(in_ins)
);

// 实例化 IR,打了一拍
IR U_IR(//3
    .clk(clk),
    .rst(rst),
    .stall(stall),
    .IRWrite(IRWrite),//2--3
    .in_ins(in_ins),
    .out_ins(out_ins)
);

// 实例化 RF
RF U_RF(//3--4，5
    .RR1(rs1),
    .RR2(rs2),
    .forward_en_A   (forward_en_A),
    .forward_en_B   (forward_en_B),
    .data_forwardA  (data_forwardA),     // 输出选择结果
    .data_forwardB  (data_forwardB),   // 输出选择结果
    .WR(WR),
    .WD(WD),
    .clk(clk),
`ifdef EXT
    .x3(x3),
    .x26(x26),
    .x27(x27),
`endif
    .RFWrite(RFWrite),
    .RD1(RD1),
    .RD2(RD2)
);

// 实例化 MUX_3to1 (寄存器选择)
MUX_3to1 U_MUX_3to1(//4--5
    .X(rd_r),
    .Y(5'd0),
    .Z(5'd31),
    .control(RegSel),
    .out(WR)
);

Flopr U_ALUOut(//4
    .clk(clk),
    .rst(rst),
    .in_data(ALU_result),
    .out_data(ALU_result_r)
);

// 实例化 MUX_3to1_LMD (写回数据选择)
MUX_3to1_LMD U_MUX_3to1_LMD(//4--5
    .X(ALU_result_r),
    .Y(DR_out),
    .Z(PCA4_r[31:2]),
    .control(WDSel),
    .out(WD)
);

// 实例化 Flopr (寄存器A)
Flopr U_A(//用不上
    .clk(clk),
    .rst(rst),
    .in_data(RD1),
    .out_data(RD1_r)
);

// 实例化 Flopr (寄存器B)
Flopr U_B(//用不上
    .clk(clk),
    .rst(rst),
    .in_data(RD2),
    .out_data(RD2_r)
);

// 实例化 EXT (立即数扩展)
EXT U_EXT(//2--3
    .imm_in(Imm12),
`ifdef EXT
    .Imm20(Imm20),
    .imm_sel(imm_sel),
`endif
    .ExtSel(ExtSel),
    .imm_out(Imm32)
);

// 实例化 MUX_2to1_A (ALU输入A选择)
MUX_2to1_A U_MUX_2to1_A(//3--4
    .X(RD1),
    .Y(5'b0),
`ifdef EXT
    .Z(PC_r2),
    .ctrl(ctrl),
`endif
    .control(ALUSrcA),
    .out(A)
);

// 实例化 MUX_3to1_B (ALU输入B选择)
MUX_3to1_B U_MUX_3to1_B(//3--4
    .X(RD2),
    .Y(Imm32),
    .Z(Offset),  
    .control(ALUSrcB),
    .out(B)
);

// 实例化 ALU
ALU U_ALU(//3--4
    .A(A),
    .B(B),
    .jalr_rs(jalr_rs),
    .ALUOp(ALUOp),
    .ALU_result(ALU_result),
    .zero(zero)
);

// 实例化 DM (数据存储器)
DM U_DM(
`ifdef EXT
    .Addr(ALU_result[12:2]),//3--4
`else    
    .Addr(ALU_result[11:2]),//3--4
`endif    
    .WD(RD2),//3--4
    .clk(clk),
    .DMCE(DMCE),//3--4
    .DMCtrl(DMCtrl),//3--4
    .RD(RD)//4--5
);

// 数据存储器输出寄存器（旁路）
assign DR_out = RD;

endmodule

//ALU
module ALU(
    input signed [31:0] A,
    input signed [31:0] B,
    input [3:0] ALUOp,
    output signed [31:0] jalr_rs,
    output zero,
    output reg signed [31:0] ALU_result//第二级，需要两级寄存器缓存
);
    initial ALU_result = 32'b0;
    wire is_sub = (ALUOp == `ALUOp_SUB);
    wire [31:0] adder_b = is_sub ? ~B : B;
    wire [31:0] adder_res = A + adder_b + is_sub;

    always @(*) begin
        case (ALUOp)
            // 根据ALUOp执行不同操作,除了JAL
            `ALUOp_ADD: ALU_result = adder_res;       // ADD ADDI LW SW JALR
            `ALUOp_SUB: ALU_result = adder_res;       // SUB BEQ BNE
            `ALUOp_AND: ALU_result = A & B;       // AND
            `ALUOp_OR : ALU_result = A | B;       // OR ORI
            `ALUOp_XOR: ALU_result = A ^ B;       // XOR
            `ALUOp_SLL: ALU_result = $unsigned(A) << B[4:0]; // SLL (移位位数取自B的低5位)
            `ALUOp_SRL: ALU_result = $unsigned(A) >> B[4:0]; // SRL (逻辑右移)
            `ALUOp_SRA: ALU_result = A >>> B[4:0]; // SRA (算术右移)
            default: ALU_result = 0;
        endcase
    end
    assign jalr_rs = ALU_result;
    assign zero = ~|ALU_result;
        // 当ALU结果为0时，设置zero标志
endmodule

//ControlUnit
module ControlUnit(
    input rst,                 // 复位信号
    input clk,                 // 时钟信号
    input zero,                // ALU零标志
    //扩展指令
`ifdef EXT 
    input signed [31:0] ALU_result,
    input [31:0] out_ins,
    output [19:0] Imm20,
    output reg imm_sel,
    output ctrl,
`endif
    input [6:0] opcode,        // 操作码 (R/I/S/B/J/U)
    input [6:0] Funct7,        // 功能码7位 (用于R型指令)
    input [2:0] Funct3,        // 功能码3位
    output PCWrite,        // 程序计数器写使能
    output InsMemRW,       // 指令存储器读写控制 (1:读, 0:写)，无
    output IRWrite,            // 指令寄存器打一拍
    output reg RFWrite,        // 寄存器文件写使能
    output DMCtrl,             // 数据存储器控制 (0:读, 1:写)
    output DMCE,           // 数据存储器片选
    output reg ExtSel,         // 立即数扩展选择，0无符号，1有符号，无
    output ALUSrcA,            // ALU输入A选择，0是寄存器读数据1,1是0,无
    output [1:0] ALUSrcB,      // ALU输入B选择,0是寄存器读数据2  1是imm32  2是offset 无
    output reg [1:0] RegSel,   // 寄存器地址选择，0是指令rd  1是0 2是31  3是0，无
    output reg [1:0] NPCOp,    // 下条PC操作选择，
                               // 0顺序执行
                               // 1比较指令跳转
                               // 2跳转寄存器并链接
                               // 3跳转指令跳转 无
    output reg [1:0] WDSel,    // 写数据选择
                               // 0选择ALU结果
                               // 1选择内存读取数据
                               // 2选择PC+4
                               // 3其他情况输出0 无
    output [3:0] ALUOp,    // ALU操作码，要补充，无
    //跳转地址的计算（分支+跳转）
    input [31:0] PCA4,
    output reg [31:0] PCA4_r,
    input [31:0] PC,
    output reg [31:0] PC_r,
    output reg [31:0] PC_r2,
    //流水线停顿
    output reg stall,
    //数据前递
    input [31:0] ALU_result_r,
    input [4:0] rs1,   
    input [4:0] rs2,
    input [4:0] rd, 
    output reg [4:0] rd_r,
    output reg forward_en_A,
    output reg forward_en_B,
    output reg [31:0] data_forwardA,     // 输出选择结果
    output reg [31:0] data_forwardB     // 输出选择结果
);
//数据前递往后推
`ifdef EXT
reg ctrl_r0;
`endif
reg IRWrite_r0;
reg IRWrite_r;
reg [3:0] ALUOp_r0;
reg ALUSrcA_r0;      
reg [1:0] ALUSrcB_r0;

reg DMCE_r0;
reg DMCtrl_r0;

reg RFWrite_r0;
reg [1:0] RegSel_r0;
reg [1:0] WDSel_r0;

reg rd_valid;           // 当前指令有rd
reg rd_valid_r;         // 当前指令有rd
reg rs1_valid;      // 当前指令有rs1
reg rs2_valid;      // 当前指令有rs2
wire [9:0] Funct10;

initial DMCE_r0=1;
initial stall=0;

assign Funct10 = {Funct7,Funct3};
`ifdef EXT
    assign Imm20 = out_ins[31:12];
`endif

assign PCWrite = ~stall;
assign InsMemRW = 1;

always@(*)begin 
`ifdef EXT
    imm_sel = 0;
    ctrl_r0 = 0;
`endif
    IRWrite_r0 =  1;
    DMCE_r0 =  1;
    NPCOp    = `NPC_PC;

    ALUSrcA_r0 = `ALUSrcA_A;
    ALUSrcB_r0 = `ALUSrcB_B;
    RFWrite_r0 = 0;
    DMCtrl_r0  = `DMCtrl_RD;
    ExtSel  = `ExtSel_ZERO;
    RegSel_r0  = `RegSel_rd;
    WDSel_r0   = `WDSel_FromALU;
    ALUOp_r0   = `ALUOp_ADD;

case(opcode)
`INSTR_RTYPE_OP:
begin
    ALUSrcA_r0 = `ALUSrcA_A;
    ALUSrcB_r0 = `ALUSrcB_B;
    RFWrite_r0 = 1;
    RegSel_r0  = `RegSel_rd;
    DMCtrl_r0  = `DMCtrl_RD;
    WDSel_r0   = `WDSel_FromALU;
    case(Funct10)
        `INSTR_ADD_FUNCT:ALUOp_r0 =`ALUOp_ADD;
        `INSTR_SUB_FUNCT:ALUOp_r0 =`ALUOp_SUB;
        `INSTR_AND_FUNCT:ALUOp_r0 =`ALUOp_AND;
        `INSTR_OR_FUNCT :ALUOp_r0 =`ALUOp_OR;
        `INSTR_XOR_FUNCT:ALUOp_r0 =`ALUOp_XOR;
        `INSTR_SLL_FUNCT:ALUOp_r0 =`ALUOp_SLL;
        `INSTR_SRL_FUNCT:ALUOp_r0 =`ALUOp_SRL;
        `INSTR_SRA_FUNCT:ALUOp_r0 =`ALUOp_SRA;
    default:ALUOp_r0 =`ALUOp_ADD;
    endcase
end

`INSTR_ITYPE_OP:
begin
    ALUSrcA_r0 =`ALUSrcA_A;
    ALUSrcB_r0 =`ALUSrcB_Imm;
    ExtSel  =`ExtSel_SIGNED;
    RFWrite_r0 =1;
    DMCtrl_r0  =`DMCtrl_RD;
    RegSel_r0  =`RegSel_rd;
    WDSel_r0   =`WDSel_FromALU;
    case(Funct3)
        `INSTR_ADDI_FUNCT:ALUOp_r0 =`ALUOp_ADD;
        `INSTR_ORI_FUNCT:ALUOp_r0  =`ALUOp_OR;
    default:ALUOp_r0 =`ALUOp_ADD;
    endcase
end
`INSTR_BTYPE_OP://后续要不要将B指令放在PC之后判断
begin
    ALUSrcA_r0 =`ALUSrcA_A;
    ALUSrcB_r0 =`ALUSrcB_B;
    ALUOp_r0   =`ALUOp_SUB;
    RFWrite_r0 =0;
    DMCtrl_r0  =`DMCtrl_RD;
    ExtSel  =`ExtSel_ZERO;
    RegSel_r0  =`RegSel_rt;
    WDSel_r0   =`WDSel_Else;
    case(Funct3)
        `INSTR_BEQ_FUNCT:
        begin
            if(zero)begin
                IRWrite_r0   = 0;             
                NPCOp   =`NPC_Offset12;
            end
            else
            begin
                IRWrite_r0   = 1;             
                NPCOp   =`NPC_PC;
            end
        end
        `INSTR_BNE_FUNCT:
        begin
            if(!zero)
            begin
                IRWrite_r0   = 0;             
                NPCOp   =`NPC_Offset12;
            end
            else
            begin
                IRWrite_r0   = 1;             
                NPCOp   =`NPC_PC;
            end
        end
`ifdef EXT
        `INSTR_BLT_FUNCT:
        begin
            if( ALU_result >= 0 )
            begin
                IRWrite_r0   = 1;             
                NPCOp   =`NPC_PC;
            end
            else
            begin
                IRWrite_r0   = 0;             
                NPCOp   =`NPC_Offset12;
            end
        end
        `INSTR_BGE_FUNCT:
        begin
            if( ALU_result >= 0 )
            begin
                IRWrite_r0   = 0;             
                NPCOp   =`NPC_Offset12;
            end
            else
            begin
                IRWrite_r0   = 1;             
                NPCOp   =`NPC_PC;
            end
        end
`endif
    default:
    begin
        IRWrite_r0   = 1;             
        NPCOp   =`NPC_PC;
    end
    endcase
end
`INSTR_LW_OP   :
begin
    ALUSrcA_r0 =`ALUSrcA_A;
    ALUSrcB_r0 =`ALUSrcB_Offset;
    ALUOp_r0   =`ALUOp_ADD;
    RFWrite_r0 =1;
    DMCE_r0 = 0;
    DMCtrl_r0  =`DMCtrl_RD;
    ExtSel  =`ExtSel_ZERO;
    RegSel_r0  =`RegSel_rd;
    WDSel_r0   =`WDSel_FromMEM;
end

`INSTR_SW_OP   :
begin
    ALUSrcA_r0 =`ALUSrcA_A;
    ALUSrcB_r0 =`ALUSrcB_Offset;
    ALUOp_r0   =`ALUOp_ADD;
    DMCE_r0 = 0;
    DMCtrl_r0  =`DMCtrl_WR;//写入内存的值一直是rd2
    RFWrite_r0 =0;
    RegSel_r0  =`RegSel_else;
    WDSel_r0   =`WDSel_Else;
end
`INSTR_JAL_OP  :
begin
    IRWrite_r0 = 0;
    NPCOp   =`NPC_Offset20;
    ALUOp_r0   =`ALUOp_ADD;
    WDSel_r0   =`WDSel_FromPC;
    RFWrite_r0 =1;
    DMCtrl_r0  =`DMCtrl_RD;
    ExtSel  =`ExtSel_ZERO;
    ALUSrcA_r0 =`ALUSrcA_A;
    ALUSrcB_r0 =`ALUSrcB_B;
    RegSel_r0  =`RegSel_rd;
end
`INSTR_JALR_OP :
begin
    IRWrite_r0 = 0;
    NPCOp   =`NPC_rs;
    ALUOp_r0   =`ALUOp_ADD;
    RFWrite_r0 =1;
    DMCtrl_r0  =`DMCtrl_RD;
    ExtSel  =`ExtSel_ZERO;
    ALUSrcA_r0 =`ALUSrcA_A;
    ALUSrcB_r0 =`ALUSrcB_Offset;
    RegSel_r0  =`RegSel_rd;
    WDSel_r0   =`WDSel_FromPC;
end
`ifdef EXT 
`INSTR_LUI_OP:
begin
    NPCOp    =`NPC_PC;
    ALUSrcA_r0 = `ALUSrcA_sa;
    ALUSrcB_r0 = `ALUSrcB_Imm;
    RFWrite_r0    = 1;
    DMCtrl_r0     = `DMCtrl_RD;
    imm_sel    = 1;
    ExtSel     = `ExtSel_ZERO;
    RegSel_r0     = `RegSel_rd;
    WDSel_r0      = `WDSel_FromALU;
    ALUOp_r0         = `ALUOp_ADD;
end
`INSTR_AUIPC_OP:
begin
    NPCOp    =`NPC_PC;
    ALUSrcA_r0  =`ALUSrcA_A;//无视
    ALUSrcB_r0  =`ALUSrcB_Imm;
    RFWrite_r0  =1;
    DMCtrl_r0   =`DMCtrl_RD;
    imm_sel  =1;
    ctrl_r0 =1;
    ExtSel   =`ExtSel_ZERO;//无视
    RegSel_r0   =`RegSel_rd;
    WDSel_r0    =`WDSel_FromALU;
    ALUOp_r0       =`ALUOp_ADD;
end
`endif
default:
begin
`ifdef EXT
    imm_sel = 0;
    ctrl_r0 = 0;
`endif
    DMCE_r0 = 1;
    IRWrite_r0  =1;
    NPCOp    =`NPC_PC;
    RFWrite_r0 =0;
    DMCtrl_r0  =`DMCtrl_RD;
    ExtSel  =`ExtSel_ZERO;
    ALUSrcA_r0 =`ALUSrcA_A;
    ALUSrcB_r0 =`ALUSrcB_B;
    RegSel_r0  =`RegSel_rd;
    WDSel_r0   =`WDSel_FromALU;
    ALUOp_r0   =`ALUOp_ADD;
end
endcase
end

//数据前递
always@(*)begin
rd_valid    =1'b0;
rs1_valid   =1'b0;
rs2_valid   =1'b0;
case(opcode)
`INSTR_RTYPE_OP:
begin
rd_valid    =1'b1;
rs1_valid   =1'b1;
rs2_valid   =1'b1;
end
`INSTR_ITYPE_OP:
begin
rd_valid    =1'b1;
rs1_valid   =1'b1;
rs2_valid   =1'b0;
end
`INSTR_BTYPE_OP:
begin
rd_valid    =1'b0;
rs1_valid   =1'b1;
rs2_valid   =1'b1;
end
`INSTR_LW_OP   :
begin
rd_valid    =1'b1;
rs1_valid   =1'b1;
rs2_valid   =1'b0;
end
`INSTR_SW_OP   :
begin
rd_valid    =1'b0;
rs1_valid   =1'b1;
rs2_valid   =1'b1;
end
`INSTR_JAL_OP  :
begin
rd_valid    =1'b1;
rs1_valid   =1'b0;
rs2_valid   =1'b0;
end
`INSTR_JALR_OP :
begin
rd_valid    =1'b1;
rs1_valid   =1'b1;
rs2_valid   =1'b0;
end
`ifdef EXT
`INSTR_LUI_OP:
begin
rd_valid     =1'b1;
rs1_valid    =1'b0;
rs2_valid    =1'b0;
end
`INSTR_AUIPC_OP:
begin
rd_valid     =1'b1;
rs1_valid    =1'b0;
rs2_valid    =1'b0;
end
`endif
default:
begin
rd_valid    =1'b0;
rs1_valid   =1'b0;
rs2_valid   =1'b0;
end
endcase
end

always@(*) begin
    data_forwardA =0;
    forward_en_A  =0;
    data_forwardB =0;
    forward_en_B  =0;
    stall = 0;

    if(rs1==rd_r && rd_valid_r && rs1_valid && rd_r!=0)
    begin       
        if(WDSel == `WDSel_FromPC) begin
            data_forwardA = PCA4_r;
            forward_en_A  =1;
        end
        else if(WDSel == `WDSel_FromALU)begin
            data_forwardA =ALU_result_r;
            forward_en_A  =1;
        end
        else begin
            stall = 1;
        end
    end
    else
    begin
        data_forwardA =0;
        forward_en_A  =0;
    end

    if(rs2==rd_r && rd_valid_r && rs2_valid && rd_r!=0)
    begin
        if(WDSel == `WDSel_FromPC) begin
            data_forwardB = PCA4_r;
            forward_en_B  =1;
        end
        else if(WDSel == `WDSel_FromALU)begin
            data_forwardB =ALU_result_r;
            forward_en_B  =1;
        end
        else begin
            stall = 1;
        end
    end
    else
    begin
        data_forwardB =0;
        forward_en_B  =0;
    end
end

//清空2级流水
always @(posedge clk or posedge rst) begin
    if (rst) begin
        IRWrite_r    <= 1;        // 复位后，输出为0
    end
    else if(stall) begin
        IRWrite_r    <= 1;
    end
    else begin
        IRWrite_r    <= IRWrite_r0;
    end
end

assign IRWrite =  IRWrite_r & IRWrite_r0;

assign ALUOp = ALUOp_r0;
assign ALUSrcA = ALUSrcA_r0;
assign ALUSrcB = ALUSrcB_r0;        

`ifdef EXT
assign ctrl = ctrl_r0;
`endif

assign DMCE = DMCE_r0;

assign DMCtrl = DMCtrl_r0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        WDSel    <= 0;        // 复位后，输出为0
    end
    else if(stall) begin
        WDSel    <= 0;
    end
    else begin
        WDSel    <= WDSel_r0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_r  <= 0;        // 复位后，输出为0
    end
    else if(stall) begin
        rd_r  <= 0;
    end
    else begin
        rd_r  <= rd;        
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_valid_r  <= 0;        // 复位后，输出为0
    end
    else if(stall) begin
        rd_valid_r  <= 0;
    end
    else begin
        rd_valid_r  <= rd_valid;        
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        RegSel  <= 0;        // 复位后，输出为0
    end
    else if(stall) begin
        RegSel  <= 0;
    end
    else begin
        RegSel  <= RegSel_r0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        RFWrite  <= 0;        // 复位后，输出为0
    end
    else if(stall) begin
        RFWrite  <= 0;
    end
    else begin
        RFWrite  <= RFWrite_r0;
    end
end
//只停顿，不考虑清空
always @(posedge clk or posedge rst) begin
    if (rst) begin
        PC_r     <= 32'h0000_2000;        // 复位后，输出为0
        PC_r2    <= 32'h0000_2000;        // 复位后，输出为0
    end
    else if(stall) begin
        PC_r     <= PC_r;
        PC_r2    <= PC_r2; 
    end
    else begin
        PC_r     <= PC;
        PC_r2    <= PC_r; 
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        PCA4_r     <= 32'h0000_2004;        // 复位后，输出为0
    end
    else begin
        PCA4_r     <= PCA4;        
    end
end

endmodule

//DM
module DM (
`ifdef EXT
    input [12:2] Addr,     // 读写对应的地址（字节地址）
`else    
    input [11:2] Addr,     // 读写对应的地址（字节地址）
`endif    
    input [31:0] WD,       // 写入的数据
    input clk,             // 时钟信号
    input DMCtrl,          // 读写控制信号
    input DMCE,            // 片选信号，低有效
    output reg [31:0] RD   // 读出的数据,不加SRAM,是reg变量
);
/*
`ifdef EXT
    reg [31:0] memory[0:2047];  // 2048个32位字的存储空间
`else    
    reg [31:0] memory[0:1023];  // 1024个32位字的存储空间
`endif   
*/
//SRAM替换

wire [63:0] Q;
wire WEB;//0是写，1是读
assign RD = Q[31:0];
assign WEB = ~DMCtrl;//为了减少翻转，应该在源头去改

//仿真时用

    TS1N65LPLL2048X64M8 U_SRAM_DM (
        .CLK   (clk),
        .CEB   (DMCE),
        .WEB   (WEB),
        `ifdef EXT
        .A     (Addr),
        `else
        .A     ({1'b0,Addr}),
        `endif
        .D     ({32'b0,WD}),
        .BWEB  ({32'hFFFF_FFFF,32'b0}),
        .Q     (Q),
        .TSEL  (2'b01)
    );
    /*
    always @(posedge clk) begin
        if (DMCtrl) begin
            memory[Addr] <= WD;  // 写入数据（将字节地址转换为字地址）
        end
        else begin
            RD <= memory[Addr];  // 读出数据（将字节地址转换为字地址）
        end
    end
*/
endmodule

//EXT
module EXT (
    input [11:0] imm_in,      // 输入的12位数据
`ifdef EXT
    input [19:0]Imm20,
    input imm_sel,
`endif
    input ExtSel,             // 控制信号
    output reg [31:0] imm_out // 扩展后的32位数据
);
`ifdef EXT

always @(imm_in or ExtSel or Imm20 or imm_sel) begin
    if(imm_sel==0)
    case (ExtSel)
        `ExtSel_ZERO:    // 无符号扩展
            imm_out = {20'b0, imm_in[11:0]};
        `ExtSel_SIGNED:  // 有符号扩展
            imm_out = {imm_in[11] ? 20'hfffff : 20'h0, imm_in[11:0]};
        default:
            imm_out = 32'b0;
    endcase
    else
        imm_out = {Imm20,12'b0};
end
`else

always @(imm_in or ExtSel) begin
    case (ExtSel)
        `ExtSel_ZERO:    // 无符号扩展
            imm_out = {20'b0, imm_in[11:0]};
        `ExtSel_SIGNED:  // 有符号扩展
            imm_out = {imm_in[11] ? 20'hfffff : 20'h0, imm_in[11:0]};
        default:
            imm_out = 32'b0;
    endcase
end
`endif
endmodule

//Flopr
module Flopr (
    input clk,               // 时钟信号
    input rst,               // 复位信号
    input [31:0] in_data,    // 输入的数据
    output reg [31:0] out_data  // 输出的数据
);
always @(posedge clk or posedge rst) begin
    if (rst) begin
        out_data <= 0;        // 复位后，输出为0
    end
    else begin
        out_data <= in_data;  // 将输入数据输出
    end
end

endmodule

//IM
module IM (
    input clk,
    input InsMemRW,           // 指令存储单元信号
    input [11:2] addr,        // 指令存储器地址
    output reg [31:0] Ins     // 取得的指令,不加SRAM是reg
);
//这里变时序
//IR变组合

wire [63:0] Q;
wire IMCE;
assign Ins = Q[31:0];
assign IMCE = ~InsMemRW;

    TS1N65LPLL2048X64M8 
    U_SRAM_IM (
        .CLK   (clk),
        .CEB   (IMCE),
        .WEB   (InsMemRW),
        .A     ({1'b0,addr}),
        .D     (),
        .BWEB  ({32'hFFFF_FFFF,32'b0}),
        .Q     (Q),
        .TSEL  (2'b01)
    );

/*
reg [31:0] memory[0:1023];    // 1024个32位字的指令存储器

always @(posedge clk) begin
    if (InsMemRW) begin
        Ins <= memory[addr];  // 根据地址取指令
    end
    else begin
        Ins <= Ins;  
    end
end
*/
endmodule

//IR
module IR (
    input [31:0] in_ins,    // 指令输入
    input clk,              // 时钟信号
    input rst,
    input stall,
    input IRWrite,          // IR寄存器写使能信号
    output reg [31:0] out_ins  // 指令输出
);
//修改为同步逻辑，进行流水线切分
always @(posedge clk or posedge rst) begin  // 时钟信号上升沿
    if(rst) begin
        out_ins <= 32'h0000_0013;    
    end
    else if(stall) begin
        out_ins <= out_ins;
    end
    else if (IRWrite) begin      //优先级，后清空
        out_ins <= in_ins;   // 输出指令
    end
    else begin
        out_ins <= 32'h0000_0013;   // 输出空泡
    end
end
endmodule

//MUX_2to1_A
module MUX_2to1_A(
    input [31:0] X,       // 临时寄存器A中的内容
    input [4:0] Y,        //0
`ifdef EXT
    input [31:0] Z,
    input ctrl,
`endif

    input control,        // 选择控制信号
    output [31:0] out     // 输出选择结果
);
`ifdef EXT
    assign out = (ctrl==1'b1)?Z:((control == 1'b0) ?  X:{27'b0,Y[4:0]} );
`else
    assign out = (control == 1'b0) ?  X:{27'b0,Y[4:0]} ;
`endif
endmodule

//MUX_3to1_B
module MUX_3to1_B(
    input [31:0] X,           // 临时寄存器B中的内容
    input [31:0] Y,           // 临时寄存器Imm中的内容
    input [11:0] Z,           // 临时寄存器Offset中的内容（12位）
    input [1:0] control,      // 选择控制信号
    output reg signed [31:0] out // 输出选择结果（32位有符号）
);

always @(X or Y or Z or control) begin
    case(control)
        `ALUSrcB_B      : out = X;                // 选择X（32位）
        `ALUSrcB_Imm    : out = Y;                // 选择Y（32位）
        `ALUSrcB_Offset : out = $signed(Z); // 选择Z（自动符号扩展为32位）
        `ALUSrcB_else   : out = X;                // 其他情况选择X
    endcase
end

endmodule

//MUX_3to1_LMD
module MUX_3to1_LMD(
    input [31:0] X,           // 临时寄存器ALU0中的内容
    input [31:0] Y,           // 临时寄存器LMD中的内容
    input [31:2] Z,           // PC+4
    input [1:0] control,      // 选择控制信号
    output reg [31:0] out     // 输出选择结果
);

always @(X or Y or Z or control) begin
    case(control)
        `WDSel_FromALU: out = X;      // 选择ALU结果
        `WDSel_FromMEM: out = Y;      // 选择内存读取数据
        `WDSel_FromPC:  out = {Z,2'b00};      // 选择PC+4
        `WDSel_Else:    out = 32'b0;  // 其他情况输出0
    endcase
end

endmodule

//MUX_3to1
module MUX_3to1(
    input [4:0] X,         // rd
    input [4:0] Y,         // 预留输入
    input [4:0] Z,         // 预留输入
    input [1:0] control,   // 选择控制信号
    output reg [4:0] out   // 输出选择结果
);

always @(X or Y or Z or control) begin
    case(control)
        `RegSel_rd:   out = X;    // 选择X
        `RegSel_rt:   out = Y;    // 选择Y
        `RegSel_31:   out = Z;    // 选择Z
        `RegSel_else: out = 0;    // 其他情况输出0
    endcase
end

endmodule

//NPC
module NPC(
    input [1:0] NPCOp,           // 控制信号
    input [12:1] Offset12,       // 比较指令的跳转偏移量
    input [20:1] Offset20,       // 跳转指令的跳转偏移量
    input [31:0] PC,             // 当前指令的地址
    input [31:0] PC_r2,           // 本条指令的地址
    input [31:0] rs,
    output reg [31:0] PCA4,      // PC+4
    output reg [31:0] NPC        // 下一条指令的地址
);

    wire signed [12:0] Offset13; // 扩展后的13位偏移量
    wire signed [20:0] Offset21; // 扩展后的21位偏移量

    // 将12位偏移量扩展为13位（最低位补0）
    assign Offset13 = $signed({Offset12[12:1], 1'b0});
    
    // 将20位偏移量扩展为21位（最低位补0）
    assign Offset21 = $signed({Offset20[20:1], 1'b0});

    always @(*) begin
        case (NPCOp)
            `NPC_PC      :  NPC = PC + 4;                                    // 顺序执行
            `NPC_Offset12:  NPC = $signed({1'b0, PC_r2}) + $signed(Offset13);   // 比较指令跳转
            `NPC_rs      :  NPC = rs & (~1);  
            `NPC_Offset20:  NPC = $signed({1'b0, PC_r2}) + $signed(Offset21);   // 跳转指令跳转
        endcase        
        PCA4 = PC_r2 + 4; // 计算PC+4
    end

endmodule

//PC
module PC (
    input clk,            // 时钟信号
    input rst,            // 复位信号
    input PCWrite,        // PC写使能信号
    input [31:0] PC_r,
    input [31:0] NPC,     // 下一条指令的地址
    output reg [31:0] PC  // 本条指令地址
);

reg [31:0] PC_reg;

initial begin
    PC_reg = 32'h0000_2000;
    PC     = 32'h0000_2000;
end

always @(posedge clk or posedge rst) begin
    // 复位信号处理
    if (rst) begin
        PC_reg <= 32'h0000_2000;  // 复位后PC的值
    end
    else if (PCWrite) begin 
        PC_reg <= NPC;//跳转地址的下一条地址 
    end
    else begin
        PC_reg <= PC_reg;         
    end
end

always @(*) begin              
    if (PCWrite == 0)
        PC = PC_r;
    else
        PC = PC_reg;
end
endmodule

//RF
module RF(
    input [4:0] RR1,      // 读取寄存器1地址
    input [4:0] RR2,      // 读取寄存器2地址
    input forward_en_A,
    input forward_en_B,
    input [31:0] data_forwardA,     // 输出选择结果
    input [31:0] data_forwardB,     // 输出选择结果
    input [4:0] WR,       // 写入寄存器地址
    input [31:0] WD,      // 写入数据
    input RFWrite,        // 寄存器写使能信号
    input clk,            // 时钟信号
`ifdef EXT
    output [31:0] x3,
    output [31:0] x26,
    output [31:0] x27,
`endif
    output [31:0] RD1,    // 读取寄存器1数据
    output [31:0] RD2    // 读取寄存器2数据
);
    reg [31:0] register [0:31]; // 32个32位寄存器
`ifdef EXT
    assign x3   = register[3];
    assign x26  = register[26];
    assign x27  = register[27];    
`endif

initial begin
    integer i;
    for (i = 0; i < 32; i = i + 1)
        register[i] = 32'h0;
end
    // 写操作：时钟上升沿触发
    always @(posedge clk) begin
        register[0] <= 32'h0;
        // 如果写寄存器地址不为0且写使能信号有效，则写入数据到指定寄存器
        if ((WR != 0) && (RFWrite == 1)) begin
            register[WR] <= WD;            
            `ifdef DEBUG
            // 如果定义了DEBUG宏，则输出寄存器的值（用于调试）
            $display("R[00-07]=%8X %8X %8X %8X %8X %8X %8X %8X", 
                     register[0], register[1], register[2], register[3],
                     register[4], register[5], register[6], register[7]);
            $display("R[08-15]=%8X %8X %8X %8X %8X %8X %8X %8X", 
                     register[8], register[9], register[10], register[11],
                     register[12], register[13], register[14], register[15]);
            $display("R[16-23]=%8X %8X %8X %8X %8X %8X %8X %8X", 
                     register[16], register[17], register[18], register[19],
                     register[20], register[21], register[22], register[23]);
            $display("R[24-31]=%8X %8X %8X %8X %8X %8X %8X %8X", 
                     register[24], register[25], register[26], register[27],
                     register[28], register[29], register[30], register[31]);
            `endif           
        end
    end
    
    // 读操作：组合逻辑
    assign RD1 = forward_en_A? data_forwardA:register[RR1];
    assign RD2 = forward_en_B? data_forwardB:register[RR2];
    
endmodule

