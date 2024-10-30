#! /usr/bin/env python3

from amaranth import *
from amaranth.lib import enum, wiring
from amaranth.lib.memory import Memory
from amaranth.lib.wiring import In, Out
from amaranth.sim import Simulator, Period
from amaranth.back import verilog


class J1(wiring.Component):
    WIDTH = 16

    def __init__(self):
        self.io_rd = Signal()
        self.io_wr = Signal()
        self.mem_addr = Signal(16)
        self.mem_wr = Signal()
        self.dout = Signal(J1.WIDTH)
        self.io_din = Signal(J1.WIDTH)
        self.code_addr = Signal(13)
        self.insn = Signal(16)

    def elaborate(self, platform):
        m = Module()

        # data stack pointer
        dsp = Signal(4)
        dspN = Signal(4)

        # top of data stack
        st0 = Signal(J1.WIDTH)
        st0N = Signal(J1.WIDTH)
        # data stack write
        dstkW = Signal()

        # program counter
        pc = Signal(13)
        pcN = Signal(13)
        pc_plus_1 = Signal(13)
        m.d.comb += pc_plus_1.eq(pc + 1)

        # return stack write
        rstkW = Signal()
        # return stack write value
        rstkD = Signal(J1.WIDTH)

        reboot = Signal(1, init=1)

        m.d.comb += [
            self.mem_addr.eq(st0[0:16]),
            self.code_addr.eq(pcN)
        ]

        # The D and R stacks
        st1 = Signal(J1.WIDTH)
        rst0 = Signal(J1.WIDTH)
        dspI = Signal(2)
        rspI = Signal(2)
        # stack2 #(.DEPTH(15)) dstack(.clk(clk), .rd(st1),  .we(dstkW), .wd(st0),   .delta(dspI));
        # stack2 #(.DEPTH(17)) rstack(.clk(clk), .rd(rst0), .we(rstkW), .wd(rstkD), .delta(rspI));

        minus = Signal(17)
        m.d.comb += minus.eq(Cat(~st0, C(1,1)) + st1 + 1)
        self.signedless = Signal()
        with m.If(st0[15] ^ st1[15]):
            m.d.comb += self.signedless.eq(st1[15])
        with m.Else():
            m.d.comb += self.signedless.eq(minus[16])

        mze = Signal() # minus==0 sign-extended
        me = Signal() # minus sign-extended

        with m.Switch(Cat(self.insn[8:16], pc[12])):
            with m.Case('1 --- -----'):
                m.d.comb += st0N.eq(self.insn)
            with m.Case('0 1-- -----'): # literal
                m.d.comb += st0N.eq(Cat(self.insn[0:15], C(0, J1.WIDTH-15)))
            with m.Case('0 000 -----'): # jump
                m.d.comb += st0N.eq(st0)
            with m.Case('0 010 -----'): # call
                m.d.comb += st0N.eq(st0)
            with m.Case('0 001 -----'): # conditional jump
                m.d.comb += st0N.eq(st1)
            with m.Case('0 011 -0000'): # ALU operations...
                m.d.comb += st0N.eq(st0)
            with m.Case('0 011 -0001'):
                m.d.comb += st0N.eq(st1)
            with m.Case('0 011 -0010'):
                m.d.comb += st0N.eq(st0 + st1)
            with m.Case('0 011 -0011'):
                m.d.comb += st0N.eq(st0 & st1)
            with m.Case('0 011 -0100'):
                m.d.comb += st0N.eq(st0 | st1)
            with m.Case('0 011 -0101'):
                m.d.comb += st0N.eq(st0 ^ st1)
            with m.Case('0 011 -0110'):
                m.d.comb += st0N.eq(~st0)

            with m.Case('0 011 -0111'): #  =
                m.d.comb += [
                    mze.eq(minus==0),
                    st0N.eq(mze.replicate(J1.WIDTH))
                ]
            with m.Case('0 011 -1000'): #  <
                m.d.comb += st0N.eq(self.signedless.replicate(J1.WIDTH))

            with m.Case('0 011 -1001'):
                m.d.comb += st0N.eq(Cat(st0[1:J1.WIDTH], st0[J1.WIDTH-1]))
            with m.Case('0 011 -1010'):
                m.d.comb += st0N.eq(Cat(C(0,1), st0[0:J1.WIDTH-1]))
            with m.Case('0 011 -1011'):
                m.d.comb += st0N.eq(rst0)
            with m.Case('0 011 -1100'):
                m.d.comb += st0N.eq(minus[0:16])
            with m.Case('0 011 -1101'):
                m.d.comb += st0N.eq(self.io_din)
            with m.Case('0 011 -1110'):
                m.d.comb += st0N.eq(Cat(dsp, C(0, J1.WIDTH-4)))
            with m.Case('0 011 -1111'): # u<
                m.d.comb += [
                    me.eq(minus[16]),
                    st0N.eq(me.replicate(J1.WIDTH))
                ]
            with m.Default():
                # st0N = {`WIDTH{1'bx}};
                m.d.comb += st0N.eq(0)


        func_T_N = Signal()
        m.d.comb += func_T_N.eq(self.insn[4:7] == 1)
        func_T_R = Signal()
        m.d.comb += func_T_R.eq(self.insn[4:7] == 2)
        func_write = Signal()
        m.d.comb += func_write.eq(self.insn[4:7] == 3)
        func_iow = Signal()
        m.d.comb += func_iow.eq(self.insn[4:7] == 4)
        func_ior = Signal()
        m.d.comb += func_ior.eq(self.insn[4:7] == 5)

        is_alu = Signal()
        m.d.comb += is_alu.eq(~pc[12] & (self.insn[13:16] == 0b011))
        mem_wr = Signal()
        m.d.comb += mem_wr.eq(~reboot & is_alu & func_write)
        m.d.comb += self.dout.eq(st1)
        m.d.comb += self.io_wr.eq(~reboot & is_alu & func_iow)
        m.d.comb += self.io_rd.eq(~reboot & is_alu & func_ior)

        with m.If(self.insn[13] == 0):
            m.d.comb += rstkD.eq(Cat(C(0,1), pc_plus_1, C(0, J1.WIDTH-14)))
        with m.Else():
            m.d.comb += rstkD.eq(st0)

        dspI = Signal(2)
        rspI = Signal(2)

        with m.Switch(Cat(self.insn[13:16], pc[12])):
            with m.Case('1 ---', '0 1--'):
                m.d.comb += [ dstkW.eq(1), dspI.eq(0b01) ]
            with m.Case('0 001'):
                m.d.comb += [ dstkW.eq(0), dspI.eq(0b11) ]
            with m.Case('0 011'):
                m.d.comb += [ dstkW.eq(func_T_N), dspI.eq(self.insn[0:2]) ]
            with m.Default():
                m.d.comb += [ dstkW.eq(0), dspI.eq(0) ]

        m.d.comb += dspN.eq(dsp + Cat(dspI, dspI[1], dspI[1]))

        with m.Switch(Cat(self.insn[13:16], pc[12])):
            with m.Case('1 ---'):
                m.d.comb += [ rstkW.eq(0), rspI.eq(0b11) ]
            with m.Case('0 010'):
                m.d.comb += [ rstkW.eq(1), rspI.eq(0b01) ]
            with m.Case('0 011'):
                m.d.comb += [ rstkW.eq(func_T_R), rspI.eq(self.insn[2:4]) ]
            with m.Default():
                m.d.comb += [ rstkW.eq(0), rspI.eq(0) ]

        with m.Switch(Cat(st0.any(), self.insn[7], self.insn[13:16], pc[12], reboot)):

            with m.Case("1 0 --- - -"):
                m.d.comb += pcN.eq(0)
            with m.Case("0 0 000 - -", "0 0 010 - -", "0 0 001 - 0"):
                m.d.comb += pcN.eq(self.insn[0:13])
            with m.Case("0 1 --- - -", "0 0 011 1 -"):
                m.d.comb += pcN.eq(rst0[1:14])
            with m.Default():
                m.d.comb += pcN.eq(pc_plus_1)

        m.d.sync += [
            pc.eq(pc_plus_1),
            dsp.eq(dspN),
            st0.eq(st0N)
        ]

        return m


async def bench(ctx):
    await ctx.tick()

def main():
    dut = J1()
    sim = Simulator(dut)
    sim.add_clock(Period(MHz=1))
    sim.reset()
    sim.add_testbench(bench)
    with sim.write_vcd("j1.vcd", gtkw_file="j1.gtkw", traces=[]):
        sim.run()

if __name__ == "__main__":
    main()