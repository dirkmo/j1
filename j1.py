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
        self.dsp = Signal(4)
        self.dspN = Signal(4)

        # top of data stack
        self.st0 = Signal(J1.WIDTH)
        self.st0N = Signal(J1.WIDTH)
        # data stack write
        self.dstkW = Signal()

        # program counter
        self.pc = Signal(13)
        self.pcN = Signal(13)
        self.pc_plus_1 = Signal(13)
        m.d.comb += self.pc_plus_1.eq(self.pc + 1)

        # return stack write
        self.rstkW = Signal()
        # return stack write value
        self.rstkD = Signal(J1.WIDTH)

        self.reboot = Signal(1, init=1)

        m.d.comb += [
            self.mem_addr.eq(self.st0[0:16]),
            self.code_addr.eq(self.pcN)
        ]

        # The D and R stacks
        self.st1 = Signal(J1.WIDTH)
        self.rst0 = Signal(J1.WIDTH)
        self.dspI = Signal(2)
        self.rspI = Signal(2)
        # stack2 #(.DEPTH(15)) dstack(.clk(clk), .rd(st1),  .we(dstkW), .wd(st0),   .delta(dspI));
        # stack2 #(.DEPTH(17)) rstack(.clk(clk), .rd(rst0), .we(rstkW), .wd(rstkD), .delta(rspI));

        self.minus = Signal(17)
        m.d.comb += self.minus.eq(Cat(~self.st0, C(1,1)) + self.st1 + 1)
        self.signedless = Signal()
        with m.If(self.st0[15] ^ self.st1[15]):
            m.d.comb += self.signedless.eq(self.st1[15])
        with m.Else():
            m.d.comb += self.signedless.eq(self.minus[16])

        mze = Signal() # minus==0 sign-extended
        me = Signal() # minus sign-extended

        with m.Switch(Cat(self.insn[8:16], self.pc[12])):
            with m.Case('1 --- -----'):
                m.d.comb += self.st0N.eq(self.insn)
            with m.Case('0 1-- -----'): # literal
                m.d.comb += self.st0N.eq(Cat(self.insn[0:15], C(0, J1.WIDTH-15)))
            with m.Case('0 000 -----'): # jump
                m.d.comb += self.st0N.eq(self.st0)
            with m.Case('0 010 -----'): # call
                m.d.comb += self.st0N.eq(self.st0)
            with m.Case('0 001 -----'): # conditional jump
                m.d.comb += self.st0N.eq(self.st1)
            with m.Case('0 011 -0000'): # ALU operations...
                m.d.comb += self.st0N.eq(self.st0)
            with m.Case('0 011 -0001'):
                m.d.comb += self.st0N.eq(self.st1)
            with m.Case('0 011 -0010'):
                m.d.comb += self.st0N.eq(self.st0 + self.st1)
            with m.Case('0 011 -0011'):
                m.d.comb += self.st0N.eq(self.st0 & self.st1)
            with m.Case('0 011 -0100'):
                m.d.comb += self.st0N.eq(self.st0 | self.st1)
            with m.Case('0 011 -0101'):
                m.d.comb += self.st0N.eq(self.st0 ^ self.st1)
            with m.Case('0 011 -0110'):
                m.d.comb += self.st0N.eq(~self.st0)

            with m.Case('0 011 -0111'): #  =
                m.d.comb += [
                    mze.eq(self.minus==0),
                    self.st0N.eq(mze.replicate(J1.WIDTH))
                ]
            with m.Case('0 011 -1000'): #  <
                m.d.comb += self.st0N.eq(self.signedless.replicate(J1.WIDTH))

            with m.Case('0 011 -1001'):
                m.d.comb += self.st0N.eq(Cat(self.st0[1:J1.WIDTH], self.st0[J1.WIDTH-1]))
            with m.Case('0 011 -1010'):
                m.d.comb += self.st0N.eq(Cat(C(0,1), self.st0[0:J1.WIDTH-1]))
            with m.Case('0 011 -1011'):
                m.d.comb += self.st0N.eq(self.rst0)
            with m.Case('0 011 -1100'):
                m.d.comb += self.st0N.eq(self.minus[0:16])
            with m.Case('0 011 -1101'):
                m.d.comb += self.st0N.eq(self.io_din)
            with m.Case('0 011 -1110'):
                m.d.comb += self.st0N.eq(Cat(self.dsp, C(0, J1.WIDTH-4)))
            with m.Case('0 011 -1111'): # u<
                m.d.comb += [
                    me.eq(self.minus[16]),
                    self.st0N.eq(me.replicate(J1.WIDTH))
                ]
            with m.Default():
                # st0N = {`WIDTH{1'bx}};
                m.d.comb += self.st0N.eq(0)


        m.d.sync += self.pc.eq(self.pc_plus_1)
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