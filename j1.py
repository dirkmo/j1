#! /usr/bin/env python3

from amaranth import *
from amaranth.lib import enum, wiring
from amaranth.lib.memory import Memory
from amaranth.lib.wiring import In, Out
from amaranth.sim import Simulator, Period
from amaranth.back import verilog


class J1(wiring.Component):
    WIDTH = 16
    DEPTH = 4 # actual depth is 2**DEPTH

    def __init__(self):
        self.io_rd = Signal()
        self.io_wr = Signal()
        self.mem_addr = Signal(16)
        self.mem_wr = Signal()
        self.dout = Signal(J1.WIDTH)
        self.mem_din = Signal(J1.WIDTH)
        self.io_din = Signal(J1.WIDTH)
        self.code_addr = Signal(13)
        self.insn = Signal(16)

    def stack(self, m, ra, rd, we, wa, wd):
        st = Array(Signal(J1.WIDTH) for _ in range(2**J1.DEPTH))
        with m.If(we):
            m.d.sync += st[wa].eq(wd)
        m.d.comb += rd.eq(st[ra])
        return st

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
        rsp = Signal(J1.WIDTH)
        rspN = Signal(J1.WIDTH)
        rstkW = Signal() # return stack write
        rstkD = Signal(J1.WIDTH) # return stack write value
        reboot = Signal(1, init=1)
        pc_plus_1 = Signal(13)
        m.d.comb += pc_plus_1.eq(pc + 1)

        m.d.comb += [
            self.mem_addr.eq(st0[0:16]),
            self.code_addr.eq(pcN)
        ]

        # The D and R stacks
        st1 = Signal(J1.WIDTH)
        rst0 = Signal(J1.WIDTH)

        dstack = self.stack(m, ra=dsp, rd=st1, we=dstkW, wa=dspN, wd=st0)
        rstack = self.stack(m, ra=rsp, rd=rst0, we=rstkW, wa=rspN, wd=rstkD)

        st1_signed = Signal(signed(J1.WIDTH))
        st0_signed = Signal(signed(J1.WIDTH))
        m.d.comb += [ st1_signed.eq(st1), st0_signed.eq(st0) ]

        with m.Switch(self.insn[8:16]):
            with m.Case('1-- -----'): # literal
                m.d.comb += st0N.eq(Cat(self.insn[0:15], C(0, J1.WIDTH-15)))
            with m.Case('000 -----'): # jump
                m.d.comb += st0N.eq(st0)
            with m.Case('010 -----'): # call
                m.d.comb += st0N.eq(st0)
            with m.Case('001 -----'): # conditional jump
                m.d.comb += st0N.eq(st1)
            with m.Case('011 -0000'): # ALU operations...
                m.d.comb += st0N.eq(st0)
            with m.Case('011 -0001'):
                m.d.comb += st0N.eq(st1)
            with m.Case('011 -0010'):
                m.d.comb += st0N.eq(st0 + st1)
            with m.Case('011 -0011'):
                m.d.comb += st0N.eq(st0 & st1)
            with m.Case('011 -0100'):
                m.d.comb += st0N.eq(st0 | st1)
            with m.Case('011 -0101'):
                m.d.comb += st0N.eq(st0 ^ st1)
            with m.Case('011 -0110'):
                m.d.comb += st0N.eq(~st0)
            with m.Case('011 -0111'): #  ==
                m.d.comb += st0N.eq(st1 == st0)
            with m.Case('011 -1000'): #  <
                m.d.comb += st0N.eq(st1_signed < st0_signed)
            with m.Case('011 -1001'): # st1 >> 1
                m.d.comb += st0N.eq(Cat(st0[1:J1.WIDTH], st0[J1.WIDTH-1]))
            with m.Case('011 -1010'): # st1 << 1
                m.d.comb += st0N.eq(Cat(C(0,1), st0[0:J1.WIDTH-1]))
            with m.Case('011 -1011'):
                m.d.comb += st0N.eq(rst0)
            with m.Case('011 -1100'):
                m.d.comb += st0N.eq(self.mem_din)
            with m.Case('011 -1101'):
                m.d.comb += st0N.eq(self.io_din)
            with m.Case('011 -1110'):
                m.d.comb += st0N.eq(Cat(dsp, rsp, C(0, J1.WIDTH-8)))
            with m.Case('011 -1111'): # u<
                m.d.comb += st0N.eq(st1 < st0)
            with m.Default():
                # st0N = {`WIDTH{1'bx}};
                m.d.comb += st0N.eq(0)


        func_T_N = Signal()
        func_T_R = Signal()
        func_write = Signal()
        func_iow = Signal()
        func_ior = Signal()
        m.d.comb += [
            func_T_N.eq(self.insn[4:7] == 1),
            func_T_R.eq(self.insn[4:7] == 2),
            func_write.eq(self.insn[4:7] == 3),
            func_iow.eq(self.insn[4:7] == 4),
            func_ior.eq(self.insn[4:7] == 5)
        ]

        is_alu = Signal()
        mem_wr = Signal()
        m.d.comb += [
            is_alu.eq(self.insn[13:16] == 0b011),
            mem_wr.eq(~reboot & is_alu & func_write),
            self.dout.eq(st1),
            self.io_wr.eq(~reboot & is_alu & func_iow),
            self.io_rd.eq(~reboot & is_alu & func_ior)
        ]

        m.d.comb += rstkD.eq(Mux(
                self.insn[13],
                st0,
                Cat(C(0,1), pc_plus_1, C(0, J1.WIDTH-14))
            ))

        dspI = Signal(J1.DEPTH)
        rspI = Signal(J1.DEPTH)

        with m.Switch(self.insn[13:16]):
            with m.Case('1--'): m.d.comb += [ dstkW.eq(1), dspI.eq(0b0001) ]
            with m.Case('001'): m.d.comb += [ dstkW.eq(0), dspI.eq(0b1111) ]
            with m.Case('011'): m.d.comb += [ dstkW.eq(func_T_N), dspI.eq(Cat(self.insn[0:2], self.insn[1], self.insn[1])) ]
            with m.Default():   m.d.comb += [ dstkW.eq(0), dspI.eq(0) ]

        m.d.comb += dspN.eq(dsp + dspI)

        with m.Switch(self.insn[13:16]):
            with m.Case('010'): m.d.comb += [ rstkW.eq(1), rspI.eq(0b0001) ]
            with m.Case('011'): m.d.comb += [ rstkW.eq(func_T_R), rspI.eq(Cat(self.insn[2:4],self.insn[3], self.insn[3])) ]
            with m.Default():   m.d.comb += [ rstkW.eq(0), rspI.eq(0) ]

        m.d.comb += rspN.eq(rsp + rspI)

        with m.Switch(Cat(st0.any(), self.insn[7], self.insn[13:16], reboot)):

            with m.Case("1 --- - -"): m.d.comb += pcN.eq(0)
            with m.Case("0 000 - -",
                        "0 010 - -",
                        "0 001 - 0"): m.d.comb += pcN.eq(self.insn[0:13])
            with m.Case("0 --- - -",
                        "0 011 1 -"): m.d.comb += pcN.eq(rst0[1:14])
            with m.Default():         m.d.comb += pcN.eq(pc_plus_1)

        m.d.sync += [
            pc.eq(pc_plus_1),
            dsp.eq(dspN),
            st0.eq(st0N),
            rsp.eq(rspN)
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