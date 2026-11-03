// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.ModelingData.Tests;

// AdvApp2Var_SysBase: the managed allocation of work zones (decision 114) and the memory routines.
public class AdvApp2Var_SysBaseTests
{
    [Fact]
    public void Mcrrqst_AllocatesAZoneAtTheOffset()
    {
        var aSys = new global::Kerf.Engine.Approximation.Advanced.SysBase();
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> aTab = default;
        int anOffset = 0, anErr = -1;
        aSys.mcrrqst_(8, 10, ref aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anErr);
        Assert.NotEqual(0, anOffset);
        // the zone is T[IOFSET] to T[IOFSET + 9], zeroed, between OCCT's start and end flags
        for (int i = 0; i < 10; i++)
        {
            Assert.Equal(0.0, aTab[anOffset + i]);
            aTab[anOffset + i] = i;
        }
        Assert.Equal(-134744073.0, aTab[anOffset - 1]);
        Assert.Equal(-134744073.0, aTab[anOffset + 10]);

        aSys.mcrdelt_(8, 10, aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(int.MaxValue, anOffset);
    }

    [Fact]
    public void Mcrrqst_IntegerZone()
    {
        var aSys = new global::Kerf.Engine.Approximation.Advanced.SysBase();
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<int> aTab = default;
        int anOffset = 0, anErr = -1;
        aSys.mcrrqst_(4, 7, ref aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(2, anOffset); // the 8-byte start flag is two ints
        aTab[anOffset + 6] = 42;
        Assert.Equal(42, aTab[anOffset + 6]);
        aSys.mcrdelt_(4, 7, aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anErr);
    }

    [Fact]
    public void Mcrrqst_Errors()
    {
        var aSys = new global::Kerf.Engine.Approximation.Advanced.SysBase();
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> aTab = default;
        int anOffset = 0, anErr = 0;
        // an invalid unit (and, in the port, a unit that is not the element size)
        aSys.mcrrqst_(3, 10, ref aTab, ref anOffset, ref anErr);
        Assert.Equal(2, anErr);
        aSys.mcrrqst_(4, 10, ref aTab, ref anOffset, ref anErr);
        Assert.Equal(2, anErr);

        // at most 32 allocations at a time
        var anOffsets = new int[33];
        for (int i = 0; i < 32; i++)
        {
            global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> aZone = default;
            aSys.mcrrqst_(8, 1, ref aZone, ref anOffsets[i], ref anErr);
            Assert.Equal(0, anErr);
        }
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> aLast = default;
        aSys.mcrrqst_(8, 1, ref aLast, ref anOffsets[32], ref anErr);
        Assert.Equal(1, anErr);

        // deleting an allocation that doesn't exist
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> anOther = new double[3];
        int anOtherOffset = 1;
        aSys.mcrdelt_(8, 1, anOther, ref anOtherOffset, ref anErr);
        Assert.Equal(3, anErr);
        Assert.Equal(1, anOtherOffset);
    }

    [Fact]
    public void Macrar8_UsesTheTableWhenLargeEnough()
    {
        var aSys = new global::Kerf.Engine.Approximation.Advanced.SysBase();
        global::Kerf.Engine.Approximation.Advanced.F2cPtr<double> aTab = new double[100];
        int anOffset = -1, anErr = -1;
        aSys.macrar8_(100, 100, ref aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anOffset);
        Assert.Equal(0, anErr);
        Assert.Equal(100, aTab.AsSpan().Length);
        aSys.macrdr8_(100, 100, aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anErr);

        aSys.macrar8_(101, 100, ref aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anErr);
        Assert.NotEqual(0, anOffset);
        aTab[anOffset + 100] = 1.0;
        aSys.macrdr8_(101, 100, aTab, ref anOffset, ref anErr);
        Assert.Equal(0, anErr);
        Assert.Equal(int.MaxValue, anOffset);
    }

    [Fact]
    public void Mcrfill_OverlappingZones()
    {
        // OCCT copies forwards or backwards so that overlapping zones are moved as memmove does
        double[] aTab = [1, 2, 3, 4, 5, 6];
        global::Kerf.Engine.Approximation.Advanced.SysBase.mcrfill_(4 * 8, new global::Kerf.Engine.Approximation.Advanced.F2cPtr<double>(aTab), new global::Kerf.Engine.Approximation.Advanced.F2cPtr<double>(aTab) + 2);
        Assert.Equal([1.0, 2, 1, 2, 3, 4], aTab);
        aTab = [1, 2, 3, 4, 5, 6];
        global::Kerf.Engine.Approximation.Advanced.SysBase.mcrfill_(4 * 8, new global::Kerf.Engine.Approximation.Advanced.F2cPtr<double>(aTab) + 2, new global::Kerf.Engine.Approximation.Advanced.F2cPtr<double>(aTab));
        Assert.Equal([3.0, 4, 5, 6, 5, 6], aTab);
        int[] anInts = [1, 2, 3];
        var aCopy = new int[3];
        global::Kerf.Engine.Approximation.Advanced.SysBase.mcrfill_(2 * 4, anInts, aCopy);
        Assert.Equal([1, 2, 0], aCopy);
    }

    [Theory]
    [InlineData(0.0, 10)]
    [InlineData(0.0, 200)]
    [InlineData(2.5, 62)]
    [InlineData(-1.0, 63)]
    [InlineData(7.0, 130)]
    public void Maitbr8_FillsTheTable(double theValue, int theSize)
    {
        var aTab = new double[theSize + 1];
        Array.Fill(aTab, 99.0);
        global::Kerf.Engine.Approximation.Advanced.SysBase.maitbr8_(theSize, aTab, theValue);
        for (int i = 0; i < theSize; i++)
        {
            Assert.Equal(theValue, aTab[i]);
        }
        Assert.Equal(99.0, aTab[theSize]);
    }

    [Fact]
    public void Raz_And_Fill()
    {
        double[] aTab = [1, 2, 3, 4];
        global::Kerf.Engine.Approximation.Advanced.SysBase.mvriraz_(3, aTab);
        Assert.Equal([0.0, 0, 0, 4], aTab);
        aTab = [1, 2, 3, 4];
        global::Kerf.Engine.Approximation.Advanced.SysBase.miraz_(16, new global::Kerf.Engine.Approximation.Advanced.F2cPtr<double>(aTab) + 1);
        Assert.Equal([1.0, 0, 0, 4], aTab);

        var anOut = new double[3];
        global::Kerf.Engine.Approximation.Advanced.SysBase.msrfill_(2, new double[] { 5, 6, 7 }, anOut);
        Assert.Equal([5.0, 6, 0], anOut);
        var anIntOut = new int[3];
        global::Kerf.Engine.Approximation.Advanced.SysBase.msifill_(3, new int[] { 5, 6, 7 }, anIntOut);
        Assert.Equal([5, 6, 7], anIntOut);

        int aNbDigits = 0;
        global::Kerf.Engine.Approximation.Advanced.SysBase.maovsr8_(ref aNbDigits);
        Assert.Equal(17, aNbDigits);
        Assert.Equal(0, global::Kerf.Engine.Approximation.Advanced.SysBase.mnfndeb_());
    }
}
