// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ChamfSpine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ChamfSpine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.ChFiDS_ChamfMethod;
using static Occt.ChFiDS_ChamfMode;

namespace Occt;

/// <summary>The spine of a chamfer, with its distances (or distance and angle) on the faces.</summary>
public sealed class ChFiDS_ChamfSpine : ChFiDS_Spine
{
    private double d1;
    private double d2;
    // bool dison1;
    private double angle;
    private ChFiDS_ChamfMethod mChamf;

    public ChFiDS_ChamfSpine()
    {
        d1 = 0.0;
        d2 = 0.0;
        angle = 0.0;
        mChamf = ChFiDS_Sym;
        myMode = ChFiDS_ClassicChamfer;
    }

    public ChFiDS_ChamfSpine(double Tol)
        : base(Tol)
    {
        d1 = 0.0;
        d2 = 0.0;
        angle = 0.0;
        mChamf = ChFiDS_Sym;
        myMode = ChFiDS_ClassicChamfer;
    }

    /// <summary>The distance of a symmetric chamfer; throws <see cref="FailureException"/> otherwise.</summary>
    public void GetDist(out double Dis)
    {
        if (mChamf != ChFiDS_Sym)
        {
            throw new FailureException("Chamfer is not symmetric");
        }

        Dis = d1;
    }

    internal void SetDist(double Dis)
    {
        // isconstant->Init(true);
        mChamf = ChFiDS_Sym;
        d1 = Dis;
    }

    /// <summary>The distances of a two-distance chamfer; throws <see cref="FailureException"/> otherwise.</summary>
    public void Dists(out double Dis1, out double Dis2)
    {
        if (mChamf != ChFiDS_TwoDist)
        {
            throw new FailureException("Chamfer is not a Two Dists Chamfer");
        }

        Dis1 = d1;
        Dis2 = d2;
    }

    internal void SetDists(double Dis1, double Dis2)
    {
        // isconstant->Init(true);
        mChamf = ChFiDS_TwoDist;
        d1 = Dis1;
        d2 = Dis2;
    }

    /// <summary>The distance and angle of a distance-angle chamfer; throws <see cref="FailureException"/> otherwise
    /// (with OCCT's message).</summary>
    public void GetDistAngle(out double Dis, out double Angle)
    {
        if (mChamf != ChFiDS_DistAngle)
        {
            throw new FailureException("Chamfer is not a Two Dists Chamfer");
        }

        Dis = d1;
        Angle = angle;
        // DisOnF1 = dison1;
    }

    internal void SetDistAngle(double Dis, double Angle)
    {
        // isconstant->Init(true);
        mChamf = ChFiDS_DistAngle;
        d1 = Dis;
        angle = Angle;
        // dison1 = DisOnF1;
    }

    internal void SetMode(ChFiDS_ChamfMode theMode)
    {
        myMode = theMode;
    }

    /// <summary>The method of the chamfer.</summary>
    public ChFiDS_ChamfMethod IsChamfer()
    {
        return mChamf;
    }
}
