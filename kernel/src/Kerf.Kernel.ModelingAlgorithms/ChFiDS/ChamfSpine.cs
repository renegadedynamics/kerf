// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ChamfSpine.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_ChamfSpine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Fillets.Data.ChamfMethod;
using static Kerf.Engine.Fillets.Data.ChamfMode;

namespace Kerf.Engine.Fillets.Data;

/// <summary>The spine of a chamfer, with its distances (or distance and angle) on the faces.</summary>
internal sealed class ChamfSpine : Spine
{
    private double _d1;
    private double _d2;
    // bool dison1;
    private double _angle;
    private ChamfMethod _mChamf;

    public ChamfSpine()
    {
        _d1 = 0.0;
        _d2 = 0.0;
        _angle = 0.0;
        _mChamf = ChamfMethod.Sym;
        _mode = ChamfMode.ClassicChamfer;
    }

    public ChamfSpine(double tol)
        : base(tol)
    {
        _d1 = 0.0;
        _d2 = 0.0;
        _angle = 0.0;
        _mChamf = ChamfMethod.Sym;
        _mode = ChamfMode.ClassicChamfer;
    }

    /// <summary>The distance of a symmetric chamfer; throws <see cref="FailureException"/> otherwise.</summary>
    public void GetDist(out double dis)
    {
        if (_mChamf != ChamfMethod.Sym)
        {
            throw new FailureException("Chamfer is not symmetric");
        }

        dis = _d1;
    }

    internal void SetDist(double dis)
    {
        // isconstant->Init(true);
        _mChamf = ChamfMethod.Sym;
        _d1 = dis;
    }

    /// <summary>The distances of a two-distance chamfer; throws <see cref="FailureException"/> otherwise.</summary>
    public void Dists(out double dis1, out double dis2)
    {
        if (_mChamf != ChamfMethod.TwoDist)
        {
            throw new FailureException("Chamfer is not a Two Dists Chamfer");
        }

        dis1 = _d1;
        dis2 = _d2;
    }

    internal void SetDists(double dis1, double dis2)
    {
        // isconstant->Init(true);
        _mChamf = ChamfMethod.TwoDist;
        _d1 = dis1;
        _d2 = dis2;
    }

    /// <summary>The distance and angle of a distance-angle chamfer; throws <see cref="FailureException"/> otherwise
    /// (with OCCT's message).</summary>
    public void GetDistAngle(out double dis, out double angle)
    {
        if (_mChamf != ChamfMethod.DistAngle)
        {
            throw new FailureException("Chamfer is not a Two Dists Chamfer");
        }

        dis = _d1;
        angle = _angle;
        // DisOnF1 = dison1;
    }

    internal void SetDistAngle(double dis, double angle)
    {
        // isconstant->Init(true);
        _mChamf = ChamfMethod.DistAngle;
        _d1 = dis;
        _angle = angle;
        // dison1 = DisOnF1;
    }

    internal void SetMode(ChamfMode mode)
    {
        _mode = mode;
    }

    /// <summary>The method of the chamfer.</summary>
    public ChamfMethod IsChamfer
    {
        get
        {
            return _mChamf;
        }
    }
}
