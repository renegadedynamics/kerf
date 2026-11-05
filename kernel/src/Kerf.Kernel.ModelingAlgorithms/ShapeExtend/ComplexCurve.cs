// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_ComplexCurve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_ComplexCurve.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_ComplexCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Healing.Extend;

/// <summary>
/// A curve made of several segments, parametrised on [0, 1]: each global parameter is mapped to a segment and a
/// local parameter on it, and derivatives are scaled by the segment's factor.
/// </summary>
internal abstract class ComplexCurve : Curve3D
{
    protected bool _closed;

    protected ComplexCurve()
    {
        _closed = false;
    }

    /// <summary>Returns the number of curves.</summary>
    public abstract int CurveCount { get; }

    /// <summary>Returns the curve of the given index.</summary>
    public abstract Curve3D Curve(int index);

    /// <summary>Returns the index of the curve holding the parameter U, and the local parameter on it.</summary>
    public abstract int LocateParameter(double u, out double uOut);

    /// <summary>Returns the global parameter of the local parameter on the segment of the given index.</summary>
    public abstract double LocalToGlobal(int index, double ulocal);

    /// <summary>Applies the transformation to each curve.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        for (int i = 1; i <= CurveCount; i++)
        {
            Curve(i).Transform(t);
        }
    }

    /// <summary>Returns 1 - U.</summary>
    public override double ReversedParameter(double u) => (1 - u);

    /// <summary>Returns 0.</summary>
    public override double FirstParameter => 0;

    /// <summary>Returns 1.</summary>
    public override double LastParameter => 1;

    public override bool IsClosed => _closed;

    /// <summary>Returns false.</summary>
    public override bool IsPeriodic => false;

    /// <summary>Returns GeomAbs_C0.</summary>
    public override Continuity Continuity => C0;

    /// <summary>Returns false if N &gt; 0.</summary>
    internal override bool IsCN(int n) => n <= 0;

    /// <summary>The point at U: found on the segment holding U, at its local parameter.</summary>
    internal override Point3D EvalD0(double u)
    {
        int ind = LocateParameter(u, out double uOut);
        return Curve(ind).EvalD0(uOut);
    }

    internal override ResD1 EvalD1(double u)
    {
        int   ind   = LocateParameter(u, out double uOut);
        ResD1 result = Curve(ind).EvalD1(uOut);
        TransformDN(ref result.D1, ind, 1);
        return result;
    }

    internal override ResD2 EvalD2(double u)
    {
        int   ind   = LocateParameter(u, out double uOut);
        ResD2 result = Curve(ind).EvalD2(uOut);
        TransformDN(ref result.D1, ind, 1);
        TransformDN(ref result.D2, ind, 2);
        return result;
    }

    internal override ResD3 EvalD3(double u)
    {
        int   ind   = LocateParameter(u, out double uOut);
        ResD3 result = Curve(ind).EvalD3(uOut);
        TransformDN(ref result.D1, ind, 1);
        TransformDN(ref result.D2, ind, 2);
        TransformDN(ref result.D3, ind, 3);
        return result;
    }

    internal override Vector3D EvalDN(double u, int n)
    {
        int    ind   = LocateParameter(u, out double uOut);
        Vector3D result = Curve(ind).EvalDN(uOut, n);
        if (n != 0)
        {
            TransformDN(ref result, ind, n);
        }

        return result;
    }

    /// <summary>Returns the scale factor of the derivatives on the segment of the given index.</summary>
    public abstract double GetScaleFactor(int ind);

    /// <summary>Checks the geometrical connectivity of the curves, and sets the closure flag.</summary>
    protected internal bool CheckConnectivity(double preci)
    {
        CheckMutable();
        int  nbC = CurveCount;
        bool ok  = true;
        for (int i = 1; i < nbC; i++)
        {
            if (i == 1)
            {
                _closed = Value(FirstParameter).IsEqual(Value(LastParameter), preci);
            }

            ok &= Curve(i)
                    .Value(Curve(i).LastParameter)
                    .IsEqual(Curve(i + 1).Value(Curve(i + 1).FirstParameter), preci);
        }

        return ok;
    }

    /// <summary>Transforms the derivative according to its order.</summary>
    protected void TransformDN(ref Vector3D v, int ind, int n)
    {
        double fact = GetScaleFactor(ind);
        for (int i = 1; i <= n; i++)
        {
            v = v.Multiplied(fact);
        }
    }
}
