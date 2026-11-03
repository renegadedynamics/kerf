// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_ComplexCurve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_ComplexCurve.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_ComplexCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A curve made of several segments, parametrised on [0, 1]: each global parameter is mapped to a segment and a
/// local parameter on it, and derivatives are scaled by the segment's factor.
/// </summary>
public abstract class ShapeExtend_ComplexCurve : Geom_Curve
{
    protected bool myClosed;

    protected ShapeExtend_ComplexCurve()
    {
        myClosed = false;
    }

    /// <summary>Returns the number of curves.</summary>
    public abstract int NbCurves();

    /// <summary>Returns the curve of the given index.</summary>
    public abstract Geom_Curve Curve(int index);

    /// <summary>Returns the index of the curve holding the parameter U, and the local parameter on it.</summary>
    public abstract int LocateParameter(double U, out double UOut);

    /// <summary>Returns the global parameter of the local parameter on the segment of the given index.</summary>
    public abstract double LocalToGlobal(int index, double Ulocal);

    /// <summary>Applies the transformation to each curve.</summary>
    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        for (int i = 1; i <= NbCurves(); i++)
        {
            Curve(i).Transform(T);
        }
    }

    /// <summary>Returns 1 - U.</summary>
    public override double ReversedParameter(double U) => (1 - U);

    /// <summary>Returns 0.</summary>
    public override double FirstParameter() => 0;

    /// <summary>Returns 1.</summary>
    public override double LastParameter() => 1;

    public override bool IsClosed() => myClosed;

    /// <summary>Returns false.</summary>
    public override bool IsPeriodic() => false;

    /// <summary>Returns GeomAbs_C0.</summary>
    public override Continuity Continuity() => C0;

    /// <summary>Returns false if N &gt; 0.</summary>
    public override bool IsCN(int N) => N <= 0;

    /// <summary>The point at U: found on the segment holding U, at its local parameter.</summary>
    public override Point3D EvalD0(double U)
    {
        int anInd = LocateParameter(U, out double UOut);
        return Curve(anInd).EvalD0(UOut);
    }

    public override ResD1 EvalD1(double U)
    {
        int   anInd   = LocateParameter(U, out double UOut);
        ResD1 aResult = Curve(anInd).EvalD1(UOut);
        TransformDN(ref aResult.D1, anInd, 1);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        int   anInd   = LocateParameter(U, out double UOut);
        ResD2 aResult = Curve(anInd).EvalD2(UOut);
        TransformDN(ref aResult.D1, anInd, 1);
        TransformDN(ref aResult.D2, anInd, 2);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        int   anInd   = LocateParameter(U, out double UOut);
        ResD3 aResult = Curve(anInd).EvalD3(UOut);
        TransformDN(ref aResult.D1, anInd, 1);
        TransformDN(ref aResult.D2, anInd, 2);
        TransformDN(ref aResult.D3, anInd, 3);
        return aResult;
    }

    public override Vector3D EvalDN(double U, int N)
    {
        int    anInd   = LocateParameter(U, out double UOut);
        Vector3D aResult = Curve(anInd).EvalDN(UOut, N);
        if (N != 0)
        {
            TransformDN(ref aResult, anInd, N);
        }

        return aResult;
    }

    /// <summary>Returns the scale factor of the derivatives on the segment of the given index.</summary>
    public abstract double GetScaleFactor(int ind);

    /// <summary>Checks the geometrical connectivity of the curves, and sets the closure flag.</summary>
    protected internal bool CheckConnectivity(double Preci)
    {
        CheckMutable();
        int  NbC = NbCurves();
        bool ok  = true;
        for (int i = 1; i < NbC; i++)
        {
            if (i == 1)
            {
                myClosed = Value(FirstParameter()).IsEqual(Value(LastParameter()), Preci);
            }

            ok &= Curve(i)
                    .Value(Curve(i).LastParameter())
                    .IsEqual(Curve(i + 1).Value(Curve(i + 1).FirstParameter()), Preci);
        }

        return ok;
    }

    /// <summary>Transforms the derivative according to its order.</summary>
    protected void TransformDN(ref Vector3D V, int ind, int N)
    {
        double fact = GetScaleFactor(ind);
        for (int i = 1; i <= N; i++)
        {
            V.Multiply(fact);
        }
    }
}
