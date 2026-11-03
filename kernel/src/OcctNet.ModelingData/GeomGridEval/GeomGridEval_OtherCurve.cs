// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Fallback batch evaluator for any curve adaptor: evaluates point by point through the adaptor's Value, D1 to
/// D3 and DN.
/// </summary>
/// <remarks>
/// OCCT keeps a reference to the adaptor, which must outlive the evaluator; the port keeps the adaptor object.
/// </remarks>
public sealed class GeomGridEval_OtherCurve : GeomGridEval.ICurveEvaluator
{
    private readonly Adaptor3d_Curve myCurve;

    /// <summary>Constructor with the curve adaptor to evaluate.</summary>
    public GeomGridEval_OtherCurve(Adaptor3d_Curve theCurve)
    {
        myCurve = theCurve;
    }

    /// <summary>Returns the curve adaptor.</summary>
    public Adaptor3d_Curve Curve() => myCurve;

    /// <summary>Evaluates all grid points; 1-based, or empty if there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Point3D> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aResult.SetValue(i - theParams.Lower() + 1, myCurve.Value(theParams.Value(i)));
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD1>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD1> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            myCurve.D1(theParams.Value(i), out Point3D aPoint, out Vector3D aD1);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD1 { Point = aPoint, D1 = aD1 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD2>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD2> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            myCurve.D2(theParams.Value(i), out Point3D aPoint, out Vector3D aD1, out Vector3D aD2);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD2 { Point = aPoint, D1 = aD1, D2 = aD2 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD3> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            myCurve.D3(theParams.Value(i), out Point3D aPoint, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD3 { Point = aPoint, D1 = aD1, D2 = aD2, D3 = aD3 };
        }
        return aResult;
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points: from the D1-D3 grids for N up to 3, with the adaptor's DN
    /// above.
    /// </summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Vector3D> aResult = new(1, aNb);

        // Reuse existing grid evaluators for orders 1-3
        if (theN == 1)
        {
            NCollection_Array1<Geom_Curve.ResD1> aD1Grid = EvaluateGridD1(theParams);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD1Grid[i].D1);
            }
        }
        else if (theN == 2)
        {
            NCollection_Array1<Geom_Curve.ResD2> aD2Grid = EvaluateGridD2(theParams);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD2Grid[i].D2);
            }
        }
        else if (theN == 3)
        {
            NCollection_Array1<Geom_Curve.ResD3> aD3Grid = EvaluateGridD3(theParams);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD3Grid[i].D3);
            }
        }
        else
        {
            // For orders > 3, use adaptor DN method
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                aResult.SetValue(i - theParams.Lower() + 1, myCurve.DN(theParams.Value(i), theN));
            }
        }
        return aResult;
    }
}
