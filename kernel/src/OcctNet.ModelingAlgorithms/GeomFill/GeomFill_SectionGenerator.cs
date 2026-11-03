// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionGenerator.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionGenerator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The sections of a surface through curves, for <see cref="GeomFill_AppSurf"/>: the profiler's curves, set to a
/// common profile, are the sections, and section P is the curve P.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="SetParam"/> keeps the caller's array, as OCCT keeps the handle, and
/// writes into it (occt-issues.md); the parameters are needed only for an approximation with known parameters.
/// </remarks>
public class GeomFill_SectionGenerator : GeomFill_Profiler, IAppBlend_SectionGenerator<int>
{
    protected NCollection_HArray1<double>? myParams;

    public GeomFill_SectionGenerator()
    {
        if (mySequence.Length() > 1)
        {
            NCollection_HArray1<double> HPar = new(1, mySequence.Length());
            for (int i = 1; i <= mySequence.Length(); i++)
            {
                HPar.ChangeValue(i) = i - 1;
            }
            SetParam(HPar);
        }
    }

    /// <summary>The parameters of the sections, kept (not copied).</summary>
    public void SetParam(NCollection_HArray1<double> Params)
    {
        int ii, L = Params.Upper() - Params.Lower() + 1;
        myParams = Params;
        for (ii = 1; ii <= L; ii++)
        {
            // OCCT: myParams is Params, so this shifts the caller's own values when Lower() != 1 (occt-issues.md)
            myParams.SetValue(ii, Params.Value(Params.Lower() + ii - 1));
        }
    }

    public void GetShape(out int NbPoles, out int NbKnots, out int Degree, out int NbPoles2d)
    {
        BSplineCurve3D C = (BSplineCurve3D)mySequence[1];
        NbPoles = C.PoleCount;
        NbKnots = C.KnotCount;
        Degree = C.Degree();
        NbPoles2d = 0;
    }

    public void Knots(NCollection_Array1<double> TKnots)
    {
        TKnots.Assign(((BSplineCurve3D)mySequence[1]).InternalKnots());
    }

    public void Mults(NCollection_Array1<int> TMults)
    {
        TMults.Assign(((BSplineCurve3D)mySequence[1]).InternalMultiplicities());
    }

    /// <summary>Used for the first and last sections; returns false: the derivatives are not computed.</summary>
    public bool Section(int P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                        NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                        NCollection_Array1<double> Weigths, NCollection_Array1<double> DWeigths)
    {
        Section(P, Poles, Poles2d, Weigths);
        return false;
    }

    public void Section(int P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Point2D> Poles2d,
                        NCollection_Array1<double> Weigths)
    {
        BSplineCurve3D C = (BSplineCurve3D)mySequence[P];

        Poles.Assign(C.InternalPoles());
        Weigths.Assign(C.InternalWeightsArray());
    }

    /// <summary>The parameter of section <paramref name="P"/>, to impose it on the approximation.</summary>
    /// <exception cref="NullReferenceException"><see cref="SetParam"/> was not called (OCCT dereferences a null handle).</exception>
    public double Parameter(int P)
    {
        return myParams!.Value(P);
    }
}
