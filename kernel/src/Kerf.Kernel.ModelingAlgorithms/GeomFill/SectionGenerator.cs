// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionGenerator.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionGenerator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The sections of a surface through curves, for <see cref="SurfaceAppSurf"/>: the profiler's curves, set to a
/// common profile, are the sections, and section P is the curve P.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="SetParam"/> keeps the caller's array, as OCCT keeps the handle, and
/// writes into it (occt-issues.md); the parameters are needed only for an approximation with known parameters.
/// </remarks>
internal class SectionGenerator : Profiler, ISectionGenerator<int>
{
    protected NCollection_HArray1<double>? _params;

    public SectionGenerator()
    {
        if (_sequence.Length() > 1)
        {
            NCollection_HArray1<double> hPar = new(1, _sequence.Length());
            for (int i = 1; i <= _sequence.Length(); i++)
            {
                hPar.ChangeValue(i) = i - 1;
            }
            SetParam(hPar);
        }
    }

    /// <summary>The parameters of the sections, kept (not copied).</summary>
    public void SetParam(NCollection_HArray1<double> Params)
    {
        int ii, l = Params.Upper() - Params.Lower() + 1;
        _params = Params;
        for (ii = 1; ii <= l; ii++)
        {
            // OCCT: myParams is Params, so this shifts the caller's own values when Lower() != 1 (occt-issues.md)
            _params.SetValue(ii, Params.Value(Params.Lower() + ii - 1));
        }
    }

    public void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d)
    {
        BSplineCurve3D c = (BSplineCurve3D)_sequence[1];
        nbPoles = c.PoleCount;
        nbKnots = c.KnotCount;
        degree = c.Degree;
        nbPoles2d = 0;
    }

    public void Knots(NCollection_Array1<double> tKnots)
    {
        tKnots.Assign(((BSplineCurve3D)_sequence[1]).InternalKnots());
    }

    public void Mults(NCollection_Array1<int> tMults)
    {
        tMults.Assign(((BSplineCurve3D)_sequence[1]).InternalMultiplicities());
    }

    /// <summary>Used for the first and last sections; returns false: the derivatives are not computed.</summary>
    public bool Section(int p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                        NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                        NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths)
    {
        Section(p, poles, poles2d, weigths);
        return false;
    }

    public void Section(int p, NCollection_Array1<Point3D> poles, NCollection_Array1<Point2D> poles2d,
                        NCollection_Array1<double> weigths)
    {
        BSplineCurve3D c = (BSplineCurve3D)_sequence[p];

        poles.Assign(c.InternalPoles());
        weigths.Assign(c.InternalWeightsArray());
    }

    /// <summary>The parameter of section <paramref name="p"/>, to impose it on the approximation.</summary>
    /// <exception cref="NullReferenceException"><see cref="SetParam"/> was not called (OCCT dereferences a null handle).</exception>
    public double Parameter(int p)
    {
        return _params!.Value(p);
    }
}
