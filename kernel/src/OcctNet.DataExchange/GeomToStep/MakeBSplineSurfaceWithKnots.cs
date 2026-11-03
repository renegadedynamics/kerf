// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineSurfaceWithKnots.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineSurfaceWithKnots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Kerf.Engine.Exchange.Step.Data.Logical;
using static Kerf.Engine.Exchange.Step.Geometry.StepBSplineSurfaceForm;
using static Kerf.Engine.Exchange.Step.Geometry.StepKnotType;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of BSplineSurface from Geom to a STEP b_spline_surface_with_knots.</summary>
public sealed class MakeBSplineSurfaceWithKnots : GeometryRoot
{
    private StepBSplineSurfaceWithKnots _bSplineSurfaceWithKnots;

    //=============================================================================
    // Creation d' une bspline_Surface_with_knots_and_rational_bspline_Surface de
    // prostep a partir d' une BSplineSurface de Geom
    //=============================================================================

    /// <summary>A B-spline surface (its weights ignored), its poles divided by the length factor.</summary>
    public MakeBSplineSurfaceWithKnots(BSplineSurface bs, in Factors localFactors)
    {
        StepBSplineSurfaceWithKnots bswk;
        int uDegree, vDegree, nu, nv, i, j, nUknots, nVknots, itampon;
        double rtampon;
        StepCartesianPoint pt;
        NCollection_HArray2<StepCartesianPoint> controlPointsList;
        StepBSplineSurfaceForm surfaceForm;
        Logical uClosed, vClosed, selfIntersect;
        NCollection_HArray1<int> uMultiplicities, vMultiplicities;
        NCollection_HArray1<double> uKnots, vKnots;
        BSplKnotDistribution uDistribution, vDistribution;
        StepKnotType knotSpec;

        uDegree = bs.UDegree();
        vDegree = bs.VDegree();

        nu = bs.UPoleCount;
        nv = bs.VPoleCount;
        NCollection_Array2<Point3D> p = bs.InternalPoles();
        controlPointsList = new NCollection_HArray2<StepCartesianPoint>(1, nu, 1, nv);
        for (i = p.LowerRow(); i <= p.UpperRow(); i++)
        {
            for (j = p.LowerCol(); j <= p.UpperCol(); j++)
            {
                MakeCartesianPoint mkPoint = new MakeCartesianPoint(p.Value(i, j), localFactors.LengthFactor());
                pt = mkPoint.Value();
                controlPointsList.SetValue(i, j, pt);
            }
        }

        surfaceForm = StepBSplineSurfaceForm.BssfUnspecified;

        if (bs.IsUClosed)
        {
            uClosed = Logical.LTrue;
        }
        else
        {
            uClosed = Logical.LFalse;
        }

        if (bs.IsVClosed)
        {
            vClosed = Logical.LTrue;
        }
        else
        {
            vClosed = Logical.LFalse;
        }

        selfIntersect = Logical.LFalse;

        nUknots = bs.UKnotCount;
        nVknots = bs.VKnotCount;
        NCollection_Array1<int> mu = bs.InternalUMultiplicities();
        uMultiplicities = new NCollection_HArray1<int>(1, nUknots);
        for (i = mu.Lower(); i <= mu.Upper(); i++)
        {
            itampon = mu.Value(i);
            uMultiplicities.SetValue(i, itampon);
        }
        NCollection_Array1<int> mv = bs.InternalVMultiplicities();
        vMultiplicities = new NCollection_HArray1<int>(1, nVknots);
        for (i = mv.Lower(); i <= mv.Upper(); i++)
        {
            itampon = mv.Value(i);
            vMultiplicities.SetValue(i, itampon);
        }

        NCollection_Array1<double> ku = bs.InternalUKnots();
        NCollection_Array1<double> kv = bs.InternalVKnots();
        uKnots = new NCollection_HArray1<double>(1, nUknots);
        vKnots = new NCollection_HArray1<double>(1, nVknots);
        for (i = ku.Lower(); i <= ku.Upper(); i++)
        {
            rtampon = ku.Value(i);
            uKnots.SetValue(i, rtampon);
        }
        for (i = kv.Lower(); i <= kv.Upper(); i++)
        {
            rtampon = kv.Value(i);
            vKnots.SetValue(i, rtampon);
        }

        uDistribution = bs.UKnotDistribution();
        vDistribution = bs.VKnotDistribution();
        if (uDistribution == NonUniform && vDistribution == NonUniform)
        {
            knotSpec = StepKnotType.KtUnspecified;
        }
        else if (uDistribution == Uniform && vDistribution == Uniform)
        {
            knotSpec = StepKnotType.KtUniformKnots;
        }
        else if (uDistribution == QuasiUniform && vDistribution == QuasiUniform)
        {
            knotSpec = StepKnotType.KtQuasiUniformKnots;
        }
        else if (uDistribution == PiecewiseBezier && vDistribution == PiecewiseBezier)
        {
            knotSpec = StepKnotType.KtPiecewiseBezierKnots;
        }
        else
        {
            knotSpec = StepKnotType.KtUnspecified;
        }

        bswk = new StepBSplineSurfaceWithKnots();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        bswk.Init(name,
                  uDegree,
                  vDegree,
                  controlPointsList,
                  surfaceForm,
                  uClosed,
                  vClosed,
                  selfIntersect,
                  uMultiplicities,
                  vMultiplicities,
                  uKnots,
                  vKnots,
                  knotSpec);

        _bSplineSurfaceWithKnots = bswk;
        _done = true;
    }

    /// <summary>A B-spline surface (default factors).</summary>
    public MakeBSplineSurfaceWithKnots(BSplineSurface bs) : this(bs, new Factors())
    {
    }

    /// <summary>The b_spline_surface_with_knots made.</summary>
    public StepBSplineSurfaceWithKnots Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeBSplineSurfaceWithKnots::Value() - no result");
        return _bSplineSurfaceWithKnots;
    }
}
