// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineSurfaceWithKnots.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeBSplineSurfaceWithKnots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Occt.StepData_Logical;
using static Occt.StepGeom_BSplineSurfaceForm;
using static Occt.StepGeom_KnotType;

namespace Occt;

/// <summary>The mapping of BSplineSurface from Geom to a STEP b_spline_surface_with_knots.</summary>
public sealed class GeomToStep_MakeBSplineSurfaceWithKnots : GeomToStep_Root
{
    private StepGeom_BSplineSurfaceWithKnots theBSplineSurfaceWithKnots;

    //=============================================================================
    // Creation d' une bspline_Surface_with_knots_and_rational_bspline_Surface de
    // prostep a partir d' une BSplineSurface de Geom
    //=============================================================================

    /// <summary>A B-spline surface (its weights ignored), its poles divided by the length factor.</summary>
    public GeomToStep_MakeBSplineSurfaceWithKnots(BSplineSurface BS, in StepData_Factors theLocalFactors)
    {
        StepGeom_BSplineSurfaceWithKnots BSWK;
        int aUDegree, aVDegree, NU, NV, i, j, NUknots, NVknots, itampon;
        double rtampon;
        StepGeom_CartesianPoint Pt;
        NCollection_HArray2<StepGeom_CartesianPoint> aControlPointsList;
        StepGeom_BSplineSurfaceForm aSurfaceForm;
        StepData_Logical aUClosed, aVClosed, aSelfIntersect;
        NCollection_HArray1<int> aUMultiplicities, aVMultiplicities;
        NCollection_HArray1<double> aUKnots, aVKnots;
        BSplKnotDistribution UDistribution, VDistribution;
        StepGeom_KnotType KnotSpec;

        aUDegree = BS.UDegree();
        aVDegree = BS.VDegree();

        NU = BS.UPoleCount;
        NV = BS.VPoleCount;
        NCollection_Array2<Point3D> P = BS.InternalPoles();
        aControlPointsList = new NCollection_HArray2<StepGeom_CartesianPoint>(1, NU, 1, NV);
        for (i = P.LowerRow(); i <= P.UpperRow(); i++)
        {
            for (j = P.LowerCol(); j <= P.UpperCol(); j++)
            {
                GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(P.Value(i, j), theLocalFactors.LengthFactor());
                Pt = MkPoint.Value();
                aControlPointsList.SetValue(i, j, Pt);
            }
        }

        aSurfaceForm = StepGeom_bssfUnspecified;

        if (BS.IsUClosed)
        {
            aUClosed = StepData_LTrue;
        }
        else
        {
            aUClosed = StepData_LFalse;
        }

        if (BS.IsVClosed)
        {
            aVClosed = StepData_LTrue;
        }
        else
        {
            aVClosed = StepData_LFalse;
        }

        aSelfIntersect = StepData_LFalse;

        NUknots = BS.UKnotCount;
        NVknots = BS.VKnotCount;
        NCollection_Array1<int> MU = BS.InternalUMultiplicities();
        aUMultiplicities = new NCollection_HArray1<int>(1, NUknots);
        for (i = MU.Lower(); i <= MU.Upper(); i++)
        {
            itampon = MU.Value(i);
            aUMultiplicities.SetValue(i, itampon);
        }
        NCollection_Array1<int> MV = BS.InternalVMultiplicities();
        aVMultiplicities = new NCollection_HArray1<int>(1, NVknots);
        for (i = MV.Lower(); i <= MV.Upper(); i++)
        {
            itampon = MV.Value(i);
            aVMultiplicities.SetValue(i, itampon);
        }

        NCollection_Array1<double> KU = BS.InternalUKnots();
        NCollection_Array1<double> KV = BS.InternalVKnots();
        aUKnots = new NCollection_HArray1<double>(1, NUknots);
        aVKnots = new NCollection_HArray1<double>(1, NVknots);
        for (i = KU.Lower(); i <= KU.Upper(); i++)
        {
            rtampon = KU.Value(i);
            aUKnots.SetValue(i, rtampon);
        }
        for (i = KV.Lower(); i <= KV.Upper(); i++)
        {
            rtampon = KV.Value(i);
            aVKnots.SetValue(i, rtampon);
        }

        UDistribution = BS.UKnotDistribution();
        VDistribution = BS.VKnotDistribution();
        if (UDistribution == NonUniform && VDistribution == NonUniform)
        {
            KnotSpec = StepGeom_ktUnspecified;
        }
        else if (UDistribution == Uniform && VDistribution == Uniform)
        {
            KnotSpec = StepGeom_ktUniformKnots;
        }
        else if (UDistribution == QuasiUniform && VDistribution == QuasiUniform)
        {
            KnotSpec = StepGeom_ktQuasiUniformKnots;
        }
        else if (UDistribution == PiecewiseBezier && VDistribution == PiecewiseBezier)
        {
            KnotSpec = StepGeom_ktPiecewiseBezierKnots;
        }
        else
        {
            KnotSpec = StepGeom_ktUnspecified;
        }

        BSWK = new StepGeom_BSplineSurfaceWithKnots();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        BSWK.Init(name,
                  aUDegree,
                  aVDegree,
                  aControlPointsList,
                  aSurfaceForm,
                  aUClosed,
                  aVClosed,
                  aSelfIntersect,
                  aUMultiplicities,
                  aVMultiplicities,
                  aUKnots,
                  aVKnots,
                  KnotSpec);

        theBSplineSurfaceWithKnots = BSWK;
        done = true;
    }

    /// <summary>A B-spline surface (default factors).</summary>
    public GeomToStep_MakeBSplineSurfaceWithKnots(BSplineSurface BS) : this(BS, new StepData_Factors())
    {
    }

    /// <summary>The b_spline_surface_with_knots made.</summary>
    public StepGeom_BSplineSurfaceWithKnots Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeBSplineSurfaceWithKnots::Value() - no result");
        return theBSplineSurfaceWithKnots;
    }
}
