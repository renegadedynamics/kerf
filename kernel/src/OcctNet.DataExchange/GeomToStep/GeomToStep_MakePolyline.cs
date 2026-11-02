// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePolyline.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePolyline.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePolyline_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of an array of points from gp to a STEP polyline.</summary>
public sealed class GeomToStep_MakePolyline : GeomToStep_Root
{
    private StepGeom_Polyline thePolyline;

    //=============================================================================
    // Creation d' une polyline Step a partir d' une Array1OfPnt
    //=============================================================================

    /// <summary>3D points, divided by the length factor (the array's lower bound must be 1, as OCCT assumes).</summary>
    public GeomToStep_MakePolyline(NCollection_Array1<gp_Pnt> P, in StepData_Factors theLocalFactors)
    {
        int N, i;
        NCollection_HArray1<StepGeom_CartesianPoint> aPoints;
        StepGeom_CartesianPoint P2;
        gp_Pnt P1;

        N = P.Length();
        aPoints = new NCollection_HArray1<StepGeom_CartesianPoint>(1, N);
        for (i = P.Lower(); i <= P.Upper(); i++)
        {
            P1 = P.Value(i);
            GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(P1, theLocalFactors.LengthFactor());
            P2 = MkPoint.Value();
            aPoints.SetValue(i, P2);
        }
        Make(aPoints);
    }

    /// <summary>3D points (default factors).</summary>
    public GeomToStep_MakePolyline(NCollection_Array1<gp_Pnt> P) : this(P, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une polyline Step a partir d' une Array1OfPnt2d
    //=============================================================================

    /// <summary>2D points (the array's lower bound must be 1).</summary>
    public GeomToStep_MakePolyline(NCollection_Array1<gp_Pnt2d> P, in StepData_Factors theLocalFactors)
    {
        int N, i;
        NCollection_HArray1<StepGeom_CartesianPoint> aPoints;
        StepGeom_CartesianPoint P2;
        gp_Pnt2d P1;

        N = P.Length();
        aPoints = new NCollection_HArray1<StepGeom_CartesianPoint>(1, N);
        for (i = P.Lower(); i <= P.Upper(); i++)
        {
            P1 = P.Value(i);
            GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(P1, theLocalFactors.LengthFactor());
            P2 = MkPoint.Value();
            aPoints.SetValue(i, P2);
        }
        Make(aPoints);
    }

    /// <summary>2D points.</summary>
    public GeomToStep_MakePolyline(NCollection_Array1<gp_Pnt2d> P) : this(P, new StepData_Factors())
    {
    }

    // The end of GeomToStep_MakePolyline_gen.pxx, the body OCCT includes in both constructors
    private void Make(NCollection_HArray1<StepGeom_CartesianPoint> aPoints)
    {
        thePolyline = new StepGeom_Polyline();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        thePolyline.Init(name, aPoints);
        done = true;
    }

    /// <summary>The polyline made.</summary>
    public StepGeom_Polyline Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakePolyline::Value() - no result");
        return thePolyline;
    }
}
