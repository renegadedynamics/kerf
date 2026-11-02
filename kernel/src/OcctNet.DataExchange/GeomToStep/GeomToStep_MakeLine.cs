// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeLine.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeLine.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeLine_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of Line from Geom and Geom2d (and Lin and Lin2d from gp) to a STEP line.</summary>
/// <remarks>
/// The line's vector is its unit direction made a vector: for a 3D line its magnitude is 1 divided by the length
/// factor, as OCCT writes it.
/// </remarks>
public sealed class GeomToStep_MakeLine : GeomToStep_Root
{
    private StepGeom_Line theLine;

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Lin de gp
    //=============================================================================

    /// <summary>A 3D line, its location divided by the length factor.</summary>
    public GeomToStep_MakeLine(in gp_Lin L, in StepData_Factors theLocalFactors)
    {
        Make(new GeomToStep_MakeCartesianPoint(L.Location(), theLocalFactors.LengthFactor()),
             new GeomToStep_MakeVector(new gp_Vec(L.Direction()), theLocalFactors));
    }

    /// <summary>A 3D line (default factors).</summary>
    public GeomToStep_MakeLine(in gp_Lin L) : this(L, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Lin2d de gp
    //=============================================================================

    /// <summary>A 2D line.</summary>
    public GeomToStep_MakeLine(in gp_Lin2d L, in StepData_Factors theLocalFactors)
    {
        Make(new GeomToStep_MakeCartesianPoint(L.Location(), theLocalFactors.LengthFactor()),
             new GeomToStep_MakeVector(new gp_Vec2d(L.Direction()), theLocalFactors));
    }

    /// <summary>A 2D line.</summary>
    public GeomToStep_MakeLine(in gp_Lin2d L) : this(L, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Line de Geom
    //=============================================================================

    /// <summary>A 3D line, its location divided by the length factor.</summary>
    public GeomToStep_MakeLine(Geom_Line Gline, in StepData_Factors theLocalFactors)
    {
        gp_Lin L;
        L = Gline.Lin();
        Make(new GeomToStep_MakeCartesianPoint(L.Location(), theLocalFactors.LengthFactor()),
             new GeomToStep_MakeVector(new gp_Vec(L.Direction()), theLocalFactors));
    }

    /// <summary>A 3D line (default factors).</summary>
    public GeomToStep_MakeLine(Geom_Line Gline) : this(Gline, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Line de Geom2d
    //=============================================================================

    /// <summary>A 2D line.</summary>
    public GeomToStep_MakeLine(Geom2d_Line Gline, in StepData_Factors theLocalFactors)
    {
        gp_Lin2d L;
        L = Gline.Lin2d();
        Make(new GeomToStep_MakeCartesianPoint(L.Location(), theLocalFactors.LengthFactor()),
             new GeomToStep_MakeVector(new gp_Vec2d(L.Direction()), theLocalFactors));
    }

    /// <summary>A 2D line.</summary>
    public GeomToStep_MakeLine(Geom2d_Line Gline) : this(Gline, new StepData_Factors())
    {
    }

    // GeomToStep_MakeLine_gen.pxx, the body OCCT includes in each constructor (its point and vector made first)
    private void Make(GeomToStep_MakeCartesianPoint MkPoint, GeomToStep_MakeVector MkVector)
    {
        StepGeom_Line Lin = new StepGeom_Line();
        StepGeom_CartesianPoint aPnt;
        StepGeom_Vector aDir;

        aPnt = MkPoint.Value();
        aDir = MkVector.Value();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Lin.Init(name, aPnt, aDir);
        theLine = Lin;
        done = true;
    }

    /// <summary>The line made.</summary>
    public StepGeom_Line Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeLine::Value() - no result");
        return theLine;
    }
}
