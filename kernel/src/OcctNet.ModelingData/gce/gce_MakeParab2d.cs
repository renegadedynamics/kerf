// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Parab2d: from a symmetry axis or an axis system and a focal length, from a directrix and a
/// focus, or from a focus and a vertex.
/// </summary>
public sealed class gce_MakeParab2d : gce_Root
{
    private Parabola2D TheParab2d = new();

    /// <summary>
    /// A parabola from its axis system and focal length (0 is accepted); fails with gce_NullFocusLength if Focal &lt; 0.
    /// </summary>
    public gce_MakeParab2d(in Frame2D A, double Focal)
    {
        if (Focal < 0.0)
        {
            TheError = gce_NullFocusLength;
        }
        else
        {
            TheParab2d = new Parabola2D(A, Focal);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A parabola from its symmetry axis, focal length (0 is accepted) and sense of parametrization; fails with
    /// gce_NullFocusLength if Focal &lt; 0.
    /// </summary>
    public gce_MakeParab2d(in Axis2D MirrorAxis, double Focal, bool Sense = true)
    {
        if (Focal < 0.0)
        {
            TheError = gce_NullFocusLength;
        }
        else
        {
            TheParab2d = new Parabola2D(MirrorAxis, Focal, Sense);
            TheError = gce_Done;
        }
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public gce_MakeParab2d(in Axis2D D, in Point2D F, bool Sense = true)
    {
        TheParab2d = new Parabola2D(D, F, Sense);
        TheError = gce_Done;
    }

    /// <summary>
    /// A parabola from its focus S and vertex Center; fails with gce_NullAxis if they coincide.
    /// </summary>
    public gce_MakeParab2d(in Point2D S, in Point2D Center, bool Sense = true)
    {
        if (S.Distance(Center) >= Constants.Resolution())
        {
            Direction2D XAxis = new(S.XY() - Center.XY());
            TheParab2d = new Parabola2D(new Axis2D(Center, XAxis), S.Distance(Center), Sense);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NullAxis;
        }
    }

    /// <summary>The constructed parabola.</summary>
    /// <exception cref="NotDoneException">No parabola was constructed.</exception>
    public Parabola2D Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeParab2d::Value() - no result");
        return TheParab2d;
    }

    /// <summary>Alias for Value().</summary>
    public Parabola2D Operator() => Value();

    /// <summary>The constructed parabola (OCCT's implicit conversion operator).</summary>
    public static explicit operator Parabola2D(gce_MakeParab2d theMaker) => theMaker.Operator();
}
