// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeConicalSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeConicalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom_ConicalSurface: from an axis system, a semi-angle and a radius, a gp_Cone, four points, or
/// two points and two radii.
/// </summary>
public sealed class GC_MakeConicalSurface : GC_Root
{
    private Geom_ConicalSurface? TheCone;

    /// <summary>
    /// A cone in the axis system A2 with semi-angle Ang and reference radius Radius; fails with gce_NegativeRadius
    /// if Radius is negative and with gce_BadAngle unless Resolution &lt; Ang &lt; π/2 − Resolution.
    /// </summary>
    public GC_MakeConicalSurface(in Frame3D A2, double Ang, double Radius)
    {
        if (Radius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else if (Ang <= Constants.Resolution() || Ang >= M_PI / 2.0 - Constants.Resolution())
        {
            TheError = gce_BadAngle;
        }
        else
        {
            TheError = gce_Done;
            TheCone = new Geom_ConicalSurface(new CoordinateSystem(A2), Ang, Radius);
        }
    }

    /// <summary>A cone from the gp_Cone C.</summary>
    public GC_MakeConicalSurface(in Cone C)
    {
        TheError = gce_Done;
        TheCone = new Geom_ConicalSurface(C);
    }

    /// <summary>A cone with axis P1P2 through P3 and P4; fails with the status of gce_MakeCone.</summary>
    public GC_MakeConicalSurface(in Point3D P1, in Point3D P2, in Point3D P3, in Point3D P4)
    {
        gce_MakeCone C = new gce_MakeCone(P1, P2, P3, P4);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCone = new Geom_ConicalSurface(C.Value());
        }
    }

    /// <summary>A cone with axis P1P2 and radii R1 at P1 and R2 at P2; fails with the status of gce_MakeCone.</summary>
    public GC_MakeConicalSurface(in Point3D P1, in Point3D P2, double R1, double R2)
    {
        gce_MakeCone C = new gce_MakeCone(P1, P2, R1, R2);
        TheError = C.Status();
        if (TheError == gce_Done)
        {
            TheCone = new Geom_ConicalSurface(C.Value());
        }
    }

    /// <summary>The constructed cone (frozen).</summary>
    /// <exception cref="NotDoneException">No cone was constructed.</exception>
    public Geom_ConicalSurface Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeConicalSurface::Value() - no result");
        TheCone!.Freeze();
        return TheCone;
    }

    /// <summary>The constructed cone (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_ConicalSurface(GC_MakeConicalSurface theMaker) => theMaker.Value();
}
