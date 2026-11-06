// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_Add2dCurve.hxx
// Port of: src/ModelingData/TKGeomBase/BndLib/BndLib_Add2dCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// Adds the bounding box of a 2D curve to a box (through <see cref="Curve2dBounds"/>). The box is enlarged by
/// the tolerance; an infinite curve opens it.
/// </summary>
internal static class Add2dCurve
{
    /// <summary>Adds the bounding box of the curve C, over its own range, enlarged by Tol, to B.</summary>
    public static void Add(Curve2DAdaptorBase aC, double aTol, ref Box2d aBox2D)
    {
        Add(aC, aC.FirstParameter(), aC.LastParameter(), aTol, ref aBox2D);
    }

    /// <summary>
    /// Adds the bounding box of the arc [U1, U2] of C, enlarged by Tol, to B. A Geom2dAdaptor_Curve goes through its
    /// curve; any other adaptor is sampled at 33 points.
    /// </summary>
    public static void Add(Curve2DAdaptorBase aC, double aU1, double aU2, double aTol, ref Box2d aBox2D)
    {
        if (aC is not CurveAdaptor2D pA)
        {
            double u, du;
            int n, j;
            Point2D p;
            n = 33;
            u = aU1;
            du = (aU2 - aU1) / (n - 1);
            for (j = 1; j < n; j++)
            {
                aC.D0(u, out p);
                u += du;
                aBox2D.Add(p);
            }
            aC.D0(aU2, out p);
            aBox2D.Add(p);
            aBox2D.Enlarge(aTol);
            return;
        }

        // OCCT dereferences the curve without checking it; an empty adaptor throws here.
        Curve2D c2D = pA.Curve()!;
        Add(c2D, aU1, aU2, aTol, ref aBox2D);
    }

    /// <summary>Adds the bounding box of the curve C, over its own range, enlarged by Tol, to B.</summary>
    public static void Add(Curve2D aC2D, double aTol, ref Box2d aBox2D)
    {
        double t1 = aC2D.FirstParameter;
        double t2 = aC2D.LastParameter;
        Add(aC2D, t1, t2, aTol, ref aBox2D);
    }

    /// <summary>Adds the bounding box of the arc [T1, T2] of C, enlarged by Tol, to B.</summary>
    public static void Add(Curve2D aC2D, double aT1, double aT2, double aTol, ref Box2d aBox2D)
    {
        new Curve2dBounds(aC2D).Add(aT1, aT2, aTol, ref aBox2D);
    }

    /// <summary>Adds the precise bounding box of the arc [T1, T2] of C to B (numerical search of the extrema; slower).</summary>
    public static void AddOptimal(Curve2D aC2D, double aT1, double aT2, double aTol, ref Box2d aBox2D)
    {
        new Curve2dBounds(aC2D).AddOptimal(aT1, aT2, aTol, ref aBox2D);
    }
}
