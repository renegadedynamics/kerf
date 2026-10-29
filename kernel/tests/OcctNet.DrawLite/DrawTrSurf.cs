// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// DrawTrSurf's Set and Get (DrawTrSurf.cxx): geometry bound to Draw variables as <see cref="DrawGeometry"/>
/// drawables, and read back by the kind of drawable the variable holds.
/// </summary>
/// <remarks>
/// As shapes are frozen when bound (decision 272), geometry is frozen when <see cref="Set(DrawInterpreter, string, Geom_Geometry?)"/>
/// binds it: a drawable's geometry is shared by the shapes made from it and by <c>copy</c>.
/// </remarks>
internal static class DrawTrSurf
{
    /// <summary>DrawTrSurf::Set(name, Geom_Geometry): a curve or surface drawable; a null geometry unbinds the name.</summary>
    public static void Set(DrawInterpreter theDI, string theName, Geom_Geometry? theGeometry)
    {
        if (theGeometry is not (Geom_Curve or Geom_Surface))
        {
            // OCCT binds a null drawable: Draw::Set with null removes the variable's drawable
            theDI.SetDrawable(theName, null);
            return;
        }

        theGeometry.Freeze();
        theDI.SetDrawable(theName, DrawGeometry.Of(theGeometry));
    }

    /// <summary>DrawTrSurf::Set(name, Geom2d_Curve).</summary>
    public static void Set(DrawInterpreter theDI, string theName, Geom2d_Curve? theCurve)
    {
        if (theCurve is null)
        {
            theDI.SetDrawable(theName, null);
            return;
        }

        theCurve.Freeze();
        theDI.SetDrawable(theName, DrawGeometry.Of(theCurve));
    }

    /// <summary>DrawTrSurf::Set(name, gp_Pnt).</summary>
    public static void Set(DrawInterpreter theDI, string theName, in gp_Pnt thePoint) => theDI.SetDrawable(theName, DrawGeometry.Of(thePoint));

    /// <summary>DrawTrSurf::Set(name, gp_Pnt2d).</summary>
    public static void Set(DrawInterpreter theDI, string theName, in gp_Pnt2d thePoint) => theDI.SetDrawable(theName, DrawGeometry.Of(thePoint));

    private static object? Geometry(DrawInterpreter theDI, string theName) => (theDI.GetDrawable(theName) as DrawGeometry)?.Geometry;

    /// <summary>DrawTrSurf::Get: the curve or surface of the name, or null.</summary>
    public static Geom_Geometry? Get(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom_Geometry;

    /// <summary>DrawTrSurf::GetCurve.</summary>
    public static Geom_Curve? GetCurve(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom_Curve;

    /// <summary>DrawTrSurf::GetBezierCurve: only a DrawTrSurf_BezierCurve, whose curve is one.</summary>
    public static Geom_BezierCurve? GetBezierCurve(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom_BezierCurve;

    /// <summary>DrawTrSurf::GetBSplineCurve.</summary>
    public static Geom_BSplineCurve? GetBSplineCurve(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom_BSplineCurve;

    /// <summary>DrawTrSurf::GetCurve2d.</summary>
    public static Geom2d_Curve? GetCurve2d(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom2d_Curve;

    /// <summary>DrawTrSurf::GetBezierCurve2d.</summary>
    public static Geom2d_BezierCurve? GetBezierCurve2d(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom2d_BezierCurve;

    /// <summary>DrawTrSurf::GetBSplineCurve2d.</summary>
    public static Geom2d_BSplineCurve? GetBSplineCurve2d(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom2d_BSplineCurve;

    /// <summary>DrawTrSurf::GetSurface.</summary>
    public static Geom_Surface? GetSurface(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom_Surface;

    /// <summary>DrawTrSurf::GetBezierSurface.</summary>
    public static Geom_BezierSurface? GetBezierSurface(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom_BezierSurface;

    /// <summary>DrawTrSurf::GetBSplineSurface.</summary>
    public static Geom_BSplineSurface? GetBSplineSurface(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geom_BSplineSurface;

    /// <summary>DrawTrSurf::GetPoint: true and the point when the name is a 3D point; the point is left otherwise.</summary>
    public static bool GetPoint(DrawInterpreter theDI, string theName, ref gp_Pnt P) => BOPTestClassifyCommands.GetPoint(theDI, theName, ref P);

    /// <summary>DrawTrSurf::GetPoint2d.</summary>
    public static bool GetPoint2d(DrawInterpreter theDI, string theName, ref gp_Pnt2d P) => BOPTestClassifyCommands.GetPoint2d(theDI, theName, ref P);

    /// <summary>DrawTrSurf::GetTriangulation.</summary>
    public static Poly_Triangulation? GetTriangulation(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Poly_Triangulation;
}
