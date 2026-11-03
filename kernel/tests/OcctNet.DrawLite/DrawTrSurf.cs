// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// DrawTrSurf's Set and Get (DrawTrSurf.cxx): geometry bound to Draw variables as <see cref="DrawGeometry"/>
/// drawables, and read back by the kind of drawable the variable holds.
/// </summary>
/// <remarks>
/// As shapes are frozen when bound (decision 272), geometry is frozen when <see cref="Set(DrawInterpreter, string, Geometry3D?)"/>
/// binds it: a drawable's geometry is shared by the shapes made from it and by <c>copy</c>.
/// </remarks>
internal static class DrawTrSurf
{
    /// <summary>DrawTrSurf::Set(name, Geom_Geometry): a curve or surface drawable; a null geometry unbinds the name.</summary>
    public static void Set(DrawInterpreter theDI, string theName, Geometry3D? theGeometry)
    {
        if (theGeometry is not (Curve3D or Surface))
        {
            // OCCT binds a null drawable: Draw::Set with null removes the variable's drawable
            theDI.SetDrawable(theName, null);
            return;
        }

        theGeometry.Freeze();
        theDI.SetDrawable(theName, DrawGeometry.Of(theGeometry));
    }

    /// <summary>DrawTrSurf::Set(name, Geom2d_Curve).</summary>
    public static void Set(DrawInterpreter theDI, string theName, Curve2D? theCurve)
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
    public static void Set(DrawInterpreter theDI, string theName, in Point3D thePoint) => theDI.SetDrawable(theName, DrawGeometry.Of(thePoint));

    /// <summary>DrawTrSurf::Set(name, gp_Pnt2d).</summary>
    public static void Set(DrawInterpreter theDI, string theName, in Point2D thePoint) => theDI.SetDrawable(theName, DrawGeometry.Of(thePoint));

    // a batten (DrawFairCurve_Batten derives from DrawTrSurf_BSplineCurve2d) and a qualified curve
    // (GeometryTest_DrawableQualifiedCurve2d, from DrawTrSurf_Curve2d) are their curves
    private static object? Geometry(DrawInterpreter theDI, string theName) => theDI.GetDrawable(theName) switch
    {
        DrawGeometry aGeometry => aGeometry.Geometry,
        DrawFairCurve aBatten => aBatten.Curve,
        DrawQualifiedCurve2d aQualified => aQualified.Curve,
        _ => null,
    };

    /// <summary>DrawTrSurf::Get: the curve or surface of the name, or null.</summary>
    public static Geometry3D? Get(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Geometry3D;

    /// <summary>DrawTrSurf::GetCurve.</summary>
    public static Curve3D? GetCurve(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Curve3D;

    /// <summary>DrawTrSurf::GetBezierCurve: only a DrawTrSurf_BezierCurve, whose curve is one.</summary>
    public static BezierCurve3D? GetBezierCurve(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as BezierCurve3D;

    /// <summary>DrawTrSurf::GetBSplineCurve.</summary>
    public static BSplineCurve3D? GetBSplineCurve(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as BSplineCurve3D;

    /// <summary>DrawTrSurf::GetCurve2d.</summary>
    public static Curve2D? GetCurve2d(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Curve2D;

    /// <summary>DrawTrSurf::GetBezierCurve2d.</summary>
    public static BezierCurve2D? GetBezierCurve2d(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as BezierCurve2D;

    /// <summary>DrawTrSurf::GetBSplineCurve2d.</summary>
    public static BSplineCurve2D? GetBSplineCurve2d(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as BSplineCurve2D;

    /// <summary>DrawTrSurf::GetSurface.</summary>
    public static Surface? GetSurface(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Surface;

    /// <summary>DrawTrSurf::GetBezierSurface.</summary>
    public static BezierSurface? GetBezierSurface(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as BezierSurface;

    /// <summary>DrawTrSurf::GetBSplineSurface.</summary>
    public static BSplineSurface? GetBSplineSurface(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as BSplineSurface;

    /// <summary>DrawTrSurf::GetPoint: true and the point when the name is a 3D point; the point is left otherwise.</summary>
    public static bool GetPoint(DrawInterpreter theDI, string theName, ref Point3D P) => BOPTestClassifyCommands.GetPoint(theDI, theName, ref P);

    /// <summary>DrawTrSurf::GetPoint2d.</summary>
    public static bool GetPoint2d(DrawInterpreter theDI, string theName, ref Point2D P) => BOPTestClassifyCommands.GetPoint2d(theDI, theName, ref P);

    /// <summary>DrawTrSurf::GetTriangulation.</summary>
    public static Triangulation? GetTriangulation(DrawInterpreter theDI, string theName) => Geometry(theDI, theName) as Triangulation;
}
