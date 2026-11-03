// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// A Draw variable (OCCT's Draw_Drawable3D): what a Tcl variable holds after <c>restore</c>, <c>explode</c> and the
/// other commands that create Draw values. Draw-lite has no viewer, so a drawable is only its value, its name, its
/// protection and the textual methods Draw's commands use (Whatis, Dump, Save, Copy).
/// </summary>
public abstract class DrawDrawable
{
    /// <summary>The name given to Draw::Set (the variable's value), as Draw_Drawable3D::Name.</summary>
    public string Name { get; internal set; } = "";

    /// <summary>A protected variable can't be written or replaced (pi, pinf, minf).</summary>
    public bool Protected { get; set; }

    /// <summary>The class name Draw writes as the first line of a saved file (DynamicType()->Name()).</summary>
    public abstract string TypeName { get; }

    /// <summary>What <c>dtyp</c> prints, as Draw_Drawable3D::Whatis.</summary>
    public virtual void Whatis(DrawResult theResult) => theResult.Append("drawable 3d");

    /// <summary>What <c>dump</c> prints, as Draw_Drawable3D::Dump.</summary>
    public abstract void Dump(TextWriter theStream);

    /// <summary>The value written by <c>bsave</c> after the type line, as Draw_Drawable3D::Save.</summary>
    public abstract void Save(TextWriter theStream);

    /// <summary>A copy for <c>copy</c>, as Draw_Drawable3D::Copy (values are immutable, so it may share them).</summary>
    public abstract DrawDrawable Copy();
}

/// <summary>A shape (DBRep_DrawableShape). The shape is frozen when it is stored (decision 272).</summary>
public sealed class DrawShape(in Shape theShape) : DrawDrawable
{
    /// <summary>The shape; rebound in place when a command meshes or cleans a shape sharing its sub-shapes (DrawShapeRebind).</summary>
    public Shape Shape { get; internal set; } = theShape;

    public override string TypeName => "DBRep_DrawableShape";

    public override void Whatis(DrawResult theResult)
    {
        if (Shape.IsNull)
        {
            return;
        }

        theResult.Append("shape ").Append(global::Kerf.Engine.Topology.TopAbs.ShapeTypeToString(Shape.Type)).Append(" ")
                 .Append(global::Kerf.Engine.Topology.TopAbs.ShapeOrientationToString(Shape.Orientation));
        if (Shape.Free())
        {
            theResult.Append(" Free");
        }

        if (Shape.Modified())
        {
            theResult.Append(" Modified");
        }

        if (Shape.IsOrientable)
        {
            theResult.Append(" Orientable");
        }

        if (Shape.IsClosed)
        {
            theResult.Append(" Closed");
        }

        if (Shape.IsInfinite)
        {
            theResult.Append(" Infinite");
        }

        if (Shape.IsConvex)
        {
            theResult.Append(" Convex");
        }
    }

    public override void Dump(TextWriter theStream) => BrepTools.Dump(Shape, theStream);

    /// <summary>DBRep_DrawableShape::Save: the shape set, then the shape (BRepTools::Write's text).</summary>
    public override void Save(TextWriter theStream) => BrepTools.Write(Shape, theStream);

    public override DrawDrawable Copy() => new DrawShape(Shape);
}

/// <summary>A number (Draw_Number): what <c>dset</c> and the props commands' x y z arguments create.</summary>
public sealed class DrawNumber(double theValue) : DrawDrawable
{
    public double Value { get; set; } = theValue;

    public override string TypeName => "Draw_Number";

    public override void Whatis(DrawResult theResult) => theResult.Append("numeric");

    public override void Dump(TextWriter theStream) => StreamUtilities.Write(theStream, Value);

    /// <summary>Draw_Number::Save: scientific, precision 15, width 30.</summary>
    public override void Save(TextWriter theStream) => theStream.Write(DrawFormat.Scientific(Value, 15).PadLeft(30) + "\n");

    public override DrawDrawable Copy() => new DrawNumber(Value);
}

/// <summary>
/// Geometry read by <c>restore</c> from a DrawTrSurf file (DrawTrSurf_Curve, _Curve2d, _Surface, their B-spline and
/// Bezier subclasses, _Point, _Polygon3D, _Polygon2D, _Triangulation).
/// </summary>
public sealed class DrawGeometry : DrawDrawable
{
    private DrawGeometry(object theGeometry) => Geometry = theGeometry;

    /// <summary>A Geom_Curve, Geom2d_Curve, Geom_Surface, gp_Pnt, gp_Pnt2d, Poly_Polygon3D, Poly_Polygon2D or Poly_Triangulation.</summary>
    public object Geometry { get; }

    public static DrawGeometry Of(object theGeometry) => new(theGeometry);

    public override string TypeName => Geometry switch
    {
        BSplineCurve3D => "DrawTrSurf_BSplineCurve",
        BezierCurve3D => "DrawTrSurf_BezierCurve",
        Curve3D => "DrawTrSurf_Curve",
        BSplineCurve2D => "DrawTrSurf_BSplineCurve2d",
        BezierCurve2D => "DrawTrSurf_BezierCurve2d",
        Curve2D => "DrawTrSurf_Curve2d",
        BSplineSurface => "DrawTrSurf_BSplineSurface",
        BezierSurface => "DrawTrSurf_BezierSurface",
        Surface => "DrawTrSurf_Surface",
        Point3D or Point2D => "DrawTrSurf_Point",
        global::Kerf.Engine.Meshing.Polygon3D => "DrawTrSurf_Polygon3D",
        global::Kerf.Engine.Meshing.Polygon2D => "DrawTrSurf_Polygon2D",
        _ => "DrawTrSurf_Triangulation",
    };

    public override void Whatis(DrawResult theResult) => theResult.Append(Geometry switch
    {
        Curve3D => " a 3d curve",
        Curve2D => "2d curve",
        Surface => "a surface",
        Point3D or Point2D => "point",
        global::Kerf.Engine.Meshing.Polygon3D => "polygon3D",
        global::Kerf.Engine.Meshing.Polygon2D => "polygon2D",
        _ => "triangulation",
    });

    public override void Dump(TextWriter theStream)
    {
        switch (Geometry)
        {
            case Curve3D aCurve:
                global::Kerf.Engine.Exchange.Brep.CurveSet.PrintCurve(aCurve, theStream);
                break;
            case Curve2D aCurve:
                global::Kerf.Engine.Exchange.Brep.Curve2dSet.PrintCurve2d(aCurve, theStream);
                break;
            case Surface aSurface:
                global::Kerf.Engine.Exchange.Brep.SurfaceSet.PrintSurface(aSurface, theStream);
                break;
            case Point3D aPoint:
                theStream.Write(string.Create(CultureInfo.InvariantCulture,
                                              $"Point : {DrawFormat.Scientific(aPoint.X, 15)}, {DrawFormat.Scientific(aPoint.Y, 15)}, {DrawFormat.Scientific(aPoint.Z, 15)}\n"));
                break;
            case Point2D aPoint:
                theStream.Write(string.Create(CultureInfo.InvariantCulture,
                                              $"Point 2d : {DrawFormat.Scientific(aPoint.X, 15)}, {DrawFormat.Scientific(aPoint.Y, 15)}\n"));
                break;
            case global::Kerf.Engine.Meshing.Polygon3D aPolygon:
                global::Kerf.Engine.Meshing.Poly.Dump(aPolygon, theStream);
                break;
            case global::Kerf.Engine.Meshing.Polygon2D aPolygon:
                global::Kerf.Engine.Meshing.Poly.Dump(aPolygon, theStream);
                break;
            case global::Kerf.Engine.Meshing.Triangulation aTriangulation:
                global::Kerf.Engine.Meshing.Poly.Dump(aTriangulation, theStream);
                break;
        }
    }

    public override void Save(TextWriter theStream)
    {
        switch (Geometry)
        {
            case Curve3D aCurve:
                global::Kerf.Engine.Exchange.Brep.CurveSet.PrintCurve(aCurve, theStream, true);
                break;
            case Curve2D aCurve:
                global::Kerf.Engine.Exchange.Brep.Curve2dSet.PrintCurve2d(aCurve, theStream, true);
                break;
            case Surface aSurface:
                global::Kerf.Engine.Exchange.Brep.SurfaceSet.PrintSurface(aSurface, theStream, true);
                break;
            case Point3D aPoint:
                theStream.Write($"1 {DrawFormat.Scientific(aPoint.X, 15)} {DrawFormat.Scientific(aPoint.Y, 15)} {DrawFormat.Scientific(aPoint.Z, 15)}\n");
                break;
            case Point2D aPoint:
                theStream.Write($"0 {DrawFormat.Scientific(aPoint.X, 15)} {DrawFormat.Scientific(aPoint.Y, 15)}\n");
                break;
            case global::Kerf.Engine.Meshing.Polygon3D aPolygon:
                global::Kerf.Engine.Meshing.Poly.Write(aPolygon, theStream);
                break;
            case global::Kerf.Engine.Meshing.Polygon2D aPolygon:
                global::Kerf.Engine.Meshing.Poly.Write(aPolygon, theStream);
                break;
            case global::Kerf.Engine.Meshing.Triangulation aTriangulation:
                global::Kerf.Engine.Meshing.Poly.Write(aTriangulation, theStream);
                break;
        }
    }

    public override DrawDrawable Copy() => new DrawGeometry(Geometry);

    /// <summary>
    /// The factories of Draw_Drawable3D::Restore for DrawTrSurf's types (DrawTrSurf::BasicCommands registers them),
    /// over GeomTools' and Poly's readers. Null when the type isn't one of them.
    /// </summary>
    internal static DrawDrawable? Restore(string theType, TextReader theStream)
    {
        object? aGeometry;
        switch (theType)
        {
            case "DrawTrSurf_Curve":
            case "DrawTrSurf_BSplineCurve":
            case "DrawTrSurf_BezierCurve":
                aGeometry = global::Kerf.Engine.Exchange.Brep.CurveSet.ReadCurve(theStream);
                break;
            case "DrawTrSurf_Curve2d":
            case "DrawTrSurf_BSplineCurve2d":
            case "DrawTrSurf_BezierCurve2d":
                aGeometry = global::Kerf.Engine.Exchange.Brep.Curve2dSet.ReadCurve2d(theStream);
                break;
            case "DrawTrSurf_Surface":
            case "DrawTrSurf_BSplineSurface":
            case "DrawTrSurf_BezierSurface":
                aGeometry = global::Kerf.Engine.Exchange.Brep.SurfaceSet.ReadSurface(theStream);
                break;
            case "DrawTrSurf_Point":
            {
                int anIs3d = 0;
                StreamUtilities.Read(theStream, ref anIs3d);
                // C++'s >> into a double, as GeomTools::GetReal reads
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(theStream, out double x);
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(theStream, out double y);
                if (anIs3d != 0)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(theStream, out double z);
                    aGeometry = new Point3D(x, y, z);
                }
                else
                {
                    aGeometry = new Point2D(x, y);
                }

                break;
            }

            case "DrawTrSurf_Polygon3D":
                aGeometry = global::Kerf.Engine.Meshing.Poly.ReadPolygon3D(theStream);
                break;
            case "DrawTrSurf_Polygon2D":
                aGeometry = global::Kerf.Engine.Meshing.Poly.ReadPolygon2D(theStream);
                break;
            case "DrawTrSurf_Triangulation":
                aGeometry = global::Kerf.Engine.Meshing.Poly.ReadTriangulation(theStream);
                break;
            default:
                return null;
        }

        return aGeometry is null ? null : new DrawGeometry(aGeometry);
    }
}
