// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Occt;

namespace OcctNet.DrawLite;

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
public sealed class DrawShape(in TopoDS_Shape theShape) : DrawDrawable
{
    public TopoDS_Shape Shape { get; } = theShape;

    public override string TypeName => "DBRep_DrawableShape";

    public override void Whatis(DrawResult theResult)
    {
        if (Shape.IsNull())
        {
            return;
        }

        theResult.Append("shape ").Append(TopAbs.ShapeTypeToString(Shape.ShapeType())).Append(" ")
                 .Append(TopAbs.ShapeOrientationToString(Shape.Orientation()));
        if (Shape.Free())
        {
            theResult.Append(" Free");
        }

        if (Shape.Modified())
        {
            theResult.Append(" Modified");
        }

        if (Shape.Orientable())
        {
            theResult.Append(" Orientable");
        }

        if (Shape.Closed())
        {
            theResult.Append(" Closed");
        }

        if (Shape.Infinite())
        {
            theResult.Append(" Infinite");
        }

        if (Shape.Convex())
        {
            theResult.Append(" Convex");
        }
    }

    public override void Dump(TextWriter theStream) => BRepTools.Dump(Shape, theStream);

    /// <summary>DBRep_DrawableShape::Save: the shape set, then the shape (BRepTools::Write's text).</summary>
    public override void Save(TextWriter theStream) => BRepTools.Write(Shape, theStream);

    public override DrawDrawable Copy() => new DrawShape(Shape);
}

/// <summary>A number (Draw_Number): what <c>dset</c> and the props commands' x y z arguments create.</summary>
public sealed class DrawNumber(double theValue) : DrawDrawable
{
    public double Value { get; set; } = theValue;

    public override string TypeName => "Draw_Number";

    public override void Whatis(DrawResult theResult) => theResult.Append("numeric");

    public override void Dump(TextWriter theStream) => Standard_Stream.Write(theStream, Value);

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
        Geom_BSplineCurve => "DrawTrSurf_BSplineCurve",
        Geom_BezierCurve => "DrawTrSurf_BezierCurve",
        Geom_Curve => "DrawTrSurf_Curve",
        Geom2d_BSplineCurve => "DrawTrSurf_BSplineCurve2d",
        Geom2d_BezierCurve => "DrawTrSurf_BezierCurve2d",
        Geom2d_Curve => "DrawTrSurf_Curve2d",
        Geom_BSplineSurface => "DrawTrSurf_BSplineSurface",
        Geom_BezierSurface => "DrawTrSurf_BezierSurface",
        Geom_Surface => "DrawTrSurf_Surface",
        gp_Pnt or gp_Pnt2d => "DrawTrSurf_Point",
        Poly_Polygon3D => "DrawTrSurf_Polygon3D",
        Poly_Polygon2D => "DrawTrSurf_Polygon2D",
        _ => "DrawTrSurf_Triangulation",
    };

    public override void Whatis(DrawResult theResult) => theResult.Append(Geometry switch
    {
        Geom_Curve => " a 3d curve",
        Geom2d_Curve => "2d curve",
        Geom_Surface => "a surface",
        gp_Pnt or gp_Pnt2d => "point",
        Poly_Polygon3D => "polygon3D",
        Poly_Polygon2D => "polygon2D",
        _ => "triangulation",
    });

    public override void Dump(TextWriter theStream)
    {
        switch (Geometry)
        {
            case Geom_Curve aCurve:
                GeomTools_CurveSet.PrintCurve(aCurve, theStream);
                break;
            case Geom2d_Curve aCurve:
                GeomTools_Curve2dSet.PrintCurve2d(aCurve, theStream);
                break;
            case Geom_Surface aSurface:
                GeomTools_SurfaceSet.PrintSurface(aSurface, theStream);
                break;
            case gp_Pnt aPoint:
                theStream.Write(string.Create(CultureInfo.InvariantCulture,
                                              $"Point : {DrawFormat.Scientific(aPoint.X(), 15)}, {DrawFormat.Scientific(aPoint.Y(), 15)}, {DrawFormat.Scientific(aPoint.Z(), 15)}\n"));
                break;
            case gp_Pnt2d aPoint:
                theStream.Write(string.Create(CultureInfo.InvariantCulture,
                                              $"Point 2d : {DrawFormat.Scientific(aPoint.X(), 15)}, {DrawFormat.Scientific(aPoint.Y(), 15)}\n"));
                break;
            case Poly_Polygon3D aPolygon:
                Poly.Dump(aPolygon, theStream);
                break;
            case Poly_Polygon2D aPolygon:
                Poly.Dump(aPolygon, theStream);
                break;
            case Poly_Triangulation aTriangulation:
                Poly.Dump(aTriangulation, theStream);
                break;
        }
    }

    public override void Save(TextWriter theStream)
    {
        switch (Geometry)
        {
            case Geom_Curve aCurve:
                GeomTools_CurveSet.PrintCurve(aCurve, theStream, true);
                break;
            case Geom2d_Curve aCurve:
                GeomTools_Curve2dSet.PrintCurve2d(aCurve, theStream, true);
                break;
            case Geom_Surface aSurface:
                GeomTools_SurfaceSet.PrintSurface(aSurface, theStream, true);
                break;
            case gp_Pnt aPoint:
                theStream.Write($"1 {DrawFormat.Scientific(aPoint.X(), 15)} {DrawFormat.Scientific(aPoint.Y(), 15)} {DrawFormat.Scientific(aPoint.Z(), 15)}\n");
                break;
            case gp_Pnt2d aPoint:
                theStream.Write($"0 {DrawFormat.Scientific(aPoint.X(), 15)} {DrawFormat.Scientific(aPoint.Y(), 15)}\n");
                break;
            case Poly_Polygon3D aPolygon:
                Poly.Write(aPolygon, theStream);
                break;
            case Poly_Polygon2D aPolygon:
                Poly.Write(aPolygon, theStream);
                break;
            case Poly_Triangulation aTriangulation:
                Poly.Write(aTriangulation, theStream);
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
                aGeometry = GeomTools_CurveSet.ReadCurve(theStream);
                break;
            case "DrawTrSurf_Curve2d":
            case "DrawTrSurf_BSplineCurve2d":
            case "DrawTrSurf_BezierCurve2d":
                aGeometry = GeomTools_Curve2dSet.ReadCurve2d(theStream);
                break;
            case "DrawTrSurf_Surface":
            case "DrawTrSurf_BSplineSurface":
            case "DrawTrSurf_BezierSurface":
                aGeometry = GeomTools_SurfaceSet.ReadSurface(theStream);
                break;
            case "DrawTrSurf_Point":
            {
                int anIs3d = 0;
                Standard_Stream.Read(theStream, ref anIs3d);
                // C++'s >> into a double, as GeomTools::GetReal reads
                GeomTools.GetReal(theStream, out double x);
                GeomTools.GetReal(theStream, out double y);
                if (anIs3d != 0)
                {
                    GeomTools.GetReal(theStream, out double z);
                    aGeometry = new gp_Pnt(x, y, z);
                }
                else
                {
                    aGeometry = new gp_Pnt2d(x, y);
                }

                break;
            }

            case "DrawTrSurf_Polygon3D":
                aGeometry = Poly.ReadPolygon3D(theStream);
                break;
            case "DrawTrSurf_Polygon2D":
                aGeometry = Poly.ReadPolygon2D(theStream);
                break;
            case "DrawTrSurf_Triangulation":
                aGeometry = Poly.ReadTriangulation(theStream);
                break;
            default:
                return null;
        }

        return aGeometry is null ? null : new DrawGeometry(aGeometry);
    }
}
