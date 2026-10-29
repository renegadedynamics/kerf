// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ShapeSet.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// A set of BRep shapes, written and read in OCCT's text format (a <c>.brep</c> file): the shape set of
/// <see cref="TopTools_ShapeSet"/> with the geometry of vertices, edges and faces (curves, 2D curves and surfaces
/// through the GeomTools sets), their polygons and triangulations.
/// </summary>
/// <remarks>
/// <para>
/// Triangulations and polygons are written when the set is made with triangles (the default), and always for a
/// face that has a triangulation and no surface. Normals are written (format version 3) when the set is made with
/// normals, and always for such a face.
/// </para>
/// <para>
/// Reading builds the shapes with <c>BRep_Builder</c>, which freezes the geometry, triangulations and polygons it
/// attaches; the set freezes the shapes once they are all read (<see cref="TopTools_ShapeSet"/>).
/// </para>
/// </remarks>
public class BRepTools_ShapeSet : TopTools_ShapeSet
{
    private readonly BRep_Builder myBuilder;
    private readonly GeomTools_SurfaceSet mySurfaces = new();
    private readonly GeomTools_CurveSet myCurves = new();
    private readonly GeomTools_Curve2dSet myCurves2d = new();
    private readonly NCollection_IndexedMap<object> myPolygons2D = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<object> myPolygons3D = new(ReferenceEqualityComparer.Instance);
    // with the flag that says whether to write the triangulation's normals
    private readonly NCollection_IndexedDataMap<Poly_Triangulation, bool> myTriangulations = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<object> myNodes = new(ReferenceEqualityComparer.Instance);
    private bool myWithTriangles;
    private bool myWithNormals;

    /// <summary>Builds an empty set, which writes triangulation data if asked, and normals if asked.</summary>
    public BRepTools_ShapeSet(bool theWithTriangles = true, bool theWithNormals = false)
    {
        myBuilder = new BRep_Builder();
        myWithTriangles = theWithTriangles;
        myWithNormals = theWithNormals;
    }

    /// <summary>Builds an empty set reading with the builder.</summary>
    internal BRepTools_ShapeSet(BRep_Builder theBuilder, bool theWithTriangles = true, bool theWithNormals = false)
    {
        myBuilder = theBuilder;
        myWithTriangles = theWithTriangles;
        myWithNormals = theWithNormals;
    }

    /// <summary>True if shapes are written with their triangulations and polygons.</summary>
    public bool IsWithTriangles() => myWithTriangles;

    /// <summary>True if triangulations are written with their normals.</summary>
    public bool IsWithNormals() => myWithNormals;

    /// <summary>
    /// Sets whether shapes are written with their triangulations and polygons (always, for a face that has a
    /// triangulation and no surface).
    /// </summary>
    public void SetWithTriangles(bool theWithTriangles)
    {
        myWithTriangles = theWithTriangles;
    }

    /// <summary>
    /// Sets whether triangulations are written with their normals (always, for a face that has a triangulation and no
    /// surface).
    /// </summary>
    public void SetWithNormals(bool theWithNormals)
    {
        myWithNormals = theWithNormals;
    }

    /// <summary>Clears the content of the set.</summary>
    public override void Clear()
    {
        mySurfaces.Clear();
        myCurves.Clear();
        myCurves2d.Clear();
        myPolygons3D.Clear();
        myPolygons2D.Clear();
        myNodes.Clear();
        myTriangulations.Clear();
        base.Clear();
    }

    private static void Out(TextWriter OS, int theValue) => Standard_Stream.Write(OS, theValue);

    private static void Out(TextWriter OS, double theValue) => Standard_Stream.Write(OS, theValue);

    // OS << std::setw(theWidth) << theValue
    private static void Setw(TextWriter OS, int theWidth, int theValue)
        => OS.Write(Standard_Stream.Setw(theWidth, theValue.ToString(CultureInfo.InvariantCulture)));

    private static void Setw(TextWriter OS, int theWidth, double theValue)
        => OS.Write(Standard_Stream.Setw(theWidth, Standard_Stream.FormatReal(theValue, Standard_Stream.Precision(OS))));

    /// <summary>Stores the geometry of the shape: its curves, surfaces, locations, polygons and triangulations.</summary>
    public override void AddGeometry(in TopoDS_Shape S)
    {
        // Add the geometry

        if (S.ShapeType() == TopAbs_VERTEX)
        {
            BRep_TVertex TV = (BRep_TVertex)S.TShape()!;
            NCollection_List<BRep_PointRepresentation>.Iterator itrp = new(TV.Points());

            while (itrp.More())
            {
                BRep_PointRepresentation PR = itrp.Value();

                if (PR.IsPointOnCurve())
                {
                    myCurves.Add(PR.Curve());
                }

                else if (PR.IsPointOnCurveOnSurface())
                {
                    myCurves2d.Add(PR.PCurve());
                    mySurfaces.Add(PR.Surface());
                }

                else if (PR.IsPointOnSurface())
                {
                    mySurfaces.Add(PR.Surface());
                }

                ChangeLocations().Add(PR.Location());
                itrp.Next();
            }
        }
        else if (S.ShapeType() == TopAbs_EDGE)
        {
            // Add the curve geometry
            BRep_TEdge TE = (BRep_TEdge)S.TShape()!;
            NCollection_List<BRep_CurveRepresentation>.Iterator itrc = new(TE.Curves());

            while (itrc.More())
            {
                BRep_CurveRepresentation CR = itrc.Value();
                if (CR.IsCurve3D())
                {
                    if (CR.Curve3D() is not null)
                    {
                        myCurves.Add(CR.Curve3D());
                        ChangeLocations().Add(CR.Location());
                    }
                }
                else if (CR.IsCurveOnSurface())
                {
                    mySurfaces.Add(CR.Surface());
                    myCurves2d.Add(CR.PCurve());
                    ChangeLocations().Add(CR.Location());
                    if (CR.IsCurveOnClosedSurface())
                    {
                        myCurves2d.Add(CR.PCurve2());
                    }
                }
                else if (CR.IsRegularity())
                {
                    mySurfaces.Add(CR.Surface());
                    ChangeLocations().Add(CR.Location());
                    mySurfaces.Add(CR.Surface2());
                    ChangeLocations().Add(CR.Location2());
                }
                else if (myWithTriangles)
                {
                    // for XML Persistence
                    if (CR.IsPolygon3D())
                    {
                        if (CR.Polygon3D() is not null)
                        {
                            myPolygons3D.Add(CR.Polygon3D()!);
                            ChangeLocations().Add(CR.Location());
                        }
                    }
                    else if (CR.IsPolygonOnTriangulation())
                    {
                        // NCollection_IndexedDataMap::Add() function use is correct because
                        // Bin(Brep)Tools_ShapeSet::AddGeometry() is called from Bin(Brep)Tools_ShapeSet::Add()
                        // that processes shapes recursively from complex to elementary ones.
                        // As a result, the TopAbs_FACE's will be processed earlier than the TopAbs_EDGE's.
                        // OCCT.NET: OCCT adds a null triangulation as a key, and crashes writing it
                        myTriangulations.Add(NotNull(CR.Triangulation()), false); // edge triangulation does not need normals
                        myNodes.Add(NotNull(CR.PolygonOnTriangulation()));
                        ChangeLocations().Add(CR.Location());
                        if (CR.IsPolygonOnClosedTriangulation())
                        {
                            myNodes.Add(NotNull(CR.PolygonOnTriangulation2()));
                        }
                    }
                    else if (CR.IsPolygonOnSurface())
                    {
                        mySurfaces.Add(CR.Surface());
                        // the 2D polygons aren't written; OCCT's map holds a null one as a key like any other
                        myPolygons2D.Add((object?)CR.Polygon() ?? GeomTools_CurveSet.theNullKey);
                        ChangeLocations().Add(CR.Location());
                        if (CR.IsPolygonOnClosedSurface())
                        {
                            myPolygons2D.Add((object?)CR.Polygon2() ?? GeomTools_CurveSet.theNullKey);
                        }
                    }
                }

                itrc.Next();
            }
        }

        else if (S.ShapeType() == TopAbs_FACE)
        {
            // Add the surface geometry
            bool needNormals = myWithNormals;
            BRep_TFace TF = (BRep_TFace)S.TShape()!;
            if (TF.Surface() is not null)
            {
                mySurfaces.Add(TF.Surface());
            }
            else
            {
                needNormals = true;
            }

            if (myWithTriangles || TF.Surface() is null)
            {
                // for XML Persistence
                Poly_Triangulation? Tr = TF.Triangulation();
                if (Tr is not null)
                {
                    myTriangulations.Add(Tr, needNormals);
                }
            }

            ChangeLocations().Add(TF.Location());
        }
    }

    // OCCT keeps null handles in its maps of polygons and triangulations, and crashes writing them; the port can't
    // write them either
    private static T NotNull<T>(T? theObject) where T : class
        => theObject ?? throw new Standard_NullObject("BRepTools_ShapeSet: a null polygon or triangulation");

    /// <summary>Dumps the geometry of the set in a readable form.</summary>
    public override void DumpGeometry(TextWriter OS)
    {
        myCurves2d.Dump(OS);
        myCurves.Dump(OS);
        DumpPolygon3D(OS);
        DumpPolygonOnTriangulation(OS);
        mySurfaces.Dump(OS);
        DumpTriangulation(OS);
    }

    /// <summary>
    /// Writes the geometry of the set: 2D curves, curves, 3D polygons, polygons on triangulations, surfaces and
    /// triangulations.
    /// </summary>
    public override void WriteGeometry(TextWriter OS, Message_ProgressRange theProgress = default)
    {
        // Make nested progress scope for processing geometry
        using var aPS = new Message_ProgressScope(theProgress, "Geometry", 100);

        myCurves2d.Write(OS, aPS.Next(20));
        if (aPS.UserBreak())
        {
            return;
        }

        myCurves.Write(OS, aPS.Next(20));
        if (aPS.UserBreak())
        {
            return;
        }

        WritePolygon3D(OS, true, aPS.Next(10));
        if (aPS.UserBreak())
        {
            return;
        }

        WritePolygonOnTriangulation(OS, true, aPS.Next(10));
        if (aPS.UserBreak())
        {
            return;
        }

        mySurfaces.Write(OS, aPS.Next(20));
        if (aPS.UserBreak())
        {
            return;
        }

        WriteTriangulation(OS, true, aPS.Next(20));
    }

    /// <summary>Reads the geometry of the set, written by <see cref="WriteGeometry(TextWriter, Message_ProgressRange)"/>.</summary>
    public override void ReadGeometry(TextReader IS, Message_ProgressRange theProgress = default)
    {
        // Make nested progress scope for processing geometry
        using var aPS = new Message_ProgressScope(theProgress, "Geometry", 100);

        myCurves2d.Read(IS, aPS.Next(20));
        if (aPS.UserBreak())
        {
            return;
        }

        myCurves.Read(IS, aPS.Next(20));
        if (aPS.UserBreak())
        {
            return;
        }

        ReadPolygon3D(IS, aPS.Next(15));
        if (aPS.UserBreak())
        {
            return;
        }

        ReadPolygonOnTriangulation(IS, aPS.Next(15));
        if (aPS.UserBreak())
        {
            return;
        }

        mySurfaces.Read(IS, aPS.Next(15));
        if (aPS.UserBreak())
        {
            return;
        }

        ReadTriangulation(IS, aPS.Next(15));
    }

    private static void PrintRegularity(GeomAbs_Shape C, TextWriter OS)
    {
        switch (C)
        {
            case GeomAbs_C0:
                OS.Write("C0");
                break;

            case GeomAbs_G1:
                OS.Write("G1");
                break;

            case GeomAbs_C1:
                OS.Write("C1");
                break;

            case GeomAbs_G2:
                OS.Write("G2");
                break;

            case GeomAbs_C2:
                OS.Write("C2");
                break;

            case GeomAbs_C3:
                OS.Write("C3");
                break;

            case GeomAbs_CN:
                OS.Write("CN");
                break;
        }
    }

    /// <summary>Dumps the geometry of the shape in a readable form.</summary>
    public override void DumpGeometry(in TopoDS_Shape S, TextWriter OS)
    {
        // Dump the geometry

        if (S.ShapeType() == TopAbs_VERTEX)
        {
            // Dump the point geometry
            TopoDS_Vertex V = TopoDS.Vertex(S);
            OS.Write("    Tolerance : ");
            Out(OS, BRep_Tool.Tolerance(V));
            OS.Write("\n");
            gp_Pnt p = BRep_Tool.Pnt(V);
            OS.Write("    - Point 3D : ");
            Out(OS, p.X());
            OS.Write(", ");
            Out(OS, p.Y());
            OS.Write(", ");
            Out(OS, p.Z());
            OS.Write("\n");

            BRep_TVertex TV = (BRep_TVertex)S.TShape()!;
            NCollection_List<BRep_PointRepresentation>.Iterator itrp = new(TV.Points());

            while (itrp.More())
            {
                BRep_PointRepresentation PR = itrp.Value();

                OS.Write("    - Parameter : ");
                Out(OS, PR.Parameter());
                if (PR.IsPointOnCurve())
                {
                    OS.Write(" on curve ");
                    Out(OS, myCurves.Index(PR.Curve()));
                }

                else if (PR.IsPointOnCurveOnSurface())
                {
                    OS.Write(" on pcurve ");
                    Out(OS, myCurves2d.Index(PR.PCurve()));
                    OS.Write(" on surface ");
                    Out(OS, mySurfaces.Index(PR.Surface()));
                }

                else if (PR.IsPointOnSurface())
                {
                    OS.Write(", ");
                    Out(OS, PR.Parameter2());
                    OS.Write(" on surface ");
                    Out(OS, mySurfaces.Index(PR.Surface()));
                }

                if (!PR.Location().IsIdentity())
                {
                    OS.Write(" location ");
                    Out(OS, Locations().Index(PR.Location()));
                }

                OS.Write("\n");

                itrp.Next();
            }
        }

        else if (S.ShapeType() == TopAbs_EDGE)
        {
            BRep_TEdge TE = (BRep_TEdge)S.TShape()!;
            gp_Pnt2d Pf, Pl;

            // Dump the curve geometry
            OS.Write("    Tolerance : ");
            Out(OS, TE.Tolerance());
            OS.Write("\n");
            if (TE.SameParameter())
            {
                OS.Write("     same parametrisation of curves\n");
            }

            if (TE.SameRange())
            {
                OS.Write("     same range on curves\n");
            }

            if (TE.Degenerated())
            {
                OS.Write("     degenerated\n");
            }

            double first, last;
            NCollection_List<BRep_CurveRepresentation>.Iterator itrc = new(TE.Curves());
            while (itrc.More())
            {
                BRep_CurveRepresentation CR = itrc.Value();
                if (CR.IsCurve3D())
                {
                    BRep_GCurve GC = (BRep_GCurve)itrc.Value();
                    GC.Range(out first, out last);
                    if (CR.Curve3D() is not null)
                    {
                        OS.Write("    - Curve 3D : ");
                        Out(OS, myCurves.Index(CR.Curve3D()));
                        if (!CR.Location().IsIdentity())
                        {
                            OS.Write(" location ");
                            Out(OS, Locations().Index(CR.Location()));
                        }

                        OS.Write(", range : ");
                        Out(OS, first);
                        OS.Write(" ");
                        Out(OS, last);
                        OS.Write("\n");
                    }
                }
                else if (CR.IsCurveOnSurface())
                {
                    BRep_GCurve GC = (BRep_GCurve)itrc.Value();
                    GC.Range(out first, out last);
                    OS.Write("    - PCurve : ");
                    Out(OS, myCurves2d.Index(CR.PCurve()));
                    if (CR.IsCurveOnClosedSurface())
                    {
                        OS.Write(", ");
                        Out(OS, myCurves2d.Index(CR.PCurve2()));
                        OS.Write(" (");
                        PrintRegularity(CR.Continuity(), OS);
                        OS.Write(")");
                    }

                    OS.Write(" on surface ");
                    Out(OS, mySurfaces.Index(CR.Surface()));
                    if (!CR.Location().IsIdentity())
                    {
                        OS.Write(" location ");
                        Out(OS, Locations().Index(CR.Location()));
                    }

                    OS.Write(", range : ");
                    Out(OS, first);
                    OS.Write(" ");
                    Out(OS, last);
                    OS.Write("\n");

                    BRep_CurveOnSurface COS = (BRep_CurveOnSurface)CR;
                    COS.UVPoints(out Pf, out Pl);
                    OS.Write("  UV Points : ");
                    Out(OS, Pf.X());
                    OS.Write(", ");
                    Out(OS, Pf.Y());
                    OS.Write(" ");
                    Out(OS, Pl.X());
                    OS.Write(", ");
                    Out(OS, Pl.Y());
                    OS.Write("\n");
                    if (CR.IsCurveOnClosedSurface())
                    {
                        BRep_CurveOnClosedSurface COCS = (BRep_CurveOnClosedSurface)CR;
                        COCS.UVPoints2(out Pf, out Pl);
                        OS.Write("  UV Points : ");
                        Out(OS, Pf.X());
                        OS.Write(", ");
                        Out(OS, Pf.Y());
                        OS.Write(" ");
                        Out(OS, Pl.X());
                        OS.Write(", ");
                        Out(OS, Pl.Y());
                        OS.Write("\n");
                    }
                }
                else if (CR.IsRegularity())
                {
                    OS.Write("    - Regularity ");
                    PrintRegularity(CR.Continuity(), OS);
                    OS.Write("   on surfaces : ");
                    Out(OS, mySurfaces.Index(CR.Surface()));
                    if (!CR.Location().IsIdentity())
                    {
                        OS.Write(" location ");
                        Out(OS, Locations().Index(CR.Location()));
                    }

                    OS.Write(", ");
                    Out(OS, mySurfaces.Index(CR.Surface2()));
                    if (!CR.Location2().IsIdentity())
                    {
                        OS.Write(" location ");
                        Out(OS, Locations().Index(CR.Location2()));
                    }

                    OS.Write("\n");
                }
                else if (CR.IsPolygon3D())
                {
                    BRep_Polygon3D GC = (BRep_Polygon3D)itrc.Value();
                    if (GC.Polygon3D() is not null)
                    {
                        OS.Write("    - Polygon 3D : ");
                        Out(OS, myPolygons3D.FindIndex(CR.Polygon3D()!));
                        if (!CR.Location().IsIdentity())
                        {
                            OS.Write(" location ");
                            Out(OS, Locations().Index(CR.Location()));
                        }
                    }
                }
                else if (CR.IsPolygonOnTriangulation())
                {
                    OS.Write("    - PolygonOnTriangulation ");
                    Out(OS, FindIndex(myNodes, CR.PolygonOnTriangulation()));
                    if (CR.IsPolygonOnClosedTriangulation())
                    {
                        OS.Write(" ");
                        Out(OS, FindIndex(myNodes, CR.PolygonOnTriangulation2()));
                    }

                    OS.Write(" on triangulation ");
                    Out(OS, CR.Triangulation() is { } aTr ? myTriangulations.FindIndex(aTr) : 0);
                    if (!CR.Location().IsIdentity())
                    {
                        OS.Write(" location ");
                        Out(OS, Locations().Index(CR.Location()));
                    }

                    OS.Write("\n");
                }

                itrc.Next();
            }
        }

        else if (S.ShapeType() == TopAbs_FACE)
        {
            TopoDS_Face F = TopoDS.Face(S);
            if (BRep_Tool.NaturalRestriction(F))
            {
                OS.Write("NaturalRestriction\n");
            }

            // Dump the surface geometry
            BRep_TFace TF = (BRep_TFace)S.TShape()!;
            if (TF.Surface() is not null)
            {
                OS.Write("    Tolerance : ");
                Out(OS, TF.Tolerance());
                OS.Write("\n");
                OS.Write("    - Surface : ");
                Out(OS, mySurfaces.Index(TF.Surface()));
                if (!S.Location().IsIdentity())
                {
                    OS.Write(" location ");
                    Out(OS, Locations().Index(S.Location()));
                }

                OS.Write("\n");
            }

            if (TF.Triangulation() is not null)
            {
                // Dump the triangulation
                OS.Write("    - Triangulation : ");
                Out(OS, myTriangulations.FindIndex(TF.Triangulation()!));
                if (!S.Location().IsIdentity())
                {
                    OS.Write(" location ");
                    Out(OS, Locations().Index(TF.Location()));
                }

                OS.Write("\n");
            }
        }

        OS.Write("\n");
    }

    // FindIndex of a handle that may be null (OCCT's maps hold the null handle as a key like any other)
    private static int FindIndex(NCollection_IndexedMap<object> theMap, object? theKey)
        => theKey is null ? 0 : theMap.FindIndex(theKey);

    /// <summary>Writes the geometry of the shape, with the indices of its curves, surfaces, polygons and locations.</summary>
    public override void WriteGeometry(in TopoDS_Shape S, TextWriter OS)
    {
        // Write the geometry

        if (S.ShapeType() == TopAbs_VERTEX)
        {
            // Write the point geometry
            TopoDS_Vertex V = TopoDS.Vertex(S);
            Out(OS, BRep_Tool.Tolerance(V));
            OS.Write("\n");
            gp_Pnt p = BRep_Tool.Pnt(V);
            Out(OS, p.X());
            OS.Write(" ");
            Out(OS, p.Y());
            OS.Write(" ");
            Out(OS, p.Z());
            OS.Write("\n");

            BRep_TVertex TV = (BRep_TVertex)S.TShape()!;
            NCollection_List<BRep_PointRepresentation>.Iterator itrp = new(TV.Points());

            while (itrp.More())
            {
                BRep_PointRepresentation PR = itrp.Value();

                Out(OS, PR.Parameter());
                if (PR.IsPointOnCurve())
                {
                    OS.Write(" 1 ");
                    Out(OS, myCurves.Index(PR.Curve()));
                }

                else if (PR.IsPointOnCurveOnSurface())
                {
                    OS.Write(" 2 ");
                    Out(OS, myCurves2d.Index(PR.PCurve()));
                    OS.Write(" ");
                    Out(OS, mySurfaces.Index(PR.Surface()));
                }

                else if (PR.IsPointOnSurface())
                {
                    OS.Write(" 3 ");
                    Out(OS, PR.Parameter2());
                    OS.Write(" ");
                    Out(OS, mySurfaces.Index(PR.Surface()));
                }

                OS.Write(" ");
                Out(OS, Locations().Index(PR.Location()));
                OS.Write("\n");

                itrp.Next();
            }

            OS.Write("0 0\n"); // end representations
        }

        else if (S.ShapeType() == TopAbs_EDGE)
        {
            // Write the curve geometry

            BRep_TEdge TE = (BRep_TEdge)S.TShape()!;

            OS.Write(" ");
            Out(OS, TE.Tolerance());
            OS.Write(" ");
            OS.Write(TE.SameParameter() ? "1" : "0");
            OS.Write(" ");
            OS.Write(TE.SameRange() ? "1" : "0");
            OS.Write(" ");
            OS.Write(TE.Degenerated() ? "1" : "0");
            OS.Write("\n");

            double first, last;
            NCollection_List<BRep_CurveRepresentation>.Iterator itrc = new(TE.Curves());
            while (itrc.More())
            {
                BRep_CurveRepresentation CR = itrc.Value();
                if (CR.IsCurve3D())
                {
                    if (CR.Curve3D() is not null)
                    {
                        BRep_GCurve GC = (BRep_GCurve)itrc.Value();
                        GC.Range(out first, out last);
                        OS.Write("1 "); // -1- Curve 3D
                        OS.Write(" ");
                        Out(OS, myCurves.Index(CR.Curve3D()));
                        OS.Write(" ");
                        Out(OS, Locations().Index(CR.Location()));
                        OS.Write(" ");
                        Out(OS, first);
                        OS.Write(" ");
                        Out(OS, last);
                        OS.Write("\n");
                    }
                }
                else if (CR.IsCurveOnSurface())
                {
                    BRep_GCurve GC = (BRep_GCurve)itrc.Value();
                    GC.Range(out first, out last);
                    if (!CR.IsCurveOnClosedSurface())
                    {
                        OS.Write("2 "); // -2- Curve on surf
                    }
                    else
                    {
                        OS.Write("3 "); // -3- Curve on closed surf
                    }

                    OS.Write(" ");
                    Out(OS, myCurves2d.Index(CR.PCurve()));
                    if (CR.IsCurveOnClosedSurface())
                    {
                        OS.Write(" ");
                        Out(OS, myCurves2d.Index(CR.PCurve2()));
                        PrintRegularity(CR.Continuity(), OS);
                    }

                    OS.Write(" ");
                    Out(OS, mySurfaces.Index(CR.Surface()));
                    OS.Write(" ");
                    Out(OS, Locations().Index(CR.Location()));
                    OS.Write(" ");
                    Out(OS, first);
                    OS.Write(" ");
                    Out(OS, last);
                    OS.Write("\n");

                    // Write UV Points // for XML Persistence higher performance
                    if (FormatNb() == (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_2)
                    {
                        gp_Pnt2d Pf, Pl;
                        if (CR.IsCurveOnClosedSurface())
                        {
                            BRep_CurveOnClosedSurface COCS = (BRep_CurveOnClosedSurface)CR;
                            COCS.UVPoints2(out Pf, out Pl);
                        }
                        else
                        {
                            BRep_CurveOnSurface COS = (BRep_CurveOnSurface)CR;
                            COS.UVPoints(out Pf, out Pl);
                        }

                        Out(OS, Pf.X());
                        OS.Write(" ");
                        Out(OS, Pf.Y());
                        OS.Write(" ");
                        Out(OS, Pl.X());
                        OS.Write(" ");
                        Out(OS, Pl.Y());
                        OS.Write("\n");
                    }
                }
                else if (CR.IsRegularity())
                {
                    OS.Write("4 "); // -4- Regularity
                    PrintRegularity(CR.Continuity(), OS);
                    OS.Write(" ");
                    Out(OS, mySurfaces.Index(CR.Surface()));
                    OS.Write(" ");
                    Out(OS, Locations().Index(CR.Location()));
                    OS.Write(" ");
                    Out(OS, mySurfaces.Index(CR.Surface2()));
                    OS.Write(" ");
                    Out(OS, Locations().Index(CR.Location2()));
                    OS.Write("\n");
                }

                else if (myWithTriangles)
                {
                    // for XML Persistence
                    if (CR.IsPolygon3D())
                    {
                        BRep_Polygon3D GC = (BRep_Polygon3D)itrc.Value();
                        if (GC.Polygon3D() is not null)
                        {
                            OS.Write("5 "); // -5- Polygon3D
                            OS.Write(" ");
                            Out(OS, myPolygons3D.FindIndex(CR.Polygon3D()!));
                            OS.Write(" ");
                            Out(OS, Locations().Index(CR.Location()));
                            OS.Write("\n");
                        }
                    }
                    else if (CR.IsPolygonOnTriangulation())
                    {
                        BRep_PolygonOnTriangulation PT = (BRep_PolygonOnTriangulation)itrc.Value();
                        if (!CR.IsPolygonOnClosedTriangulation())
                        {
                            OS.Write("6 "); // -6- Polygon on triangulation
                        }
                        else
                        {
                            OS.Write("7 "); // -7- Polygon on closed triangulation
                        }

                        OS.Write(" ");
                        Out(OS, FindIndex(myNodes, PT.PolygonOnTriangulation()));
                        if (CR.IsPolygonOnClosedTriangulation())
                        {
                            OS.Write(" ");
                            Out(OS, FindIndex(myNodes, PT.PolygonOnTriangulation2()));
                        }

                        OS.Write(" ");
                        Out(OS, PT.Triangulation() is { } aTr ? myTriangulations.FindIndex(aTr) : 0);
                        OS.Write(" ");
                        Out(OS, Locations().Index(CR.Location()));
                        OS.Write("\n");
                    }
                }

                itrc.Next();
            }

            OS.Write("0\n"); // end of the list of representations
        }

        else if (S.ShapeType() == TopAbs_FACE)
        {
            BRep_TFace TF = (BRep_TFace)S.TShape()!;
            TopoDS_Face F = TopoDS.Face(S);

            if (TF.Surface() is not null)
            {
                OS.Write(BRep_Tool.NaturalRestriction(F) ? "1" : "0");
                OS.Write(" ");
                // Write the surface geometry
                OS.Write(" ");
                Out(OS, TF.Tolerance());
                OS.Write(" ");
                Out(OS, mySurfaces.Index(TF.Surface()));
                OS.Write(" ");
                Out(OS, Locations().Index(TF.Location()));
                OS.Write("\n");
            }
            else // For correct reading of null face
            {
                Out(OS, 0);
                OS.Write(" ");
                OS.Write(" ");
                Out(OS, TF.Tolerance());
                OS.Write(" ");
                Out(OS, 0);
                OS.Write(" ");
                Out(OS, 0);
                OS.Write("\n");
            }

            if (myWithTriangles || TF.Surface() is null)
            {
                // for XML Persistence
                if (TF.Triangulation() is not null)
                {
                    Out(OS, 2);
                    OS.Write(" ");
                    // Write the triangulation
                    OS.Write(" ");
                    Out(OS, myTriangulations.FindIndex(TF.Triangulation()!));
                }
            }
        }
    }

    private static GeomAbs_Shape ReadRegularity(TextReader IS)
    {
        char[] buffer = new char[255];
        Standard_Stream.ReadWord(IS, buffer);
        switch (buffer[0])
        {
            case 'C':
                switch (buffer[1])
                {
                    case '0':
                        return GeomAbs_C0;

                    case '1':
                        return GeomAbs_C1;

                    case '2':
                        return GeomAbs_C2;

                    case '3':
                        return GeomAbs_C3;

                    case 'N':
                        return GeomAbs_CN;
                }

                break;

            case 'G':
                switch (buffer[1])
                {
                    case '1':
                        return GeomAbs_G1;

                    case '2':
                        return GeomAbs_G2;
                }

                break;
        }

        return GeomAbs_C0;
    }

    /// <summary>Reads the geometry of a shape of type T and creates the shape in S, unfrozen.</summary>
    /// <exception cref="Storage_StreamFormatError">The input can't be read safely.</exception>
    internal override void ReadGeometry(TopAbs_ShapeEnum T, TextReader IS, ref TopoDS_Shape S)
    {
        // Read the geometry

        // OCCT leaves most of them uninitialised
        int val = 0, c = 0, pc = 0, pc2 = 0, s = 0, s2 = 0, l = 0, l2 = 0, t = 0, pt = 0, pt2 = 0;
        double tol, X, Y, Z, first, last, p1, p2 = 0;
        double PfX, PfY, PlX, PlY;
        gp_Pnt2d aPf = new(), aPl = new();
        bool closed;
        GeomAbs_Shape reg = GeomAbs_C0;
        switch (T)
        {
            //---------
            // vertex
            //---------

            case TopAbs_VERTEX:
            {
                TopoDS_Vertex V = new();

                // Read the point geometry
                GeomTools.GetReal(IS, out tol);
                GeomTools.GetReal(IS, out X);
                GeomTools.GetReal(IS, out Y);
                GeomTools.GetReal(IS, out Z);
                myBuilder.MakeVertex(ref V, new gp_Pnt(X, Y, Z), tol);
                S = V;
                BRep_TVertex TV = (BRep_TVertex)V.TShape()!;

                NCollection_List<BRep_PointRepresentation> lpr = TV.ChangePoints();
                TopLoc_Location L = new();

                do
                {
                    GeomTools.GetReal(IS, out p1);
                    Standard_Stream.Read(IS, ref val);
                    // OCCT.NET: untrusted input: OCCT loops for ever on a stream that has failed
                    Standard_Stream.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadGeometry");

                    BRep_PointRepresentation? PR = null;
                    switch (val)
                    {
                        case 1:
                        {
                            Standard_Stream.Read(IS, ref c);

                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:09 2002 Begin
                            if (myCurves.Curve(c) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:13 2002 End

                            BRep_PointOnCurve POC = new(p1, myCurves.Curve(c), L);
                            PR = POC;
                        }
                            break;

                        case 2:
                        {
                            Standard_Stream.Read(IS, ref pc);
                            Standard_Stream.Read(IS, ref s);

                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:09 2002 Begin
                            if (myCurves2d.Curve2d(pc) is null || mySurfaces.Surface(s) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:13 2002 End

                            BRep_PointOnCurveOnSurface POC = new(p1, myCurves2d.Curve2d(pc), mySurfaces.Surface(s), L);
                            PR = POC;
                        }
                            break;

                        case 3:
                        {
                            GeomTools.GetReal(IS, out p2);
                            Standard_Stream.Read(IS, ref s);

                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:09 2002 Begin
                            if (mySurfaces.Surface(s) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 13:59:13 2002 End

                            BRep_PointOnSurface POC = new(p1, p2, mySurfaces.Surface(s), L);
                            PR = POC;
                        }
                            break;
                    }

                    if (val > 0)
                    {
                        Standard_Stream.Read(IS, ref l);
                        if (PR is not null)
                        {
                            PR.Location(Locations().Location(l));
                            lpr.Append(PR);
                        }
                    }
                } while (val > 0);
            }
                break;

            //---------
            // edge
            //---------

            case TopAbs_EDGE:

                // Create an edge
            {
                TopoDS_Edge E = new();

                myBuilder.MakeEdge(ref E);
                S = E;

                // Read the curve geometry
                GeomTools.GetReal(IS, out tol);
                Standard_Stream.Read(IS, ref val);
                myBuilder.SameParameter(E, (val == 1));
                Standard_Stream.Read(IS, ref val);
                myBuilder.SameRange(E, (val == 1));
                Standard_Stream.Read(IS, ref val);
                myBuilder.Degenerated(E, (val == 1));

                do
                {
                    Standard_Stream.Read(IS, ref val);
                    // OCCT.NET: untrusted input: OCCT loops for ever on a stream that has failed
                    Standard_Stream.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadGeometry");
                    switch (val)
                    {
                        case 1: // -1- Curve 3D
                            Standard_Stream.Read(IS, ref c);
                            Standard_Stream.Read(IS, ref l);
                            if (myCurves.Curve(c) is not null)
                            {
                                myBuilder.UpdateEdge(E, myCurves.Curve(c), Locations().Location(l), tol);
                            }

                            GeomTools.GetReal(IS, out first);
                            GeomTools.GetReal(IS, out last);
                            if (myCurves.Curve(c) is not null)
                            {
                                bool Only3d = true;
                                myBuilder.Range(E, first, last, Only3d);
                            }

                            break;

                        case 2: // -2- Curve on surf
                        case 3: // -3- Curve on closed surf
                            closed = (val == 3);
                            Standard_Stream.Read(IS, ref pc);
                            if (closed)
                            {
                                Standard_Stream.Read(IS, ref pc2);
                                reg = ReadRegularity(IS);
                            }

                            // surface, location
                            Standard_Stream.Read(IS, ref s);
                            Standard_Stream.Read(IS, ref l);

                            // range
                            GeomTools.GetReal(IS, out first);
                            GeomTools.GetReal(IS, out last);

                            // read UV Points // for XML Persistence higher performance
                            if (FormatNb() == (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_2)
                            {
                                GeomTools.GetReal(IS, out PfX);
                                GeomTools.GetReal(IS, out PfY);
                                GeomTools.GetReal(IS, out PlX);
                                GeomTools.GetReal(IS, out PlY);
                                aPf = new gp_Pnt2d(PfX, PfY);
                                aPl = new gp_Pnt2d(PlX, PlY);
                            }

                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:11:16 2002 Begin
                            if (myCurves2d.Curve2d(pc) is null || (closed && myCurves2d.Curve2d(pc2) is null)
                                || mySurfaces.Surface(s) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:11:17 2002 End

                            if (closed)
                            {
                                if (FormatNb() == (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_2)
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myCurves2d.Curve2d(pc),
                                                         myCurves2d.Curve2d(pc2),
                                                         mySurfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol,
                                                         aPf,
                                                         aPl);
                                }
                                else
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myCurves2d.Curve2d(pc),
                                                         myCurves2d.Curve2d(pc2),
                                                         mySurfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol);
                                }

                                myBuilder.Continuity(E,
                                                     mySurfaces.Surface(s),
                                                     mySurfaces.Surface(s),
                                                     Locations().Location(l),
                                                     Locations().Location(l),
                                                     reg);
                            }
                            else
                            {
                                if (FormatNb() == (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_2)
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myCurves2d.Curve2d(pc),
                                                         mySurfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol,
                                                         aPf,
                                                         aPl);
                                }
                                else
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myCurves2d.Curve2d(pc),
                                                         mySurfaces.Surface(s),
                                                         Locations().Location(l),
                                                         tol);
                                }
                            }

                            myBuilder.Range(E, mySurfaces.Surface(s), Locations().Location(l), first, last);
                            break;

                        case 4: // -4- Regularity
                            reg = ReadRegularity(IS);
                            Standard_Stream.Read(IS, ref s);
                            Standard_Stream.Read(IS, ref l);
                            Standard_Stream.Read(IS, ref s2);
                            Standard_Stream.Read(IS, ref l2);
                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:13 2002 Begin
                            if (mySurfaces.Surface(s) is null || mySurfaces.Surface(s2) is null)
                            {
                                break;
                            }
                            //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:14 2002 End
                            myBuilder.Continuity(E,
                                                 mySurfaces.Surface(s),
                                                 mySurfaces.Surface(s2),
                                                 Locations().Location(l),
                                                 Locations().Location(l2),
                                                 reg);
                            break;

                        case 5: // -5- Polygon3D
                            Standard_Stream.Read(IS, ref c);
                            Standard_Stream.Read(IS, ref l);
                            // szy-02.05.2004
                            // myBuilder.UpdateEdge(E,occ::down_cast<Poly_Polygon3D>(myPolygons3D(c)));
                            if (c > 0 && c <= myPolygons3D.Extent())
                            {
                                myBuilder.UpdateEdge(E, myPolygons3D[c] as Poly_Polygon3D, Locations().Location(l));
                            }

                            break;

                        case 6:
                        case 7:
                            closed = (val == 7);
                            Standard_Stream.Read(IS, ref pt);
                            if (closed)
                            {
                                Standard_Stream.Read(IS, ref pt2);
                            }

                            Standard_Stream.Read(IS, ref t);
                            Standard_Stream.Read(IS, ref l);
                            if (closed)
                            {
                                if (t > 0 && t <= myTriangulations.Extent() && pt > 0 && pt <= myNodes.Extent()
                                    && pt2 > 0 && pt2 <= myNodes.Extent())
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myNodes[pt] as Poly_PolygonOnTriangulation,
                                                         myNodes[pt2] as Poly_PolygonOnTriangulation,
                                                         myTriangulations.FindKey(t),
                                                         Locations().Location(l));
                                }
                            }
                            else
                            {
                                if (t > 0 && t <= myTriangulations.Extent() && pt > 0 && pt <= myNodes.Extent())
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myNodes[pt] as Poly_PolygonOnTriangulation,
                                                         myTriangulations.FindKey(t),
                                                         Locations().Location(l));
                                }
                            }
                            // range

                            break;
                    }
                } while (val > 0);
            }
                break;

            //---------
            // wire
            //---------

            case TopAbs_WIRE:
            {
                TopoDS_Wire W = new();
                myBuilder.MakeWire(ref W);
                S = W;
            }
                break;

            //---------
            // face
            //---------

            case TopAbs_FACE:
            {
                // create a face :
                TopoDS_Face F = new();
                //    std::streampos pos;
                myBuilder.MakeFace(ref F);
                S = F;

                Standard_Stream.Read(IS, ref val); // natural restriction
                if (val == 0 || val == 1)
                {
                    GeomTools.GetReal(IS, out tol);
                    Standard_Stream.Read(IS, ref s);
                    Standard_Stream.Read(IS, ref l);
                    //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:13 2002 Begin
                    if (mySurfaces.Surface(s) is not null)
                    {
                        //  Modified by Sergey KHROMOV - Wed Apr 24 12:39:14 2002 End
                        myBuilder.UpdateFace(F, mySurfaces.Surface(s), Locations().Location(l), tol);
                        myBuilder.NaturalRestriction(F, (val == 1));
                    }
                    //      pos = IS.tellg();
                    //      IS >> val;
                }
                else if (val == 2)
                {
                    // only triangulation
                    Standard_Stream.Read(IS, ref s);
                    // OCCT.NET: untrusted input: OCCT reads outside its map
                    if (s < 1 || s > myTriangulations.Extent())
                    {
                        throw new Storage_StreamFormatError("BRepTools_ShapeSet::ReadGeometry: no triangulation "
                                                            + s.ToString(CultureInfo.InvariantCulture));
                    }

                    myBuilder.UpdateFace(F, myTriangulations.FindKey(s));
                }
                //    else pos = IS.tellg();

                // BUC60769

                if (val == 2)
                {
                    break;
                }

                char[] @string = new char[260];
                Standard_Stream.GetLine(IS, @string, 256, '\n');
                Standard_Stream.GetLine(IS, @string, 256, '\n');

                if (@string[0] == '2')
                {
                    // cas triangulation
                    s = Standard_Stream.Atoi(@string.AsSpan(2));
                    if (s > 0 && s <= myTriangulations.Extent())
                    {
                        myBuilder.UpdateFace(F, myTriangulations.FindKey(s));
                    }
                }
                //    else IS.seekg(pos);
            }
                break;

            //---------
            // shell
            //---------

            case TopAbs_SHELL:
            {
                TopoDS_Shell Sh = new();
                myBuilder.MakeShell(ref Sh);
                S = Sh;
            }
                break;

            //---------
            // solid
            //---------

            case TopAbs_SOLID:
            {
                TopoDS_Solid So = new();
                myBuilder.MakeSolid(ref So);
                S = So;
            }
                break;

            //---------
            // compsolid
            //---------

            case TopAbs_COMPSOLID:
            {
                TopoDS_CompSolid CS = new();
                myBuilder.MakeCompSolid(ref CS);
                S = CS;
            }
                break;

            //---------
            // compound
            //---------

            case TopAbs_COMPOUND:
            {
                TopoDS_Compound Co = new();
                myBuilder.MakeCompound(ref Co);
                S = Co;
            }
                break;

            default:
                break;
        }
    }

    /// <summary>Adds S2 to S1, with the set's builder.</summary>
    internal override void AddShapes(ref TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        myBuilder.Add(S1, S2);
    }

    /// <summary>Completes a face read from a version 1 file: <c>BRepTools.Update</c> computes its UV points.</summary>
    internal override void Check(TopAbs_ShapeEnum T, ref TopoDS_Shape S)
    {
        if (T == TopAbs_FACE)
        {
            TopoDS_Face F = TopoDS.Face(S);
            BRepTools.Update(F);
        }
    }

    /// <summary>
    /// Writes the polygons on triangulations: compact, in the format <see cref="ReadPolygonOnTriangulation"/> reads,
    /// or in a readable form.
    /// </summary>
    public void WritePolygonOnTriangulation(TextWriter OS, bool Compact = true, Message_ProgressRange theProgress = default)
    {
        int i, j, nbpOntri = myNodes.Extent();

        using var aPS = new Message_ProgressScope(theProgress, "Polygons On Triangulation", nbpOntri);
        if (Compact)
        {
            OS.Write("PolygonOnTriangulations ");
            Out(OS, nbpOntri);
            OS.Write("\n");
        }
        else
        {
            OS.Write(" -------\n");
            OS.Write("Dump of ");
            Out(OS, nbpOntri);
            OS.Write(" PolygonOnTriangulations\n");
            OS.Write(" -------\n");
        }

        Poly_PolygonOnTriangulation Poly;
        NCollection_HArray1<double>? Param;
        for (i = 1; i <= nbpOntri && aPS.More(); i++, aPS.Next())
        {
            Poly = (Poly_PolygonOnTriangulation)myNodes[i];
            NCollection_Array1<int> Nodes = Poly.Nodes();
            if (!Compact)
            {
                OS.Write("  ");
                Out(OS, i);
                OS.Write(" : PolygonOnTriangulation with ");
                Out(OS, Nodes.Length());
                OS.Write(" Nodes\n");
            }
            else
            {
                Out(OS, Nodes.Length());
                OS.Write(" ");
            }

            if (!Compact)
            {
                OS.Write("  ");
            }

            for (j = 1; j <= Nodes.Length(); j++)
            {
                Out(OS, Nodes.Value(j));
                OS.Write(" ");
            }

            OS.Write("\n");

            // writing parameters:
            Param = Poly.Parameters();
            if (Compact)
            {
                OS.Write("p ");
            }

            // write the deflection
            if (!Compact)
            {
                OS.Write("  Deflection : ");
            }

            Out(OS, Poly.Deflection());
            OS.Write(" ");
            if (!Compact)
            {
                OS.Write("\n");
            }

            if (Param is not null)
            {
                if (!Compact)
                {
                    OS.Write("  Parameters :");
                }
                else
                {
                    OS.Write("1 ");
                }

                if (!Compact)
                {
                    OS.Write("  ");
                }

                for (j = 1; j <= Param.Length(); j++)
                {
                    Out(OS, Param.Value(j));
                    OS.Write(" ");
                }

                OS.Write("\n");
            }
            else
            {
                OS.Write("0 \n");
            }
        }
    }

    /// <summary>Dumps the polygons on triangulations in a readable form.</summary>
    public void DumpPolygonOnTriangulation(TextWriter OS)
    {
        WritePolygonOnTriangulation(OS, false);
    }

    /// <summary>Reads the polygons on triangulations, written by <see cref="WritePolygonOnTriangulation"/>.</summary>
    /// <exception cref="Storage_StreamFormatError">The input can't be read safely.</exception>
    public void ReadPolygonOnTriangulation(TextReader IS, Message_ProgressRange theProgress = default)
    {
        char[] buffer = new char[255];
        Standard_Stream.ReadWord(IS, buffer);
        if (!Standard_Stream.CString(buffer).Contains("PolygonOnTriangulations", StringComparison.Ordinal))
        {
            return;
        }

        int i, j, val = 0, nbpol = 0, nbnodes = 0;
        int hasparameters = 0;
        double par;
        Poly_PolygonOnTriangulation Poly;
        Standard_Stream.Read(IS, ref nbpol);
        // OCCT.NET: untrusted input: a polygon takes at least 9 characters ("0 \np 0 0 \n")
        Standard_Stream.CheckCount(IS, Math.Max(nbpol, 0), 8, "polygons on triangulations");
        // OCC19559
        using var aPS = new Message_ProgressScope(theProgress, "Polygons On Triangulation", nbpol);
        for (i = 1; i <= nbpol && aPS.More(); i++, aPS.Next())
        {
            Standard_Stream.Read(IS, ref nbnodes);
            // OCCT.NET: untrusted input: OCCT allocates for any count, and again for each polygon on a failed stream
            Standard_Stream.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadPolygonOnTriangulation");
            Standard_Stream.CheckCount(IS, nbnodes, 2, "polygon nodes");
            NCollection_Array1<int> Nodes = new(1, nbnodes);
            for (j = 1; j <= nbnodes; j++)
            {
                Standard_Stream.Read(IS, ref val);
                Nodes[j] = val;
            }

            Standard_Stream.ReadWord(IS, buffer);
            //      if (!strcasecmp(buffer, "p")) {
            double def;
            GeomTools.GetReal(IS, out def);
            Standard_Stream.Read(IS, ref hasparameters);
            if (hasparameters != 0)
            {
                NCollection_Array1<double> Param1 = new(1, nbnodes);
                for (j = 1; j <= nbnodes; j++)
                {
                    GeomTools.GetReal(IS, out par);
                    Param1[j] = par;
                }

                Poly = new Poly_PolygonOnTriangulation(Nodes, Param1);
            }
            else
            {
                Poly = new Poly_PolygonOnTriangulation(Nodes);
            }

            Poly.Deflection(def);
            //      }
            //      else {
            //      IS.seekg(ppp);
            //      Poly = new Poly_PolygonOnTriangulation(Nodes);
            //      }
            myNodes.Add(Poly);
        }
        //  }
        //  else IS.seekg(pos);
    }

    /// <summary>Writes the 3D polygons: compact, in the format <see cref="ReadPolygon3D"/> reads, or in a readable form.</summary>
    public void WritePolygon3D(TextWriter OS, bool Compact = true, Message_ProgressRange theProgress = default)
    {
        int i, j, nbpol = myPolygons3D.Extent();

        using var aPS = new Message_ProgressScope(theProgress, "3D Polygons", nbpol);

        if (Compact)
        {
            OS.Write("Polygon3D ");
            Out(OS, nbpol);
            OS.Write("\n");
        }
        else
        {
            OS.Write(" -------\n");
            OS.Write("Dump of ");
            Out(OS, nbpol);
            OS.Write(" Polygon3Ds\n");
            OS.Write(" -------\n");
        }

        Poly_Polygon3D P;
        for (i = 1; i <= nbpol && aPS.More(); i++, aPS.Next())
        {
            P = (Poly_Polygon3D)myPolygons3D[i];
            if (Compact)
            {
                Out(OS, P.NbNodes());
                OS.Write(" ");
                OS.Write(P.HasParameters() ? "1" : "0");
                OS.Write("\n");
            }
            else
            {
                OS.Write("  ");
                Out(OS, i);
                OS.Write(" : Polygon3D with ");
                Out(OS, P.NbNodes());
                OS.Write(" Nodes\n");
                OS.Write(P.HasParameters() ? "with" : "without");
                OS.Write(" parameters\n");
            }

            // write the deflection
            if (!Compact)
            {
                OS.Write("Deflection : ");
            }

            Out(OS, P.Deflection());
            OS.Write("\n");

            // write the nodes
            if (!Compact)
            {
                OS.Write("\nNodes :\n");
            }

            int i1, nbNodes = P.NbNodes();
            NCollection_Array1<gp_Pnt> Nodes = P.Nodes();
            for (j = 1; j <= nbNodes; j++)
            {
                if (!Compact)
                {
                    Setw(OS, 10, j);
                    OS.Write(" : ");
                }

                if (!Compact)
                {
                    Setw(OS, 17, Nodes[j].X());
                }
                else
                {
                    Out(OS, Nodes[j].X());
                }

                OS.Write(" ");
                if (!Compact)
                {
                    Setw(OS, 17, Nodes[j].Y());
                }
                else
                {
                    Out(OS, Nodes[j].Y());
                }

                OS.Write(" ");
                if (!Compact)
                {
                    Setw(OS, 17, Nodes[j].Z());
                }
                else
                {
                    Out(OS, Nodes[j].Z());
                }

                if (!Compact)
                {
                    OS.Write("\n");
                }
                else
                {
                    OS.Write(" ");
                }
            }

            OS.Write("\n");

            if (P.HasParameters())
            {
                if (!Compact)
                {
                    OS.Write("\nParameters :\n");
                }

                NCollection_Array1<double> Param = P.Parameters();
                for (i1 = 1; i1 <= nbNodes; i1++)
                {
                    Out(OS, Param[i1]);
                    OS.Write(" ");
                }

                OS.Write("\n");
            }
        }
    }

    /// <summary>Dumps the 3D polygons in a readable form.</summary>
    public void DumpPolygon3D(TextWriter OS)
    {
        WritePolygon3D(OS, false);
    }

    /// <summary>Reads the 3D polygons, written by <see cref="WritePolygon3D"/>.</summary>
    /// <exception cref="Storage_StreamFormatError">The input can't be read safely.</exception>
    public void ReadPolygon3D(TextReader IS, Message_ProgressRange theProgress = default)
    {
        char[] buffer = new char[255];
        //  int i, j, p, val, nbpol, nbnodes, hasparameters;
        int i, j, p, nbpol = 0, nbnodes = 0, hasparameters = 0;
        double d, x, y, z;

        Standard_Stream.ReadWord(IS, buffer);
        if (!Standard_Stream.CString(buffer).Contains("Polygon3D", StringComparison.Ordinal))
        {
            return;
        }

        Poly_Polygon3D P;
        Standard_Stream.Read(IS, ref nbpol);
        // OCCT.NET: untrusted input: a polygon takes at least 6 characters ("0 0\n0\n\n")
        Standard_Stream.CheckCount(IS, Math.Max(nbpol, 0), 6, "3D polygons");
        // OCC19559
        using var aPS = new Message_ProgressScope(theProgress, "3D Polygons", nbpol);
        for (i = 1; i <= nbpol && aPS.More(); i++, aPS.Next())
        {
            Standard_Stream.Read(IS, ref nbnodes);
            Standard_Stream.Read(IS, ref hasparameters);
            // OCCT.NET: untrusted input: OCCT allocates for any count, and again for each polygon on a failed stream
            Standard_Stream.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadPolygon3D");
            Standard_Stream.CheckCount(IS, nbnodes, 6, "polygon nodes");
            NCollection_Array1<gp_Pnt> Nodes = new(1, nbnodes);
            GeomTools.GetReal(IS, out d);
            for (j = 1; j <= nbnodes; j++)
            {
                GeomTools.GetReal(IS, out x);
                GeomTools.GetReal(IS, out y);
                GeomTools.GetReal(IS, out z);
                Nodes[j].SetCoord(x, y, z);
            }

            if (hasparameters != 0)
            {
                NCollection_Array1<double> Param = new(1, nbnodes);
                for (p = 1; p <= nbnodes; p++)
                {
                    GeomTools.GetReal(IS, out Param[p]);
                }

                P = new Poly_Polygon3D(Nodes, Param);
            }
            else
            {
                P = new Poly_Polygon3D(Nodes);
            }

            P.Deflection(d);
            myPolygons3D.Add(P);
        }
    }

    /// <summary>
    /// Writes the triangulations: compact, in the format <see cref="ReadTriangulation"/> reads, or in a readable form.
    /// </summary>
    public void WriteTriangulation(TextWriter OS, bool Compact = true, Message_ProgressRange theProgress = default)
    {
        int i, j, nbNodes, nbtri = myTriangulations.Extent();
        int nbTriangles = 0, n1, n2, n3;

        using var aPS = new Message_ProgressScope(theProgress, "Triangulations", nbtri);

        if (Compact)
        {
            OS.Write("Triangulations ");
            Out(OS, nbtri);
            OS.Write("\n");
        }
        else
        {
            OS.Write(" -------\n");
            OS.Write("Dump of ");
            Out(OS, nbtri);
            OS.Write(" Triangulations\n");
            OS.Write(" -------\n");
        }

        Poly_Triangulation T;
        for (i = 1; i <= nbtri && aPS.More(); i++, aPS.Next())
        {
            T = myTriangulations.FindKey(i);
            bool toWriteNormals = myTriangulations[i];
            if (Compact)
            {
                Out(OS, T.NbNodes());
                OS.Write(" ");
                Out(OS, T.NbTriangles());
                OS.Write(" ");
                OS.Write(T.HasUVNodes() ? "1" : "0");
                OS.Write(" ");
                if (FormatNb() >= (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
                {
                    OS.Write(T.HasNormals() && toWriteNormals ? "1" : "0");
                    OS.Write(" ");
                }
            }
            else
            {
                OS.Write("  ");
                Out(OS, i);
                OS.Write(" : Triangulation with ");
                Out(OS, T.NbNodes());
                OS.Write(" Nodes and ");
                Out(OS, T.NbTriangles());
                OS.Write(" Triangles\n");
                OS.Write("      ");
                OS.Write(T.HasUVNodes() ? "with" : "without");
                OS.Write(" UV nodes\n");
                if (FormatNb() >= (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
                {
                    OS.Write("      ");
                    OS.Write(T.HasNormals() && toWriteNormals ? "with" : "without");
                    OS.Write(" normals\n");
                }
            }

            // write the deflection

            if (!Compact)
            {
                OS.Write("  Deflection : ");
            }

            Out(OS, T.Deflection());
            OS.Write("\n");

            // write the 3d nodes

            if (!Compact)
            {
                OS.Write("\n3D Nodes :\n");
            }

            nbNodes = T.NbNodes();
            for (j = 1; j <= nbNodes; j++)
            {
                gp_Pnt aNode = T.Node(j);
                if (!Compact)
                {
                    Setw(OS, 10, j);
                    OS.Write(" : ");
                }

                if (!Compact)
                {
                    Setw(OS, 17, aNode.X());
                }
                else
                {
                    Out(OS, aNode.X());
                }

                OS.Write(" ");
                if (!Compact)
                {
                    Setw(OS, 17, aNode.Y());
                }
                else
                {
                    Out(OS, aNode.Y());
                }

                OS.Write(" ");
                if (!Compact)
                {
                    Setw(OS, 17, aNode.Z());
                }
                else
                {
                    Out(OS, aNode.Z());
                }

                if (!Compact)
                {
                    OS.Write("\n");
                }
                else
                {
                    OS.Write(" ");
                }
            }

            if (T.HasUVNodes())
            {
                if (!Compact)
                {
                    OS.Write("\nUV Nodes :\n");
                }

                for (j = 1; j <= nbNodes; j++)
                {
                    gp_Pnt2d aNode2d = T.UVNode(j);
                    if (!Compact)
                    {
                        Setw(OS, 10, j);
                        OS.Write(" : ");
                    }

                    if (!Compact)
                    {
                        Setw(OS, 17, aNode2d.X());
                    }
                    else
                    {
                        Out(OS, aNode2d.X());
                    }

                    OS.Write(" ");
                    if (!Compact)
                    {
                        Setw(OS, 17, aNode2d.Y());
                    }
                    else
                    {
                        Out(OS, aNode2d.Y());
                    }

                    if (!Compact)
                    {
                        OS.Write("\n");
                    }
                    else
                    {
                        OS.Write(" ");
                    }
                }
            }

            if (!Compact)
            {
                OS.Write("\nTriangles :\n");
            }

            nbTriangles = T.NbTriangles();
            for (j = 1; j <= nbTriangles; j++)
            {
                if (!Compact)
                {
                    Setw(OS, 10, j);
                    OS.Write(" : ");
                }

                T.Triangle(j).Get(out n1, out n2, out n3);
                if (!Compact)
                {
                    Setw(OS, 10, n1);
                }
                else
                {
                    Out(OS, n1);
                }

                OS.Write(" ");
                if (!Compact)
                {
                    Setw(OS, 10, n2);
                }
                else
                {
                    Out(OS, n2);
                }

                OS.Write(" ");
                if (!Compact)
                {
                    Setw(OS, 10, n3);
                }
                else
                {
                    Out(OS, n3);
                }

                if (!Compact)
                {
                    OS.Write("\n");
                }
                else
                {
                    OS.Write(" ");
                }
            }

            if (FormatNb() >= (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
            {
                if (T.HasNormals() && toWriteNormals)
                {
                    if (!Compact)
                    {
                        OS.Write("\nNormals :\n");
                    }

                    for (j = 1; j <= nbNodes; j++)
                    {
                        // std::setw(17) applies to the first component only
                        bool isSetw = false;
                        if (!Compact)
                        {
                            Setw(OS, 10, j);
                            OS.Write(" : ");
                            isSetw = true;
                        }

                        NCollection_Vec3<float> aNorm;
                        for (int k = 0; k < 3; ++k)
                        {
                            T.Normal(j, out aNorm);
                            if (isSetw)
                            {
                                Setw(OS, 17, aNorm[k]);
                                isSetw = false;
                            }
                            else
                            {
                                Out(OS, aNorm[k]);
                            }

                            OS.Write(!Compact ? "\n" : " ");
                        }
                    }
                }
            }

            OS.Write("\n");
        }
    }

    /// <summary>Dumps the triangulations in a readable form.</summary>
    public void DumpTriangulation(TextWriter OS)
    {
        WriteTriangulation(OS, false);
    }

    /// <summary>Reads the triangulations, written by <see cref="WriteTriangulation"/>.</summary>
    /// <exception cref="Storage_StreamFormatError">The input can't be read safely.</exception>
    public void ReadTriangulation(TextReader IS, Message_ProgressRange theProgress = default)
    {
        char[] buffer = new char[255];
        int i, j, nbtri = 0;
        double d, x, y, z;
        int nbNodes = 0, nbTriangles = 0;
        bool hasUV = false;
        bool hasNormals = false;

        Poly_Triangulation T;

        Standard_Stream.ReadWord(IS, buffer);
        if (!Standard_Stream.CString(buffer).Contains("Triangulations", StringComparison.Ordinal))
        {
            return;
        }

        Standard_Stream.Read(IS, ref nbtri);
        // OCCT.NET: untrusted input: a triangulation takes at least 10 characters ("0 0 0 0 0\n\n")
        Standard_Stream.CheckCount(IS, Math.Max(nbtri, 0), 10, "triangulations");
        // OCC19559
        using var aPS = new Message_ProgressScope(theProgress, "Triangulations", nbtri);
        for (i = 1; i <= nbtri && aPS.More(); i++, aPS.Next())
        {
            Standard_Stream.Read(IS, ref nbNodes);
            Standard_Stream.Read(IS, ref nbTriangles);
            Standard_Stream.Read(IS, ref hasUV);
            if (FormatNb() >= (int)TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
            {
                Standard_Stream.Read(IS, ref hasNormals);
            }

            // OCCT.NET: untrusted input: OCCT allocates for any count, and again for each triangulation on a failed
            // stream; a node takes at least 6 characters, a triangle 6
            Standard_Stream.CheckNotFailed(IS, "BRepTools_ShapeSet::ReadTriangulation");
            Standard_Stream.CheckCount(IS, nbNodes, hasUV ? 10 : 6, "triangulation nodes");
            Standard_Stream.CheckCount(IS, nbTriangles, 6, "triangles");
            Standard_Stream.CheckCount(IS, (long)nbNodes * (hasUV ? 10 : 6) + (long)nbTriangles * 6, 1, "triangulation data");
            GeomTools.GetReal(IS, out d);

            T = new Poly_Triangulation(nbNodes, nbTriangles, hasUV, hasNormals);

            for (j = 1; j <= nbNodes; j++)
            {
                GeomTools.GetReal(IS, out x);
                GeomTools.GetReal(IS, out y);
                GeomTools.GetReal(IS, out z);
                T.SetNode(j, new gp_Pnt(x, y, z));
            }

            if (hasUV)
            {
                for (j = 1; j <= nbNodes; j++)
                {
                    GeomTools.GetReal(IS, out x);
                    GeomTools.GetReal(IS, out y);
                    T.SetUVNode(j, new gp_Pnt2d(x, y));
                }
            }

            // read the triangles
            // OCCT's n1, n2 and n3 are uninitialised, and left so when the stream has failed
            int n1 = 0, n2 = 0, n3 = 0;
            for (j = 1; j <= nbTriangles; j++)
            {
                Standard_Stream.Read(IS, ref n1);
                Standard_Stream.Read(IS, ref n2);
                Standard_Stream.Read(IS, ref n3);
                T.SetTriangle(j, new Poly_Triangle(n1, n2, n3));
            }

            if (hasNormals)
            {
                double aNormX, aNormY, aNormZ;
                for (j = 1; j <= nbNodes; j++)
                {
                    GeomTools.GetReal(IS, out aNormX);
                    GeomTools.GetReal(IS, out aNormY);
                    GeomTools.GetReal(IS, out aNormZ);
                    T.SetNormal(j, new NCollection_Vec3<float>((float)aNormX, (float)aNormY, (float)aNormZ));
                }
            }

            T.Deflection(d);
            myTriangulations.Add(T, hasNormals);
        }
    }
}
