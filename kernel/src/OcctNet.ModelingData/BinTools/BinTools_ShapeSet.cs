// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// A set of shapes with their geometry, written and read in OCCT's table-based binary BRep format (versions 1 to 4):
/// a header line, the locations, the 2D curves, curves, 3D polygons, polygons on triangulations, surfaces and
/// triangulations, then the TShapes, sub-shapes first, each referring to its sub-shapes by index.
/// </summary>
/// <remarks>
/// <para>
/// <see cref="Read(Stream, Message_ProgressRange)"/> freezes the shapes it creates once it has read them all (or has
/// failed), so every shape a set hands out after a read is frozen, as phase3-design.md, section 4.2, requires. The
/// writers write the seven flags OCCT writes (Free to Convex: the Locked bit isn't part of this format).
/// </para>
/// <para>
/// Reading a vertex in versions 1, 2 and 4 steps back in the stream, as OCCT does, so the stream must be seekable;
/// <see cref="BinTools.Read(out TopoDS_Shape, Stream, Message_ProgressRange)"/> buffers a stream that isn't.
/// </para>
/// </remarks>
public class BinTools_ShapeSet : BinTools_ShapeSetBase
{
    // index and its shape (started from 1)
    private readonly NCollection_IndexedMap<TopoDS_Shape> myShapes = new(TopTools_ShapeMapHasher.Instance);
    private readonly BinTools_LocationSet myLocations = new();
    private readonly BRep_Builder myBuilder = new();
    private readonly BinTools_SurfaceSet mySurfaces = new();
    private readonly BinTools_CurveSet myCurves = new();
    private readonly BinTools_Curve2dSet myCurves2d = new();
    private readonly NCollection_IndexedMap<Poly_Polygon2D> myPolygons2D = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<Poly_Polygon3D> myPolygons3D = new(ReferenceEqualityComparer.Instance);

    // with a flag telling whether the triangulation's normals are written
    private readonly NCollection_IndexedDataMap<Poly_Triangulation, bool> myTriangulations = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<Poly_PolygonOnTriangulation> myNodes = new(ReferenceEqualityComparer.Instance);

    // OCCT.NET: untrusted input: the nesting depth of each shape read, by index - 1
    private readonly List<int> myDepths = [];

    /// <summary>An empty set, in the current format version, without triangulations.</summary>
    public BinTools_ShapeSet()
    {
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
        myShapes.Clear();
        myLocations.Clear();
        myDepths.Clear();
    }

    /// <summary>
    /// Adds the shape, its sub-shapes (first) and their geometry to the set, and returns its index; 0 for a null shape.
    /// </summary>
    public int Add(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return 0;
        }

        // OCCT.NET: a shape nested too deeply for the stack throws InsufficientExecutionStackException
        System.Runtime.CompilerServices.RuntimeHelpers.EnsureSufficientExecutionStack();

        myLocations.Add(theShape.Location());
        TopoDS_Shape aS2 = theShape;
        aS2.Location(new TopLoc_Location());
        int anIndex = myShapes.FindIndex(aS2);
        if (anIndex == 0)
        {
            AddShape(aS2);
            for (var its = new TopoDS_Iterator(aS2, false, false); its.More(); its.Next())
            {
                Add(its.Value());
            }

            anIndex = myShapes.Add(aS2);
        }

        return anIndex;
    }

    /// <summary>The shape of index I.</summary>
    /// <exception cref="Standard_OutOfRange">There is no shape of index I.</exception>
    public TopoDS_Shape Shape(int theIndx)
    {
        return myShapes[theIndx];
    }

    /// <summary>The index of the shape (its TShape and location), 0 if it isn't in the set.</summary>
    public int Index(in TopoDS_Shape theShape)
    {
        return myShapes.FindIndex(theShape);
    }

    /// <summary>The locations of the set.</summary>
    public BinTools_LocationSet Locations()
    {
        return myLocations;
    }

    /// <summary>The locations of the set, to change.</summary>
    public BinTools_LocationSet ChangeLocations()
    {
        return myLocations;
    }

    /// <summary>The number of shapes.</summary>
    public int NbShapes()
    {
        return myShapes.Extent();
    }

    /// <summary>Adds the geometry of the shape (not its sub-shapes) to the set.</summary>
    public virtual void AddShape(in TopoDS_Shape S)
    {
        // Add the geometry
        if (S.ShapeType() == TopAbs_VERTEX)
        {
            BRep_TVertex TV = (BRep_TVertex)S.TShape()!;
            foreach (BRep_PointRepresentation PR in TV.Points())
            {
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
            }
        }
        else if (S.ShapeType() == TopAbs_EDGE)
        {
            // Add the curve geometry
            BRep_TEdge TE = (BRep_TEdge)S.TShape()!;
            foreach (BRep_CurveRepresentation CR in TE.Curves())
            {
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
                else if (IsWithTriangles())
                {
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
                        // Add() is right: the shapes are added from complex to elementary ones, so a face's
                        // triangulation is added (with its normals flag) before its edges'.
                        if (CR.Triangulation() is null)
                        {
                            // OCCT adds the null handle and dereferences it in WriteTriangulation, which throws (a caught
                            // access violation); the port's maps can't hold null, so it throws here
                            throw new Standard_Failure("BinTools_ShapeSet: a polygon on a null triangulation");
                        }

                        myTriangulations.Add(CR.Triangulation()!, false); // edge triangulation does not need normals
                        myNodes.Add(CR.PolygonOnTriangulation()!);
                        ChangeLocations().Add(CR.Location());
                        if (CR.IsPolygonOnClosedTriangulation())
                        {
                            myNodes.Add(CR.PolygonOnTriangulation2()!);
                        }
                    }
                    else if (CR.IsPolygonOnSurface())
                    {
                        mySurfaces.Add(CR.Surface());
                        // OCCT.NET: the 2D polygons are never written; the port's map can't hold a null one
                        if (CR.Polygon() is not null)
                        {
                            myPolygons2D.Add(CR.Polygon()!);
                        }

                        ChangeLocations().Add(CR.Location());
                        if (CR.IsPolygonOnClosedSurface())
                        {
                            if (CR.Polygon2() is not null)
                            {
                                myPolygons2D.Add(CR.Polygon2()!);
                            }
                        }
                    }
                }
            }
        }
        else if (S.ShapeType() == TopAbs_FACE)
        {
            // Add the surface geometry
            bool needNormals = IsWithNormals();
            BRep_TFace TF = (BRep_TFace)S.TShape()!;
            if (TF.Surface() is not null)
            {
                mySurfaces.Add(TF.Surface());
            }
            else
            {
                needNormals = true;
            }

            if (IsWithTriangles() || TF.Surface() is null)
            {
                Poly_Triangulation? Tr = TF.Triangulation();
                if (Tr is not null)
                {
                    myTriangulations.Add(Tr, needNormals);
                }
            }

            ChangeLocations().Add(TF.Location());
        }
    }

    /// <summary>Writes the geometry of the set: 2D curves, curves, 3D polygons, polygons on triangulations, surfaces, triangulations.</summary>
    public virtual void WriteGeometry(Stream OS, Message_ProgressRange theRange = default)
    {
        using var aPS = new Message_ProgressScope(theRange, "Writing geometry", 6);
        myCurves2d.Write(OS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        myCurves.Write(OS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        WritePolygon3D(OS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        WritePolygonOnTriangulation(OS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        mySurfaces.Write(OS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        WriteTriangulation(OS, aPS.Next());
    }

    /// <summary>
    /// Writes the set: the header line, the locations, the geometry, then the shapes, sub-shapes first, each with its
    /// type, geometry, flags and sub-shapes.
    /// </summary>
    public override void Write(Stream OS, Message_ProgressRange theRange = default)
    {
        // write the copyright
        BinTools.PutText(OS, "\n");
        BinTools.PutText(OS, THE_ASCII_VERSIONS[FormatNb()]);
        BinTools.PutText(OS, "\n");

        //-----------------------------------------
        // write the locations
        //-----------------------------------------

        myLocations.Write(OS);

        //-----------------------------------------
        // write the geometry
        //-----------------------------------------

        using var aPS = new Message_ProgressScope(theRange, "Writing geometry", 2);

        WriteGeometry(OS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        //-----------------------------------------
        // write the shapes
        //-----------------------------------------

        int i, nbShapes = myShapes.Extent();
        using var aPSinner = new Message_ProgressScope(aPS.Next(), "Writing shapes", nbShapes);
        BinTools.PutText(OS, "\nTShapes ");
        BinTools.PutText(OS, nbShapes);
        BinTools.PutText(OS, "\n");

        // subshapes are written first
        for (i = 1; i <= nbShapes && aPSinner.More(); i++, aPSinner.Next())
        {
            TopoDS_Shape S = myShapes[i];

            // Type
            OS.WriteByte((byte)S.ShapeType());

            // Geometry
            WriteShape(S, OS);

            // Flags (OCCT writes no Locked bit in this format)
            BinTools.PutBool(OS, S.Free());
            BinTools.PutBool(OS, S.Modified());
            BinTools.PutBool(OS, S.Checked());
            BinTools.PutBool(OS, S.Orientable());
            BinTools.PutBool(OS, S.Closed());
            BinTools.PutBool(OS, S.Infinite());
            BinTools.PutBool(OS, S.Convex());

            // sub-shapes
            for (var its = new TopoDS_Iterator(S, false, false); its.More(); its.Next())
            {
                Write(its.Value(), OS);
            }

            Write(new TopoDS_Shape(), OS); // Null shape to end the list
        }
    }

    /// <summary>
    /// Clears the set and reads one written by <see cref="Write(Stream, Message_ProgressRange)"/>. The shapes are frozen.
    /// If the stream has no header line of a known version, writes a message to the console and returns, as OCCT does.
    /// </summary>
    /// <exception cref="Standard_Failure">The stream isn't a binary shape set OCCT can read; for input that can't be
    /// read safely, a <see cref="Storage_StreamFormatError"/>.</exception>
    public override void Read(Stream IS, Message_ProgressRange theRange = default)
    {
        try
        {
            ReadSet(IS, theRange);
        }
        catch (Exception anException) when (BinTools.IsSignal(anException))
        {
            // OCCT.NET: untrusted input: a malformed stream raised an exception that OCCT doesn't catch here
            throw new Storage_StreamFormatError("BinTools_ShapeSet::Read: " + anException.Message, anException);
        }
        finally
        {
            // design 4.2: the shapes freeze once the whole set is read; on a failure too, since the set hands them out
            for (int i = 1; i <= myShapes.Extent(); i++)
            {
                myShapes[i].TShape()!.FreezeGraph();
            }
        }
    }

    private void ReadSet(Stream IS, Message_ProgressRange theRange)
    {
        Clear();

        // Check the version
        bool anIsSetFormat = false;
        bool isFail;
        bool isEof = false;
        do
        {
            isFail = !BinTools.GetLine(IS, out string vers, ref isEof);

            // OCCT compares C strings
            int aNul = vers.IndexOf('\0', StringComparison.Ordinal);
            if (aNul >= 0)
            {
                vers = vers[..aNul];
            }

            // BUC60769 PTV 18.10.2000: remove possible '\r' at the end of the line
            int lv = vers.Length;
            if (lv > 0)
            {
                for (lv--; lv > 0 && (vers[lv] == '\r' || vers[lv] == '\n'); lv--)
                {
                    vers = vers[..lv];
                }
            }

            for (int i = (int)BinTools_FormatVersionBounds.BinTools_FormatVersion_LOWER;
                 i <= (int)BinTools_FormatVersionBounds.BinTools_FormatVersion_UPPER;
                 ++i)
            {
                if (vers == THE_ASCII_VERSIONS[i])
                {
                    SetFormatNb(i);
                    anIsSetFormat = true;
                    break;
                }
            }

            if (anIsSetFormat)
            {
                break;
            }
        } while (!isFail);

        if (isFail)
        {
            Console.Out.Write("BinTools_ShapeSet::Read: File was not written with this version of the topology" + "\n");
            return;
        }

        //-----------------------------------------
        // read the locations
        //-----------------------------------------

        myLocations.Read(IS);

        //-----------------------------------------
        // read the geometry
        //-----------------------------------------

        using var aPSouter = new Message_ProgressScope(theRange, "Reading", 2);
        ReadGeometry(IS, aPSouter.Next());
        if (!aPSouter.More())
        {
            return;
        }

        //-----------------------------------------
        // read the shapes
        //-----------------------------------------

        string? buffer = BinTools.ReadWord(IS);
        if (buffer is null || buffer != "TShapes")
        {
            throw new Standard_Failure("BinTools_ShapeSet::Read: Not a TShape table" + "\n");
        }

        int nbShapes = BinTools.ReadFormattedInteger(IS);
        BinTools.Get(IS); // remove lf
        // OCCT.NET: untrusted input: a shape takes at least 9 bytes (its type, 7 flags and the end of its sub-shapes)
        BinTools.CheckCount(IS, nbShapes, 9, "shapes");
        using var aPSinner = new Message_ProgressScope(aPSouter.Next(), "Reading Shapes", nbShapes);
        for (int i = 1; i <= nbShapes && aPSinner.More(); i++, aPSinner.Next())
        {
            TopoDS_Shape S = new();
            TopAbs_ShapeEnum T = (TopAbs_ShapeEnum)BinTools.Get(IS);
            ReadShape(T, IS, ref S);
            ReadFlagsAndSubs(ref S, T, IS, nbShapes);
            myShapes.Add(S);

            // OCCT.NET: untrusted input: cap the nesting, which recursive writers and explorers follow
            int aDepth = 1;
            for (var its = new TopoDS_Iterator(S, false, false); its.More(); its.Next())
            {
                int aSub = myShapes.FindIndex(its.Value().Located(new TopLoc_Location()));
                aDepth = Math.Max(aDepth, myDepths[aSub - 1] + 1);
            }

            if (aDepth > BinTools.MaxShapeNesting)
            {
                throw new Storage_StreamFormatError("BinTools_ShapeSet::Read: shapes nested too deeply");
            }

            myDepths.Add(aDepth);
        }
    }

    /// <summary>Writes a reference to the shape: its orientation, the index of its TShape and of its location.</summary>
    public override void Write(in TopoDS_Shape S, Stream OS)
    {
        if (S.IsNull())
        {
            OS.WriteByte((byte)'*');
        }
        else
        {
            // {TopAbs_FORWARD, TopAbs_REVERSED, TopAbs_INTERNAL, TopAbs_EXTERNAL}
            OS.WriteByte((byte)S.Orientation());
            BinTools.PutInteger(OS, myShapes.Extent() - myShapes.FindIndex(S.Located(new TopLoc_Location())) + 1);
            BinTools.PutInteger(OS, Locations().Index(S.Location()));
        }
    }

    // Reads the flags and the sub-shapes of a shape just created by ReadShape
    internal virtual void ReadFlagsAndSubs(ref TopoDS_Shape S, TopAbs_ShapeEnum T, Stream IS, int nbShapes)
    {
        // Set the flags
        BinTools.GetBool(IS, out bool aFree);
        BinTools.GetBool(IS, out bool aMod);
        BinTools.GetBool(IS, out bool aChecked);
        BinTools.GetBool(IS, out bool anOrient);
        BinTools.GetBool(IS, out bool aClosed);
        BinTools.GetBool(IS, out bool anInf);
        BinTools.GetBool(IS, out bool aConv);

        // sub-shapes
        TopoDS_Shape SS = new();
        do
        {
            ReadSubs(ref SS, IS, nbShapes);
            if (!SS.IsNull())
            {
                AddShapes(ref S, SS);
            }
        } while (!SS.IsNull());

        S.Free(aFree);
        S.Modified(aMod);
        if (FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_1)
        {
            aChecked = false; // force check at reading
        }

        S.Checked(aChecked);
        S.Orientable(anOrient);
        S.Closed(aClosed);
        S.Infinite(anInf);
        S.Convex(aConv);

        // check
        if (FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_1)
        {
            if (T == TopAbs_FACE)
            {
                TopoDS_Face F = TopoDS.Face(S);
                BRepTools.Update(F);
            }
        }
    }

    /// <summary>
    /// Reads a reference to a shape written by <see cref="Write(in TopoDS_Shape, Stream)"/>: one of the
    /// <paramref name="nbshapes"/> shapes of the set, or a null shape for the end of a list.
    /// </summary>
    public virtual void ReadSubs(ref TopoDS_Shape S, Stream IS, int nbshapes)
    {
        int aChar = BinTools.ReadFormattedChar(IS);
        if (aChar == '*')
        {
            S = new TopoDS_Shape();
        }
        else
        {
            TopAbs_Orientation anOrient;
            // OCCT.NET: untrusted input: OCCT casts any character
            anOrient = BinTools.CheckOrientation(aChar);
            BinTools.GetInteger(IS, out int anIndx);
            long aShapeIndex = (long)nbshapes - anIndx + 1;
            // OCCT.NET: untrusted input: OCCT reads outside its map
            BinTools.CheckIndex(aShapeIndex is >= int.MinValue and <= int.MaxValue ? (int)aShapeIndex : 0,
                                1,
                                myShapes.Extent(),
                                "shape");
            S = Shape((int)aShapeIndex);
            S.Orientation(anOrient);

            BinTools.GetInteger(IS, out int l);
            // OCCT.NET: untrusted input
            BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");
            S.Location(myLocations.Location(l), false);
        }
    }

    /// <summary>Reads the geometry of the set, as <see cref="WriteGeometry"/> writes it.</summary>
    public virtual void ReadGeometry(Stream IS, Message_ProgressRange theRange = default)
    {
        using var aPS = new Message_ProgressScope(theRange, "Reading geometry", 6);
        myCurves2d.Read(IS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        myCurves.Read(IS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        ReadPolygon3D(IS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        ReadPolygonOnTriangulation(IS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        mySurfaces.Read(IS, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        ReadTriangulation(IS, aPS.Next());
    }

    /// <summary>Writes the geometry of the shape (not its sub-shapes).</summary>
    public virtual void WriteShape(in TopoDS_Shape S, Stream OS)
    {
        // Write the geometry
        try
        {
            if (S.ShapeType() == TopAbs_VERTEX)
            {
                // Write the point geometry
                TopoDS_Vertex V = TopoDS.Vertex(S);
                BinTools.PutReal(OS, BRep_Tool.Tolerance(V));
                gp_Pnt p = BRep_Tool.Pnt(V);
                Write(OS, p);

                BRep_TVertex TV = (BRep_TVertex)S.TShape()!;
                foreach (BRep_PointRepresentation PR in TV.Points())
                {
                    if (PR.IsPointOnCurve())
                    {
                        OS.WriteByte(1); // 1
                        BinTools.PutReal(OS, PR.Parameter());
                        BinTools.PutInteger(OS, myCurves.Index(PR.Curve()));
                    }
                    else if (PR.IsPointOnCurveOnSurface())
                    {
                        OS.WriteByte(2); // 2
                        BinTools.PutReal(OS, PR.Parameter());
                        BinTools.PutInteger(OS, myCurves2d.Index(PR.PCurve()));
                        BinTools.PutInteger(OS, mySurfaces.Index(PR.Surface()));
                    }
                    else if (PR.IsPointOnSurface())
                    {
                        // OCCT writes the second parameter first, and the readers take it as the first: the parameters
                        // of a point on a surface are swapped on each round trip (occt-issues.md)
                        OS.WriteByte(3); // 3
                        BinTools.PutReal(OS, PR.Parameter2());
                        BinTools.PutReal(OS, PR.Parameter());
                        BinTools.PutInteger(OS, mySurfaces.Index(PR.Surface()));
                    }

                    BinTools.PutInteger(OS, Locations().Index(PR.Location()));
                }

                OS.WriteByte(0);
            }
            else if (S.ShapeType() == TopAbs_EDGE)
            {
                // Write the curve geometry
                BRep_TEdge TE = (BRep_TEdge)S.TShape()!;

                BinTools.PutReal(OS, TE.Tolerance());

                bool aVal = TE.SameParameter();
                BinTools.PutBool(OS, aVal);
                aVal = TE.SameRange();
                BinTools.PutBool(OS, aVal);
                aVal = TE.Degenerated();
                BinTools.PutBool(OS, aVal);

                double first, last;
                foreach (BRep_CurveRepresentation CR in TE.Curves())
                {
                    if (CR.IsCurve3D())
                    {
                        if (CR.Curve3D() is not null)
                        {
                            BRep_GCurve GC = (BRep_GCurve)CR;
                            GC.Range(out first, out last);
                            OS.WriteByte(1); // CURVE_3D;
                            BinTools.PutInteger(OS, myCurves.Index(CR.Curve3D()));
                            BinTools.PutInteger(OS, Locations().Index(CR.Location()));
                            BinTools.PutReal(OS, first);
                            BinTools.PutReal(OS, last);
                        }
                    }
                    else if (CR.IsCurveOnSurface())
                    {
                        BRep_GCurve GC = (BRep_GCurve)CR;
                        GC.Range(out first, out last);
                        if (!CR.IsCurveOnClosedSurface())
                        {
                            // -2- Curve on surf
                            OS.WriteByte(2);
                        }
                        else
                        {
                            // -3- Curve on closed surf
                            OS.WriteByte(3);
                        }

                        BinTools.PutInteger(OS, myCurves2d.Index(CR.PCurve()));
                        if (CR.IsCurveOnClosedSurface())
                        {
                            //+ int|char
                            BinTools.PutInteger(OS, myCurves2d.Index(CR.PCurve2()));
                            OS.WriteByte((byte)CR.Continuity());
                        }

                        BinTools.PutInteger(OS, mySurfaces.Index(CR.Surface()));
                        BinTools.PutInteger(OS, Locations().Index(CR.Location()));
                        BinTools.PutReal(OS, first);
                        BinTools.PutReal(OS, last);

                        // Write UV Points for higher performance
                        if (FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_2
                            || FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_3)
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

                            BinTools.PutReal(OS, Pf.X());
                            BinTools.PutReal(OS, Pf.Y());
                            BinTools.PutReal(OS, Pl.X());
                            BinTools.PutReal(OS, Pl.Y());
                        }
                    }
                    else if (CR.IsRegularity())
                    {
                        // -4- Regularity
                        OS.WriteByte(4);
                        OS.WriteByte((byte)CR.Continuity());
                        BinTools.PutInteger(OS, mySurfaces.Index(CR.Surface()));
                        BinTools.PutInteger(OS, Locations().Index(CR.Location()));
                        BinTools.PutInteger(OS, mySurfaces.Index(CR.Surface2()));
                        BinTools.PutInteger(OS, Locations().Index(CR.Location2()));
                    }
                    else if (IsWithTriangles())
                    {
                        if (CR.IsPolygon3D())
                        {
                            BRep_Polygon3D GC = (BRep_Polygon3D)CR;
                            if (GC.Polygon3D() is not null)
                            {
                                // -5- Polygon3D
                                OS.WriteByte(5);
                                BinTools.PutInteger(OS, myPolygons3D.FindIndex(CR.Polygon3D()!));
                                BinTools.PutInteger(OS, Locations().Index(CR.Location()));
                            }
                        }
                        else if (CR.IsPolygonOnTriangulation())
                        {
                            BRep_PolygonOnTriangulation PT = (BRep_PolygonOnTriangulation)CR;
                            if (!CR.IsPolygonOnClosedTriangulation())
                            {
                                // -6- Polygon on triangulation
                                OS.WriteByte(6);
                            }
                            else
                            {
                                // -7- Polygon on closed triangulation
                                OS.WriteByte(7);
                            }

                            BinTools.PutInteger(OS, myNodes.FindIndex(PT.PolygonOnTriangulation()!));

                            if (CR.IsPolygonOnClosedTriangulation())
                            {
                                BinTools.PutInteger(OS, myNodes.FindIndex(PT.PolygonOnTriangulation2()!));
                            }

                            BinTools.PutInteger(OS, myTriangulations.FindIndex(PT.Triangulation()!));
                            BinTools.PutInteger(OS, Locations().Index(CR.Location()));
                        }
                    }
                }

                OS.WriteByte(0); // end of the list of representations
            }
            else if (S.ShapeType() == TopAbs_FACE)
            {
                BRep_TFace TF = (BRep_TFace)S.TShape()!;
                TopoDS_Face F = TopoDS.Face(S);

                // Write the surface geometry
                bool aNatRes = BRep_Tool.NaturalRestriction(F);
                BinTools.PutBool(OS, aNatRes);
                BinTools.PutReal(OS, TF.Tolerance());
                BinTools.PutInteger(OS, TF.Surface() is not null ? mySurfaces.Index(TF.Surface()) : 0);
                BinTools.PutInteger(OS, Locations().Index(TF.Location()));

                if (IsWithTriangles() || TF.Surface() is null)
                {
                    if (TF.Triangulation() is not null)
                    {
                        OS.WriteByte(2);
                        // Write the triangulation
                        BinTools.PutInteger(OS, myTriangulations.FindIndex(TF.Triangulation()!));
                    }
                    else
                    {
                        OS.WriteByte(1);
                    }
                }
                else
                {
                    OS.WriteByte(0); // without triangulation
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::WriteGeometry(S,OS)", anException);
        }
    }

    // Reads the geometry of a shape of type T into a new shape S (not frozen: the set freezes it after reading)
    internal virtual void ReadShape(TopAbs_ShapeEnum T, Stream IS, ref TopoDS_Shape S)
    {
        // Read the geometry
        int val, c, pc, pc2 = 0, s, s2, l, l2, t, pt, pt2 = 0;
        double tol, X, Y, Z, first, last, p1 = 0.0, p2;
        double PfX, PfY, PlX, PlY;
        gp_Pnt2d aPf = new(), aPl = new();
        bool closed, bval;
        GeomAbs_Shape reg = GeomAbs_Shape.GeomAbs_C0;
        try
        {
            switch (T)
            {
                //---------
                // vertex
                //---------
                case TopAbs_VERTEX:
                {
                    TopoDS_Vertex V = new();
                    // Read the point geometry
                    BinTools.GetReal(IS, out tol);
                    BinTools.GetReal(IS, out X);
                    BinTools.GetReal(IS, out Y);
                    BinTools.GetReal(IS, out Z);
                    gp_Pnt aPnt = new(X, Y, Z);
                    myBuilder.MakeVertex(ref V, aPnt, tol);
                    S = V;
                    BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
                    NCollection_List<BRep_PointRepresentation> lpr = TV.ChangePoints();
                    TopLoc_Location L = new();

                    do
                    {
                        if (FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_3)
                        {
                            val = BinTools.Get(IS); // case {0|1|2|3}
                            if (val > 0 && val <= 3)
                            {
                                BinTools.GetReal(IS, out p1);
                            }
                        }
                        else
                        {
                            // OCCT (versions 1, 2 and 4): reads the parameter before the type, as older files store
                            // them, and steps back when the byte after it isn't a type. A version 4 file stores the
                            // type first, so a parameter whose last byte is 1, 2 or 3 (a positive real below about
                            // 1e-303) is misread (occt-issues.md).
                            if (!IS.CanSeek)
                            {
                                throw new Standard_ProgramError("BinTools_ShapeSet: reading a vertex needs a seekable stream");
                            }

                            long aPos = IS.Position;
                            BinTools.GetReal(IS, out p1);
                            val = BinTools.Get(IS); // case {0|1|2|3}
                            if (val != 1 && val != 2 && val != 3)
                            {
                                IS.Seek(aPos, SeekOrigin.Begin);
                                val = BinTools.Get(IS);
                                if (val > 0 && val <= 3)
                                {
                                    BinTools.GetReal(IS, out p1);
                                }
                            }
                        }

                        BRep_PointRepresentation? PR = null;
                        switch (val)
                        {
                            case 0:
                                break;

                            case 1:
                            {
                                BinTools.GetInteger(IS, out c);
                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(c, 0, myCurves.NbCurves(), "curve");
                                if (myCurves.Curve(c) is null)
                                {
                                    break;
                                }

                                BRep_PointOnCurve POC = new(p1, myCurves.Curve(c), L);
                                PR = POC;
                            }
                                break;

                            case 2:
                            {
                                BinTools.GetInteger(IS, out pc);
                                BinTools.GetInteger(IS, out s);
                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(pc, 1, myCurves2d.NbCurves(), "2D curve");
                                BinTools.CheckIndex(s, 1, mySurfaces.NbSurfaces(), "surface");
                                if (myCurves2d.Curve2d(pc) is null || mySurfaces.Surface(s) is null)
                                {
                                    break;
                                }

                                BRep_PointOnCurveOnSurface POC = new(p1, myCurves2d.Curve2d(pc), mySurfaces.Surface(s), L);
                                PR = POC;
                            }
                                break;

                            case 3:
                            {
                                BinTools.GetReal(IS, out p2);
                                BinTools.GetInteger(IS, out s);
                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(s, 1, mySurfaces.NbSurfaces(), "surface");
                                if (mySurfaces.Surface(s) is null)
                                {
                                    break;
                                }

                                // OCCT: p1 is the second parameter written (see WriteShape)
                                BRep_PointOnSurface POC = new(p1, p2, mySurfaces.Surface(s), L);
                                PR = POC;
                            }
                                break;

                            default:
                            {
                                throw new Standard_Failure("BinTools_SurfaceSet::ReadGeometry: UnExpected BRep_PointRepresentation = "
                                                           + val + "\n");
                            }
                        }

                        if (val > 0)
                        {
                            BinTools.GetInteger(IS, out l); // Locations index
                            // OCCT.NET: untrusted input
                            BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");

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
                {
                    // Create an edge
                    TopoDS_Edge E = new();
                    myBuilder.MakeEdge(ref E);
                    S = E;

                    // Read the curve geometry
                    BinTools.GetReal(IS, out tol);
                    BinTools.GetBool(IS, out bval);
                    myBuilder.SameParameter(E, bval);

                    BinTools.GetBool(IS, out bval);
                    myBuilder.SameRange(E, bval);

                    BinTools.GetBool(IS, out bval);
                    myBuilder.Degenerated(E, bval);

                    do
                    {
                        val = BinTools.Get(IS); //{0|1|2|3|4|5|6|7}
                        // -0- no representation
                        // -1- Curve 3D
                        // -2- Curve on surf
                        // -3- Curve on closed surf
                        // -4- Regularity
                        // -5- Polygon3D
                        // -6- Polygon on triangulation
                        // -7- Polygon on closed triangulation

                        switch (val)
                        {
                            case 0:
                                break;

                            case 1: // -1- Curve 3D
                                BinTools.GetInteger(IS, out c);
                                BinTools.GetInteger(IS, out l);
                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(c, 0, myCurves.NbCurves(), "curve");
                                BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");
                                if (myCurves.Curve(c) is not null)
                                {
                                    myBuilder.UpdateEdge(E, myCurves.Curve(c), Locations().Location(l), tol);
                                }

                                BinTools.GetReal(IS, out first);
                                BinTools.GetReal(IS, out last);
                                if (myCurves.Curve(c) is not null)
                                {
                                    bool Only3d = true;
                                    myBuilder.Range(E, first, last, Only3d);
                                }

                                break;

                            case 2: // -2- Curve on surf
                            case 3: // -3- Curve on closed surf
                                closed = (val == 3);
                                BinTools.GetInteger(IS, out pc);
                                if (closed)
                                {
                                    BinTools.GetInteger(IS, out pc2);
                                    reg = (GeomAbs_Shape)BinTools.Get(IS);
                                }

                                // surface, location
                                BinTools.GetInteger(IS, out s);
                                BinTools.GetInteger(IS, out l);

                                // range
                                BinTools.GetReal(IS, out first);
                                BinTools.GetReal(IS, out last);

                                // read UV Points // for XML Persistence higher performance
                                if (FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_2
                                    || FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_3)
                                {
                                    BinTools.GetReal(IS, out PfX);
                                    BinTools.GetReal(IS, out PfY);
                                    BinTools.GetReal(IS, out PlX);
                                    BinTools.GetReal(IS, out PlY);
                                    aPf = new gp_Pnt2d(PfX, PfY);
                                    aPl = new gp_Pnt2d(PlX, PlY);
                                }

                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(pc, 1, myCurves2d.NbCurves(), "2D curve");
                                if (closed)
                                {
                                    BinTools.CheckIndex(pc2, 1, myCurves2d.NbCurves(), "2D curve");
                                }

                                BinTools.CheckIndex(s, 1, mySurfaces.NbSurfaces(), "surface");
                                BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");

                                if (myCurves2d.Curve2d(pc) is null || (closed && myCurves2d.Curve2d(pc2) is null)
                                    || mySurfaces.Surface(s) is null)
                                {
                                    break;
                                }

                                if (closed)
                                {
                                    if (FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_2
                                        || FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_3)
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
                                    if (FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_2
                                        || FormatNb() == (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_3)
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
                                reg = (GeomAbs_Shape)BinTools.Get(IS);
                                BinTools.GetInteger(IS, out s);
                                BinTools.GetInteger(IS, out l);
                                BinTools.GetInteger(IS, out s2);
                                BinTools.GetInteger(IS, out l2);
                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(s, 1, mySurfaces.NbSurfaces(), "surface");
                                BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");
                                BinTools.CheckIndex(s2, 1, mySurfaces.NbSurfaces(), "surface");
                                BinTools.CheckIndex(l2, 0, myLocations.NbLocations(), "location");
                                if (mySurfaces.Surface(s) is null || mySurfaces.Surface(s2) is null)
                                {
                                    break;
                                }

                                myBuilder.Continuity(E,
                                                     mySurfaces.Surface(s),
                                                     mySurfaces.Surface(s2),
                                                     Locations().Location(l),
                                                     Locations().Location(l2),
                                                     reg);
                                break;

                            case 5: // -5- Polygon3D
                                BinTools.GetInteger(IS, out c);
                                BinTools.GetInteger(IS, out l);
                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(c, 1, myPolygons3D.Extent(), "3D polygon");
                                BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");
                                //??? Bug?  myBuilder.UpdateEdge(E,myPolygons3D(c));
                                myBuilder.UpdateEdge(E, myPolygons3D[c], Locations().Location(l));
                                break;

                            case 6: // -6- Polygon on triangulation
                            case 7: // -7- Polygon on closed triangulation
                                closed = (val == 7);
                                BinTools.GetInteger(IS, out pt);
                                if (closed)
                                {
                                    BinTools.GetInteger(IS, out pt2);
                                }

                                BinTools.GetInteger(IS, out t);
                                BinTools.GetInteger(IS, out l);
                                // OCCT.NET: untrusted input
                                BinTools.CheckIndex(pt, 1, myNodes.Extent(), "polygon on triangulation");
                                if (closed)
                                {
                                    BinTools.CheckIndex(pt2, 1, myNodes.Extent(), "polygon on triangulation");
                                }

                                BinTools.CheckIndex(t, 1, myTriangulations.Extent(), "triangulation");
                                BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");
                                if (closed)
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myNodes[pt],
                                                         myNodes[pt2],
                                                         myTriangulations.FindKey(t),
                                                         Locations().Location(l));
                                }
                                else
                                {
                                    myBuilder.UpdateEdge(E,
                                                         myNodes[pt],
                                                         myTriangulations.FindKey(t),
                                                         Locations().Location(l));
                                }

                                // range
                                break;

                            default:
                            {
                                throw new Standard_Failure("Unexpected Curve Representation =" + val + "\n");
                            }
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
                    myBuilder.MakeFace(ref F);
                    S = F;
                    BinTools.GetBool(IS, out bval); // NaturalRestriction flag
                    BinTools.GetReal(IS, out tol);
                    BinTools.GetInteger(IS, out s); // surface indx
                    BinTools.GetInteger(IS, out l); // location indx
                    // OCCT.NET: untrusted input
                    if (s > 0)
                    {
                        BinTools.CheckIndex(s, 1, mySurfaces.NbSurfaces(), "surface");
                    }

                    BinTools.CheckIndex(l, 0, myLocations.NbLocations(), "location");
                    myBuilder.UpdateFace(F, s > 0 ? mySurfaces.Surface(s) : null, Locations().Location(l), tol);
                    myBuilder.NaturalRestriction(F, bval);

                    byte aByte = (byte)BinTools.Get(IS);
                    // triangulation case
                    if (aByte == 2)
                    {
                        BinTools.GetInteger(IS, out s);
                        // OCCT.NET: untrusted input
                        BinTools.CheckIndex(s, 1, myTriangulations.Extent(), "triangulation");
                        myBuilder.UpdateFace(F, myTriangulations.FindKey(s));
                    }
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
                {
                    throw new Standard_Failure("Unexpected topology type = " + (int)T + "\n");
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::ReadGeometry(S,OS)", anException);
        }
    }

    // Inserts S2 in S1 (the builder's Add)
    internal virtual void AddShapes(ref TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        myBuilder.Add(S1, S2);
    }

    /// <summary>Writes the polygons on triangulations ("PolygonOnTriangulations", the count, then each polygon).</summary>
    public void WritePolygonOnTriangulation(Stream OS, Message_ProgressRange theRange = default)
    {
        int aNbPol = myNodes.Extent();
        BinTools.PutText(OS, "PolygonOnTriangulations ");
        BinTools.PutText(OS, aNbPol);
        BinTools.PutText(OS, "\n");
        try
        {
            using var aPS = new Message_ProgressScope(theRange, "Writing polygons on triangulation", aNbPol);
            for (int aPolIter = 1; aPolIter <= aNbPol && aPS.More(); ++aPolIter, aPS.Next())
            {
                Poly_PolygonOnTriangulation aPoly = myNodes.FindKey(aPolIter);
                NCollection_Array1<int> aNodes = aPoly.Nodes();
                BinTools.PutInteger(OS, aNodes.Length());
                for (int aNodeIter = 1; aNodeIter <= aNodes.Length(); ++aNodeIter)
                {
                    BinTools.PutInteger(OS, aNodes.Value(aNodeIter));
                }

                // write the deflection
                BinTools.PutReal(OS, aPoly.Deflection());

                // writing parameters
                if (aPoly.Parameters() is NCollection_HArray1<double> aParam)
                {
                    BinTools.PutBool(OS, true);
                    for (int aNodeIter = 1; aNodeIter <= aParam.Length(); ++aNodeIter)
                    {
                        BinTools.PutReal(OS, aParam.Value(aNodeIter));
                    }
                }
                else
                {
                    BinTools.PutBool(OS, false);
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::WritePolygonOnTriangulation(..)", anException);
        }
    }

    /// <summary>Reads the polygons on triangulations written by <see cref="WritePolygonOnTriangulation"/>.</summary>
    public void ReadPolygonOnTriangulation(Stream IS, Message_ProgressRange theRange = default)
    {
        string? aHeader = BinTools.ReadWord(IS);
        if (aHeader is null || !aHeader.Contains("PolygonOnTriangulations", StringComparison.Ordinal))
        {
            throw new Standard_Failure("BinTools_ShapeSet::ReadPolygonOnTriangulation: Not a PolygonOnTriangulation section");
        }

        int aNbPol = BinTools.ReadFormattedInteger(IS);
        BinTools.Get(IS); // remove LF
        // OCCT.NET: untrusted input: a polygon takes at least 13 bytes
        BinTools.CheckCount(IS, aNbPol, 13, "polygons on triangulation");
        try
        {
            using var aPS = new Message_ProgressScope(theRange, "Reading Polygones on triangulation", aNbPol);
            for (int aPolIter = 1; aPolIter <= aNbPol && aPS.More(); ++aPolIter, aPS.Next())
            {
                BinTools.GetInteger(IS, out int aNbNodes);
                // OCCT.NET: untrusted input
                BinTools.CheckCount(IS, aNbNodes, 4, "nodes");
                Poly_PolygonOnTriangulation aPoly = new(aNbNodes, false);
                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    BinTools.GetInteger(IS, out int aNode);
                    aPoly.SetNode(aNodeIter, aNode);
                }

                BinTools.GetReal(IS, out double aDefl);
                aPoly.Deflection(aDefl);

                BinTools.GetBool(IS, out bool hasParameters);
                if (hasParameters)
                {
                    // OCCT.NET: untrusted input
                    BinTools.CheckCount(IS, aNbNodes, 8, "parameters");
                    NCollection_HArray1<double> aParams = new(1, aNbNodes);
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        BinTools.GetReal(IS, out aParams.ChangeValue(aNodeIter));
                    }

                    aPoly.SetParameters(aParams);
                }

                myNodes.Add(aPoly);
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::ReadPolygonOnTriangulation(..)", anException);
        }
    }

    /// <summary>Writes the 3D polygons ("Polygon3D", the count, then each polygon).</summary>
    public void WritePolygon3D(Stream OS, Message_ProgressRange theRange = default)
    {
        int aNbPol = myPolygons3D.Extent();
        BinTools.PutText(OS, "Polygon3D ");
        BinTools.PutText(OS, aNbPol);
        BinTools.PutText(OS, "\n");
        try
        {
            using var aPS = new Message_ProgressScope(theRange, "Writing polygons 3D", aNbPol);
            for (int aPolIter = 1; aPolIter <= aNbPol && aPS.More(); ++aPolIter, aPS.Next())
            {
                Poly_Polygon3D aPoly = myPolygons3D.FindKey(aPolIter);
                BinTools.PutInteger(OS, aPoly.NbNodes());
                BinTools.PutBool(OS, aPoly.HasParameters());

                // write the deflection
                BinTools.PutReal(OS, aPoly.Deflection());

                // write the nodes
                int aNbNodes = aPoly.NbNodes();
                NCollection_Array1<gp_Pnt> aNodes = aPoly.Nodes();
                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    gp_Pnt aPnt = aNodes.Value(aNodeIter);
                    BinTools.PutReal(OS, aPnt.X());
                    BinTools.PutReal(OS, aPnt.Y());
                    BinTools.PutReal(OS, aPnt.Z());
                }

                if (aPoly.HasParameters())
                {
                    NCollection_Array1<double> aParam = aPoly.Parameters();
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        BinTools.PutReal(OS, aParam.Value(aNodeIter));
                    }
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::WritePolygon3D(..)", anException);
        }
    }

    /// <summary>Reads the 3D polygons written by <see cref="WritePolygon3D"/>.</summary>
    public void ReadPolygon3D(Stream IS, Message_ProgressRange theRange = default)
    {
        string? aHeader = BinTools.ReadWord(IS);
        if (aHeader is null || !aHeader.Contains("Polygon3D", StringComparison.Ordinal))
        {
            throw new Standard_Failure("BinTools_ShapeSet::ReadPolygon3D: Not a Polygon3D section");
        }

        int aNbPol = BinTools.ReadFormattedInteger(IS);
        BinTools.Get(IS); // remove LF
        // OCCT.NET: untrusted input: a polygon takes at least 13 bytes
        BinTools.CheckCount(IS, aNbPol, 13, "3D polygons");
        try
        {
            using var aPS = new Message_ProgressScope(theRange, "Reading polygones 3D", aNbPol);
            for (int aPolIter = 1; aPolIter <= aNbPol && aPS.More(); ++aPolIter, aPS.Next())
            {
                BinTools.GetInteger(IS, out int aNbNodes);
                BinTools.GetBool(IS, out bool hasParameters);
                BinTools.GetReal(IS, out double aDefl);
                // OCCT.NET: untrusted input
                BinTools.CheckCount(IS, aNbNodes, hasParameters ? 32 : 24, "nodes");
                Poly_Polygon3D aPoly = new(aNbNodes, hasParameters);
                aPoly.Deflection(aDefl);

                NCollection_Array1<gp_Pnt> aNodes = aPoly.ChangeNodes();
                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    BinTools.GetReal(IS, out double aX);
                    BinTools.GetReal(IS, out double aY);
                    BinTools.GetReal(IS, out double aZ);
                    aNodes[aNodeIter] = new gp_Pnt(aX, aY, aZ);
                }

                if (hasParameters)
                {
                    NCollection_Array1<double> aParam = aPoly.ChangeParameters();
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        BinTools.GetReal(IS, out aParam[aNodeIter]);
                    }
                }

                myPolygons3D.Add(aPoly);
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::ReadPolygon3D(..)", anException);
        }
    }

    /// <summary>Writes the triangulations ("Triangulations", the count, then each triangulation).</summary>
    public void WriteTriangulation(Stream OS, Message_ProgressRange theRange = default)
    {
        int aNbTriangulations = myTriangulations.Extent();
        BinTools.PutText(OS, "Triangulations ");
        BinTools.PutText(OS, aNbTriangulations);
        BinTools.PutText(OS, "\n");

        try
        {
            using var aPS = new Message_ProgressScope(theRange, "Writing triangulation", aNbTriangulations);
            for (int aTriangulationIter = 1; aTriangulationIter <= aNbTriangulations && aPS.More();
                 ++aTriangulationIter, aPS.Next())
            {
                Poly_Triangulation aTriangulation = myTriangulations.FindKey(aTriangulationIter);
                bool NeedToWriteNormals = myTriangulations.FindFromIndex(aTriangulationIter);
                int aNbNodes = aTriangulation.NbNodes();
                int aNbTriangles = aTriangulation.NbTriangles();
                BinTools.PutInteger(OS, aNbNodes);
                BinTools.PutInteger(OS, aNbTriangles);
                BinTools.PutBool(OS, aTriangulation.HasUVNodes());
                if (FormatNb() >= (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_4)
                {
                    BinTools.PutBool(OS, aTriangulation.HasNormals() && NeedToWriteNormals);
                }

                BinTools.PutReal(OS, aTriangulation.Deflection());

                // write the 3d nodes
                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    gp_Pnt aPnt = aTriangulation.Node(aNodeIter);
                    BinTools.PutReal(OS, aPnt.X());
                    BinTools.PutReal(OS, aPnt.Y());
                    BinTools.PutReal(OS, aPnt.Z());
                }

                if (aTriangulation.HasUVNodes())
                {
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        gp_Pnt2d aUV = aTriangulation.UVNode(aNodeIter);
                        BinTools.PutReal(OS, aUV.X());
                        BinTools.PutReal(OS, aUV.Y());
                    }
                }

                for (int aTriIter = 1; aTriIter <= aNbTriangles; ++aTriIter)
                {
                    Poly_Triangle aTri = aTriangulation.Triangle(aTriIter);
                    BinTools.PutInteger(OS, aTri.Value(1));
                    BinTools.PutInteger(OS, aTri.Value(2));
                    BinTools.PutInteger(OS, aTri.Value(3));
                }

                // write the normals
                if (FormatNb() >= (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_4)
                {
                    if (aTriangulation.HasNormals() && NeedToWriteNormals)
                    {
                        for (int aNormalIter = 1; aNormalIter <= aNbNodes; ++aNormalIter)
                        {
                            aTriangulation.Normal(aNormalIter, out NCollection_Vec3<float> aNormal);
                            BinTools.PutShortReal(OS, aNormal.x());
                            BinTools.PutShortReal(OS, aNormal.y());
                            BinTools.PutShortReal(OS, aNormal.z());
                        }
                    }
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::WriteTriangulation(..)", anException);
        }
    }

    /// <summary>Reads the triangulations written by <see cref="WriteTriangulation"/>.</summary>
    public void ReadTriangulation(Stream IS, Message_ProgressRange theRange = default)
    {
        string? aHeader = BinTools.ReadWord(IS);
        if (aHeader is null || !aHeader.Contains("Triangulations", StringComparison.Ordinal))
        {
            throw new Standard_Failure("BinTools_ShapeSet::Triangulation: Not a Triangulation section");
        }

        int aNbTriangulations = BinTools.ReadFormattedInteger(IS);
        BinTools.Get(IS); // remove LF
        // OCCT.NET: untrusted input: a triangulation takes at least 17 bytes
        BinTools.CheckCount(IS, aNbTriangulations, 17, "triangulations");

        try
        {
            using var aPS = new Message_ProgressScope(theRange, "Reading triangulation", aNbTriangulations);
            for (int aTriangulationIter = 1; aTriangulationIter <= aNbTriangulations && aPS.More();
                 ++aTriangulationIter, aPS.Next())
            {
                bool hasNormals = false;
                BinTools.GetInteger(IS, out int aNbNodes);
                BinTools.GetInteger(IS, out int aNbTriangles);
                BinTools.GetBool(IS, out bool hasUV);
                if (FormatNb() >= (int)BinTools_FormatVersion.BinTools_FormatVersion_VERSION_4)
                {
                    BinTools.GetBool(IS, out hasNormals);
                }

                BinTools.GetReal(IS, out double aDefl); // deflection
                // OCCT.NET: untrusted input
                BinTools.CheckCount(IS, aNbNodes, 24 + (hasUV ? 16 : 0) + (hasNormals ? 12 : 0), "nodes");
                BinTools.CheckCount(IS, aNbTriangles, 12, "triangles");
                BinTools.CheckCount(IS,
                                    aNbNodes * (24L + (hasUV ? 16 : 0) + (hasNormals ? 12 : 0)) + aNbTriangles * 12L,
                                    1,
                                    "bytes of a triangulation");
                Poly_Triangulation aTriangulation = new(aNbNodes, aNbTriangles, hasUV, hasNormals);
                aTriangulation.Deflection(aDefl);

                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    BinTools.GetReal(IS, out double aX);
                    BinTools.GetReal(IS, out double aY);
                    BinTools.GetReal(IS, out double aZ);
                    aTriangulation.SetNode(aNodeIter, new gp_Pnt(aX, aY, aZ));
                }

                if (hasUV)
                {
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        BinTools.GetReal(IS, out double aU);
                        BinTools.GetReal(IS, out double aV);
                        aTriangulation.SetUVNode(aNodeIter, new gp_Pnt2d(aU, aV));
                    }
                }

                // read the triangles
                for (int aTriIter = 1; aTriIter <= aNbTriangles; ++aTriIter)
                {
                    BinTools.GetInteger(IS, out int aTriNode0);
                    BinTools.GetInteger(IS, out int aTriNode1);
                    BinTools.GetInteger(IS, out int aTriNode2);
                    aTriangulation.SetTriangle(aTriIter, new Poly_Triangle(aTriNode0, aTriNode1, aTriNode2));
                }

                if (hasNormals)
                {
                    for (int aNormalIter = 1; aNormalIter <= aNbNodes; ++aNormalIter)
                    {
                        BinTools.GetShortReal(IS, out float aX);
                        BinTools.GetShortReal(IS, out float aY);
                        BinTools.GetShortReal(IS, out float aZ);
                        aTriangulation.SetNormal(aNormalIter, new NCollection_Vec3<float>(aX, aY, aZ));
                    }
                }

                myTriangulations.Add(aTriangulation, hasNormals);
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeSet::ReadTriangulation(..)", anException);
        }
    }
}
