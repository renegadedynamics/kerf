// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeWriter.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Writes shapes in OCCT's position-based binary format, which <see cref="BinTools_ShapeReader"/> reads: each shape,
/// location, curve, surface, polygon and triangulation is written once, where it is first met, and referred to after
/// that by its offset back from the reference. OCCT uses it for shapes in binary documents. The writer remembers what
/// it has written across calls of <see cref="Write(in TopoDS_Shape, Stream)"/> until <see cref="Clear"/>.
/// </summary>
/// <remarks>The format has no version and no Locked bit: a shape's flags are Free to Convex, in one byte.</remarks>
public class BinTools_ShapeWriter : BinTools_ShapeSetBase
{
    // positions of the objects already stored
    private readonly NCollection_DataMap<TopoDS_Shape, ulong> myShapePos = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<Placement, ulong> myLocationPos = new();
    private readonly NCollection_DataMap<Geom_Curve, ulong> myCurvePos = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<Geom2d_Curve, ulong> myCurve2dPos = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<Geom_Surface, ulong> mySurfacePos = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<Polygon3D, ulong> myPolygon3dPos = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<PolygonOnTriangulation, ulong> myPolygonPos = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_DataMap<Triangulation, ulong> myTriangulationPos = new(ReferenceEqualityComparer.Instance);

    /// <summary>A writer that has written nothing yet.</summary>
    public BinTools_ShapeWriter()
    {
    }

    /// <summary>Forgets what has been written.</summary>
    public override void Clear()
    {
        base.Clear();
        myShapePos.Clear();
        myLocationPos.Clear();
        myCurvePos.Clear();
        myCurve2dPos.Clear();
        mySurfacePos.Clear();
        myPolygon3dPos.Clear();
        myPolygonPos.Clear();
        myTriangulationPos.Clear();
    }

    /// <summary>Writes the shape, or a reference to it if it has been written already.</summary>
    public override void Write(in TopoDS_Shape theShape, Stream theStream)
    {
        var anOStream = new BinTools_OStream(theStream);
        WriteShape(anOStream, theShape);
    }

    /// <summary>Writes the location, or a reference to it if it has been written already.</summary>
    public virtual void WriteLocation(BinTools_OStream theStream, in Placement theLocation)
    {
        if (theLocation.IsIdentity)
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptyLocation;
            return;
        }

        if (myLocationPos.Find(theLocation, out ulong aLoc))
        {
            theStream.WriteReference(aLoc);
            return;
        }

        ulong aNewLoc = theStream.Position();
        try
        {
            Placement aL2 = theLocation.NextLocation();
            bool isSimple = aL2.IsIdentity;
            int aPower = theLocation.FirstPower();
            Placement aL1 = new(theLocation.FirstDatum());
            bool elementary = (isSimple && aPower == 1);
            if (elementary)
            {
                theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_SimpleLocation << theLocation.Transformation();
            }
            else
            {
                theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_Location;
                WriteLocation(theStream, aL1);
                theStream = theStream << aPower;
                while (!aL2.IsIdentity)
                {
                    aL1 = new Placement(aL2.FirstDatum());
                    aPower = aL2.FirstPower();
                    aL2 = aL2.NextLocation();
                    WriteLocation(theStream, aL1);
                    theStream = theStream << aPower;
                }

                theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_LocationEnd;
            }

            myLocationPos.Bind(theLocation, aNewLoc);
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeWriter::WriteLocation", anException);
        }
    }

    // Writes the shape (type, orientation, location, geometry, flags, sub-shapes), or a reference to it
    private void WriteShape(BinTools_OStream theStream, in TopoDS_Shape theShape)
    {
        // OCCT.NET: a shape nested too deeply for the stack throws InsufficientExecutionStackException
        System.Runtime.CompilerServices.RuntimeHelpers.EnsureSufficientExecutionStack();

        if (theShape.IsNull())
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptyShape;
            return;
        }

        TopoDS_Shape aShape = theShape.Located(new Placement());
        if (myShapePos.Find(aShape, out ulong anExisting))
        {
            // shape is already there, so, write reference to it
            theStream.WriteReference(anExisting);
            WriteLocation(theStream, theShape.Location());
            theStream = theStream << (byte)theShape.Orientation();
            return;
        }

        ulong aNewPos = theStream.Position();
        myShapePos.Bind(aShape, aNewPos);
        theStream.WriteShape(aShape.ShapeType(), aShape.Orientation());
        WriteLocation(theStream, theShape.Location());

        // OCCT.NET: the geometry is written by a separate method (a smaller frame for the recursion)
        WriteShapeGeometry(theStream, aShape);

        theStream.PutBools(aShape.Free(),
                           aShape.Modified(),
                           aShape.Checked(),
                           aShape.Orientable(),
                           aShape.Closed(),
                           aShape.Infinite(),
                           aShape.Convex());

        // process sub-shapes
        for (var aSub = new TopoDS_Iterator(aShape, false, false); aSub.More(); aSub.Next())
        {
            WriteShape(theStream, aSub.Value());
        }

        theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EndShape;
    }

    // The geometry of a shape, apart from WriteShape so that its recursion keeps a small stack frame
    private void WriteShapeGeometry(BinTools_OStream theStream, in TopoDS_Shape aShape)
    {
        try
        {
            switch (aShape.ShapeType())
            {
                case TopAbs_VERTEX:
                {
                    TopoDS_Vertex aV = TopoDS.Vertex(aShape);
                    theStream = theStream << BRep_Tool.Tolerance(aV);
                    Point3D aP = BRep_Tool.Pnt(aV);
                    theStream = theStream << aP;
                    BRep_TVertex aTV = (BRep_TVertex)aShape.TShape()!;
                    foreach (BRep_PointRepresentation aPR in aTV.Points())
                    {
                        if (aPR.IsPointOnCurve())
                        {
                            theStream = theStream << (byte)1; // 1
                            theStream = theStream << aPR.Parameter();
                            WriteCurve(theStream, aPR.Curve());
                        }
                        else if (aPR.IsPointOnCurveOnSurface())
                        {
                            theStream = theStream << (byte)2; // 2
                            theStream = theStream << aPR.Parameter();
                            WriteCurve(theStream, aPR.PCurve());
                            WriteSurface(theStream, aPR.Surface());
                        }
                        else if (aPR.IsPointOnSurface())
                        {
                            // OCCT: the second parameter first, read back as the first (occt-issues.md)
                            theStream = theStream << (byte)3; // 3
                            theStream = theStream << aPR.Parameter2() << aPR.Parameter();
                            WriteSurface(theStream, aPR.Surface());
                        }

                        WriteLocation(theStream, aPR.Location());
                    }

                    theStream = theStream << (byte)0;
                }
                    break;

                case TopAbs_EDGE:
                {
                    BRep_TEdge aTE = (BRep_TEdge)aShape.TShape()!;
                    theStream = theStream << aTE.Tolerance();
                    theStream.PutBools(aTE.SameParameter(), aTE.SameRange(), aTE.Degenerated());
                    double aFirst, aLast;
                    foreach (BRep_CurveRepresentation aCR in aTE.Curves())
                    {
                        if (aCR.IsCurve3D())
                        {
                            if (aCR.Curve3D() is not null)
                            {
                                BRep_GCurve aGC = (BRep_GCurve)aCR;
                                aGC.Range(out aFirst, out aLast);
                                theStream = theStream << (byte)1; // -1- CURVE_3D;
                                WriteCurve(theStream, aCR.Curve3D());
                                WriteLocation(theStream, aCR.Location());
                                theStream = theStream << aFirst << aLast;
                            }
                        }
                        else if (aCR.IsCurveOnSurface())
                        {
                            BRep_GCurve GC = (BRep_GCurve)aCR;
                            GC.Range(out aFirst, out aLast);
                            if (!aCR.IsCurveOnClosedSurface())
                            {
                                theStream = theStream << (byte)2; // -2- Curve on surf
                            }
                            else
                            {
                                theStream = theStream << (byte)3; // -3- Curve on closed surf
                            }

                            WriteCurve(theStream, aCR.PCurve());
                            if (aCR.IsCurveOnClosedSurface())
                            {
                                //+ int|char
                                WriteCurve(theStream, aCR.PCurve2());
                                theStream = theStream << (byte)aCR.Continuity();
                            }

                            WriteSurface(theStream, aCR.Surface());
                            WriteLocation(theStream, aCR.Location());
                            theStream = theStream << aFirst << aLast;
                        }
                        else if (aCR.IsRegularity())
                        {
                            theStream = theStream << (byte)4; // -4- Regularity
                            theStream = theStream << (byte)aCR.Continuity();
                            WriteSurface(theStream, aCR.Surface());
                            WriteLocation(theStream, aCR.Location());
                            WriteSurface(theStream, aCR.Surface2());
                            WriteLocation(theStream, aCR.Location2());
                        }
                        else if (IsWithTriangles())
                        {
                            if (aCR.IsPolygon3D())
                            {
                                BRep_Polygon3D aGC = (BRep_Polygon3D)aCR;
                                if (aGC.Polygon3D() is not null)
                                {
                                    theStream = theStream << (byte)5; // -5- Polygon3D
                                    WritePolygon(theStream, aCR.Polygon3D());
                                    WriteLocation(theStream, aCR.Location());
                                }
                            }
                            else if (aCR.IsPolygonOnTriangulation())
                            {
                                BRep_PolygonOnTriangulation aPT = (BRep_PolygonOnTriangulation)aCR;
                                if (!aCR.IsPolygonOnClosedTriangulation())
                                {
                                    theStream = theStream << (byte)6; // -6- Polygon on triangulation
                                }
                                else
                                {
                                    theStream = theStream << (byte)7; // -7- Polygon on closed triangulation
                                }

                                WritePolygon(theStream, aPT.PolygonOnTriangulation());
                                if (aCR.IsPolygonOnClosedTriangulation())
                                {
                                    WritePolygon(theStream, aPT.PolygonOnTriangulation2());
                                }

                                // edge triangulation does not need normals
                                WriteTriangulation(theStream, aPT.Triangulation(), false);
                                WriteLocation(theStream, aCR.Location());
                            }
                        }
                    }

                    theStream = theStream << (byte)0;
                }
                    break;

                case TopAbs_FACE:
                {
                    BRep_TFace aTF = (BRep_TFace)aShape.TShape()!;
                    TopoDS_Face aF = TopoDS.Face(aShape);

                    // Write the surface geometry
                    theStream = theStream << BRep_Tool.NaturalRestriction(aF) << aTF.Tolerance();
                    WriteSurface(theStream, aTF.Surface());
                    WriteLocation(theStream, aTF.Location());

                    if (IsWithTriangles() || aTF.Surface() is null)
                    {
                        if (aTF.Triangulation() is not null)
                        {
                            theStream = theStream << (byte)2;
                            WriteTriangulation(theStream,
                                               aTF.Triangulation(),
                                               aTF.Triangulation()!.HasNormals
                                                   && (IsWithNormals() || aTF.Surface() is null));
                        }
                        else
                        {
                            theStream = theStream << (byte)1;
                        }
                    }
                    else
                    {
                        theStream = theStream << (byte)0; // without triangulation
                    }
                }
                    break;

                default:
                {
                }
                    break;
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeWriter::WriteShape", anException);
        }
    }

    private void WriteCurve(BinTools_OStream theStream, Geom_Curve? theCurve)
    {
        if (theCurve is null)
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptyCurve;
            return;
        }

        if (myCurvePos.Find(theCurve, out ulong aCurve))
        {
            theStream.WriteReference(aCurve);
            return;
        }

        myCurvePos.Bind(theCurve, theStream.Position());
        theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_Curve;
        BinTools_CurveSet.WriteCurve(theCurve, theStream);
    }

    private void WriteCurve(BinTools_OStream theStream, Geom2d_Curve? theCurve)
    {
        if (theCurve is null)
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptyCurve2d;
            return;
        }

        if (myCurve2dPos.Find(theCurve, out ulong aCurve))
        {
            theStream.WriteReference(aCurve);
            return;
        }

        myCurve2dPos.Bind(theCurve, theStream.Position());
        theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_Curve2d;
        BinTools_Curve2dSet.WriteCurve2d(theCurve, theStream);
    }

    private void WriteSurface(BinTools_OStream theStream, Geom_Surface? theSurface)
    {
        if (theSurface is null)
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptySurface;
            return;
        }

        if (mySurfacePos.Find(theSurface, out ulong aSurface))
        {
            theStream.WriteReference(aSurface);
            return;
        }

        mySurfacePos.Bind(theSurface, theStream.Position());
        theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_Surface;
        BinTools_SurfaceSet.WriteSurface(theSurface, theStream);
    }

    private void WritePolygon(BinTools_OStream theStream, Polygon3D? thePolygon)
    {
        if (thePolygon is null)
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptyPolygon3d;
            return;
        }

        if (myPolygon3dPos.Find(thePolygon, out ulong aPolygon))
        {
            theStream.WriteReference(aPolygon);
            return;
        }

        myPolygon3dPos.Bind(thePolygon, theStream.Position());
        theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_Polygon3d;

        int aNbNodes = thePolygon.NodeCount;
        theStream = theStream << aNbNodes << thePolygon.HasParameters << thePolygon.Deflection();
        NCollection_Array1<Point3D> aNodes = thePolygon.Nodes();
        for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
        {
            theStream = theStream << aNodes.Value(aNodeIter);
        }

        if (thePolygon.HasParameters)
        {
            NCollection_Array1<double> aParam = thePolygon.Parameters();
            for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
            {
                theStream = theStream << aParam.Value(aNodeIter);
            }
        }
    }

    private void WritePolygon(BinTools_OStream theStream, PolygonOnTriangulation? thePolygon)
    {
        if (thePolygon is null)
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptyPolygonOnTriangulation;
            return;
        }

        if (myPolygonPos.Find(thePolygon, out ulong aPolygon))
        {
            theStream.WriteReference(aPolygon);
            return;
        }

        myPolygonPos.Bind(thePolygon, theStream.Position());
        theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_PolygonOnTriangulation;

        NCollection_Array1<int> aNodes = thePolygon.Nodes();
        theStream = theStream << aNodes.Length();
        for (int aNodeIter = 1; aNodeIter <= aNodes.Length(); ++aNodeIter)
        {
            theStream = theStream << aNodes.Value(aNodeIter);
        }

        theStream = theStream << thePolygon.Deflection();
        if (thePolygon.Parameters() is NCollection_HArray1<double> aParam)
        {
            theStream = theStream << true;
            for (int aNodeIter = 1; aNodeIter <= aParam.Length(); ++aNodeIter)
            {
                theStream = theStream << aParam.Value(aNodeIter);
            }
        }
        else
        {
            theStream = theStream << false;
        }
    }

    private void WriteTriangulation(BinTools_OStream theStream, Triangulation? theTriangulation, bool theNeedToWriteNormals)
    {
        if (theTriangulation is null)
        {
            theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_EmptyTriangulation;
            return;
        }

        if (myTriangulationPos.Find(theTriangulation, out ulong aTriangulation))
        {
            theStream.WriteReference(aTriangulation);
            return;
        }

        myTriangulationPos.Bind(theTriangulation, theStream.Position());
        theStream = theStream << BinTools_ObjectType.BinTools_ObjectType_Triangulation;

        int aNbNodes = theTriangulation.NodeCount;
        int aNbTriangles = theTriangulation.TriangleCount;
        theStream = theStream << aNbNodes << aNbTriangles << theTriangulation.HasUVNodes;
        theStream = theStream << theNeedToWriteNormals << theTriangulation.Deflection();

        // write the 3d nodes
        for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
        {
            theStream = theStream << theTriangulation.Node(aNodeIter);
        }

        if (theTriangulation.HasUVNodes)
        {
            for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
            {
                theStream = theStream << theTriangulation.UVNode(aNodeIter);
            }
        }

        for (int aTriIter = 1; aTriIter <= aNbTriangles; ++aTriIter)
        {
            theStream = theStream << theTriangulation.Triangle(aTriIter);
        }

        if (theNeedToWriteNormals)
        {
            for (int aNormalIter = 1; aNormalIter <= aNbNodes; ++aNormalIter)
            {
                theTriangulation.Normal(aNormalIter, out NCollection_Vec3<float> aNormal);
                theStream = theStream << aNormal;
            }
        }
    }
}
