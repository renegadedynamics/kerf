// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeReader.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeReader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Reads shapes written by <see cref="BinTools_ShapeWriter"/>: objects are read where they are first met, and
/// references to them are followed by seeking back in the stream, which must therefore be seekable. The reader
/// remembers what it has read across calls of <see cref="Read(Stream, ref TopoDS_Shape)"/> until <see cref="Clear"/>.
/// </summary>
/// <remarks>
/// The shapes read are frozen. On untrusted input the reader also checks that references point inside the stream and
/// don't form cycles (OCCT recurses forever), and caps the nesting of shapes, locations and references at
/// <see cref="BinTools.MaxShapeNesting"/> and of geometry at <see cref="BinTools.MaxGeometryNesting"/>; failures
/// throw <see cref="StreamFormatErrorException"/>.
/// </remarks>
public class BinTools_ShapeReader : BinTools_ShapeSetBase
{
    private static readonly BRep_Builder aBuilder = new();

    // positions of the objects already read
    private readonly NCollection_DataMap<ulong, TopoDS_Shape> myShapePos = new();
    private readonly NCollection_DataMap<ulong, Placement> myLocationPos = new();
    private readonly NCollection_DataMap<ulong, Geom_Curve?> myCurvePos = new();
    private readonly NCollection_DataMap<ulong, Geom2d_Curve?> myCurve2dPos = new();
    private readonly NCollection_DataMap<ulong, Geom_Surface?> mySurfacePos = new();
    private readonly NCollection_DataMap<ulong, Polygon3D> myPolygon3dPos = new();
    private readonly NCollection_DataMap<ulong, PolygonOnTriangulation> myPolygonPos = new();
    private readonly NCollection_DataMap<ulong, Triangulation> myTriangulationPos = new();

    // OCCT.NET: untrusted input: the positions of the shapes and locations being read, and the depth of the recursion
    private readonly HashSet<ulong> myShapesInProgress = [];
    private readonly HashSet<ulong> myLocationsInProgress = [];
    private int myDepth;

    /// <summary>A reader that has read nothing yet.</summary>
    public BinTools_ShapeReader()
    {
    }

    /// <summary>Forgets what has been read.</summary>
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
        myShapesInProgress.Clear();
        myLocationsInProgress.Clear();
        myDepth = 0;
    }

    /// <summary>Reads a shape from the stream, which must be seekable. The shape is frozen.</summary>
    /// <exception cref="FailureException">The stream doesn't hold a shape OCCT can read; for input that can't be read
    /// safely, a <see cref="StreamFormatErrorException"/>.</exception>
    public override void Read(Stream theStream, ref TopoDS_Shape theShape)
    {
        var aStream = new BinTools_IStream(theStream);
        try
        {
            theShape = ReadShape(aStream);
        }
        catch (Exception anException) when (BinTools.IsSignal(anException))
        {
            // OCCT.NET: untrusted input: a malformed stream raised an exception that OCCT doesn't catch here
            throw new StreamFormatErrorException("BinTools_ShapeReader::Read: " + anException.Message, anException);
        }
        finally
        {
            myShapesInProgress.Clear();
            myLocationsInProgress.Clear();
            myDepth = 0;
        }

        // design 4.2: a shape read is frozen
        if (!theShape.IsNull())
        {
            theShape.TShape()!.FreezeGraph();
        }
    }

    // OCCT.NET: untrusted input: bounds the recursion through nested shapes, locations and references
    private void Enter()
    {
        if (!System.Runtime.CompilerServices.RuntimeHelpers.TryEnsureSufficientExecutionStack())
        {
            throw new StreamFormatErrorException("BinTools_ShapeReader: objects nested too deeply for the stack");
        }

        if (++myDepth > BinTools.MaxShapeNesting)
        {
            throw new StreamFormatErrorException("BinTools_ShapeReader: objects nested too deeply");
        }
    }

    private TopoDS_Shape ReadShape(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            return ReadShapeAt(theStream);
        }
        finally
        {
            myDepth--;
        }
    }

    private TopoDS_Shape ReadShapeAt(BinTools_IStream theStream)
    {
        TopoDS_Shape aResult = new();
        ulong aPosition = theStream.Position();
        BinTools_ObjectType aType = theStream.ReadType();
        if (aType == BinTools_ObjectType.BinTools_ObjectType_EmptyShape
            || aType == BinTools_ObjectType.BinTools_ObjectType_EndShape)
        {
            return aResult;
        }

        if (theStream.IsReference())
        {
            ulong aRef = theStream.ReadReference();
            if (myShapePos.Find(aRef, out TopoDS_Shape aFound))
            {
                // the shape is already retrieved, just add location
                aResult = aFound;
            }
            else
            {
                if (myShapesInProgress.Contains(aRef))
                {
                    // OCCT.NET: untrusted input: OCCT reads the shape again, forever
                    throw new StreamFormatErrorException("BinTools_ShapeReader: a shape contains itself");
                }

                ulong aCurrent = theStream.Position();
                theStream.GoTo(aRef); // go to the referenced position
                aResult = ReadShape(theStream);
                theStream.GoTo(aCurrent); // returns to the current position
            }

            aResult.Location(ReadLocation(theStream), false);
            // OCCT.NET: untrusted input: OCCT casts any byte
            aResult.Orientation(BinTools.CheckOrientation(theStream.ReadByte()));
            return aResult;
        }

        // read the shape
        TopAbs_ShapeEnum aShapeType = theStream.ShapeType();
        TopAbs_Orientation aShapeOrientation = theStream.ShapeOrientation();
        Placement aShapeLocation = ReadLocation(theStream);
        myShapesInProgress.Add(aPosition);
        // OCCT.NET: the geometry is read by a separate method (a smaller frame for the recursion)
        aResult = ReadShapeGeometry(theStream, aShapeType);

        // read flags and subs
        theStream.ReadBools(out bool aFree,
                            out bool aMod,
                            out bool aChecked,
                            out bool anOrient,
                            out bool aClosedFlag,
                            out bool anInf,
                            out bool aConv);

        // sub-shapes
        for (TopoDS_Shape aSub = ReadShape(theStream); !aSub.IsNull(); aSub = ReadShape(theStream))
        {
            aBuilder.Add(aResult, aSub);
        }

        aResult.Free(aFree);
        aResult.Modified(aMod);
        aResult.Checked(aChecked);
        aResult.Orientable(anOrient);
        aResult.Closed(aClosedFlag);
        aResult.Infinite(anInf);
        aResult.Convex(aConv);
        myShapePos.Bind(aPosition, aResult);
        myShapesInProgress.Remove(aPosition);
        aResult.Orientation(aShapeOrientation);
        aResult.Location(aShapeLocation, false);
        return aResult;
    }

    // The geometry of a shape, apart from ReadShapeAt so that its recursion keeps a small stack frame
    private TopoDS_Shape ReadShapeGeometry(BinTools_IStream theStream, TopAbs_ShapeEnum aShapeType)
    {
        TopoDS_Shape aResult = new();
        double aTol;
        try
        {
            switch (aShapeType)
            {
                case TopAbs_VERTEX:
                {
                    TopoDS_Vertex aV = new();
                    // Read the point geometry
                    aTol = theStream.ReadReal();
                    Point3D aPnt = theStream.ReadPnt();
                    aBuilder.MakeVertex(ref aV, aPnt, aTol);
                    aResult = aV;
                    BRep_TVertex aTV = (BRep_TVertex)aV.TShape()!;
                    NCollection_List<BRep_PointRepresentation> aLpr = aTV.ChangePoints();
                    Placement anEmptyLoc = new();
                    while (theStream)
                    {
                        byte aPrsType = theStream.ReadByte();
                        if (aPrsType == 0)
                        {
                            // end of the cycle
                            break;
                        }

                        double aParam = theStream.ReadReal();
                        BRep_PointRepresentation? aPR = null;
                        switch (aPrsType)
                        {
                            case 1:
                            {
                                Geom_Curve? aCurve = ReadCurve(theStream);
                                if (aCurve is not null)
                                {
                                    aPR = new BRep_PointOnCurve(aParam, aCurve, anEmptyLoc);
                                }
                            }
                                break;

                            case 2:
                            {
                                Geom2d_Curve? aCurve2d = ReadCurve2d(theStream);
                                Geom_Surface? aSurface = ReadSurface(theStream);
                                // OCCT: tests that the surface is null, not that it isn't, so a point on a curve on a
                                // surface is read only when its surface is missing (occt-issues.md)
                                if (aCurve2d is not null && aSurface is null)
                                {
                                    aPR = new BRep_PointOnCurveOnSurface(aParam, aCurve2d, aSurface, anEmptyLoc);
                                }
                            }
                                break;

                            case 3:
                            {
                                double aParam2 = theStream.ReadReal();
                                Geom_Surface? aSurface = ReadSurface(theStream);
                                if (aSurface is not null)
                                {
                                    // OCCT: aParam is the second parameter written (see BinTools_ShapeWriter)
                                    aPR = new BRep_PointOnSurface(aParam, aParam2, aSurface, anEmptyLoc);
                                }
                            }
                                break;

                            default:
                            {
                                throw new FailureException("BinTools_ShapeReader::Read: UnExpected BRep_PointRepresentation = "
                                                           + aPrsType + "\n");
                            }
                        }

                        Placement aPRLoc = ReadLocation(theStream);
                        if (aPR is not null)
                        {
                            aPR.Location(aPRLoc);
                            aLpr.Append(aPR);
                        }
                    }
                }
                    break;

                case TopAbs_EDGE:
                {
                    TopoDS_Edge aE = new();
                    aBuilder.MakeEdge(ref aE);
                    aResult = aE;

                    // Read the curve geometry
                    aTol = theStream.ReadReal();
                    theStream.ReadBools(out bool aSameParameter, out bool aSameRange, out bool aDegenerated);
                    aBuilder.SameParameter(aE, aSameParameter);
                    aBuilder.SameRange(aE, aSameRange);
                    aBuilder.Degenerated(aE, aDegenerated);
                    double aFirst, aLast;
                    while (theStream)
                    {
                        byte aPrsType = theStream.ReadByte(); //{0|1|2|3|4|5|6|7}
                        if (aPrsType == 0)
                        {
                            break;
                        }

                        switch (aPrsType)
                        {
                            case 1: // -1- Curve 3D
                            {
                                Geom_Curve? aCurve = ReadCurve(theStream);
                                Placement aLoc = ReadLocation(theStream);
                                aFirst = theStream.ReadReal();
                                aLast = theStream.ReadReal();
                                if (aCurve is not null)
                                {
                                    aBuilder.UpdateEdge(aE, aCurve, aLoc, aTol);
                                    aBuilder.Range(aE, aFirst, aLast, true);
                                }
                            }
                                break;

                            case 2: // -2- Curve on surf
                            case 3: // -3- Curve on closed surf
                            {
                                bool aClosed = (aPrsType == 3);
                                Geom2d_Curve? aCurve2d_2 = null, aCurve2d_1 = ReadCurve2d(theStream);
                                Continuity aReg = Continuity.C0;
                                if (aClosed)
                                {
                                    aCurve2d_2 = ReadCurve2d(theStream);
                                    aReg = (Continuity)theStream.ReadByte();
                                }

                                Geom_Surface? aSurface = ReadSurface(theStream);
                                Placement aLoc = ReadLocation(theStream);

                                // range
                                aFirst = theStream.ReadReal();
                                aLast = theStream.ReadReal();
                                if (aCurve2d_1 is not null && (!aClosed || aCurve2d_2 is not null) && aSurface is not null)
                                {
                                    if (aClosed)
                                    {
                                        aBuilder.UpdateEdge(aE, aCurve2d_1, aCurve2d_2, aSurface, aLoc, aTol);
                                        aBuilder.Continuity(aE, aSurface, aSurface, aLoc, aLoc, aReg);
                                    }
                                    else
                                    {
                                        aBuilder.UpdateEdge(aE, aCurve2d_1, aSurface, aLoc, aTol);
                                    }

                                    aBuilder.Range(aE, aSurface, aLoc, aFirst, aLast);
                                }
                            }
                                break;

                            case 4: // -4- Regularity
                            {
                                Continuity aReg = (Continuity)theStream.ReadByte();
                                Geom_Surface? aSurface1 = ReadSurface(theStream);
                                Placement aLoc1 = ReadLocation(theStream);
                                Geom_Surface? aSurface2 = ReadSurface(theStream);
                                Placement aLoc2 = ReadLocation(theStream);
                                if (aSurface1 is not null && aSurface2 is not null)
                                {
                                    aBuilder.Continuity(aE, aSurface1, aSurface2, aLoc1, aLoc2, aReg);
                                }
                            }
                                break;

                            case 5: // -5- Polygon3D
                            {
                                Polygon3D? aPolygon = ReadPolygon3d(theStream);
                                Placement aLoc = ReadLocation(theStream);
                                aBuilder.UpdateEdge(aE, aPolygon, aLoc);
                            }
                                break;

                            case 6: // -6- Polygon on triangulation
                            case 7: // -7- Polygon on closed triangulation
                            {
                                bool aClosed = (aPrsType == 7);
                                PolygonOnTriangulation? aPoly2 = null, aPoly1 = ReadPolygon(theStream);
                                if (aClosed)
                                {
                                    aPoly2 = ReadPolygon(theStream);
                                }

                                Triangulation? aTriangulation = ReadTriangulation(theStream);
                                Placement aLoc = ReadLocation(theStream);
                                if (aClosed)
                                {
                                    aBuilder.UpdateEdge(aE, aPoly1, aPoly2, aTriangulation, aLoc);
                                }
                                else
                                {
                                    aBuilder.UpdateEdge(aE, aPoly1, aTriangulation, aLoc);
                                }

                                // range
                            }
                                break;

                            default:
                            {
                                throw new FailureException("Unexpected Curve Representation =" + aPrsType + "\n");
                            }
                        }
                    }
                }
                    break;

                case TopAbs_WIRE:
                {
                    TopoDS_Wire aW = new();
                    aBuilder.MakeWire(ref aW);
                    aResult = aW;
                }
                    break;

                case TopAbs_FACE:
                {
                    TopoDS_Face aF = new();
                    aBuilder.MakeFace(ref aF);
                    aResult = aF;
                    bool aNatRes = theStream.ReadBool();
                    aTol = theStream.ReadReal();
                    Geom_Surface? aSurface = ReadSurface(theStream);
                    Placement aLoc = ReadLocation(theStream);
                    aBuilder.UpdateFace(aF, aSurface, aLoc, aTol);
                    aBuilder.NaturalRestriction(aF, aNatRes);
                    if (theStream.ReadByte() == 2)
                    {
                        // triangulation
                        aBuilder.UpdateFace(aF, ReadTriangulation(theStream));
                    }
                }
                    break;

                case TopAbs_SHELL:
                {
                    TopoDS_Shell aSh = new();
                    aBuilder.MakeShell(ref aSh);
                    aResult = aSh;
                }
                    break;

                case TopAbs_SOLID:
                {
                    TopoDS_Solid aSo = new();
                    aBuilder.MakeSolid(ref aSo);
                    aResult = aSo;
                }
                    break;

                case TopAbs_COMPSOLID:
                {
                    TopoDS_CompSolid aCS = new();
                    aBuilder.MakeCompSolid(ref aCS);
                    aResult = aCS;
                }
                    break;

                case TopAbs_COMPOUND:
                {
                    TopoDS_Compound aC = new();
                    aBuilder.MakeCompound(ref aC);
                    aResult = aC;
                }
                    break;

                default:
                {
                    throw new FailureException("Unexpected topology type = " + (int)aShapeType + "\n");
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_ShapeReader::Read", anException);
        }

        return aResult;
    }

    /// <summary>Reads a location written by <see cref="BinTools_ShapeWriter.WriteLocation"/>, or a reference to one.</summary>
    public Placement ReadLocation(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            return ReadLocationAt(theStream);
        }
        finally
        {
            myDepth--;
        }
    }

    private Placement ReadLocationAt(BinTools_IStream theStream)
    {
        ulong aPosition = theStream.Position();
        BinTools_ObjectType aType = theStream.ReadType();
        if (aType == BinTools_ObjectType.BinTools_ObjectType_EmptyLocation
            || aType == BinTools_ObjectType.BinTools_ObjectType_LocationEnd)
        {
            return new Placement();
        }

        if (theStream.IsReference())
        {
            ulong aRef = theStream.ReadReference();
            if (myLocationPos.Find(aRef, out Placement aFound))
            {
                // the location is already retrieved
                return aFound;
            }

            if (myLocationsInProgress.Contains(aRef))
            {
                // OCCT.NET: untrusted input: OCCT reads the location again, forever
                throw new StreamFormatErrorException("BinTools_ShapeReader: a location contains itself");
            }

            ulong aCurrent = theStream.Position();
            theStream.GoTo(aRef); // go to the referenced position
            Placement aResult = ReadLocation(theStream);
            theStream.GoTo(aCurrent); // returns to the current position
            return aResult;
        }

        // read the location directly from the stream
        Placement aLoc = new();
        if (aType == BinTools_ObjectType.BinTools_ObjectType_SimpleLocation)
        {
            Transform3D aTrsf = theStream.ReadTrsf();
            aLoc = new Placement(aTrsf);
        }
        else if (aType == BinTools_ObjectType.BinTools_ObjectType_Location)
        {
            myLocationsInProgress.Add(aPosition);
            for (Placement aNextLoc = ReadLocation(theStream); !aNextLoc.IsIdentity;
                 aNextLoc = ReadLocation(theStream))
            {
                // OCCT.NET: untrusted input: bounds the recursion of Powered and Multiplied
                aLoc = BinTools.PoweredTimes(aNextLoc, theStream.ReadInteger(), aLoc);
            }

            myLocationsInProgress.Remove(aPosition);
        }

        myLocationPos.Bind(aPosition, aLoc);
        return aLoc;
    }

    private Geom_Curve? ReadCurve(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            Geom_Curve? aResult = null;
            ulong aPosition = theStream.Position();
            theStream.ReadType();
            if (theStream.IsReference())
            {
                // get by reference
                ulong aRef = theStream.ReadReference();
                if (myCurvePos.Find(aRef, out Geom_Curve? aFound))
                {
                    // the curve is already retrieved
                    return aFound;
                }

                ulong aCurrent = theStream.Position();
                theStream.GoTo(aRef); // go to the referenced position
                aResult = ReadCurve(theStream);
                theStream.GoTo(aCurrent); // returns to the current position
            }
            else if (theStream.LastType() == BinTools_ObjectType.BinTools_ObjectType_Curve)
            {
                // read from the stream
                BinTools_CurveSet.ReadCurve(theStream.Stream(), out aResult);
                theStream.UpdatePosition();
                myCurvePos.Bind(aPosition, aResult);
            }

            return aResult;
        }
        finally
        {
            myDepth--;
        }
    }

    private Geom2d_Curve? ReadCurve2d(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            Geom2d_Curve? aResult = null;
            ulong aPosition = theStream.Position();
            theStream.ReadType();
            if (theStream.IsReference())
            {
                // get by reference
                ulong aRef = theStream.ReadReference();
                if (myCurve2dPos.Find(aRef, out Geom2d_Curve? aFound))
                {
                    // the curve is already retrieved
                    return aFound;
                }

                ulong aCurrent = theStream.Position();
                theStream.GoTo(aRef); // go to the referenced position
                aResult = ReadCurve2d(theStream);
                theStream.GoTo(aCurrent); // returns to the current position
            }
            else if (theStream.LastType() == BinTools_ObjectType.BinTools_ObjectType_Curve2d)
            {
                // read from the stream
                BinTools_Curve2dSet.ReadCurve2d(theStream.Stream(), out aResult);
                theStream.UpdatePosition();
                myCurve2dPos.Bind(aPosition, aResult);
            }

            return aResult;
        }
        finally
        {
            myDepth--;
        }
    }

    private Geom_Surface? ReadSurface(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            Geom_Surface? aResult = null;
            ulong aPosition = theStream.Position();
            theStream.ReadType();
            if (theStream.IsReference())
            {
                // get by reference
                ulong aRef = theStream.ReadReference();
                if (mySurfacePos.Find(aRef, out Geom_Surface? aFound))
                {
                    // the surface is already retrieved
                    return aFound;
                }

                ulong aCurrent = theStream.Position();
                theStream.GoTo(aRef); // go to the referenced position
                aResult = ReadSurface(theStream);
                theStream.GoTo(aCurrent); // returns to the current position
            }
            else if (theStream.LastType() == BinTools_ObjectType.BinTools_ObjectType_Surface)
            {
                // read from the stream
                BinTools_SurfaceSet.ReadSurface(theStream.Stream(), out aResult);
                theStream.UpdatePosition();
                mySurfacePos.Bind(aPosition, aResult);
            }

            return aResult;
        }
        finally
        {
            myDepth--;
        }
    }

    private Polygon3D? ReadPolygon3d(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            Polygon3D? aResult = null;
            ulong aPosition = theStream.Position();
            theStream.ReadType();
            if (theStream.IsReference())
            {
                // get by reference
                ulong aRef = theStream.ReadReference();
                if (myPolygon3dPos.Find(aRef, out Polygon3D aFound))
                {
                    // the polygon is already retrieved
                    return aFound;
                }

                ulong aCurrent = theStream.Position();
                theStream.GoTo(aRef); // go to the referenced position
                aResult = ReadPolygon3d(theStream);
                theStream.GoTo(aCurrent); // returns to the current position
            }
            else if (theStream.LastType() == BinTools_ObjectType.BinTools_ObjectType_Polygon3d)
            {
                // read from the stream
                int aNbNodes = theStream.ReadInteger();
                bool aHasParameters = theStream.ReadBool();
                // OCCT.NET: untrusted input
                BinTools.CheckCount(theStream.Stream(), aNbNodes, aHasParameters ? 32 : 24, "nodes");
                aResult = new Polygon3D(aNbNodes, aHasParameters);
                aResult.Deflection(theStream.ReadReal());
                NCollection_Array1<Point3D> aNodes = aResult.ChangeNodes();
                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    aNodes[aNodeIter] = theStream.ReadPnt();
                }

                if (aHasParameters)
                {
                    NCollection_Array1<double> aParam = aResult.ChangeParameters();
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        aParam[aNodeIter] = theStream.ReadReal();
                    }
                }

                myPolygon3dPos.Bind(aPosition, aResult);
            }

            return aResult;
        }
        finally
        {
            myDepth--;
        }
    }

    private PolygonOnTriangulation? ReadPolygon(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            PolygonOnTriangulation? aResult = null;
            ulong aPosition = theStream.Position();
            theStream.ReadType();
            if (theStream.IsReference())
            {
                // get by reference
                ulong aRef = theStream.ReadReference();
                if (myPolygonPos.Find(aRef, out PolygonOnTriangulation aFound))
                {
                    // the polygon is already retrieved
                    return aFound;
                }

                ulong aCurrent = theStream.Position();
                theStream.GoTo(aRef); // go to the referenced position
                aResult = ReadPolygon(theStream);
                theStream.GoTo(aCurrent); // returns to the current position
            }
            else if (theStream.LastType() == BinTools_ObjectType.BinTools_ObjectType_PolygonOnTriangulation)
            {
                // read from the stream
                int aNbNodes = theStream.ReadInteger();
                // OCCT.NET: untrusted input
                BinTools.CheckCount(theStream.Stream(), aNbNodes, 4, "nodes");
                aResult = new PolygonOnTriangulation(aNbNodes, false);
                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    aResult.SetNode(aNodeIter, theStream.ReadInteger());
                }

                aResult.Deflection(theStream.ReadReal());
                if (theStream.ReadBool())
                {
                    // OCCT.NET: untrusted input
                    BinTools.CheckCount(theStream.Stream(), aNbNodes, 8, "parameters");
                    NCollection_HArray1<double> aParams = new(1, aNbNodes);
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        aParams.ChangeValue(aNodeIter) = theStream.ReadReal();
                    }

                    aResult.SetParameters(aParams);
                }

                myPolygonPos.Bind(aPosition, aResult);
            }

            return aResult;
        }
        finally
        {
            myDepth--;
        }
    }

    private Triangulation? ReadTriangulation(BinTools_IStream theStream)
    {
        Enter();
        try
        {
            Triangulation? aResult = null;
            ulong aPosition = theStream.Position();
            BinTools_ObjectType aType = theStream.ReadType();
            if (theStream.IsReference())
            {
                // get by reference
                ulong aRef = theStream.ReadReference();
                if (myTriangulationPos.Find(aRef, out Triangulation aFound))
                {
                    // the triangulation is already retrieved
                    return aFound;
                }

                ulong aCurrent = theStream.Position();
                theStream.GoTo(aRef); // go to the referenced position
                aResult = ReadTriangulation(theStream);
                theStream.GoTo(aCurrent); // returns to the current position
            }
            else if (aType == BinTools_ObjectType.BinTools_ObjectType_Triangulation)
            {
                // read from the stream
                int aNbNodes = theStream.ReadInteger();
                int aNbTriangles = theStream.ReadInteger();
                bool aHasUV = theStream.ReadBool();
                bool aHasNormals = theStream.ReadBool();
                // OCCT.NET: untrusted input
                int aNodeBytes = 24 + (aHasUV ? 16 : 0) + (aHasNormals ? 12 : 0);
                BinTools.CheckCount(theStream.Stream(), aNbNodes, aNodeBytes, "nodes");
                BinTools.CheckCount(theStream.Stream(), aNbTriangles, 12, "triangles");
                BinTools.CheckCount(theStream.Stream(),
                                    (long)aNbNodes * aNodeBytes + aNbTriangles * 12L,
                                    1,
                                    "bytes of a triangulation");
                aResult = new Triangulation(aNbNodes, aNbTriangles, aHasUV, aHasNormals);
                aResult.Deflection(theStream.ReadReal());
                for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                {
                    aResult.SetNode(aNodeIter, theStream.ReadPnt());
                }

                if (aHasUV)
                {
                    for (int aNodeIter = 1; aNodeIter <= aNbNodes; ++aNodeIter)
                    {
                        double aU = theStream.ReadReal();
                        double aV = theStream.ReadReal();
                        aResult.SetUVNode(aNodeIter, new Point2D(aU, aV));
                    }
                }

                // read the triangles
                for (int aTriIter = 1; aTriIter <= aNbTriangles; ++aTriIter)
                {
                    int aN1 = theStream.ReadInteger();
                    int aN2 = theStream.ReadInteger();
                    int aN3 = theStream.ReadInteger();
                    aResult.SetTriangle(aTriIter, new Triangle1(aN1, aN2, aN3));
                }

                if (aHasNormals)
                {
                    for (int aNormalIter = 1; aNormalIter <= aNbNodes; ++aNormalIter)
                    {
                        float aX = theStream.ReadShortReal();
                        float aY = theStream.ReadShortReal();
                        float aZ = theStream.ReadShortReal();
                        aResult.SetNormal(aNormalIter, new NCollection_Vec3<float>(aX, aY, aZ));
                    }
                }

                myTriangulationPos.Bind(aPosition, aResult);
            }

            return aResult;
        }
        finally
        {
            myDepth--;
        }
    }
}
