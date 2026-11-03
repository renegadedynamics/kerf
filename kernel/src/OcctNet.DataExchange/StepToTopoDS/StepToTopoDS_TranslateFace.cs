// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateFace.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.StepToTopoDS_TranslateFaceError;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// Translates a face_surface (its surface and its bounds: edge loops, poly loops, vertex loops) into a TopoDS_Face, or
/// a triangulated face or surface set into a face carrying a triangulation.
/// </summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
public sealed class StepToTopoDS_TranslateFace : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateFaceError myError;
    private Shape myResult = new Shape();

    // Set nodes to the triangulation from an array
    private static void SetNodes(Triangulation theMesh, NCollection_HArray1<Xyz> theNodes, NCollection_HArray1<int> thePnindices,
                                 double theLengthFactor)
    {
        if (thePnindices is not null)
        {
            for (int aPnIndex = 1; aPnIndex <= theMesh.NodeCount; ++aPnIndex)
            {
                Xyz aPoint = theNodes.Value(thePnindices.Value(aPnIndex));
                theMesh.SetNode(aPnIndex, new Point3D(theLengthFactor * aPoint));
            }
        }
        else
        {
            for (int aPnIndex = 1; aPnIndex <= theMesh.NodeCount; ++aPnIndex)
            {
                theMesh.SetNode(aPnIndex, new Point3D(theLengthFactor * theNodes.Value(aPnIndex)));
            }
        }
    }

    // Set normals to the triangulation from an array
    private static void SetNormals(Triangulation theMesh, NCollection_HArray2<double> theNormals, int theNormNum, int theNbNodes)
    {
        if (theNormals.RowLength() != 3)
        {
            return;
        }

        Xyz aNorm = new Xyz();
        if (theNormNum == 1)
        {
            aNorm.SetX(theNormals.Value(1, 1));
            aNorm.SetY(theNormals.Value(1, 2));
            aNorm.SetZ(theNormals.Value(1, 3));
            for (int aNodeIndex = 1; aNodeIndex <= theNbNodes; ++aNodeIndex)
            {
                theMesh.SetNormal(aNodeIndex, new Direction3D(aNorm));
            }
        }
        else if (theNbNodes == theNormNum)
        {
            for (int aNormIndex = 1; aNormIndex <= theNormNum; ++aNormIndex)
            {
                aNorm.SetX(theNormals.Value(aNormIndex, 1));
                aNorm.SetY(theNormals.Value(aNormIndex, 2));
                aNorm.SetZ(theNormals.Value(aNormIndex, 3));
                theMesh.SetNormal(aNormIndex, new Direction3D(aNorm));
            }
        }
    }

    // Set triangles to the triangulation from an array
    private static void SetTriangles(Triangulation theMesh, NCollection_HArray2<int> theTriangles, int theTrianStripsNum,
                                     NCollection_HArray1<Transient> theTrianStrips, int theTrianFansNum,
                                     NCollection_HArray1<Transient> theTrianFans)
    {
        if (theTrianStripsNum == 0 && theTrianFansNum == 0)
        {
            for (int aTrianIndex = 1; aTrianIndex <= theMesh.TriangleCount; ++aTrianIndex)
            {
                theMesh.SetTriangle(aTrianIndex, new Triangle1(theTriangles.Value(aTrianIndex, 1), theTriangles.Value(aTrianIndex, 2),
                                                                   theTriangles.Value(aTrianIndex, 3)));
            }
        }
        else
        {
            int aTriangleIndex = 1;
            for (int aTrianStripIndex = 1; aTrianStripIndex <= theTrianStripsNum; ++aTrianStripIndex)
            {
                NCollection_HArray1<int> aTriangleStrip = theTrianStrips.Value(aTrianStripIndex) as NCollection_HArray1<int>;
                for (int anIndex = 3; anIndex <= aTriangleStrip.Length(); anIndex += 2)
                {
                    if (aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 2)
                        && aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 1))
                    {
                        theMesh.SetTriangle(aTriangleIndex++, new Triangle1(aTriangleStrip.Value(anIndex - 2), aTriangleStrip.Value(anIndex),
                                                                                aTriangleStrip.Value(anIndex - 1)));
                    }
                }

                for (int anIndex = 4; anIndex <= aTriangleStrip.Length(); anIndex += 2)
                {
                    if (aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 2)
                        && aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 1))
                    {
                        theMesh.SetTriangle(aTriangleIndex++, new Triangle1(aTriangleStrip.Value(anIndex - 2),
                                                                                aTriangleStrip.Value(anIndex - 1),
                                                                                aTriangleStrip.Value(anIndex)));
                    }
                }
            }

            for (int aTrianFanIndex = 1; aTrianFanIndex <= theTrianFansNum; ++aTrianFanIndex)
            {
                NCollection_HArray1<int> aTriangleFan = theTrianFans.Value(aTrianFanIndex) as NCollection_HArray1<int>;
                for (int anIndex = 3; anIndex <= aTriangleFan.Length(); ++anIndex)
                {
                    if (aTriangleFan.Value(anIndex) != aTriangleFan.Value(anIndex - 2)
                        && aTriangleFan.Value(anIndex - 1) != aTriangleFan.Value(anIndex - 2))
                    {
                        theMesh.SetTriangle(aTriangleIndex++, new Triangle1(aTriangleFan.Value(1), aTriangleFan.Value(anIndex),
                                                                                aTriangleFan.Value(anIndex - 1)));
                    }
                }
            }
        }
    }

    // GetSimpleFaceElements: OCCT's template, once per instantiation
    private static void GetSimpleFaceElements(StepVisual_TriangulatedFace theFace, out NCollection_HArray1<Xyz> theNodes,
                                              out NCollection_HArray2<double> theNormals, out NCollection_HArray2<int> theTriangles,
                                              out int theNormNb, out int theTriNb, ref NCollection_HArray1<int> thePnindices)
    {
        theNodes = theFace.Coordinates().Points();
        theNormals = theFace.Normals();
        theTriangles = theFace.Triangles();
        theNormNb = theFace.NbNormals();
        theTriNb = theFace.NbTriangles();
        int aPnIndNb = theFace.NbPnindex();
        if (aPnIndNb > 0)
        {
            thePnindices = new NCollection_HArray1<int>(1, aPnIndNb);
            for (int anIndx = 1; anIndx <= aPnIndNb; ++anIndx)
            {
                thePnindices.SetValue(anIndx, theFace.PnindexValue(anIndx));
            }
        }
    }

    private static void GetSimpleFaceElements(StepVisual_TriangulatedSurfaceSet theFace, out NCollection_HArray1<Xyz> theNodes,
                                              out NCollection_HArray2<double> theNormals, out NCollection_HArray2<int> theTriangles,
                                              out int theNormNb, out int theTriNb, ref NCollection_HArray1<int> thePnindices)
    {
        theNodes = theFace.Coordinates().Points();
        theNormals = theFace.Normals();
        theTriangles = theFace.Triangles();
        theNormNb = theFace.NbNormals();
        theTriNb = theFace.NbTriangles();
        int aPnIndNb = theFace.NbPnindex();
        if (aPnIndNb > 0)
        {
            thePnindices = new NCollection_HArray1<int>(1, aPnIndNb);
            for (int anIndx = 1; anIndx <= aPnIndNb; ++anIndx)
            {
                thePnindices.SetValue(anIndx, theFace.PnindexValue(anIndx));
            }
        }
    }

    // GetComplexFaceElements: OCCT's template, once per instantiation
    private static void GetComplexFaceElements(StepVisual_ComplexTriangulatedFace theFace, out NCollection_HArray1<Xyz> theNodes,
                                               out NCollection_HArray2<double> theNormals,
                                               out NCollection_HArray1<Transient> theTriangleStrips,
                                               out NCollection_HArray1<Transient> theTriangleFans, out int theNormNb,
                                               out int theTriStripsNb, out int theTriFansNb, ref NCollection_HArray1<int> thePnindices)
    {
        theNodes = theFace.Coordinates().Points();
        theNormals = theFace.Normals();
        theTriangleStrips = theFace.TriangleStrips();
        theTriangleFans = theFace.TriangleFans();
        theNormNb = theFace.NbNormals();
        theTriStripsNb = theFace.NbTriangleStrips();
        theTriFansNb = theFace.NbTriangleFans();
        int aPnIndNb = theFace.NbPnindex();
        if (aPnIndNb > 0)
        {
            thePnindices = new NCollection_HArray1<int>(1, aPnIndNb);
            for (int anIndx = 1; anIndx <= aPnIndNb; ++anIndx)
            {
                thePnindices.SetValue(anIndx, theFace.PnindexValue(anIndx));
            }
        }
    }

    private static void GetComplexFaceElements(StepVisual_ComplexTriangulatedSurfaceSet theFace, out NCollection_HArray1<Xyz> theNodes,
                                               out NCollection_HArray2<double> theNormals,
                                               out NCollection_HArray1<Transient> theTriangleStrips,
                                               out NCollection_HArray1<Transient> theTriangleFans, out int theNormNb,
                                               out int theTriStripsNb, out int theTriFansNb, ref NCollection_HArray1<int> thePnindices)
    {
        theNodes = theFace.Coordinates().Points();
        theNormals = theFace.Normals();
        theTriangleStrips = theFace.TriangleStrips();
        theTriangleFans = theFace.TriangleFans();
        theNormNb = theFace.NbNormals();
        theTriStripsNb = theFace.NbTriangleStrips();
        theTriFansNb = theFace.NbTriangleFans();
        int aPnIndNb = theFace.NbPnindex();
        if (aPnIndNb > 0)
        {
            thePnindices = new NCollection_HArray1<int>(1, aPnIndNb);
            for (int anIndx = 1; anIndx <= aPnIndNb; ++anIndx)
            {
                thePnindices.SetValue(anIndx, theFace.PnindexValue(anIndx));
            }
        }
    }

    private static Triangulation CreatePolyTriangulation(StepVisual_TessellatedItem theTI, in StepData_Factors theLocalFactors)
    {
        Triangulation aMesh;
        if (theTI is null)
        {
            return null;
        }

        NCollection_HArray1<Xyz> aNodes;
        NCollection_HArray2<double> aNormals;
        NCollection_HArray2<int> aTriangles = null;
        int aNormNum = 0;
        int aTrianNum = 0;
        NCollection_HArray1<int> aPnindices = null;

        NCollection_HArray1<Transient> aTriaStrips = null;
        NCollection_HArray1<Transient> aTriaFans = null;
        int aTrianStripsNum = 0;
        int aTrianFansNum = 0;

        if (theTI is StepVisual_TriangulatedFace aTF)
        {
            GetSimpleFaceElements(aTF, out aNodes, out aNormals, out aTriangles, out aNormNum, out aTrianNum, ref aPnindices);
        }
        else if (theTI is StepVisual_TriangulatedSurfaceSet aTSS)
        {
            GetSimpleFaceElements(aTSS, out aNodes, out aNormals, out aTriangles, out aNormNum, out aTrianNum, ref aPnindices);
        }
        else if (theTI is StepVisual_ComplexTriangulatedFace aCTF)
        {
            GetComplexFaceElements(aCTF, out aNodes, out aNormals, out aTriaStrips, out aTriaFans, out aNormNum, out aTrianStripsNum,
                                   out aTrianFansNum, ref aPnindices);
        }
        else if (theTI is StepVisual_ComplexTriangulatedSurfaceSet aCTSS)
        {
            GetComplexFaceElements(aCTSS, out aNodes, out aNormals, out aTriaStrips, out aTriaFans, out aNormNum, out aTrianStripsNum,
                                   out aTrianFansNum, ref aPnindices);
        }
        else
        {
            return null;
        }

        const bool aHasUVNodes = false;
        bool aHasNormals = (aNormNum > 0);
        int aNbNodes = aPnindices is not null ? aPnindices.Length() : aNodes.Length();

        if (aTrianStripsNum == 0 && aTrianFansNum == 0)
        {
            aMesh = new Triangulation(aNbNodes, aTrianNum, aHasUVNodes, aHasNormals);
        }
        else
        {
            int aNbTriaStrips = 0;
            int aNbTriaFans = 0;

            for (int aTrianStripIndex = 1; aTrianStripIndex <= aTrianStripsNum; ++aTrianStripIndex)
            {
                NCollection_HArray1<int> aTriangleStrip = aTriaStrips.Value(aTrianStripIndex) as NCollection_HArray1<int>;
                for (int anIndex = 3; anIndex <= aTriangleStrip.Length(); anIndex += 2)
                {
                    if (aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 2)
                        && aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 1))
                    {
                        ++aNbTriaStrips;
                    }
                }

                for (int anIndex = 4; anIndex <= aTriangleStrip.Length(); anIndex += 2)
                {
                    if (aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 2)
                        && aTriangleStrip.Value(anIndex) != aTriangleStrip.Value(anIndex - 1))
                    {
                        ++aNbTriaStrips;
                    }
                }
            }

            for (int aTrianFanIndex = 1; aTrianFanIndex <= aTrianFansNum; ++aTrianFanIndex)
            {
                NCollection_HArray1<int> aTriangleFan = aTriaFans.Value(aTrianFanIndex) as NCollection_HArray1<int>;
                aNbTriaFans += aTriangleFan.Length() - 2;
            }

            aMesh = new Triangulation(aNbNodes, aNbTriaStrips + aNbTriaFans, aHasUVNodes, aHasNormals);
        }

        SetNodes(aMesh, aNodes, aPnindices, theLocalFactors.LengthFactor());

        if (aHasNormals)
        {
            SetNormals(aMesh, aNormals, aNormNum, aNbNodes);
        }

        SetTriangles(aMesh, aTriangles, aTrianStripsNum, aTriaStrips, aTrianFansNum, aTriaFans);

        return aMesh;
    }

    public StepToTopoDS_TranslateFace()
    {
        myError = StepToTopoDS_TranslateFaceOther;
        done = false;
    }

    /// <summary>Translates a face_surface.</summary>
    public StepToTopoDS_TranslateFace(StepShape_FaceSurface FS, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        Init(FS, T, NMTool, theLocalFactors);
    }

    public StepToTopoDS_TranslateFace(StepShape_FaceSurface FS, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool)
        : this(FS, T, NMTool, new StepData_Factors())
    {
    }

    /// <summary>Translates a triangulated_face or a complex_triangulated_face.</summary>
    public StepToTopoDS_TranslateFace(StepVisual_TessellatedFace theTF, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                                      bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom, in StepData_Factors theLocalFactors)
    {
        Init(theTF, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, theLocalFactors);
    }

    public StepToTopoDS_TranslateFace(StepVisual_TessellatedFace theTF, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                                      bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        : this(theTF, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors())
    {
    }

    /// <summary>Translates a triangulated_surface_set or a complex_triangulated_surface_set.</summary>
    public StepToTopoDS_TranslateFace(StepVisual_TessellatedSurfaceSet theTSS, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                                      in StepData_Factors theLocalFactors)
    {
        Init(theTSS, theTool, theNMTool, theLocalFactors);
    }

    public StepToTopoDS_TranslateFace(StepVisual_TessellatedSurfaceSet theTSS, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool)
        : this(theTSS, theTool, theNMTool, new StepData_Factors())
    {
    }

    private static bool isReversed(StepGeom_Surface theStepSurf)
    {
        if (theStepSurf is StepGeom_RectangularTrimmedSurface aRTS)
        {
            return isReversed(aRTS.BasisSurface());
        }
        else
        {
            StepGeom_ToroidalSurface aStepTorSur = theStepSurf as StepGeom_ToroidalSurface;
            return aStepTorSur is not null && aStepTorSur.MajorRadius() < 0.0;
        }
    }

    /// <summary>Translates a face_surface.</summary>
    /// <remarks>Needs <see cref="ShapeAlgo.Init"/> (B-spline surfaces are made periodic through its container).</remarks>
    public void Init(StepShape_FaceSurface theFaceSurface, StepToTopoDS_Tool theTopoDSTool, StepToTopoDS_NMTool theTopoDSToolNM,
                     in StepData_Factors theLocalFactors)
    {
        done = true;
        if (theTopoDSTool.IsBound(theFaceSurface))
        {
            myResult = ShapeCasts.Face(theTopoDSTool.Find(theFaceSurface));
            myError = StepToTopoDS_TranslateFaceDone;
            done = true;
            return;
        }

        // Within a context of this method this object is used for message handling only.
        Transfer_TransientProcess aMessageHandler = theTopoDSTool.TransientProcess();

        // ----------------------------------------------
        // Map the Face Geometry and create a TopoDS_Face
        // ----------------------------------------------
        StepGeom_Surface aStepGeomSurface = theFaceSurface.FaceGeometry();
        // sln 01.10.2001 BUC61003. If corresponding entity was read with error StepSurface may be NULL.
        // In this case we exit from function
        if (aStepGeomSurface is null)
        {
            aMessageHandler.AddFail(aStepGeomSurface, " Surface has not been created");
            myError = StepToTopoDS_TranslateFaceOther;
            done = false;
            return;
        }

        // [BEGIN] Added to process non-manifold topology (ssv; 14.11.2010)
        if (theTopoDSToolNM.IsActive() && theTopoDSToolNM.IsBound(aStepGeomSurface))
        {
            Shape anExistingShape = theTopoDSToolNM.Find(aStepGeomSurface);
            // Reverse shape's orientation for the next shell
            anExistingShape.Reverse();
            myResult = anExistingShape;
            myError = StepToTopoDS_TranslateFaceDone;
            done = true;
            return;
        }
        // [END] Added to process non-manifold topology (ssv; 14.11.2010)

        if (aStepGeomSurface is StepGeom_OffsetSurface) //: d4 abv 12 Mar 98
        {
            aMessageHandler.AddWarning(aStepGeomSurface, " Type OffsetSurface is out of scope of AP 214");
        }

        Surface aGeomSurface = StepToGeom.MakeSurface(aStepGeomSurface, theLocalFactors);
        if (aGeomSurface is null)
        {
            aMessageHandler.AddFail(aStepGeomSurface, " Surface has not been created");
            myError = StepToTopoDS_TranslateFaceOther;
            done = false;
            return;
        }

        // pdn to force bsplsurf to be periodic
        if (aStepGeomSurface is StepGeom_BSplineSurface)
        {
            Surface periodicSurf = ShapeAlgo.AlgoContainer().ConvertToPeriodic(aGeomSurface);
            if (periodicSurf is not null)
            {
                aMessageHandler.AddWarning(aStepGeomSurface, "Surface forced to be periodic");
                aGeomSurface = periodicSurf;
            }
        }

        // fix for bug 0026376 Solid Works wrote face based on toroidal surface having negative major
        // radius seems that such case is interpreted  by "Solid Works" and "ProE" as face having reversed
        // orientation.
        bool aSameSense = isReversed(aStepGeomSurface) ? !theFaceSurface.SameSense() : theFaceSurface.SameSense();

        // -- Statistics --
        theTopoDSTool.AddContinuity(aGeomSurface);

        Face aResultFace = new Face();
        var aFaceBuilder = new BrepBuilder();
        aFaceBuilder.MakeFace(ref aResultFace, aGeomSurface, Kerf.Engine.Foundation.Precision.Confusion());

        // ----------------------------------
        // Iterate on each FaceBounds (Wires)
        // ----------------------------------
        // - Simple sewing criterion (CKY, Jan97)
        // Periodic surface (typically a cylinder)
        // 2 face bounds, each with an edge loop from a single edge.
        // This edge is closed, c-a-d vertex-begin = vertex-end (for the two edges)
        // Is it sufficient (check that these are two outer-bounds... ?? How?)
        // Then we can say: face with two edges whose seam is missing
        // The seam is between the two vertex
        for (int aBoundIndex = 1; aBoundIndex <= theFaceSurface.NbBounds(); ++aBoundIndex)
        {
            StepShape_FaceBound aFaceBound = theFaceSurface.BoundsValue(aBoundIndex);
            if (aFaceBound is null)
            {
                continue;
            }

            StepShape_Loop aFaceLoop = aFaceBound.Bound();
            if (aFaceLoop is null)
            {
                continue;
            }

            // ------------------------
            // The Loop is a VertexLoop
            // ------------------------
            if (aFaceLoop is StepShape_VertexLoop)
            {
                //: S4136      STF.Closed() = false;
                //  PROBLEM if SPHERE or TORE
                // It will be necessary to make a complete wire, provided that the point carries on the face
                // In the meantime, we do nothing

                // abv 10.07.00 pr1sy.stp: vertex_loop can be wrong; so just make natural bounds
                if ((aGeomSurface is SphericalSurface || aGeomSurface is BSplineSurface || aGeomSurface is RevolutionSurface)
                    && (theFaceSurface.NbBounds() == 1))
                {
                    // Modification to create natural bounds for face based on the spherical and Bspline
                    // surface and having only one bound represented by Vertex loop was made.
                    // According to the specification of ISO - 10303 part 42:
                    // "If the face has only one bound and this is of type vertex_loop, then the interior of
                    // the face is the domain of the face_surface.face_geometry. In such a case the underlying
                    // surface shall be closed (e.g. a spherical_surface.)"
                    // - natural bounds are applied only in case if VertexLoop is only the one defined face
                    // bound.
                    var anAuxiliaryFaceBuilder = new BRepBuilderAPI_MakeFace(aGeomSurface, Precision());
                    for (var aFaceIt = new Iterator(anAuxiliaryFaceBuilder.InternalShape()); aFaceIt.More(); aFaceIt.Next())
                    {
                        aFaceBuilder.Add(aResultFace, aFaceIt.Value());
                    }

                    continue;
                }

                if (aGeomSurface is ToroidalSurface)
                {
                    continue;
                }

                StepShape_VertexLoop aVertexLoop = aFaceLoop as StepShape_VertexLoop;
                if (aGeomSurface is PlaneSurface)
                {
                    aMessageHandler.AddWarning(aVertexLoop, "VertexLoop on plane is ignored");
                    continue; // smh : BUC60809
                }

                var aVertexLoopTranslator = new StepToTopoDS_TranslateVertexLoop();
                aVertexLoopTranslator.SetPrecision(Precision()); // gka
                aVertexLoopTranslator.SetMaxTol(MaxTol());
                aVertexLoopTranslator.Init(aVertexLoop, theTopoDSTool, theTopoDSToolNM, theLocalFactors);
                if (aVertexLoopTranslator.IsDone())
                {
                    aFaceBuilder.Add(aResultFace, aVertexLoopTranslator.InternalValue());
                }
                else
                {
                    aMessageHandler.AddWarning(aVertexLoop, " a VertexLoop not mapped to TopoDS");
                }
            }

            // ----------------------
            // The Loop is a PolyLoop
            // ----------------------
            else if (aFaceLoop is StepShape_PolyLoop)
            {
                StepShape_PolyLoop aPolyLoop = aFaceLoop as StepShape_PolyLoop;
                aResultFace.Orientation(theFaceSurface.SameSense() ? Forward : Reversed);
                var aPolyLoopTranslator = new StepToTopoDS_TranslatePolyLoop();
                aPolyLoopTranslator.SetPrecision(Precision()); // gka
                aPolyLoopTranslator.SetMaxTol(MaxTol());
                aPolyLoopTranslator.Init(aPolyLoop, theTopoDSTool, aGeomSurface, aResultFace, theLocalFactors);
                if (aPolyLoopTranslator.IsDone())
                {
                    Wire aPolyLoopWire = ShapeCasts.Wire(aPolyLoopTranslator.InternalValue());
                    aPolyLoopWire.Orientation(aFaceBound.Orientation() ? Forward : Reversed);
                    aFaceBuilder.Add(aResultFace, aPolyLoopWire);
                }
                else
                {
                    aMessageHandler.AddWarning(aPolyLoop, " a PolyLoop not mapped to TopoDS");
                }
            }

            // -----------------------
            // The Loop is an EdgeLoop
            // -----------------------
            else if (aFaceLoop is StepShape_EdgeLoop)
            {
                var anEdgeLoopTranslator = new StepToTopoDS_TranslateEdgeLoop();
                anEdgeLoopTranslator.SetPrecision(Precision()); // gka
                anEdgeLoopTranslator.SetMaxTol(MaxTol());
                anEdgeLoopTranslator.Init(aFaceBound, aResultFace, aGeomSurface, aStepGeomSurface, aSameSense, theTopoDSTool, theTopoDSToolNM,
                                          theLocalFactors);

                if (anEdgeLoopTranslator.IsDone())
                {
                    Wire anEdgeLoopWire = ShapeCasts.Wire(anEdgeLoopTranslator.InternalValue());

                    // STEP Face_Surface orientation :
                    // if the topological orientation is opposite to the geometric
                    // orientation of the surface => the underlying topological
                    // orientation are not implicitly reversed
                    // this is the case in CAS.CADE => If the face_surface is reversed,
                    // the wire orientation has to be explicitly reversed
                    if (aFaceBound.Orientation())
                    {
                        anEdgeLoopWire.Orientation(aSameSense ? Forward : Reversed);
                    }
                    else
                    {
                        anEdgeLoopWire.Orientation(aSameSense ? Reversed : Forward);
                    }

                    // -----------------------------
                    // The Wire is added to the Face
                    // -----------------------------
                    aFaceBuilder.Add(aResultFace, anEdgeLoopWire);
                }
                else
                {
                    // There was a problem in the mapping: We lost Face (so to speak...).
                    // No way today to recover at least all the geometries (Points, 3D Curves, Surface).
                    aMessageHandler.AddFail(aFaceLoop, " EdgeLoop not mapped to TopoDS");

                    // CKY JAN-97: a Wire is missing, well we continue anyway
                    // unless OuterBound: that's still not quite normal...
                    if (aFaceBound is StepShape_FaceOuterBound)
                    {
                        aMessageHandler.AddWarning(theFaceSurface, "No Outer Bound : Face not done");
                    }

                    continue;
                }
            }
            else
            {
                // Type not yet implemented or non sens
                aMessageHandler.AddFail(aFaceLoop, " Type of loop not yet implemented");
                continue;
            }
        }

        aResultFace.Orientation(theFaceSurface.SameSense() ? Forward : Reversed);
        theTopoDSTool.Bind(theFaceSurface, aResultFace);

        // Register face in NM tool (ssv; 14.11.2010)
        if (theTopoDSToolNM.IsActive())
        {
            theTopoDSToolNM.Bind(aStepGeomSurface, aResultFace);
        }

        myResult = aResultFace;
        myError = StepToTopoDS_TranslateFaceDone;
        done = true;
    }

    public void Init(StepShape_FaceSurface theFaceSurface, StepToTopoDS_Tool theTopoDSTool, StepToTopoDS_NMTool theTopoDSToolNM)
        => Init(theFaceSurface, theTopoDSTool, theTopoDSToolNM, new StepData_Factors());

    /// <summary>Translates a triangulated_face or a complex_triangulated_face.</summary>
    /// <remarks>
    /// With a geometric link to a face already translated in the transient process, the triangulation is added to that
    /// face (OCCT writes it into the face in place: the face must be the reader's working one, unfrozen).
    /// </remarks>
    public void Init(StepVisual_TessellatedFace theTF, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                     bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom, in StepData_Factors theLocalFactors)
    {
        if (theTF is null)
        {
            return;
        }

        Transfer_TransientProcess aTP = theTool.TransientProcess();
        var aB = new BrepBuilder();
        Face aF = new Face();

        if (theTF.HasGeometricLink())
        {
            TransferBRep_ShapeBinder aBinder = aTP.Find(theTF.GeometricLink().Face()) as TransferBRep_ShapeBinder;
            if (aBinder is not null)
            {
                aF = aBinder.Face();
                theHasGeom = true;
            }

            if (!aF.IsNull && Tool.Surface(aF) is not null && theReadTessellatedWhenNoBRepOnly)
            {
                // Face has BRep but OnNoBRep param is specified
                return;
            }
        }

        if (aF.IsNull)
        {
            aB.MakeFace(ref aF);
            theHasGeom = false;
        }

        Triangulation aMesh;
        if (theTF is StepVisual_TriangulatedFace aTriaF)
        {
            aMesh = createMesh(aTriaF, theLocalFactors);
        }
        else if (theTF is StepVisual_ComplexTriangulatedFace aCompTriaF)
        {
            aMesh = createMesh(aCompTriaF, theLocalFactors);
        }
        else
        {
            aTP.AddWarning(theTF, " Triangulated or ComplexTriangulated entity is supported only.");
            return;
        }

        if (aMesh is null)
        {
            aTP.AddWarning(theTF, " Poly triangulation is not set to TopoDS face.");
            return;
        }

        aB.UpdateFace(aF, aMesh);

        if (theNMTool.IsActive())
        {
            theNMTool.Bind(theTF, aF);
        }

        aTP.Bind(theTF, new TransferBRep_ShapeBinder(aF));

        myResult = aF;
        myError = StepToTopoDS_TranslateFaceDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedFace theTF, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                     bool theReadTessellatedWhenNoBRepOnly, ref bool theHasGeom)
        => Init(theTF, theTool, theNMTool, theReadTessellatedWhenNoBRepOnly, ref theHasGeom, new StepData_Factors());

    /// <summary>Translates a triangulated_surface_set or a complex_triangulated_surface_set.</summary>
    public void Init(StepVisual_TessellatedSurfaceSet theTSS, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool,
                     in StepData_Factors theLocalFactors)
    {
        if (theTSS is null)
        {
            return;
        }

        Transfer_TransientProcess aTP = theTool.TransientProcess();
        var aB = new BrepBuilder();
        Face aF = new Face();
        aB.MakeFace(ref aF);

        Triangulation aMesh;
        if (theTSS is StepVisual_TriangulatedSurfaceSet aTriaSS)
        {
            aMesh = createMesh(aTriaSS, theLocalFactors);
        }
        else if (theTSS is StepVisual_ComplexTriangulatedSurfaceSet aCompTriaSS)
        {
            aMesh = createMesh(aCompTriaSS, theLocalFactors);
        }
        else
        {
            aTP.AddWarning(theTSS, " Triangulated or ComplexTriangulated entity is supported only.");
            return;
        }

        if (aMesh is null)
        {
            aTP.AddWarning(theTSS, " Poly triangulation is not set to TopoDS face.");
            return;
        }

        aB.UpdateFace(aF, aMesh);

        if (theNMTool.IsActive())
        {
            theNMTool.Bind(theTSS, aF);
        }

        myResult = aF;
        myError = StepToTopoDS_TranslateFaceDone;
        done = true;
    }

    public void Init(StepVisual_TessellatedSurfaceSet theTSS, StepToTopoDS_Tool theTool, StepToTopoDS_NMTool theNMTool)
        => Init(theTSS, theTool, theNMTool, new StepData_Factors());

    // Creates a Poly_Triangulation from a simple or complex triangulated face or surface set
    private Triangulation createMesh(StepVisual_TessellatedItem theTI, in StepData_Factors theLocalFactors)
        => CreatePolyTriangulation(theTI, theLocalFactors);

    /// <summary>The face, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateFace::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateFaceError Error() => myError;
}
