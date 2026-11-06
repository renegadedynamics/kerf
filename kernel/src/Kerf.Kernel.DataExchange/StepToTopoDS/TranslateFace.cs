// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateFace.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Healing;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>
/// Translates a face_surface (its surface and its bounds: edge loops, poly loops, vertex loops) into a TopoDS_Face, or
/// a triangulated face or surface set into a face carrying a triangulation.
/// </summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
internal sealed class TranslateFace : ImportRoot
{
    private TranslateFaceError _error;
    private Shape _result = new Shape();

    // Set nodes to the triangulation from an array
    private static void SetNodes(Triangulation mesh, NCollection_HArray1<Xyz> nodes, NCollection_HArray1<int> pnindices,
                                 double lengthFactor)
    {
        if (pnindices is not null)
        {
            for (int pnIndex = 1; pnIndex <= mesh.NodeCount; ++pnIndex)
            {
                Xyz point = nodes.Value(pnindices.Value(pnIndex));
                mesh.SetNode(pnIndex, new Point3D(lengthFactor * point));
            }
        }
        else
        {
            for (int pnIndex = 1; pnIndex <= mesh.NodeCount; ++pnIndex)
            {
                mesh.SetNode(pnIndex, new Point3D(lengthFactor * nodes.Value(pnIndex)));
            }
        }
    }

    // Set normals to the triangulation from an array
    private static void SetNormals(Triangulation mesh, NCollection_HArray2<double> normals, int normNum, int nbNodes)
    {
        if (normals.RowLength() != 3)
        {
            return;
        }

        Xyz norm = new Xyz();
        if (normNum == 1)
        {
            norm.SetX(normals.Value(1, 1));
            norm.SetY(normals.Value(1, 2));
            norm.SetZ(normals.Value(1, 3));
            for (int nodeIndex = 1; nodeIndex <= nbNodes; ++nodeIndex)
            {
                mesh.SetNormal(nodeIndex, new Direction3D(norm));
            }
        }
        else if (nbNodes == normNum)
        {
            for (int normIndex = 1; normIndex <= normNum; ++normIndex)
            {
                norm.SetX(normals.Value(normIndex, 1));
                norm.SetY(normals.Value(normIndex, 2));
                norm.SetZ(normals.Value(normIndex, 3));
                mesh.SetNormal(normIndex, new Direction3D(norm));
            }
        }
    }

    // Set triangles to the triangulation from an array
    private static void SetTriangles(Triangulation mesh, NCollection_HArray2<int> triangles, int trianStripsNum,
                                     NCollection_HArray1<Transient> trianStrips, int trianFansNum,
                                     NCollection_HArray1<Transient> trianFans)
    {
        if (trianStripsNum == 0 && trianFansNum == 0)
        {
            for (int trianIndex = 1; trianIndex <= mesh.TriangleCount; ++trianIndex)
            {
                mesh.SetTriangle(trianIndex, new Triangle1(triangles.Value(trianIndex, 1), triangles.Value(trianIndex, 2),
                                                                   triangles.Value(trianIndex, 3)));
            }
        }
        else
        {
            int triangleIndex = 1;
            for (int trianStripIndex = 1; trianStripIndex <= trianStripsNum; ++trianStripIndex)
            {
                NCollection_HArray1<int> triangleStrip = trianStrips.Value(trianStripIndex) as NCollection_HArray1<int>;
                for (int index = 3; index <= triangleStrip.Length(); index += 2)
                {
                    if (triangleStrip.Value(index) != triangleStrip.Value(index - 2)
                        && triangleStrip.Value(index) != triangleStrip.Value(index - 1))
                    {
                        mesh.SetTriangle(triangleIndex++, new Triangle1(triangleStrip.Value(index - 2), triangleStrip.Value(index),
                                                                                triangleStrip.Value(index - 1)));
                    }
                }

                for (int index = 4; index <= triangleStrip.Length(); index += 2)
                {
                    if (triangleStrip.Value(index) != triangleStrip.Value(index - 2)
                        && triangleStrip.Value(index) != triangleStrip.Value(index - 1))
                    {
                        mesh.SetTriangle(triangleIndex++, new Triangle1(triangleStrip.Value(index - 2),
                                                                                triangleStrip.Value(index - 1),
                                                                                triangleStrip.Value(index)));
                    }
                }
            }

            for (int trianFanIndex = 1; trianFanIndex <= trianFansNum; ++trianFanIndex)
            {
                NCollection_HArray1<int> triangleFan = trianFans.Value(trianFanIndex) as NCollection_HArray1<int>;
                for (int index = 3; index <= triangleFan.Length(); ++index)
                {
                    if (triangleFan.Value(index) != triangleFan.Value(index - 2)
                        && triangleFan.Value(index - 1) != triangleFan.Value(index - 2))
                    {
                        mesh.SetTriangle(triangleIndex++, new Triangle1(triangleFan.Value(1), triangleFan.Value(index),
                                                                                triangleFan.Value(index - 1)));
                    }
                }
            }
        }
    }

    // GetSimpleFaceElements: OCCT's template, once per instantiation
    private static void GetSimpleFaceElements(TriangulatedFace face, out NCollection_HArray1<Xyz> nodes,
                                              out NCollection_HArray2<double> normals, out NCollection_HArray2<int> triangles,
                                              out int normNb, out int triNb, ref NCollection_HArray1<int> pnindices)
    {
        nodes = face.Coordinates().Points();
        normals = face.Normals();
        triangles = face.Triangles();
        normNb = face.NbNormals();
        triNb = face.NbTriangles();
        int pnIndNb = face.NbPnindex();
        if (pnIndNb > 0)
        {
            pnindices = new NCollection_HArray1<int>(1, pnIndNb);
            for (int indx = 1; indx <= pnIndNb; ++indx)
            {
                pnindices.SetValue(indx, face.PnindexValue(indx));
            }
        }
    }

    private static void GetSimpleFaceElements(TriangulatedSurfaceSet face, out NCollection_HArray1<Xyz> nodes,
                                              out NCollection_HArray2<double> normals, out NCollection_HArray2<int> triangles,
                                              out int normNb, out int triNb, ref NCollection_HArray1<int> pnindices)
    {
        nodes = face.Coordinates().Points();
        normals = face.Normals();
        triangles = face.Triangles();
        normNb = face.NbNormals();
        triNb = face.NbTriangles();
        int pnIndNb = face.NbPnindex();
        if (pnIndNb > 0)
        {
            pnindices = new NCollection_HArray1<int>(1, pnIndNb);
            for (int indx = 1; indx <= pnIndNb; ++indx)
            {
                pnindices.SetValue(indx, face.PnindexValue(indx));
            }
        }
    }

    // GetComplexFaceElements: OCCT's template, once per instantiation
    private static void GetComplexFaceElements(ComplexTriangulatedFace face, out NCollection_HArray1<Xyz> nodes,
                                               out NCollection_HArray2<double> normals,
                                               out NCollection_HArray1<Transient> triangleStrips,
                                               out NCollection_HArray1<Transient> triangleFans, out int normNb,
                                               out int triStripsNb, out int triFansNb, ref NCollection_HArray1<int> pnindices)
    {
        nodes = face.Coordinates().Points();
        normals = face.Normals();
        triangleStrips = face.TriangleStrips();
        triangleFans = face.TriangleFans();
        normNb = face.NbNormals();
        triStripsNb = face.NbTriangleStrips();
        triFansNb = face.NbTriangleFans();
        int pnIndNb = face.NbPnindex();
        if (pnIndNb > 0)
        {
            pnindices = new NCollection_HArray1<int>(1, pnIndNb);
            for (int indx = 1; indx <= pnIndNb; ++indx)
            {
                pnindices.SetValue(indx, face.PnindexValue(indx));
            }
        }
    }

    private static void GetComplexFaceElements(ComplexTriangulatedSurfaceSet face, out NCollection_HArray1<Xyz> nodes,
                                               out NCollection_HArray2<double> normals,
                                               out NCollection_HArray1<Transient> triangleStrips,
                                               out NCollection_HArray1<Transient> triangleFans, out int normNb,
                                               out int triStripsNb, out int triFansNb, ref NCollection_HArray1<int> pnindices)
    {
        nodes = face.Coordinates().Points();
        normals = face.Normals();
        triangleStrips = face.TriangleStrips();
        triangleFans = face.TriangleFans();
        normNb = face.NbNormals();
        triStripsNb = face.NbTriangleStrips();
        triFansNb = face.NbTriangleFans();
        int pnIndNb = face.NbPnindex();
        if (pnIndNb > 0)
        {
            pnindices = new NCollection_HArray1<int>(1, pnIndNb);
            for (int indx = 1; indx <= pnIndNb; ++indx)
            {
                pnindices.SetValue(indx, face.PnindexValue(indx));
            }
        }
    }

    private static Triangulation CreatePolyTriangulation(TessellatedItem ti, in Factors localFactors)
    {
        Triangulation mesh;
        if (ti is null)
        {
            return null;
        }

        NCollection_HArray1<Xyz> nodes;
        NCollection_HArray2<double> normals;
        NCollection_HArray2<int> triangles = null;
        int normNum = 0;
        int trianNum = 0;
        NCollection_HArray1<int> pnindices = null;

        NCollection_HArray1<Transient> triaStrips = null;
        NCollection_HArray1<Transient> triaFans = null;
        int trianStripsNum = 0;
        int trianFansNum = 0;

        if (ti is TriangulatedFace tf)
        {
            GetSimpleFaceElements(tf, out nodes, out normals, out triangles, out normNum, out trianNum, ref pnindices);
        }
        else if (ti is TriangulatedSurfaceSet tss)
        {
            GetSimpleFaceElements(tss, out nodes, out normals, out triangles, out normNum, out trianNum, ref pnindices);
        }
        else if (ti is ComplexTriangulatedFace ctf)
        {
            GetComplexFaceElements(ctf, out nodes, out normals, out triaStrips, out triaFans, out normNum, out trianStripsNum,
                                   out trianFansNum, ref pnindices);
        }
        else if (ti is ComplexTriangulatedSurfaceSet ctss)
        {
            GetComplexFaceElements(ctss, out nodes, out normals, out triaStrips, out triaFans, out normNum, out trianStripsNum,
                                   out trianFansNum, ref pnindices);
        }
        else
        {
            return null;
        }

        const bool hasUVNodes = false;
        bool hasNormals = (normNum > 0);
        int nbNodes = pnindices is not null ? pnindices.Length() : nodes.Length();

        if (trianStripsNum == 0 && trianFansNum == 0)
        {
            mesh = new Triangulation(nbNodes, trianNum, hasUVNodes, hasNormals);
        }
        else
        {
            int nbTriaStrips = 0;
            int nbTriaFans = 0;

            for (int trianStripIndex = 1; trianStripIndex <= trianStripsNum; ++trianStripIndex)
            {
                NCollection_HArray1<int> triangleStrip = triaStrips.Value(trianStripIndex) as NCollection_HArray1<int>;
                for (int index = 3; index <= triangleStrip.Length(); index += 2)
                {
                    if (triangleStrip.Value(index) != triangleStrip.Value(index - 2)
                        && triangleStrip.Value(index) != triangleStrip.Value(index - 1))
                    {
                        ++nbTriaStrips;
                    }
                }

                for (int index = 4; index <= triangleStrip.Length(); index += 2)
                {
                    if (triangleStrip.Value(index) != triangleStrip.Value(index - 2)
                        && triangleStrip.Value(index) != triangleStrip.Value(index - 1))
                    {
                        ++nbTriaStrips;
                    }
                }
            }

            for (int trianFanIndex = 1; trianFanIndex <= trianFansNum; ++trianFanIndex)
            {
                NCollection_HArray1<int> triangleFan = triaFans.Value(trianFanIndex) as NCollection_HArray1<int>;
                nbTriaFans += triangleFan.Length() - 2;
            }

            mesh = new Triangulation(nbNodes, nbTriaStrips + nbTriaFans, hasUVNodes, hasNormals);
        }

        SetNodes(mesh, nodes, pnindices, localFactors.LengthFactor());

        if (hasNormals)
        {
            SetNormals(mesh, normals, normNum, nbNodes);
        }

        SetTriangles(mesh, triangles, trianStripsNum, triaStrips, trianFansNum, triaFans);

        return mesh;
    }

    public TranslateFace()
    {
        _error = TranslateFaceError.TranslateFaceOther;
        _done = false;
    }

    /// <summary>Translates a face_surface.</summary>
    public TranslateFace(StepFaceSurface fs, ImportTool t, NMTool nmTool, in Factors localFactors)
    {
        Init(fs, t, nmTool, localFactors);
    }

    public TranslateFace(StepFaceSurface fs, ImportTool t, NMTool nmTool)
        : this(fs, t, nmTool, new Factors())
    {
    }

    /// <summary>Translates a triangulated_face or a complex_triangulated_face.</summary>
    public TranslateFace(TessellatedFace tf, ImportTool tool, NMTool nmTool,
                                      bool readTessellatedWhenNoBRepOnly, ref bool hasGeom, in Factors localFactors)
    {
        Init(tf, tool, nmTool, readTessellatedWhenNoBRepOnly, ref hasGeom, localFactors);
    }

    public TranslateFace(TessellatedFace tf, ImportTool tool, NMTool nmTool,
                                      bool readTessellatedWhenNoBRepOnly, ref bool hasGeom)
        : this(tf, tool, nmTool, readTessellatedWhenNoBRepOnly, ref hasGeom, new Factors())
    {
    }

    /// <summary>Translates a triangulated_surface_set or a complex_triangulated_surface_set.</summary>
    public TranslateFace(TessellatedSurfaceSet tss, ImportTool tool, NMTool nmTool,
                                      in Factors localFactors)
    {
        Init(tss, tool, nmTool, localFactors);
    }

    public TranslateFace(TessellatedSurfaceSet tss, ImportTool tool, NMTool nmTool)
        : this(tss, tool, nmTool, new Factors())
    {
    }

    private static bool IsReversed(StepSurface stepSurf)
    {
        if (stepSurf is StepRectangularTrimmedSurface rts)
        {
            return IsReversed(rts.BasisSurface());
        }
        else
        {
            StepToroidalSurface stepTorSur = stepSurf as StepToroidalSurface;
            return stepTorSur is not null && stepTorSur.MajorRadius() < 0.0;
        }
    }

    /// <summary>Translates a face_surface.</summary>
    /// <remarks>Needs <see cref="ShapeAlgo.Init"/> (B-spline surfaces are made periodic through its container).</remarks>
    public void Init(StepFaceSurface faceSurface, ImportTool topoDSTool, NMTool topoDSToolNM,
                     in Factors localFactors)
    {
        _done = true;
        if (topoDSTool.IsBound(faceSurface))
        {
            _result = ShapeCasts.Face(topoDSTool.Find(faceSurface));
            _error = TranslateFaceError.TranslateFaceDone;
            _done = true;
            return;
        }

        // Within a context of this method this object is used for message handling only.
        TransientProcess messageHandler = topoDSTool.TransientProcess();

        // ----------------------------------------------
        // Map the Face Geometry and create a TopoDS_Face
        // ----------------------------------------------
        StepSurface stepGeomSurface = faceSurface.FaceGeometry();
        // sln 01.10.2001 BUC61003. If corresponding entity was read with error StepSurface may be NULL.
        // In this case we exit from function
        if (stepGeomSurface is null)
        {
            messageHandler.AddFail(stepGeomSurface, " Surface has not been created");
            _error = TranslateFaceError.TranslateFaceOther;
            _done = false;
            return;
        }

        // [BEGIN] Added to process non-manifold topology (ssv; 14.11.2010)
        if (topoDSToolNM.IsActive && topoDSToolNM.IsBound(stepGeomSurface))
        {
            Shape existingShape = topoDSToolNM.Find(stepGeomSurface);
            // Reverse shape's orientation for the next shell
            existingShape.Reverse();
            _result = existingShape;
            _error = TranslateFaceError.TranslateFaceDone;
            _done = true;
            return;
        }
        // [END] Added to process non-manifold topology (ssv; 14.11.2010)

        if (stepGeomSurface is StepOffsetSurface) //: d4 abv 12 Mar 98
        {
            messageHandler.AddWarning(stepGeomSurface, " Type OffsetSurface is out of scope of AP 214");
        }

        Surface geomSurface = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurface(stepGeomSurface, localFactors);
        if (geomSurface is null)
        {
            messageHandler.AddFail(stepGeomSurface, " Surface has not been created");
            _error = TranslateFaceError.TranslateFaceOther;
            _done = false;
            return;
        }

        // pdn to force bsplsurf to be periodic
        if (stepGeomSurface is StepBSplineSurface)
        {
            Surface periodicSurf = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().ConvertToPeriodic(geomSurface);
            if (periodicSurf is not null)
            {
                messageHandler.AddWarning(stepGeomSurface, "Surface forced to be periodic");
                geomSurface = periodicSurf;
            }
        }

        // fix for bug 0026376 Solid Works wrote face based on toroidal surface having negative major
        // radius seems that such case is interpreted  by "Solid Works" and "ProE" as face having reversed
        // orientation.
        bool sameSense = IsReversed(stepGeomSurface) ? !faceSurface.SameSense() : faceSurface.SameSense();

        // -- Statistics --
        topoDSTool.AddContinuity(geomSurface);

        Face resultFace = new Face();
        var faceBuilder = new BrepBuilder();
        faceBuilder.MakeFace(ref resultFace, geomSurface, Kerf.Engine.Foundation.Precision.Confusion());

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
        for (int boundIndex = 1; boundIndex <= faceSurface.NbBounds(); ++boundIndex)
        {
            StepFaceBound faceBound = faceSurface.BoundsValue(boundIndex);
            if (faceBound is null)
            {
                continue;
            }

            StepLoop faceLoop = faceBound.Bound();
            if (faceLoop is null)
            {
                continue;
            }

            // ------------------------
            // The Loop is a VertexLoop
            // ------------------------
            if (faceLoop is StepVertexLoop)
            {
                //: S4136      STF.Closed() = false;
                //  PROBLEM if SPHERE or TORE
                // It will be necessary to make a complete wire, provided that the point carries on the face
                // In the meantime, we do nothing

                // abv 10.07.00 pr1sy.stp: vertex_loop can be wrong; so just make natural bounds
                if ((geomSurface is SphericalSurface || geomSurface is BSplineSurface || geomSurface is RevolutionSurface)
                    && (faceSurface.NbBounds() == 1))
                {
                    // Modification to create natural bounds for face based on the spherical and Bspline
                    // surface and having only one bound represented by Vertex loop was made.
                    // According to the specification of ISO - 10303 part 42:
                    // "If the face has only one bound and this is of type vertex_loop, then the interior of
                    // the face is the domain of the face_surface.face_geometry. In such a case the underlying
                    // surface shall be closed (e.g. a spherical_surface.)"
                    // - natural bounds are applied only in case if VertexLoop is only the one defined face
                    // bound.
                    var auxiliaryFaceBuilder = new MakeFace(geomSurface, Precision());
                    for (var faceIt = new Iterator(auxiliaryFaceBuilder.InternalShape()); faceIt.More(); faceIt.Next())
                    {
                        faceBuilder.Add(resultFace, faceIt.Value());
                    }

                    continue;
                }

                if (geomSurface is ToroidalSurface)
                {
                    continue;
                }

                StepVertexLoop vertexLoop = faceLoop as StepVertexLoop;
                if (geomSurface is PlaneSurface)
                {
                    messageHandler.AddWarning(vertexLoop, "VertexLoop on plane is ignored");
                    continue; // smh : BUC60809
                }

                var vertexLoopTranslator = new TranslateVertexLoop();
                vertexLoopTranslator.SetPrecision(Precision()); // gka
                vertexLoopTranslator.SetMaxTol(MaxTol());
                vertexLoopTranslator.Init(vertexLoop, topoDSTool, topoDSToolNM, localFactors);
                if (vertexLoopTranslator.IsDone)
                {
                    faceBuilder.Add(resultFace, vertexLoopTranslator.InternalValue());
                }
                else
                {
                    messageHandler.AddWarning(vertexLoop, " a VertexLoop not mapped to TopoDS");
                }
            }

            // ----------------------
            // The Loop is a PolyLoop
            // ----------------------
            else if (faceLoop is StepPolyLoop)
            {
                StepPolyLoop polyLoop = faceLoop as StepPolyLoop;
                resultFace.Orient(faceSurface.SameSense() ? Forward : Reversed);
                var polyLoopTranslator = new TranslatePolyLoop();
                polyLoopTranslator.SetPrecision(Precision()); // gka
                polyLoopTranslator.SetMaxTol(MaxTol());
                polyLoopTranslator.Init(polyLoop, topoDSTool, geomSurface, resultFace, localFactors);
                if (polyLoopTranslator.IsDone)
                {
                    Wire polyLoopWire = ShapeCasts.Wire(polyLoopTranslator.InternalValue());
                    polyLoopWire.Orient(faceBound.Orientation() ? Forward : Reversed);
                    faceBuilder.Add(resultFace, polyLoopWire);
                }
                else
                {
                    messageHandler.AddWarning(polyLoop, " a PolyLoop not mapped to TopoDS");
                }
            }

            // -----------------------
            // The Loop is an EdgeLoop
            // -----------------------
            else if (faceLoop is StepEdgeLoop)
            {
                var edgeLoopTranslator = new TranslateEdgeLoop();
                edgeLoopTranslator.SetPrecision(Precision()); // gka
                edgeLoopTranslator.SetMaxTol(MaxTol());
                edgeLoopTranslator.Init(faceBound, resultFace, geomSurface, stepGeomSurface, sameSense, topoDSTool, topoDSToolNM,
                                          localFactors);

                if (edgeLoopTranslator.IsDone)
                {
                    Wire edgeLoopWire = ShapeCasts.Wire(edgeLoopTranslator.InternalValue());

                    // STEP Face_Surface orientation :
                    // if the topological orientation is opposite to the geometric
                    // orientation of the surface => the underlying topological
                    // orientation are not implicitly reversed
                    // this is the case in CAS.CADE => If the face_surface is reversed,
                    // the wire orientation has to be explicitly reversed
                    if (faceBound.Orientation())
                    {
                        edgeLoopWire.Orient(sameSense ? Forward : Reversed);
                    }
                    else
                    {
                        edgeLoopWire.Orient(sameSense ? Reversed : Forward);
                    }

                    // -----------------------------
                    // The Wire is added to the Face
                    // -----------------------------
                    faceBuilder.Add(resultFace, edgeLoopWire);
                }
                else
                {
                    // There was a problem in the mapping: We lost Face (so to speak...).
                    // No way today to recover at least all the geometries (Points, 3D Curves, Surface).
                    messageHandler.AddFail(faceLoop, " EdgeLoop not mapped to TopoDS");

                    // CKY JAN-97: a Wire is missing, well we continue anyway
                    // unless OuterBound: that's still not quite normal...
                    if (faceBound is StepFaceOuterBound)
                    {
                        messageHandler.AddWarning(faceSurface, "No Outer Bound : Face not done");
                    }

                    continue;
                }
            }
            else
            {
                // Type not yet implemented or non sens
                messageHandler.AddFail(faceLoop, " Type of loop not yet implemented");
                continue;
            }
        }

        resultFace.Orient(faceSurface.SameSense() ? Forward : Reversed);
        topoDSTool.Bind(faceSurface, resultFace);

        // Register face in NM tool (ssv; 14.11.2010)
        if (topoDSToolNM.IsActive)
        {
            topoDSToolNM.Bind(stepGeomSurface, resultFace);
        }

        _result = resultFace;
        _error = TranslateFaceError.TranslateFaceDone;
        _done = true;
    }

    public void Init(StepFaceSurface faceSurface, ImportTool topoDSTool, NMTool topoDSToolNM)
        => Init(faceSurface, topoDSTool, topoDSToolNM, new Factors());

    /// <summary>Translates a triangulated_face or a complex_triangulated_face.</summary>
    /// <remarks>
    /// With a geometric link to a face already translated in the transient process, the triangulation is added to that
    /// face (OCCT writes it into the face in place: the face must be the reader's working one, unfrozen).
    /// </remarks>
    public void Init(TessellatedFace tf, ImportTool tool, NMTool nmTool,
                     bool readTessellatedWhenNoBRepOnly, ref bool hasGeom, in Factors localFactors)
    {
        if (tf is null)
        {
            return;
        }

        TransientProcess tp = tool.TransientProcess();
        var b = new BrepBuilder();
        Face f = new Face();

        if (tf.HasGeometricLink)
        {
            ShapeBinder binder = tp.Find(tf.GeometricLink().Face()) as ShapeBinder;
            if (binder is not null)
            {
                f = binder.Face();
                hasGeom = true;
            }

            if (!f.IsNull && Tool.Surface(f) is not null && readTessellatedWhenNoBRepOnly)
            {
                // Face has BRep but OnNoBRep param is specified
                return;
            }
        }

        if (f.IsNull)
        {
            b.MakeFace(ref f);
            hasGeom = false;
        }

        Triangulation mesh;
        if (tf is TriangulatedFace triaF)
        {
            mesh = CreateMesh(triaF, localFactors);
        }
        else if (tf is ComplexTriangulatedFace compTriaF)
        {
            mesh = CreateMesh(compTriaF, localFactors);
        }
        else
        {
            tp.AddWarning(tf, " Triangulated or ComplexTriangulated entity is supported only.");
            return;
        }

        if (mesh is null)
        {
            tp.AddWarning(tf, " Poly triangulation is not set to TopoDS face.");
            return;
        }

        b.UpdateFace(f, mesh);

        if (nmTool.IsActive)
        {
            nmTool.Bind(tf, f);
        }

        tp.Bind(tf, new ShapeBinder(f));

        _result = f;
        _error = TranslateFaceError.TranslateFaceDone;
        _done = true;
    }

    public void Init(TessellatedFace tf, ImportTool tool, NMTool nmTool,
                     bool readTessellatedWhenNoBRepOnly, ref bool hasGeom)
        => Init(tf, tool, nmTool, readTessellatedWhenNoBRepOnly, ref hasGeom, new Factors());

    /// <summary>Translates a triangulated_surface_set or a complex_triangulated_surface_set.</summary>
    public void Init(TessellatedSurfaceSet tss, ImportTool tool, NMTool nmTool,
                     in Factors localFactors)
    {
        if (tss is null)
        {
            return;
        }

        TransientProcess tp = tool.TransientProcess();
        var b = new BrepBuilder();
        Face f = new Face();
        b.MakeFace(ref f);

        Triangulation mesh;
        if (tss is TriangulatedSurfaceSet triaSS)
        {
            mesh = CreateMesh(triaSS, localFactors);
        }
        else if (tss is ComplexTriangulatedSurfaceSet compTriaSS)
        {
            mesh = CreateMesh(compTriaSS, localFactors);
        }
        else
        {
            tp.AddWarning(tss, " Triangulated or ComplexTriangulated entity is supported only.");
            return;
        }

        if (mesh is null)
        {
            tp.AddWarning(tss, " Poly triangulation is not set to TopoDS face.");
            return;
        }

        b.UpdateFace(f, mesh);

        if (nmTool.IsActive)
        {
            nmTool.Bind(tss, f);
        }

        _result = f;
        _error = TranslateFaceError.TranslateFaceDone;
        _done = true;
    }

    public void Init(TessellatedSurfaceSet tss, ImportTool tool, NMTool nmTool)
        => Init(tss, tool, nmTool, new Factors());

    // Creates a Poly_Triangulation from a simple or complex triangulated face or surface set
    private Triangulation CreateMesh(TessellatedItem ti, in Factors localFactors)
        => CreatePolyTriangulation(ti, localFactors);

    /// <summary>The face, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslateFace::Value() - no result");
        return _result;
    }

    public TranslateFaceError Error() => _error;
}
