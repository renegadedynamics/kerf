// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// This is the implementation of the extension of the 3D offset algorithm
// to work in mode Complete and Join Type Intersection.
// Currently only the Planar cases are supported.

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Offsets;

// OFFSET_DEBUG blocks (the BRepAlgoAPI_Check of the edges and the compounds built only to be shown) are left out.

//=======================================================================
// function : BRepOffset_BuildOffsetFaces
// purpose  : Auxiliary local class that is used here for building splits
//           of offset faces, that are further used for building volumes.
//=======================================================================

/// <summary>
/// Builds the splits of the offset faces in the Complete mode with the Intersection join (planar shapes), used to
/// build the volumes of the offset. A local class of BRepOffset_MakeOffset_1.cxx.
/// </summary>
/// <remarks>
/// A working object of the offset (decision 21): it writes into the offset's own working shapes (the AsDes edges, the
/// splits it builds, vertex tolerances of micro edges), never into the caller's. Maps that OCCT holds by value inside
/// other maps (lists, maps and indexed maps of shapes) are copied wherever OCCT copies them. Shape-keyed maps iterate in
/// insertion order where OCCT's iterate by address (decision 1032).
/// </remarks>
internal sealed class BuildOffsetFaces
{
    //=======================================================================
    // function : AddToContainer
    // purpose  : Set of methods to add a shape into container
    //=======================================================================
    // Kerf: the overloads of OCCT's templates (TakeModified) dispatch on the container's type at run time.
    private static bool AddToContainer(in Shape s, object container)
    {
        switch (container)
        {
            case NCollection_List<Shape> list:
                list.Append(s);
                return true;
            case HashSet<Shape> map:
                return map.Add(s);
            case IndexedSet<Shape> iMap:
            {
                int nb = iMap.Count;
                int ind = iMap.Add(s) + 1;
                return ind > nb;
            }
            case Shape sOut:
                new BrepBuilder().Add(sOut, s);
                return true;
            case Compound cOut:
                new BrepBuilder().Add(cOut, s);
                return true;
            default:
                throw new ProgramErrorException("BRepOffset_BuildOffsetFaces::AddToContainer");
        }
    }

    private static void AddToContainer(in Shape key, in Shape value,
                                       Dictionary<Shape, NCollection_List<Shape>> map)
    {
        if (map.TryGetValue(key, out NCollection_List<Shape>? pList))
        {
            pList.Append(value);
        }
        else
        {
            NCollection_List<Shape> list = new();
            list.Append(value);
            map[key] = list;
        }
    }

    //=======================================================================
    // function : TakeModified
    // purpose  : Check if the shape has images in the given images map.
    //           Puts in the output map either the images or the shape itself.
    //=======================================================================
    private static bool TakeModified(in Shape s,
                                     Dictionary<Shape, NCollection_List<Shape>> images,
                                     object container, object? mFence)
    {
        if (images.TryGetValue(s, out NCollection_List<Shape>? pLSIm))
        {
            NCollection_List<Shape>.Iterator itLSIm = new(pLSIm);
            for (; itLSIm.More(); itLSIm.Next())
            {
                Shape sIm = itLSIm.Value();
                if (mFence is null || AddToContainer(sIm, mFence))
                {
                    AddToContainer(sIm, container);
                }
            }
            return true;
        }
        else
        {
            if (mFence is null || AddToContainer(s, mFence))
            {
                AddToContainer(s, container);
            }
            return false;
        }
    }

    private static bool TakeModified(in Shape s,
                                     Dictionary<Shape, NCollection_List<Shape>> images,
                                     object mapOut)
    {
        return TakeModified(s, images, mapOut, null);
    }

    //=======================================================================
    // function : hasData
    // purpose  : Checks if container has any data in it
    //=======================================================================
    private static bool HasData<TK, TV>(Dictionary<TK, TV>? data) where TK : notnull =>
        data is not null && data.Count != 0;

    private static bool HasData(NCollection_List<Shape>? data) => data is not null && !data.IsEmpty();

    //=======================================================================
    // function : AppendToList
    // purpose  : Add to a list only unique elements
    //=======================================================================
    private static void AppendToList(NCollection_List<Shape> list, in Shape shape)
    {
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            Shape s = it.Value();
            if (s.IsSame(shape))
            {
                return;
            }
        }
        list.Append(shape);
    }

    //=======================================================================
    // function : ProcessMicroEdge
    // purpose  : Checking if the edge is micro edge
    //=======================================================================
    private static bool ProcessMicroEdge(in Edge edge, IntersectionContext ctx)
    {
        Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
        if (v1.IsNull || v2.IsNull)
        {
            return false;
        }

        bool bMicro = AlgoTools.IsMicroEdge(edge, ctx);
        if (bMicro && new EdgeCurveAdaptor(edge).GetType() == Line)
        {
            double len = Tool.Pnt(v1).DistanceTo(Tool.Pnt(v2));
            new BrepBuilder().UpdateVertex(v1, len / 2.0);
            new BrepBuilder().UpdateVertex(v2, len / 2.0);
        }

        return bMicro;
    }

    // OCCT's BOPAlgo_Builder::Modified (a const reference to the history's list), as a list of its own.
    private static NCollection_List<Shape> ModifiedOf(BuilderShape gf, in Shape s) =>
        new(gf.Modified(s));

    //=================================================================================================

    private static void UpdateOrigins(NCollection_List<Shape> la,
                                      Dictionary<Shape, NCollection_List<Shape>> origins,
                                      BooleanBuilder gf)
    {
        for (NCollection_List<Shape>.Iterator itA = new(la); itA.More(); itA.Next())
        {
            Shape s = itA.Value();

            NCollection_List<Shape> lsIm = ModifiedOf(gf, s);
            if (lsIm.IsEmpty())
            {
                continue;
            }

            NCollection_List<Shape> lsEmpt = new();
            if (!origins.TryGetValue(s, out NCollection_List<Shape>? pLS))
            {
                pLS = lsEmpt;
                pLS.Append(s);
            }

            for (NCollection_List<Shape>.Iterator it = new(lsIm); it.More(); it.Next())
            {
                Shape sIm = it.Value();
                if (origins.TryGetValue(sIm, out NCollection_List<Shape>? pLSOr))
                {
                    // merge two lists
                    for (NCollection_List<Shape>.Iterator it1 = new(pLS); it1.More(); it1.Next())
                    {
                        AppendToList(pLSOr, it1.Value());
                    }
                }
                else
                {
                    origins[sIm] = new NCollection_List<Shape>(pLS);
                }
            }
        }
    }

    //=================================================================================================

    private static void UpdateImages(NCollection_List<Shape> la,
                                     Dictionary<Shape, NCollection_List<Shape>> images,
                                     BooleanBuilder gf, HashSet<Shape> modified)
    {
        for (NCollection_List<Shape>.Iterator it = new(la); it.More(); it.Next())
        {
            Shape s = it.Value();

            if (!images.TryGetValue(s, out NCollection_List<Shape>? pLSIm))
            {
                NCollection_List<Shape> lsIm = ModifiedOf(gf, s);
                if (lsIm.Extent() != 0)
                {
                    images[s] = lsIm;
                    modified.Add(s);
                }
                continue;
            }

            HashSet<Shape> mFence = new(ShapeHasher.Instance);
            NCollection_List<Shape> lsImNew = new();

            bool bModified = false;

            // check modifications of the images
            for (NCollection_List<Shape>.Iterator it1 = new(pLSIm); it1.More(); it1.Next())
            {
                Shape sIm = it1.Value();
                bModified |= TakeModified(sIm, gf.Images(), lsImNew, mFence);
            }

            if (bModified)
            {
                pLSIm.Assign(lsImNew);
                modified.Add(s);
            }
        }
    }

    //=======================================================================
    // function : FindCommonParts
    // purpose  : Looking for the parts of type <theType> contained in both lists
    //=======================================================================
    private static void FindCommonParts(NCollection_List<Shape> ls1, NCollection_List<Shape> ls2,
                                        NCollection_List<Shape> lsc, ShapeType type = ShapeType.Edge)
    {
        // map shapes in the first list
        IndexedSet<Shape> ms1 = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator it = new(ls1); it.More(); it.Next())
        {
            Shape s = it.Value();
            Exploration.MapShapes(s, type, ms1);
        }
        if (ms1.Count == 0)
        {
            return;
        }

        // check for such shapes in the other list
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator it = new(ls2); it.More(); it.Next())
        {
            Shape s = it.Value();

            Explorer exp = new(s, type);
            for (; exp.More(); exp.Next())
            {
                Shape st = exp.Current();
                if (ms1.Contains(st) && mFence.Add(st))
                {
                    lsc.Append(st);
                }
            }
        }
    }

    //=======================================================================
    // function : NbPoints
    // purpose  : Defines number of sample points to get average direction of the edge
    //=======================================================================
    private static int NbPoints(in Edge edge)
    {
        EdgeCurveAdaptor bac = new(edge);
        switch (bac.GetType())
        {
            case Line:
                return 1;
            default:
                return 11;
        }
    }

    //=======================================================================
    // function : FindShape
    // purpose  : Looking for the same sub-shape in the shape
    //=======================================================================
    private static bool FindShape(in Shape sWhat, in Shape sWhere, Analyse? analyse,
                                  ref Shape res)
    {
        bool bFound = false;
        ShapeType type = sWhat.Type;
        Explorer exp = new(sWhere, type);
        for (; exp.More(); exp.Next())
        {
            Shape s = exp.Current();
            if (s.IsSame(sWhat))
            {
                res = s;
                bFound = true;
                break;
            }
        }

        if (!bFound && analyse is not null)
        {
            NCollection_List<Shape>? pLD = analyse.Descendants(sWhere);
            if (pLD is not null)
            {
                for (NCollection_List<Shape>.Iterator it = new(pLD); it.More(); it.Next())
                {
                    Shape s = it.Value();
                    if (s.IsSame(sWhat))
                    {
                        res = s;
                        bFound = true;
                        break;
                    }
                }
            }
        }

        return bFound;
    }

    //=======================================================================
    // function : BuildSplitsOfTrimmedFace
    // purpose  : Building the splits of offset face
    //=======================================================================
    private static void BuildSplitsOfTrimmedFace(in Face face, in Shape edges,
                                                 NCollection_List<Shape> lfImages,
                                                 ProgressRange range)
    {
        Splitter splitter = new();
        //
        splitter.AddArgument(face);
        splitter.AddArgument(edges);
        splitter.SetToFillHistory(false);
        splitter.Perform(range);
        if (splitter.HasErrors)
        {
            return;
        }
        //
        // splits of the offset shape
        for (Explorer exp = new(splitter.InternalShape(), ShapeType.Face); exp.More(); exp.Next())
        {
            lfImages.Append(exp.Current());
        }
    }

    //=======================================================================
    // function : BuildSplitsOfFace
    // purpose  : Building the splits of offset face
    //=======================================================================
    private static void BuildSplitsOfFace(in Face face, in Shape edges,
                                          Dictionary<Shape, Shape> facesOrigins,
                                          NCollection_List<Shape> lfImages)
    {
        lfImages.Clear();
        //
        // take edges to split the face
        NCollection_List<Shape> le = new();
        Explorer exp = new(edges, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            e.Orient(Forward);
            le.Append(e);
            e.Orient(Reversed);
            le.Append(e);
        }
        //
        Face ff = face;
        Orientation or = face.Orientation;
        ff.Orient(Forward);
        //
        // build pcurves for edges on the face
        BrepLibrary.BuildPCurveForEdgesOnPlane(le, ff);
        //
        // build splits of faces
        BuilderFace bf = new();
        bf.SetFace(ff);
        bf.SetShapes(le);
        bf.Perform();
        if (bf.HasErrors)
        {
            return;
        }
        //
        NCollection_List<Shape> lfSp = bf.Areas();
        NCollection_List<Shape>.Iterator itLF = new(lfSp);
        for (; itLF.More(); itLF.Next())
        {
            ref Shape fSp = ref itLF.ChangeValue();
            fSp.Orient(or);
            lfImages.Append(fSp);
            //
            facesOrigins[fSp] = face;
        }
    }

    //=======================================================================
    // function : GetAverageTangent
    // purpose  : Computes average tangent vector along the curve
    //=======================================================================
    private static Vector3D GetAverageTangent(in Shape s, int nbP)
    {
        Vector3D va = new();
        Explorer exp = new(s, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            //
            Curve3D c = Tool.Curve(e, out double t1, out double t2)!;
            //
            Vector3D vSum = new();
            double t = t1;
            double dt = (t2 - t1) / nbP;
            while (t <= t2)
            {
                c.D1(t, out Point3D p, out Vector3D v);
                vSum += v.Normalized();
                t += dt;
            }
            //
            if (e.Orientation == Reversed)
            {
                vSum = vSum.Reversed();
            }
            //
            va += vSum;
        }
        return va;
    }

    // Input data
    private NCollection_List<Shape>? _faces; //!< Input faces which have to be split
    private AsDes? _asDes; //!< Ascendants/descendants of the edges faces
    private Analyse? _analyzer; //!< Analyzer of the input parameters

    //!< Origins of the offset edges (binding between offset edge and original edge)
    private Dictionary<Shape, NCollection_List<Shape>>? _edgesOrigins;
    //!< Origins of the offset faces (binding between offset face and original face)
    private Dictionary<Shape, Shape>? _facesOrigins;
    //!< Binding between trimmed and infinite edge
    private Dictionary<Shape, Shape>? _eTrimEInf;

    // Intermediate data
    //!< Images of the offset edges
    private readonly Dictionary<Shape, NCollection_List<Shape>> _oeImages = new(ShapeHasher.Instance);
    //!< Origins of the splits of offset edges
    private readonly Dictionary<Shape, NCollection_List<Shape>> _oeOrigins = new(ShapeHasher.Instance);
    //!< Images of the offset edges
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _ofImages = new(ShapeHasher.Instance);

    //!< Edges considered invalid (orientation is changed) in some split of face
    private IndexedSet<Shape> _invalidEdges = new(ShapeHasher.Instance);
    //!< Edges considered valid (orientation is kept) in some split of face
    private readonly IndexedSet<Shape> _validEdges = new(ShapeHasher.Instance);
    //!< Edges considered inverted (vertices swapped) in some split of face
    private readonly IndexedSet<Shape> _invertedEdges = new(ShapeHasher.Instance);
    //!< Splits of edges to be avoided when building splits of faces
    private IndexedSet<Shape> _edgesToAvoid = new(ShapeHasher.Instance);
    //!< Edges marked invalid on the current step and to be avoided on the next step
    private readonly HashSet<Shape> _lastInvEdges = new(ShapeHasher.Instance);
    //!< Edges to be used for building splits
    private readonly HashSet<Shape> _modifiedEdges = new(ShapeHasher.Instance);
    //!< Edges located fully inside solids built from the splits of offset faces
    private readonly IndexedSet<Shape> _insideEdges = new(ShapeHasher.Instance);

    //!< Invalid faces - splits of offset faces consisting of invalid edges
    private IndexedMap<Shape, NCollection_List<Shape>> _invalidFaces = new(ShapeHasher.Instance);
    //!< Artificially invalid faces - valid faces intentionally marked invalid to be rebuilt on the future steps in the
    //!  situations when invalid edges are present, but invalid faces not
    private readonly Dictionary<Shape, IndexedSet<Shape>> _artInvalidFaces = new(ShapeHasher.Instance);
    //!< Count number of the same face being marked invalid to avoid infinite rebuilding of the same face
    private readonly Dictionary<Shape, int> _alreadyInvFaces = new(ShapeHasher.Instance);
    //!< Images of the hole faces of the original face
    private readonly Dictionary<Shape, NCollection_List<Shape>> _fNewHoles = new(ShapeHasher.Instance);

    //!< Intersection information, used to collect intersection pairs during rebuild
    private readonly Dictionary<Shape, NCollection_List<Shape>> _ssInterfs = new(ShapeHasher.Instance);
    //!< Intersection information, used to collect intersection pairs during rebuild
    private readonly Dictionary<Shape, NCollection_List<Shape>> _ssInterfsArt = new(ShapeHasher.Instance);
    //!< All possible intersection pairs, used to avoid some of the intersections
    private readonly Dictionary<Shape, Dictionary<Shape, HashSet<Shape>>> _intersectionPairs = new(ShapeHasher.Instance);

    //!< Faces that have to be rebuilt (invalid and close to invalid faces)
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _facesToRebuild = new(ShapeHasher.Instance);
    //!< Faces that have to be avoided when rebuilding splits of the same offset face
    private readonly HashSet<Shape> _fSelfRebAvoid = new(ShapeHasher.Instance);

    private Shape _solids = new(); //!< Solids built from the splits of faces

    // Auxiliary tools
    private readonly IntersectionContext _context;

    // Output
    private readonly ShapeImage _image; //!< History of modifications

    //! Constructor, taking the history tool to be filled
    public BuildOffsetFaces(ShapeImage image)
    {
        _image = image;
        _context = new IntersectionContext();
    }

    //! Sets faces to build splits
    public void SetFaces(NCollection_List<Shape> faces) => _faces = faces;

    //! Sets ascendants/descendants information
    public void SetAsDesInfo(AsDes asDes) => _asDes = asDes;

    //! Sets the analysis info of the input shape
    public void SetAnalysis(Analyse analyse) => _analyzer = analyse;

    //! Sets origins of the offset edges (from original shape)
    public void SetEdgesOrigins(Dictionary<Shape, NCollection_List<Shape>> edgesOrigins) =>
        _edgesOrigins = edgesOrigins;

    //! Sets origins of the offset faces (from original shape)
    public void SetFacesOrigins(Dictionary<Shape, Shape> facesOrigins) =>
        _facesOrigins = facesOrigins;

    //! Sets infinite (extended) edges for the trimmed ones
    public void SetInfEdges(Dictionary<Shape, Shape> eTrimEInf) => _eTrimEInf = eTrimEInf;

    //=================================================================================================

    //! Build splits of already trimmed faces
    public void BuildSplitsOfTrimmedFaces(ProgressRange range)
    {
        if (!HasData(_faces))
        {
            return;
        }

        Dictionary<Shape, NCollection_List<Shape>> edgesOrigins = new(ShapeHasher.Instance);
        if (_edgesOrigins is null)
        {
            _edgesOrigins = edgesOrigins;
        }

        using ProgressScope ps = new(range, "Building splits of trimmed faces", 5);

        // Fuse all edges
        IntersectTrimmedEdges(ps.Next(1));

        using ProgressScope psLoop = new(ps.Next(4), null, _faces!.Extent());
        for (NCollection_List<Shape>.Iterator itLF = new(_faces); itLF.More(); itLF.Next())
        {
            if (!psLoop.More())
            {
                return;
            }
            Face f = ShapeCasts.Face(itLF.Value());

            Shape ce = new();
            bool bFound = GetEdges(f, ref ce);

            // split the face by the edges
            if (!bFound)
            {
                if (!_image.HasImage(f))
                {
                    _ofImages.ValueAt(_ofImages.Add(f, new NCollection_List<Shape>())).Append(f);
                }
                continue;
            }

            NCollection_List<Shape> lfImages = new();
            BuildSplitsOfTrimmedFace(f, ce, lfImages, psLoop.Next());

            _ofImages.Add(f, lfImages);
        }
        // Fill history for faces and edges
        FillHistory();
    }

    //=================================================================================================

    //! Building splits of not-trimmed offset faces.
    //! For the cases in which invalidities will be found, these invalidities will be rebuilt.
    public void BuildSplitsOfExtendedFaces(ProgressRange range)
    {
        // Check input data
        if (!HasData(_faces) || !HasData(_edgesOrigins) || !HasData(_facesOrigins) || !HasData(_eTrimEInf))
        {
            return;
        }

        using ProgressScope ps = new(range, "Building splits of extended faces", 100.0);
        // Scope has to be added into a loop of undefined size.
        // In general there are about 2 to 5 loops performed, each time
        // decreasing complexity. So reserve for each next loop smaller time.
        // Reserve also 4% on filling gaps after the faces are built.
        double whole = 100.0 - 4.0;

        // Fusing all trimmed offset edges to avoid self-intersections in the splits
        IntersectTrimmedEdges(ps.Next());
        if (!ps.More())
        {
            return;
        }
        // vertices to avoid
        HashSet<Shape> vertsToAvoid = new(ShapeHasher.Instance);

        // Limit total number of attempts by 10. This should be extra, as each invalid face can be
        // rebuilt only three times. So, in general, there are about 2-5 loops done.
        const int nbMaxAttempts = 10;
        // First progress portion is the half of the whole. Each next step is half of the previous:
        // 48%, 24%, 12%, 6% and so on. This way in three loops it will already be 84%,
        // and in four - 90%. So even if the loop will stop earlier, the not advanced scope
        // will be acceptable.
        double part = whole / 2.0;
        for (int iCount = 1; iCount <= nbMaxAttempts; ++iCount, part /= 2.0)
        {
            if (!ps.More())
            {
                return;
            }
            // Clear the data before further faces construction
            _invalidFaces.Clear();
            _artInvalidFaces.Clear();
            _invalidEdges.Clear();
            _invertedEdges.Clear();
            _ssInterfs.Clear();
            _ssInterfsArt.Clear();
            _intersectionPairs.Clear();
            _solids.Nullify();
            _facesToRebuild.Clear();
            _fSelfRebAvoid.Clear();

            // Split progress range on
            // * building faces basing on currently available edges and
            // * rebuilding faces basing on edges classification
            using ProgressScope psLoop = new(ps.Next(part), null, 10.0);

            // Build splits of the faces having new intersection edges
            BuildSplitsOfFaces(psLoop.Next(7.0));
            if (_invalidFaces.Count == 0)
            {
                break;
            }

            // Find faces to rebuild
            FindFacesToRebuild();
            if (_facesToRebuild.Count == 0)
            {
                break;
            }

            // Perform new intersections
            _modifiedEdges.Clear();
            IntersectFaces(vertsToAvoid, psLoop.Next(3.0));
        }

        // Fill possible gaps in the splits of offset faces to increase possibility of
        // creating closed volume from these splits
        FillGaps(ps.Next(4.0));

        // Fill history for faces and edges
        FillHistory();
    }

    //=======================================================================
    // function : UpdateIntersectedEdges
    // purpose  : Saving connection from trimmed edges to not trimmed ones
    //=======================================================================
    private void UpdateIntersectedEdges(NCollection_List<Shape> la, BooleanBuilder gf)
    {
        for (NCollection_List<Shape>.Iterator itA = new(la); itA.More(); itA.Next())
        {
            Shape s = itA.Value();
            if (!_eTrimEInf!.TryGetValue(s, out Shape pEInf))
            {
                continue;
            }

            NCollection_List<Shape> lsIm = ModifiedOf(gf, s);
            if (lsIm.IsEmpty())
            {
                continue;
            }

            for (NCollection_List<Shape>.Iterator it = new(lsIm); it.More(); it.Next())
            {
                Shape eIm = it.Value();
                if (!_eTrimEInf.ContainsKey(eIm))
                {
                    _eTrimEInf[eIm] = pEInf;
                }
            }
        }
    }

    //=================================================================================================

    //! Intersection of the trimmed edges among themselves
    private void IntersectTrimmedEdges(ProgressRange range)
    {
        // get edges to intersect from descendants of the offset faces
        NCollection_List<Shape> ls = new();
        //
        using ProgressScope ps = new(range, null, 2);
        NCollection_List<Shape>.Iterator itLF = new(_faces!);
        for (; itLF.More(); itLF.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Face f = ShapeCasts.Face(itLF.Value());
            //
            NCollection_List<Shape> le = _asDes!.Descendant(f);
            NCollection_List<Shape>.Iterator itLE = new(le);
            for (; itLE.More(); itLE.Next())
            {
                Edge e = ShapeCasts.Edge(itLE.Value());
                //
                if (ProcessMicroEdge(e, _context))
                {
                    continue;
                }
                //
                if (_modifiedEdges.Add(e))
                {
                    ls.Append(e);
                }
            }
        }
        //
        if (ls.Extent() < 2)
        {
            // nothing to intersect
            return;
        }
        //
        // perform intersection of the edges
        BooleanBuilder gfe = new();
        gfe.SetArguments(ls);
        gfe.Perform(ps.Next());
        if (gfe.HasErrors)
        {
            return;
        }
        //
        NCollection_List<Shape> la = new();
        // fill map with edges images
        using ProgressScope psLoop = new(ps.Next(), null, ls.Extent());
        for (NCollection_List<Shape>.Iterator it = new(ls); it.More(); it.Next(), psLoop.Next())
        {
            if (!psLoop.More())
            {
                return;
            }
            Shape e = it.Value();
            NCollection_List<Shape> leIm = ModifiedOf(gfe, e);
            if (leIm.IsEmpty())
            {
                continue;
            }
            //
            la.Append(e);
            // save images
            _oeImages[e] = new NCollection_List<Shape>(leIm);
            // save origins
            NCollection_List<Shape>.Iterator itLE = new(leIm);
            for (; itLE.More(); itLE.Next())
            {
                Shape eIm = itLE.Value();
                if (_oeOrigins.TryGetValue(eIm, out NCollection_List<Shape>? pLEOr))
                {
                    AppendToList(pLEOr, e);
                }
                else
                {
                    NCollection_List<Shape> l = new();
                    l.Append(e);
                    _oeOrigins[eIm] = l;
                }
            }
        }
        //
        UpdateOrigins(la, _edgesOrigins!, gfe);
        UpdateIntersectedEdges(la, gfe);
    }

    //=======================================================================
    // function : CheckConnectionsOfFace
    // purpose  : Checks number of connections for theFace with theLF
    //           Returns true if number of connections more than 1
    //=======================================================================
    private static bool CheckConnectionsOfFace(in Shape face, NCollection_List<Shape> lf)
    {
        IndexedSet<Shape> shapeVert = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator fImIterator = new(lf); fImIterator.More(); fImIterator.Next())
        {
            Shape shape = fImIterator.Value();
            if (shape.IsSame(face))
            {
                continue;
            }
            Exploration.MapShapes(shape, ShapeType.Vertex, shapeVert);
        }
        int nbConnections = 0;
        IndexedSet<Shape> faceVertices = new(ShapeHasher.Instance);
        Exploration.MapShapes(face, ShapeType.Vertex, faceVertices);
        for (int iV = 0; iV < faceVertices.Count; ++iV)
        {
            Shape vert = faceVertices[iV];
            if (shapeVert.Contains(vert))
            {
                ++nbConnections;
            }
            if (nbConnections > 1)
            {
                return true;
            }
        }
        return false;
    }

    //=======================================================================
    // function : BuildSplitsOfFaces
    // purpose  : Building the splits of offset faces and
    //           looking for the invalid splits
    //=======================================================================
    private void BuildSplitsOfFaces(ProgressRange range)
    {
        BrepBuilder bb = new();
        int i, nb;
        //
        // processed faces
        NCollection_List<Shape> lfDone = new();
        // extended face - map of neutral edges, i.e. in one split - valid and in other - invalid
        Dictionary<Shape, HashSet<Shape>> dmfmne = new(ShapeHasher.Instance);
        // map of valid edges for each face
        Dictionary<Shape, HashSet<Shape>> dmfmve = new(ShapeHasher.Instance);
        // map of invalid edges for each face
        Dictionary<Shape, IndexedSet<Shape>> dmfmie = new(ShapeHasher.Instance);
        // map of valid inverted edges for the face
        Dictionary<Shape, HashSet<Shape>> dmfmvie = new(ShapeHasher.Instance);
        // map of splits to check for internals
        IndexedSet<Shape> mfToCheckInt = new(ShapeHasher.Instance);
        // map of edges created from vertex and marked as invalid
        HashSet<Shape> mEdgeInvalidByVertex = new(ShapeHasher.Instance);
        // map of edges created from vertex and marked as valid
        HashSet<Shape> mEdgeValidByVertex = new(ShapeHasher.Instance);
        // connection map from old edges to new ones
        Dictionary<Shape, NCollection_List<Shape>> dmeOrLEIm = new(ShapeHasher.Instance);
        //
        // Outer range
        using ProgressScope psOuter = new(range, null, 10.0);
        // build splits of faces
        using ProgressScope psbf = new(psOuter.Next(3.0), "Building faces", 2 * _faces!.Extent());
        NCollection_List<Shape>.Iterator itLF = new(_faces);
        for (; itLF.More(); itLF.Next(), psbf.Next())
        {
            if (!psbf.More())
            {
                return;
            }
            Face f = ShapeCasts.Face(itLF.Value());
            //
            NCollection_List<Shape>? pLFIm = _ofImages.TryGetValue(f, out NCollection_List<Shape> found) ? found : null;
            if (pLFIm is not null && pLFIm.IsEmpty())
            {
                continue;
            }
            // get edges by which the face should be split
            Shape ce = new();
            IndexedSet<Shape> mapEInv = new(ShapeHasher.Instance);
            bool bFound = GetEdges(f, ref ce, mapEInv);
            if (!bFound)
            {
                continue;
            }
            //
            // OFFSET_DEBUG: the self-intersection check of the edges is left out.
            // build splits
            NCollection_List<Shape> lfImages = new();
            BuildSplitsOfFace(f, ce, _facesOrigins!, lfImages);
            //
            if (mapEInv.Count != 0)
            {
                // check if all possible faces are built
                HashSet<Shape> menInv = new(ShapeHasher.Instance);
                bool bArtificialCase = lfImages.IsEmpty() || CheckIfArtificial(f, lfImages, ce, mapEInv, menInv);
                //
                // try to build splits using invalid edges
                Compound ce1 = new();
                bb.MakeCompound(ref ce1);
                bb.Add(ce1, ce);
                for (i = 1; i <= mapEInv.Count; ++i)
                {
                    bb.Add(ce1, mapEInv[i - 1]);
                }
                //
                NCollection_List<Shape> lfImages1 = new();
                BuildSplitsOfFace(f, ce1, _facesOrigins!, lfImages1);
                //
                // check if the rebuilding has added some new faces to the splits
                for (NCollection_List<Shape>.Iterator aItLFIm = new(lfImages1); aItLFIm.More();)
                {
                    bool bAllInv = true;
                    // Additional check for artificial case
                    // if current image face consist only of edges from aMapEInv and aMENInv
                    // then recheck current face for the further processing
                    bool toReCheckFace = bArtificialCase;
                    Shape fIm = aItLFIm.Value();
                    Explorer expE = new(fIm, ShapeType.Edge);
                    for (; expE.More(); expE.Next())
                    {
                        Shape e = expE.Current();
                        if (!mapEInv.Contains(e))
                        {
                            bAllInv = false;
                            if (!menInv.Contains(e))
                            {
                                toReCheckFace = false;
                                break;
                            }
                        }
                    }
                    // if current image face is to recheck then check number of connections for this face
                    // with other image faces for current face
                    if (!expE.More() && toReCheckFace)
                    {
                        toReCheckFace = CheckConnectionsOfFace(fIm, lfImages1);
                    }
                    // do not delete image face from further processing if aToReCheckFace is true
                    if (!expE.More() && !toReCheckFace)
                    {
                        if (bAllInv)
                        {
                            mfToCheckInt.Add(fIm);
                        }
                        lfImages1.Remove(aItLFIm);
                    }
                    else
                    {
                        aItLFIm.Next();
                    }
                }
                //
                if (bArtificialCase)
                {
                    if (lfImages.Extent() == lfImages1.Extent())
                    {
                        bArtificialCase = false;
                    }
                    else
                    {
                        lfImages.Assign(lfImages1);
                    }
                }
                //
                if (bArtificialCase)
                {
                    // make the face invalid
                    IndexedSet<Shape> meInv = new(ShapeHasher.Instance);
                    //
                    // OCCT: "*pLFIm = aLFImages" dereferences a null pointer when the face has no splits yet
                    // (an access violation natively, a NullReferenceException here; occt-issues.md).
                    pLFIm!.Assign(lfImages);
                    NCollection_List<Shape>.Iterator aItLFIm = new(lfImages);
                    for (; aItLFIm.More(); aItLFIm.Next())
                    {
                        Shape fIm = aItLFIm.Value();
                        Explorer expE = new(fIm, ShapeType.Edge);
                        for (; expE.More(); expE.Next())
                        {
                            Shape e = expE.Current();
                            if (mapEInv.Contains(e))
                            {
                                _invalidEdges.Add(e);
                                meInv.Add(e);
                            }
                            else
                            {
                                _validEdges.Add(e);
                            }
                        }
                    }
                    //
                    _artInvalidFaces[f] = new IndexedSet<Shape>(meInv);
                    dmfmie[f] = new IndexedSet<Shape>(meInv);
                    lfDone.Append(f);
                    //
                    continue;
                }
            }

            // find invalid edges
            FindInvalidEdges(f, lfImages, dmfmve, dmfmne, dmfmie, dmfmvie, dmeOrLEIm, mEdgeInvalidByVertex,
                             mEdgeValidByVertex, psbf.Next());

            // save the new splits
            if (pLFIm is null)
            {
                pLFIm = _ofImages.ValueAt(_ofImages.Add(f, new NCollection_List<Shape>()));
            }
            else
            {
                pLFIm.Clear();
            }
            pLFIm.Append(lfImages);
            //
            lfDone.Append(f);
        }
        //
        if (_invalidEdges.Count == 0 && _artInvalidFaces.Count == 0 && dmfmie.Count == 0)
        {
            return;
        }

        // Additional step to find invalid edges by checking unclassified edges
        // in the splits of SD faces
        FindInvalidEdges(lfDone, dmfmie, dmfmve, dmfmne);

        // Additional step to mark inverted edges located inside loops
        // of invalid edges as invalid as well
        MakeInvertedEdgesInvalid(lfDone);

        // OFFSET_DEBUG: the compounds of invalid, valid and inverted edges, and of all splits, are left out.

        // Build Edge-Face connectivity map to find faces which removal
        // may potentially lead to creation of the holes in the faces
        // preventing from obtaining closed volume in the result
        IndexedMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance);
        int nbF = _ofImages.Count;
        for (i = 1; i <= nbF; ++i)
        {
            NCollection_List<Shape>.Iterator itLFIm = new(_ofImages.ValueAt(i - 1));
            for (; itLFIm.More(); itLFIm.Next())
            {
                Exploration.MapShapesAndAncestors(itLFIm.Value(), ShapeType.Edge, ShapeType.Face, efMap);
            }
        }

        HashSet<Shape> emptyMap = new(ShapeHasher.Instance);
        // invalid faces inside the holes
        IndexedSet<Shape> mfInvInHole = new(ShapeHasher.Instance);
        // all hole faces
        Compound fHoles = new();
        bb.MakeCompound(ref fHoles);
        // Find the faces containing only the inverted edges and the invalid ones
        NCollection_List<Shape> invertedFaces = new();

        using ProgressScope psif = new(psOuter.Next(2.0), "Checking validity of faces", lfDone.Extent());
        // find invalid faces
        // considering faces containing only invalid edges as invalid
        itLF.Initialize(lfDone);
        for (; itLF.More(); itLF.Next(), psif.Next())
        {
            if (!psif.More())
            {
                return;
            }
            Face f = ShapeCasts.Face(itLF.Value());
            NCollection_List<Shape> lfImages = _ofImages.GetValue(f);
            //
            NCollection_List<Shape> lfInv = new();
            bool bArtificialCase = _artInvalidFaces.ContainsKey(f);
            if (bArtificialCase)
            {
                lfInv.Assign(lfImages);
            }
            else
            {
                // neutral edges
                if (!dmfmne.TryGetValue(f, out HashSet<Shape>? pMNE))
                {
                    pMNE = emptyMap;
                }
                // find faces inside holes wires
                HashSet<Shape> mfHoles = new(ShapeHasher.Instance);
                Face fOr = ShapeCasts.Face(_facesOrigins!.GetValue(f));
                FindFacesInsideHoleWires(fOr, f, lfImages, dmeOrLEIm, efMap, mfHoles);
                //
                foreach (Shape itMH in mfHoles)
                {
                    bb.Add(fHoles, itMH);
                }
                //
                // find invalid faces
                FindInvalidFaces(lfImages, dmfmve, dmfmie, pMNE, mEdgeInvalidByVertex, mEdgeValidByVertex, mfHoles,
                                 mfInvInHole, lfInv, invertedFaces);
            }
            //
            if (lfInv.Extent() != 0)
            {
                if (_alreadyInvFaces.ContainsKey(f))
                {
                    if (_alreadyInvFaces.GetValue(f) > 2)
                    {
                        if (lfInv.Extent() == lfImages.Extent() && !bArtificialCase)
                        {
                            lfImages.Clear();
                        }
                        //
                        lfInv.Clear();
                    }
                }
                _invalidFaces.Add(f, lfInv);
            }
        }
        //
        if (_invalidFaces.Count == 0)
        {
            _invalidEdges.Clear();
            return;
        }
        //
        // OFFSET_DEBUG: the compound of invalid faces is left out.
        //
        IndexedSet<Shape> meRemoved = new(ShapeHasher.Instance);
        // remove invalid splits of faces using inverted edges
        RemoveInvalidSplitsByInvertedEdges(meRemoved);
        if (_invalidFaces.Count == 0)
        {
            _invalidEdges.Clear();
            return;
        }
        //
        // remove invalid splits from valid splits
        RemoveInvalidSplitsFromValid(dmfmvie);
        //
        // remove inside faces
        RemoveInsideFaces(invertedFaces, mfToCheckInt, mfInvInHole, fHoles, meRemoved, _insideEdges,
                          psOuter.Next(5.0));
        //
        // make compound of valid splits
        Compound cfIm = new();
        bb.MakeCompound(ref cfIm);
        //
        nb = _ofImages.Count;
        for (i = 1; i <= nb; ++i)
        {
            NCollection_List<Shape> lfIm = _ofImages.ValueAt(i - 1);
            itLF.Initialize(lfIm);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                bb.Add(cfIm, fIm);
            }
        }
        //
        IndexedMap<Shape, NCollection_List<Shape>> dmef = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(cfIm, ShapeType.Edge, ShapeType.Face, dmef);
        //
        // filter maps of images and origins
        FilterEdgesImages(cfIm);
        //
        // filter invalid faces
        FilterInvalidFaces(dmef, meRemoved.Count != 0 ? _insideEdges : meRemoved);
        nb = _invalidFaces.Count;
        if (nb == 0)
        {
            _invalidEdges.Clear();
            return;
        }
        //
        // OFFSET_DEBUG: the compound of invalid faces is left out.
        //
        // filter invalid edges
        HashSet<Shape> meUseInRebuild = new(ShapeHasher.Instance);
        FilterInvalidEdges(dmfmie, meRemoved, meRemoved.Count != 0 ? _insideEdges : meRemoved, meUseInRebuild);
        //
        // Check additionally validity of edges originated from vertices.
        CheckEdgesCreatedByVertex();

        // OFFSET_DEBUG: the compound of invalid edges is left out.
        //
        _lastInvEdges.Clear();
        nb = _invalidEdges.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape e = _invalidEdges[i - 1];
            _lastInvEdges.Add(e);
            if (!meUseInRebuild.Contains(e))
            {
                _edgesToAvoid.Add(e);
            }
        }
        //
        nb = _invalidFaces.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape f = _invalidFaces.KeyAt(i - 1);
            if (_alreadyInvFaces.ContainsKey(f))
            {
                _alreadyInvFaces.GetValueRef(f)++;
            }
            else
            {
                _alreadyInvFaces[f] = 1;
            }
        }
    }

    //=======================================================================
    // function : GetEdges
    // purpose  : Getting edges from AsDes map to build the splits of faces
    //=======================================================================
    private bool GetEdges(in Face face, ref Shape edges,
                          IndexedSet<Shape>? invMap = null)
    {
        // get boundary edges
        HashSet<Shape> mfBounds = new(ShapeHasher.Instance);
        Explorer exp = new(face, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Shape e = exp.Current();
            if (_oeImages.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
            {
                for (NCollection_List<Shape>.Iterator itLE0 = new(pLEIm); itLE0.More(); itLE0.Next())
                {
                    mfBounds.Add(itLE0.Value());
                }
            }
            else
            {
                mfBounds.Add(e);
            }
        }

        BrepBuilder bb = new();
        bool bFound = false, bUpdate = false;
        // the resulting edges
        Compound anEdges = new();
        bb.MakeCompound(ref anEdges);
        // Fence map
        HashSet<Shape> meFence = new(ShapeHasher.Instance);
        // the edges by which the offset face should be split
        NCollection_List<Shape> le = _asDes!.Descendant(face);
        NCollection_List<Shape>.Iterator itLE = new(le);
        for (; itLE.More(); itLE.Next())
        {
            Edge e = ShapeCasts.Edge(itLE.Value());
            //
            if (!bUpdate)
            {
                bUpdate = _modifiedEdges.Contains(e);
            }
            //
            if (_oeImages.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
            {
                NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    Edge eIm = ShapeCasts.Edge(itLEIm.Value());
                    //
                    if (!meFence.Add(eIm))
                    {
                        continue;
                    }

                    if (_edgesToAvoid.Contains(eIm))
                    {
                        invMap?.Add(eIm);
                        if (!bUpdate)
                        {
                            bUpdate = _lastInvEdges.Contains(eIm);
                        }
                        continue;
                    }
                    // check for micro edge
                    if (ProcessMicroEdge(eIm, _context))
                    {
                        continue;
                    }
                    //
                    bb.Add(anEdges, eIm);
                    if (!bFound)
                    {
                        bFound = !mfBounds.Contains(eIm);
                    }
                    //
                    if (!bUpdate)
                    {
                        bUpdate = _modifiedEdges.Contains(eIm);
                    }
                }
            }
            else
            {
                if (_edgesToAvoid.Contains(e))
                {
                    invMap?.Add(e);
                    if (!bUpdate)
                    {
                        bUpdate = _lastInvEdges.Contains(e);
                    }
                    continue;
                }
                //
                if (ProcessMicroEdge(e, _context))
                {
                    continue;
                }
                bb.Add(anEdges, e);
                if (!bFound)
                {
                    bFound = !mfBounds.Contains(e);
                }
            }
        }
        //
        edges = anEdges;
        return bFound && bUpdate;
    }

    //=======================================================================
    // function : CheckIfArtificial
    // purpose  : Checks if the face is artificially invalid
    //=======================================================================
    private bool CheckIfArtificial(in Shape f, NCollection_List<Shape> lfImages, in Shape ce,
                                   IndexedSet<Shape> mapEInv, HashSet<Shape> menInv)
    {
        // all boundary edges should be used
        IndexedSet<Shape> meUsed = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLFIm = new(lfImages);
        for (; itLFIm.More(); itLFIm.Next())
        {
            Shape fIm = itLFIm.Value();
            Exploration.MapShapes(fIm, ShapeType.Edge, meUsed);
            Exploration.MapShapes(fIm, ShapeType.Vertex, meUsed);
        }
        //
        IndexedMap<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(ce, ShapeType.Vertex, ShapeType.Edge, mve);
        //
        int i, nb = mapEInv.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape eInv = mapEInv[i - 1];
            Explorer expV = new(eInv, ShapeType.Vertex);
            for (; expV.More(); expV.Next())
            {
                Shape veInv = expV.Current();
                if (mve.TryGetValue(veInv, out NCollection_List<Shape> pLENInv))
                {
                    NCollection_List<Shape>.Iterator itLEInv = new(pLENInv);
                    for (; itLEInv.More(); itLEInv.Next())
                    {
                        Shape enInv = itLEInv.Value();
                        if (!meUsed.Contains(enInv))
                        {
                            menInv.Add(enInv);
                        }
                    }
                }
            }
        }
        //
        if (menInv.Count == 0)
        {
            return false;
        }
        //
        IndexedSet<Shape> meFound = new(ShapeHasher.Instance);
        Exploration.MapShapes(ce, ShapeType.Edge, meFound);
        //
        NCollection_List<Shape> le = _asDes!.Descendant(f);
        NCollection_List<Shape>.Iterator itLE = new(le);
        for (; itLE.More(); itLE.Next())
        {
            Edge e = ShapeCasts.Edge(itLE.Value());
            //
            if (_oeImages.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
            {
                bool bChecked = false;
                NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    Edge eIm = ShapeCasts.Edge(itLEIm.Value());
                    if (!meFound.Contains(eIm) || menInv.Contains(eIm))
                    {
                        continue;
                    }
                    //
                    bChecked = true;
                    if (meUsed.Contains(eIm))
                    {
                        break;
                    }
                }
                //
                if (bChecked && !itLEIm.More())
                {
                    break;
                }
            }
            else
            {
                if (meFound.Contains(e) && !menInv.Contains(e) && !meUsed.Contains(e))
                {
                    break;
                }
            }
        }
        //
        return itLE.More();
    }

    //=================================================================================================

    //! Looks for the invalid edges (edges with changed orientation)
    //! in the splits of offset faces
    private void FindInvalidEdges(in Face f, NCollection_List<Shape> lfImages,
                                  Dictionary<Shape, HashSet<Shape>> dmfmve,
                                  Dictionary<Shape, HashSet<Shape>> dmfmne,
                                  Dictionary<Shape, IndexedSet<Shape>> dmfmie,
                                  Dictionary<Shape, HashSet<Shape>> dmfmvie,
                                  Dictionary<Shape, NCollection_List<Shape>> dmeOrLEIm,
                                  HashSet<Shape> edgesInvalidByVertex,
                                  HashSet<Shape> edgesValidByVertex, ProgressRange range)
    {
        // Edge is considered as invalid in the following cases:
        // 1. Its orientation on the face has changed comparing to the originals edge and face;
        // 2. The vertices of the edge have changed places comparing to the originals edge and face.
        //
        // The edges created from vertices, i.e. as intersection between two faces connected only
        // by VERTEX, will also be checked on validity. For these edges the correct orientation will
        // be defined by the edges on the original face adjacent to the connection vertex

        // original face
        Face fOr = ShapeCasts.Face(_facesOrigins!.GetValue(f));
        // invalid edges
        IndexedSet<Shape> meInv = new(ShapeHasher.Instance);
        // valid edges
        HashSet<Shape> meVal = new(ShapeHasher.Instance);
        // internal edges
        HashSet<Shape> meInt = new(ShapeHasher.Instance);
        //
        // maps for checking the inverted edges
        IndexedMap<Shape, NCollection_List<Shape>> dmve = new(ShapeHasher.Instance),
                                                                                 dmef = new(ShapeHasher.Instance);
        IndexedSet<Shape> mEdges = new(ShapeHasher.Instance);
        // back map from the original shapes to their offset images
        Dictionary<Shape, NCollection_List<Shape>> images = new(ShapeHasher.Instance);
        //
        using ProgressScope ps = new(range, "Checking validity of edges", 2 * lfImages.Extent());
        NCollection_List<Shape>.Iterator itLF = new(lfImages);
        for (; itLF.More(); itLF.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Face fIm = ShapeCasts.Face(itLF.Value());
            //
            Explorer exp = new(fIm, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Shape e = exp.Current();
                // keep all edges
                mEdges.Add(e);
                //
                // keep connection from edges to faces
                if (!dmef.TryGetValue(e, out NCollection_List<Shape> pLF))
                {
                    pLF = dmef.ValueAt(dmef.Add(e, new NCollection_List<Shape>()));
                }
                AppendToList(pLF, fIm);
                //
                // keep connection from vertices to edges
                Iterator aItV = new(e);
                for (; aItV.More(); aItV.Next())
                {
                    Shape v = aItV.Value();
                    //
                    if (!dmve.TryGetValue(v, out NCollection_List<Shape> pLE))
                    {
                        pLE = dmve.ValueAt(dmve.Add(v, new NCollection_List<Shape>()));
                    }
                    AppendToList(pLE, e);
                }

                // back map from original edges to their offset images
                if (!_edgesOrigins!.TryGetValue(e, out NCollection_List<Shape>? pLOr))
                {
                    continue;
                }
                for (NCollection_List<Shape>.Iterator itOr = new(pLOr); itOr.More(); itOr.Next())
                {
                    Shape sOr = itOr.Value();
                    Shape sInF = new();
                    if (!FindShape(sOr, fOr, _analyzer, ref sInF))
                    {
                        continue;
                    }
                    if (!images.TryGetValue(sInF, out NCollection_List<Shape>? pImages))
                    {
                        pImages = new NCollection_List<Shape>();
                        images[sInF] = pImages;
                    }
                    AppendToList(pImages, e);
                }
            }
        }
        //
        // the map will be used to find the edges on the original face
        // adjacent to the same vertex. It will be filled at first necessity;
        IndexedMap<Shape, NCollection_List<Shape>> dmvefOr = new(ShapeHasher.Instance);
        //
        itLF.Initialize(lfImages);
        for (; itLF.More(); itLF.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Face fIm = ShapeCasts.Face(itLF.Value());
            //
            // valid edges for this split
            HashSet<Shape> mve = new(ShapeHasher.Instance);
            // invalid edges for this split
            IndexedSet<Shape> mie = new(ShapeHasher.Instance);
            //
            Explorer exp = new(fIm, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge eIm = ShapeCasts.Edge(exp.Current());
                //
                if (eIm.Orientation == Internal)
                {
                    meInt.Add(eIm);
                    continue;
                }
                //
                if (!_edgesOrigins!.TryGetValue(eIm, out NCollection_List<Shape>? pLEOr) || pLEOr.IsEmpty())
                {
                    continue;
                }
                //
                int nbVOr = 0;
                NCollection_List<Shape>.Iterator itLEO = new(pLEOr);
                for (; itLEO.More(); itLEO.Next())
                {
                    if (itLEO.Value().Type == ShapeType.Vertex)
                    {
                        ++nbVOr;
                    }
                }
                if (nbVOr > 1 && (pLEOr.Extent() - nbVOr) > 1)
                {
                    continue;
                }
                //
                HashSet<Shape> me = new(ShapeHasher.Instance), mv = new(ShapeHasher.Instance),
                                              mf = new(ShapeHasher.Instance);
                bool bInvalid = false, bChecked = false;
                int nbP = NbPoints(eIm), nbInv = 0;
                bool bUseVertex = nbVOr == 0 ? false
                                              : (nbVOr == 1 && dmef.GetValue(eIm).Extent() == 1
                                                 && !_oeOrigins.ContainsKey(eIm));
                //
                itLEO.Initialize(pLEOr);
                for (; itLEO.More(); itLEO.Next())
                {
                    Shape sOr = itLEO.Value();
                    bool bVertex = (sOr.Type == ShapeType.Vertex);
                    //
                    Shape eOrF = new();
                    if (bVertex)
                    {
                        // for some cases it is impossible to check the validity of the edge
                        if (!bUseVertex)
                        {
                            continue;
                        }
                        // find edges on the original face adjacent to this vertex
                        if (dmvefOr.Count == 0)
                        {
                            // fill the map
                            Exploration.MapShapesAndAncestors(fOr, ShapeType.Vertex, ShapeType.Edge, dmvefOr);
                        }
                        //
                        if (dmvefOr.TryGetValue(sOr, out NCollection_List<Shape> pLEFOr))
                        {
                            Compound ceOr = new();
                            new BrepBuilder().MakeCompound(ref ceOr);
                            // Avoid classification of edges originated from vertices
                            // located between tangent edges
                            bool bAllTgt = true;
                            NCollection_List<Shape>.Iterator itLEFOr = new(pLEFOr);
                            Vector3D vRef = GetAverageTangent(itLEFOr.Value(), nbP);
                            for (; itLEFOr.More(); itLEFOr.Next())
                            {
                                Shape eOr = itLEFOr.Value();
                                new BrepBuilder().Add(ceOr, eOr);

                                Vector3D vCur = GetAverageTangent(eOr, nbP);
                                if (!vRef.IsParallel(vCur, global::Kerf.Engine.Foundation.Precision.Angular()))
                                {
                                    bAllTgt = false;
                                }
                            }
                            if (!bAllTgt)
                            {
                                eOrF = ceOr;
                            }
                        }
                    }
                    else
                    {
                        FindShape(sOr, fOr, _analyzer, ref eOrF);
                        //
                        if (!dmeOrLEIm.TryGetValue(sOr, out NCollection_List<Shape>? pLEIm))
                        {
                            pLEIm = new NCollection_List<Shape>();
                            dmeOrLEIm[sOr] = pLEIm;
                        }
                        AppendToList(pLEIm, eIm);
                    }
                    //
                    if (eOrF.IsNull)
                    {
                        // the edge has not been found
                        continue;
                    }

                    if (bVertex)
                    {
                        HashSet<Shape> mvTotal = new(ShapeHasher.Instance);
                        int nbChecked = 0;
                        // Just check if the original edges sharing the vertex do not share it any more.
                        for (Iterator it = new(eOrF); it.More(); it.Next())
                        {
                            Shape eOr = it.Value();
                            if (!images.TryGetValue(eOr, out NCollection_List<Shape>? lIm))
                            {
                                continue;
                            }
                            ++nbChecked;
                            IndexedMap<Shape, NCollection_List<Shape>> mvLoc = new(ShapeHasher.Instance);
                            for (NCollection_List<Shape>.Iterator itLIM = new(lIm); itLIM.More(); itLIM.Next())
                            {
                                Exploration.MapShapesAndAncestors(itLIM.Value(), ShapeType.Vertex, ShapeType.Edge, mvLoc);
                            }
                            for (int i0 = 0; i0 < mvLoc.Count; ++i0)
                            {
                                if (mvLoc.ValueAt(i0).Extent() > 1 && !mvTotal.Add(mvLoc.KeyAt(i0)))
                                {
                                    bInvalid = true;
                                    edgesInvalidByVertex.Add(eIm);
                                    break;
                                }
                            }
                            if (bInvalid)
                            {
                                break;
                            }
                        }
                        if (!bInvalid && nbChecked < 2)
                        {
                            continue;
                        }
                        else
                        {
                            edgesValidByVertex.Add(eIm);
                        }
                    }
                    else
                    {
                        //
                        // Check orientations of the image edge and original edge.
                        // In case the 3d curves are having the same direction the orientations
                        // must be the same. Otherwise the orientations should also be different.
                        //
                        // get average tangent vector for each curve taking into account
                        // the orientations of the edges, i.e. the edge is reversed
                        // the vector is reversed as well
                        Vector3D vSum1 = GetAverageTangent(eIm, nbP);
                        Vector3D vSum2 = GetAverageTangent(eOrF, nbP);
                        //
                        vSum1 = vSum1.Normalized();
                        vSum2 = vSum2.Normalized();
                        //
                        double cos = vSum1.Dot(vSum2);
                        if (Math.Abs(cos) < 0.9999)
                        {
                            continue;
                        }
                        //
                        me.Add(eOrF);
                        Explorer expE = new(eOrF, ShapeType.Vertex);
                        for (; expE.More(); expE.Next())
                        {
                            Shape v = expE.Current();
                            mv.Add(v);
                        }
                        if (_analyzer is not null)
                        {
                            for (NCollection_List<Shape>.Iterator itFA = new(_analyzer.Ancestors(eOrF)); itFA.More(); itFA.Next())
                            {
                                mf.Add(itFA.Value());
                            }
                        }
                        //
                        if (cos < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            bInvalid = true;
                            nbInv++;
                        }
                    }
                    bChecked = true;
                }
                //
                if (!bChecked)
                {
                    continue;
                }
                //
                bool bLocalOnly = (nbVOr > 1 && (pLEOr.Extent() - nbVOr) > 1);
                int nbE = me.Count, nbV = mv.Count;
                if (nbE > 1 && nbV == 2 * nbE)
                {
                    bool bSkip = true;

                    // It seems the edge originated from not connected edges and cannot be
                    // considered as correctly classified as it may fill some undesired parts.
                    // Still, allow the edge to be accounted for local analysis if it is:
                    // * originated from more than two faces
                    // * unanimously considered valid or invalid
                    // * not a boundary edge in the splits
                    if (mf.Count > 2 && (nbInv == 0 || nbInv == nbE))
                    {
                        if (lfImages.Extent() > 2)
                        {
                            Iterator itV = new(eIm);
                            for (; itV.More(); itV.Next())
                            {
                                NCollection_List<Shape>.Iterator itE = new(dmve.GetValue(itV.Value()));
                                for (; itE.More(); itE.Next())
                                {
                                    if (dmef.GetValue(itE.Value()).Extent() < 2)
                                    {
                                        break;
                                    }
                                }
                                if (itE.More())
                                {
                                    break;
                                }
                            }
                            bSkip = itV.More();
                        }
                    }
                    if (bSkip)
                    {
                        continue;
                    }
                    else
                    {
                        bLocalOnly = true;
                    }
                }
                //
                if (bInvalid)
                {
                    if (!bLocalOnly)
                    {
                        _invalidEdges.Add(eIm);
                    }
                    mie.Add(eIm);
                    meInv.Add(eIm);
                    continue;
                }
                //
                // check if the edge has been inverted
                bool bInverted = nbE == 0 || bLocalOnly ? false : CheckInverted(eIm, fOr, dmve, mEdges);
                //
                if (!bInverted || nbVOr == 0)
                {
                    if (!bLocalOnly)
                    {
                        _validEdges.Add(eIm);
                    }
                    mve.Add(eIm);
                    meVal.Add(eIm);
                }
            }
            //
            // valid edges
            if (mve.Count != 0)
            {
                dmfmve[fIm] = mve;
            }
            //
            // invalid edges
            if (mie.Count != 0)
            {
                dmfmie[fIm] = mie;
            }
        }
        //
        // process invalid edges:
        // check for the inverted edges
        HashSet<Shape> mvie = new(ShapeHasher.Instance);
        // fill neutral edges
        HashSet<Shape> mne = new(ShapeHasher.Instance);
        //
        int i, nbEInv = meInv.Count;
        for (i = 1; i <= nbEInv; ++i)
        {
            Shape eIm = meInv[i - 1];
            //
            // neutral edges - on the splits of the same offset face
            // it is valid for one split and invalid for other
            if (meVal.Contains(eIm))
            {
                mne.Add(eIm);
                continue;
            }
            //
            // the inverted images of the origins of invalid edges should also be invalid
            if (!_invertedEdges.Contains(eIm))
            {
                continue;
            }
            //
            if (!_oeOrigins.TryGetValue(eIm, out NCollection_List<Shape>? pLOEOr))
            {
                continue;
            }
            //
            NCollection_List<Shape>.Iterator itLOEOr = new(pLOEOr);
            for (; itLOEOr.More(); itLOEOr.Next())
            {
                Shape oeOr = itLOEOr.Value();
                NCollection_List<Shape> leIm1 = _oeImages.GetValue(oeOr);
                //
                NCollection_List<Shape>.Iterator itLEIm1 = new(leIm1);
                for (; itLEIm1.More(); itLEIm1.Next())
                {
                    Shape eIm1 = itLEIm1.Value();
                    if (mEdges.Contains(eIm1) && !meInv.Contains(eIm1) && !meInt.Contains(eIm1)
                        && _invertedEdges.Contains(eIm1))
                    {
                        _invalidEdges.Add(eIm1);
                        mvie.Add(eIm1);
                    }
                }
            }
        }
        //
        if (mne.Count != 0)
        {
            dmfmne[f] = mne;
        }
        //
        if (mvie.Count != 0)
        {
            dmfmvie[f] = mvie;
        }
    }

    //=======================================================================
    // function : addAsNeutral
    // purpose  : Adds as the edge into corresponding maps making it neutral
    //=======================================================================
    private static void AddAsNeutral(in Shape e, in Shape fInv, in Shape fVal,
                                     Dictionary<Shape, IndexedSet<Shape>> locInvEdges,
                                     Dictionary<Shape, HashSet<Shape>> locValidEdges)
    {
        if (!locInvEdges.TryGetValue(fInv, out IndexedSet<Shape>? pMEInv))
        {
            pMEInv = new IndexedSet<Shape>(ShapeHasher.Instance);
            locInvEdges[fInv] = pMEInv;
        }
        pMEInv.Add(e);

        if (!locValidEdges.TryGetValue(fVal, out HashSet<Shape>? pMEVal))
        {
            pMEVal = new HashSet<Shape>(ShapeHasher.Instance);
            locValidEdges[fVal] = pMEVal;
        }
        pMEVal.Add(e);
    }

    //=======================================================================
    // function : FindInvalidEdges
    // purpose  : Additional method to look for invalid edges
    //=======================================================================
    private void FindInvalidEdges(NCollection_List<Shape> lfOffset,
                                  Dictionary<Shape, IndexedSet<Shape>> locInvEdges,
                                  Dictionary<Shape, HashSet<Shape>> locValidEdges,
                                  Dictionary<Shape, HashSet<Shape>> neutralEdges)
    {
        // 1. Find edges unclassified in faces
        // 2. Find SD faces in which the same edge is classified
        // 3. Check if the edge is neutral in face in which it wasn't classified

        IndexedMap<Shape, HashSet<Shape>> meUnclassified = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> fSplitFOffset = new(ShapeHasher.Instance);

        // Avoid artificial faces
        HashSet<Shape> newFaces = new(ShapeHasher.Instance);
        if (_analyzer is not null)
        {
            HashSet<Shape> mapNewTmp = new(ShapeHasher.Instance);
            for (NCollection_List<Shape>.Iterator it = new(_analyzer.NewFaces()); it.More(); it.Next())
            {
                mapNewTmp.Add(it.Value());
            }

            for (NCollection_List<Shape>.Iterator it = new(lfOffset); it.More(); it.Next())
            {
                Shape fOffset = it.Value();
                Shape fOrigin = _facesOrigins!.GetValue(fOffset);
                if (mapNewTmp.Contains(fOrigin))
                {
                    newFaces.Add(fOffset);
                }
            }
        }

        IndexedMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator itLFO = new(lfOffset); itLFO.More(); itLFO.Next())
        {
            Shape f = itLFO.Value();
            if (newFaces.Contains(f))
            {
                continue;
            }

            NCollection_List<Shape> lfImages = _ofImages.GetValue(f);
            for (NCollection_List<Shape>.Iterator itLF = new(lfImages); itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();

                Exploration.MapShapesAndAncestors(fIm, ShapeType.Edge, ShapeType.Face, efMap);

                locInvEdges.TryGetValue(fIm, out IndexedSet<Shape>? pMEInvalid);
                locValidEdges.TryGetValue(fIm, out HashSet<Shape>? pMEValid);

                for (Explorer expE = new(fIm, ShapeType.Edge); expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (_invalidEdges.Contains(e) != _validEdges.Contains(e))
                    {
                        // edge is classified in some face

                        if ((pMEInvalid is null || !pMEInvalid.Contains(e)) && (pMEValid is null || !pMEValid.Contains(e)))
                        {
                            // but not in the current one
                            if (!meUnclassified.TryGetValue(e, out HashSet<Shape> pMap))
                            {
                                pMap = meUnclassified.ValueAt(meUnclassified.Add(e, new HashSet<Shape>(ShapeHasher.Instance)));
                            }
                            pMap.Add(fIm);

                            fSplitFOffset[fIm] = f;
                        }
                    }
                }
            }
        }

        if (meUnclassified.Count == 0)
        {
            return;
        }

        // Analyze unclassified edges
        int nbE = meUnclassified.Count;
        for (int iE = 0; iE < nbE; ++iE)
        {
            Shape e = meUnclassified.KeyAt(iE);
            HashSet<Shape> mfUnclassified = meUnclassified.ValueAt(iE);

            NCollection_List<Shape> lf = efMap.GetValue(e);

            for (NCollection_List<Shape>.Iterator itLF = new(lf); itLF.More(); itLF.Next())
            {
                Shape fClassified = itLF.Value();
                if (mfUnclassified.Contains(fClassified))
                {
                    continue;
                }

                ToolSet edgeSetClass = new();
                edgeSetClass.Add(fClassified, ShapeType.Edge);

                Shape eClassified = new();
                FindShape(e, fClassified, null, ref eClassified);
                Orientation oriClass = eClassified.Orientation;

                AlgoTools3D.GetNormalToFaceOnEdge(ShapeCasts.Edge(eClassified), ShapeCasts.Face(fClassified),
                                                           out Direction3D dnClass);

                locInvEdges.TryGetValue(fClassified, out IndexedSet<Shape>? pMEInvalid);
                bool isInvalid = pMEInvalid is not null && pMEInvalid.Contains(e);

                // Kerf: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
                foreach (Shape fUnclassified in mfUnclassified)
                {
                    ToolSet edgeSetUnclass = new();
                    edgeSetUnclass.Add(fUnclassified, ShapeType.Edge);

                    if (edgeSetClass.IsEqual(edgeSetUnclass))
                    {
                        AlgoTools3D.GetNormalToFaceOnEdge(ShapeCasts.Edge(e), ShapeCasts.Face(fUnclassified),
                                                                   out Direction3D dnUnclass);

                        bool isSameOri = dnClass.IsEqual(dnUnclass, global::Kerf.Engine.Foundation.Precision.Angular());

                        // Among other splits of the same face find those where the edge is contained with
                        // different orientation
                        Shape fOffset = fSplitFOffset.GetValue(fUnclassified);
                        NCollection_List<Shape> lfSplits = _ofImages.GetValue(fOffset);
                        NCollection_List<Shape>.Iterator itLFSp = new(lfSplits);
                        for (; itLFSp.More(); itLFSp.Next())
                        {
                            Shape fSp = itLFSp.Value();

                            if (!fSp.IsSame(fUnclassified) && mfUnclassified.Contains(fSp))
                            {
                                Shape eUnclassified = new();
                                FindShape(e, fSp, null, ref eUnclassified);

                                Orientation oriUnclass = eUnclassified.Orientation;
                                if (!isSameOri)
                                {
                                    oriUnclass = global::Kerf.Engine.Topology.TopAbs.Reverse(oriUnclass);
                                }

                                if (oriClass != oriUnclass)
                                {
                                    // make the edge neutral for the face
                                    if (!neutralEdges.TryGetValue(fOffset, out HashSet<Shape>? pMENeutral))
                                    {
                                        pMENeutral = new HashSet<Shape>(ShapeHasher.Instance);
                                        neutralEdges[fOffset] = pMENeutral;
                                    }
                                    pMENeutral.Add(e);

                                    if (isInvalid && isSameOri)
                                    {
                                        // make edge invalid in aFUnclassified and valid in aFSp
                                        AddAsNeutral(e, fClassified, fSp, locInvEdges, locValidEdges);
                                    }
                                    else
                                    {
                                        // make edge invalid in aFSp and valid in aFUnclassified
                                        AddAsNeutral(e, fSp, fClassified, locInvEdges, locValidEdges);
                                    }
                                }
                            }
                        }
                        if (itLFSp.More())
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : MakeInvertedEdgesInvalid
    // purpose  : Makes inverted edges located inside loop of invalid edges, invalid as well
    //=======================================================================
    private void MakeInvertedEdgesInvalid(NCollection_List<Shape> lfOffset)
    {
        if (_invalidEdges.Count == 0 || _invertedEdges.Count == 0)
        {
            return;
        }

        // Map all invalid edges
        Compound cbeInv = new();
        new BrepBuilder().MakeCompound(ref cbeInv);
        for (int i = 0; i < _invalidEdges.Count; ++i)
        {
            new BrepBuilder().Add(cbeInv, _invalidEdges[i]);
        }

        // Make loops of invalid edges
        NCollection_List<Shape> lcb = new();
        AlgoTools.MakeConnexityBlocks(cbeInv, ShapeType.Vertex, ShapeType.Edge, lcb);

        // Analyze each loop on closeness and use only closed ones
        Dictionary<Shape, Shape> dmvcb = new(ShapeHasher.Instance);

        for (NCollection_List<Shape>.Iterator itLCB = new(lcb); itLCB.More(); itLCB.Next())
        {
            Shape cb = itLCB.Value();

            IndexedMap<Shape, NCollection_List<Shape>> dmve = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(cb, ShapeType.Vertex, ShapeType.Edge, dmve);
            bool isClosed = true;
            for (int iV = 0; iV < dmve.Count; ++iV)
            {
                if (dmve.ValueAt(iV).Extent() != 2)
                {
                    isClosed = false;
                    break;
                }
            }
            if (!isClosed)
            {
                continue;
            }

            // Bind loop to each vertex of the loop
            for (int iV = 0; iV < dmve.Count; ++iV)
            {
                dmvcb[dmve.KeyAt(iV)] = cb;
            }
        }

        // Check if any inverted edges of offset faces are locked inside the loops of invalid edges.
        // Make such edges invalid as well.
        for (NCollection_List<Shape>.Iterator itLF = new(lfOffset); itLF.More(); itLF.Next())
        {
            NCollection_List<Shape> lfIm = _ofImages.GetValue(itLF.Value());
            for (NCollection_List<Shape>.Iterator itLFIm = new(lfIm); itLFIm.More(); itLFIm.Next())
            {
                for (Explorer expE = new(itLFIm.Value(), ShapeType.Edge); expE.More(); expE.Next())
                {
                    Edge e = ShapeCasts.Edge(expE.Current());
                    if (!_invalidEdges.Contains(e) && _invertedEdges.Contains(e))
                    {
                        bool bCB1 = dmvcb.TryGetValue(Exploration.FirstVertex(e), out Shape pCB1);
                        bool bCB2 = dmvcb.TryGetValue(Exploration.LastVertex(e), out Shape pCB2);
                        if (bCB1 && bCB2 && pCB1.IsSame(pCB2))
                        {
                            _invalidEdges.Add(e);
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FindInvalidFaces
    // purpose  : Looking for the invalid faces by analyzing their invalid edges
    //=======================================================================
    private void FindInvalidFaces(NCollection_List<Shape> lfImages,
                                  Dictionary<Shape, HashSet<Shape>> dmfmve,
                                  Dictionary<Shape, IndexedSet<Shape>> dmfmie,
                                  HashSet<Shape> meNeutral,
                                  HashSet<Shape> edgesInvalidByVertex,
                                  HashSet<Shape> edgesValidByVertex,
                                  HashSet<Shape> mfHoles,
                                  IndexedSet<Shape> mfInvInHole,
                                  NCollection_List<Shape> invFaces,
                                  NCollection_List<Shape> invertedFaces)
    {
        // The face should be considered as invalid in the following cases:
        // 1. It has been reverted, i.e. at least two not connected edges
        //    have changed orientation (i.e. invalid). In this case all edges,
        //    should be invalid for that face, because edges have also been reverted;
        // 2. All checked edges of the face are invalid for this face;
        // The face should be removed from the splits in the following cases:
        // 1. All checked edges of the face are invalid for this one, but valid for
        //    some other face in this list of splits.
        // The face will be kept in the following cases:
        // 1. Some of the edges are valid for this face.
        bool bHasValid, bAllValid, bAllInvalid, bHasReallyInvalid, bAllInvNeutral;
        bool bValid, bValidLoc, bInvalid, bInvalidLoc, bNeutral, bInverted;
        bool bIsInvalidByInverted, bHasInverted;
        int nbChecked;
        //
        bool bTreatInvertedAsInvalid = (lfImages.Extent() == 1);
        //
        // neutral edges to remove
        IndexedSet<Shape> menRem = new(ShapeHasher.Instance);
        //
        // faces for post treat
        NCollection_List<Shape> lfpt = new();
        //
        IndexedMap<Shape, NCollection_List<Shape>> dmef = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLF = new(lfImages);
        for (; itLF.More(); itLF.Next())
        {
            Face fIm = ShapeCasts.Face(itLF.Value());
            Exploration.MapShapesAndAncestors(fIm, ShapeType.Edge, ShapeType.Face, dmef);
        }

        itLF.Initialize(lfImages);
        for (; itLF.More();)
        {
            Face fIm = ShapeCasts.Face(itLF.Value());
            //
            // valid edges for this split
            dmfmve.TryGetValue(fIm, out HashSet<Shape>? pMVE);
            // invalid edges for this split
            dmfmie.TryGetValue(fIm, out IndexedSet<Shape>? pMIE);
            //
            bHasValid = false;
            bAllValid = true;
            bAllInvalid = true;
            bHasReallyInvalid = false;
            bAllInvNeutral = true;
            bIsInvalidByInverted = true;
            bHasInverted = false;
            nbChecked = 0;
            //
            Wire wIm = BrepTools.OuterWire(fIm);
            Explorer exp = new(wIm, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Shape eIm = exp.Current();
                //
                bValid = _validEdges.Contains(eIm);
                bInvalid = _invalidEdges.Contains(eIm);
                bNeutral = meNeutral.Contains(eIm);
                //
                if (!bValid && !bInvalid && !bNeutral)
                {
                    // edge has not been checked for some reason
                    continue;
                }

                // skip not-boundary edges originated from vertex
                if ((edgesInvalidByVertex.Contains(eIm) || edgesValidByVertex.Contains(eIm))
                    && dmef.GetValue(eIm).Extent() != 1)
                {
                    continue;
                }

                ++nbChecked;
                //
                bInvalidLoc = pMIE is not null && pMIE.Contains(eIm);
                bHasReallyInvalid = bInvalid && bInvalidLoc && !bValid && !edgesInvalidByVertex.Contains(eIm);
                if (bHasReallyInvalid)
                {
                    break;
                }
                //
                bValidLoc = pMVE is not null && pMVE.Contains(eIm);
                bInverted = _invertedEdges.Contains(eIm);
                if (!bInvalid && !bInvalidLoc && bTreatInvertedAsInvalid)
                {
                    bInvalid = bInverted;
                }
                //
                if (bValidLoc && bNeutral)
                {
                    bHasValid = true;
                }
                //
                bAllValid &= bValidLoc;
                bAllInvalid &= (bInvalid || bInvalidLoc);
                bAllInvNeutral &= (bAllInvalid && bNeutral);
                bIsInvalidByInverted &= (bInvalidLoc || bInverted);
                bHasInverted |= bInverted;
            }
            //
            if (nbChecked == 0)
            {
                itLF.Next();
                continue;
            }
            //
            if (!bHasReallyInvalid && (bAllInvNeutral && !bHasValid) && (nbChecked > 1))
            {
                if (bHasInverted)
                {
                    // The part seems to be filled due to overlapping of parts rather than
                    // due to multi-connection of faces. No need to remove the part.
                    itLF.Next();
                    continue;
                }
                // remove edges from neutral
                Exploration.MapShapes(fIm, ShapeType.Edge, menRem);
                // remove face
                lfImages.Remove(itLF);
                continue;
            }
            //
            if (bHasReallyInvalid || (bAllInvalid && !(bHasValid || bAllValid) && (!bAllInvNeutral || (nbChecked != 1))))
            {
                invFaces.Append(fIm);
                if (mfHoles.Contains(fIm))
                {
                    mfInvInHole.Add(fIm);
                }
                itLF.Next();
                continue;
            }
            //
            if (mfHoles.Contains(fIm))
            {
                // remove edges from neutral
                Exploration.MapShapes(fIm, ShapeType.Edge, menRem);
                // remove face
                lfImages.Remove(itLF);
                continue;
            }
            //
            if (bIsInvalidByInverted && !(bHasValid || bAllValid))
            {
                // The face contains only the inverted and locally invalid edges
                invertedFaces.Append(fIm);
            }

            if (!bAllInvNeutral)
            {
                lfpt.Append(fIm);
            }
            else
            {
                // remove edges from neutral
                Exploration.MapShapes(fIm, ShapeType.Edge, menRem);
            }
            itLF.Next();
        }
        //
        if (lfpt.IsEmpty() || menRem.Count == 0)
        {
            return;
        }

        // check the splits once more
        itLF.Initialize(lfpt);
        for (; itLF.More(); itLF.Next())
        {
            Face fIm = ShapeCasts.Face(itLF.Value());
            //
            // valid edges for this split
            dmfmve.TryGetValue(fIm, out HashSet<Shape>? pMVE);
            //
            bHasValid = false;
            bAllValid = true;
            bAllInvalid = true;
            //
            Wire wIm = BrepTools.OuterWire(fIm);
            Explorer exp = new(wIm, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Shape eIm = exp.Current();
                //
                bValid = _validEdges.Contains(eIm);
                bInvalid = _invalidEdges.Contains(eIm);
                bNeutral = meNeutral.Contains(eIm) && !menRem.Contains(eIm);
                bValidLoc = pMVE is not null && pMVE.Contains(eIm);
                //
                if (!bInvalid && bTreatInvertedAsInvalid)
                {
                    bInvalid = _invertedEdges.Contains(eIm);
                }
                //
                if (bValidLoc && bNeutral)
                {
                    bHasValid = true;
                }
                //
                bAllValid = bAllValid && bValidLoc;
                bAllInvalid = bAllInvalid && bInvalid;
            }
            //
            if (bAllInvalid && !bHasValid && !bAllValid)
            {
                invFaces.Append(fIm);
            }
        }
    }

    //=======================================================================
    // function : FindFacesInsideHoleWires
    // purpose  : Find faces inside holes wires from the original face
    //=======================================================================
    private void FindFacesInsideHoleWires(in Face fOrigin, in Face fOffset,
                                          NCollection_List<Shape> lfImages,
                                          Dictionary<Shape, NCollection_List<Shape>> dmeOrLEIm,
                                          IndexedMap<Shape, NCollection_List<Shape>> efMap,
                                          HashSet<Shape> mfHoles)
    {
        if (lfImages.IsEmpty())
        {
            return;
        }
        //
        // find all hole wires in the original face
        NCollection_List<Shape> lHoleWires = new();
        Wire outerWire = BrepTools.OuterWire(fOrigin);
        Explorer expW = new(fOrigin, ShapeType.Wire);
        for (; expW.More(); expW.Next())
        {
            Wire holeWire = ShapeCasts.Wire(expW.Current());
            if (!holeWire.IsSame(outerWire) && holeWire.Orientation != Internal)
            {
                lHoleWires.Append(holeWire);
            }
        }
        //
        if (lHoleWires.IsEmpty())
        {
            // no holes in the face
            return;
        }
        //
        if (!_fNewHoles.TryGetValue(fOrigin, out NCollection_List<Shape>? pLFNewHoles))
        {
            pLFNewHoles = new NCollection_List<Shape>();
            _fNewHoles[fOrigin] = pLFNewHoles;
        }
        if (pLFNewHoles.IsEmpty())
        {
            //
            // find the faces representing holes in the images of the faces:
            // 1. for each original hole wire try to build its image
            // 2. build the new planar face from the images
            //
            // map vertices and edges of the splits
            IndexedSet<Shape> meSplits = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator itLF0 = new(lfImages);
            for (; itLF0.More(); itLF0.Next())
            {
                Exploration.MapShapes(itLF0.Value(), ShapeType.Edge, meSplits);
            }
            //
            NCollection_List<Shape>.Iterator itLW = new(lHoleWires);
            for (; itLW.More(); itLW.Next())
            {
                Wire holeWire = ShapeCasts.Wire(itLW.Value());
                // find images of all edges of the original wire
                IndexedSet<Shape> meImWire = new(ShapeHasher.Instance);
                Iterator itE = new(holeWire);
                for (; itE.More(); itE.Next())
                {
                    Shape eOr = itE.Value();
                    if (!dmeOrLEIm.TryGetValue(eOr, out NCollection_List<Shape>? pLEIm) || pLEIm.IsEmpty())
                    {
                        continue;
                    }
                    NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
                    for (; itLEIm.More(); itLEIm.Next())
                    {
                        Shape eIm = itLEIm.Value();
                        if (meSplits.Contains(eIm))
                        {
                            meImWire.Add(eIm);
                        }
                    }
                }
                //
                if (meImWire.Count == 0)
                {
                    continue;
                }
                //
                // build new planar face using these edges
                NCollection_List<Shape> le = new();
                int i, nbE0 = meImWire.Count;
                for (i = 1; i <= nbE0; ++i)
                {
                    le.Append(meImWire[i - 1].Oriented(Forward));
                    le.Append(meImWire[i - 1].Oriented(Reversed));
                }
                //
                BuilderFace bf = new();
                bf.SetFace(ShapeCasts.Face(fOffset.Oriented(Forward)));
                bf.SetShapes(le);
                bf.Perform();
                //
                NCollection_List<Shape> lfNew = bf.Areas();
                if (lfNew.IsEmpty())
                {
                    continue;
                }
                //
                // check if outer edges in the new faces are not inverted
                // because the inverted edges mean that the hole has been
                // filled during offset and there will be no faces to remove
                IndexedMap<Shape, NCollection_List<Shape>> dmefNew = new(ShapeHasher.Instance);
                NCollection_List<Shape>.Iterator itLFNew = new(lfNew);
                for (; itLFNew.More(); itLFNew.Next())
                {
                    Exploration.MapShapesAndAncestors(itLFNew.Value(), ShapeType.Edge, ShapeType.Face, dmefNew);
                }
                //
                nbE0 = dmefNew.Count;
                for (i = 1; i <= nbE0; ++i)
                {
                    if (dmefNew.ValueAt(i - 1).Extent() == 1)
                    {
                        Shape e = dmefNew.KeyAt(i - 1);
                        if (_invertedEdges.Contains(e))
                        {
                            break;
                        }
                    }
                }
                //
                if (i <= nbE0)
                {
                    continue;
                }
                //
                itLFNew.Initialize(lfNew);
                for (; itLFNew.More(); itLFNew.Next())
                {
                    pLFNewHoles.Append(itLFNew.Value());
                }
            }
        }

        // Build Edge-Face map for splits of current offset face
        IndexedMap<Shape, NCollection_List<Shape>> efSplitsMap = new(ShapeHasher.Instance);
        // Build Edge-Face map for holes
        IndexedMap<Shape, NCollection_List<Shape>> efHolesMap = new(ShapeHasher.Instance);

        // among the splits of the offset face find those that are
        // located inside the hole faces
        NCollection_List<Shape>.Iterator itLF = new(lfImages);
        for (; itLF.More(); itLF.Next())
        {
            Face fIm = ShapeCasts.Face(itLF.Value());
            Exploration.MapShapesAndAncestors(fIm, ShapeType.Edge, ShapeType.Face, efSplitsMap);
            // get the point inside the face and classify it relatively hole faces
            Point3D p3D = new();
            Point2D p2D = new();
            int iErr = AlgoTools3D.PointInFace(fIm, ref p3D, ref p2D, _context);
            if (iErr != 0)
            {
                continue;
            }
            //
            double tol = Tool.Tolerance(fIm);
            //
            NCollection_List<Shape>.Iterator itLFNew = new(pLFNewHoles);
            for (; itLFNew.More(); itLFNew.Next())
            {
                Face fNew = ShapeCasts.Face(itLFNew.Value());
                if (_context.IsValidPointForFace(p3D, fNew, tol))
                {
                    // the face is classified as IN
                    mfHoles.Add(fIm);
                    Exploration.MapShapesAndAncestors(fIm, ShapeType.Edge, ShapeType.Face, efHolesMap);
                    break;
                }
            }
        }

        // Out of all found holes find those which cannot be removed
        // by checking their connectivity to splits of other offset faces.
        // These are the faces, which will create uncovered holes if removed.
        int nbE = efHolesMap.Count;
        for (int i = 0; i < nbE; ++i)
        {
            Shape edge = efHolesMap.KeyAt(i);
            NCollection_List<Shape> lfHoles = efHolesMap.ValueAt(i);
            // Check if the edge is outer for holes
            if (lfHoles.Extent() != 1)
            {
                continue;
            }

            Shape fHole = lfHoles.First();
            if (!mfHoles.Contains(fHole))
            {
                // Already removed
                continue;
            }

            // Check if the edge is not outer for splits
            NCollection_List<Shape> lSplits = efSplitsMap.GetValue(edge);
            if (lSplits.Extent() == 1)
            {
                continue;
            }

            // Check if edge is only connected to splits of the current offset face
            NCollection_List<Shape> lfAll = efMap.GetValue(edge);
            if (lfAll.Extent() == 2)
            {
                // Avoid removal of the hole from the splits
                mfHoles.Remove(fHole);
            }
        }
    }

    //=======================================================================
    // function : CheckInverted
    // purpose  : Checks if the edge has been inverted
    //=======================================================================
    private bool CheckInverted(in Edge eIm, in Face fOr,
                               IndexedMap<Shape, NCollection_List<Shape>> dmve,
                               IndexedSet<Shape> mEdges)
    {
        // It is necessary to compare the direction from first vertex
        // to the last vertex on the original edge with the
        // same direction on the new edge. If the directions
        // will be different - the edge has been inverted.
        //
        Vertex vo1 = new(), vo2 = new(); // vertices on the original edge
        //
        int i;
        // find vertices of the offset shape
        Exploration.Vertices(eIm, out Vertex vi1, out Vertex vi2); // vertices on the offset edge
        //
        // find images
        NCollection_List<Shape> leImages = new();
        if (_oeOrigins.ContainsKey(eIm))
        {
            Wire images = new();
            new BrepBuilder().MakeWire(ref images);
            //
            HashSet<Shape> mImFence = new(ShapeHasher.Instance);
            NCollection_List<Shape> lOffsetOr = _oeOrigins.GetValue(eIm);
            NCollection_List<Shape>.Iterator itOffset = new(lOffsetOr);
            for (; itOffset.More(); itOffset.Next())
            {
                Shape eOffsetOr = itOffset.Value();
                NCollection_List<Shape> lImages = _oeImages.GetValue(eOffsetOr);
                //
                NCollection_List<Shape>.Iterator itImages = new(lImages);
                for (; itImages.More(); itImages.Next())
                {
                    Edge im = ShapeCasts.Edge(itImages.Value());
                    if (mEdges.Contains(im) && mImFence.Add(im))
                    {
                        new BrepBuilder().Add(images, im);
                        leImages.Append(im);
                    }
                }
            }
            //
            // find alone vertices
            Vertex vw1, vw2;
            IndexedMap<Shape, NCollection_List<Shape>> dmImVE = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(images, ShapeType.Vertex, ShapeType.Edge, dmImVE);
            //
            NCollection_List<Shape> lvAlone = new();
            int nb = dmImVE.Count;
            for (i = 1; i <= nb; ++i)
            {
                NCollection_List<Shape> lImE = dmImVE.ValueAt(i - 1);
                if (lImE.Extent() == 1)
                {
                    lvAlone.Append(dmImVE.KeyAt(i - 1));
                }
            }
            //
            if (lvAlone.Extent() > 1)
            {
                vw1 = ShapeCasts.Vertex(lvAlone.First());
                vw2 = ShapeCasts.Vertex(lvAlone.Last());
                //
                // check distances
                Point3D pi1_ = Tool.Pnt(vi1);
                Point3D pw1 = Tool.Pnt(vw1);
                Point3D pw2 = Tool.Pnt(vw2);
                //
                double dist1 = pi1_.SquareDistanceTo(pw1);
                double dist2 = pi1_.SquareDistanceTo(pw2);
                //
                if (dist1 < dist2)
                {
                    vi1 = vw1;
                    vi2 = vw2;
                }
                else
                {
                    vi1 = vw2;
                    vi2 = vw1;
                }
            }
        }
        else
        {
            leImages.Append(eIm);
        }
        //
        // Find edges connected to these vertices
        NCollection_List<Shape> lie1 = dmve.GetValue(vi1);
        NCollection_List<Shape> lie2 = dmve.GetValue(vi2);
        //
        // Find vertices on the original face corresponding to vertices on the offset edge
        //
        // find original edges for both lists
        NCollection_List<Shape> loe1 = new(), loe2 = new();
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> lie = i == 0 ? lie1 : lie2;
            NCollection_List<Shape> loe = i == 0 ? loe1 : loe2;
            //
            HashSet<Shape> mFence = new(ShapeHasher.Instance);
            //
            NCollection_List<Shape>.Iterator itLIE = new(lie);
            for (; itLIE.More(); itLIE.Next())
            {
                Shape ei = itLIE.Value();
                if (_edgesOrigins!.ContainsKey(ei))
                {
                    NCollection_List<Shape> leOrigins = _edgesOrigins.GetValue(ei);
                    //
                    NCollection_List<Shape>.Iterator aItLOE = new(leOrigins);
                    for (; aItLOE.More(); aItLOE.Next())
                    {
                        Shape eo = aItLOE.Value();
                        if (eo.Type == ShapeType.Edge && mFence.Add(eo))
                        {
                            Shape eOin = new();
                            if (FindShape(eo, fOr, null, ref eOin))
                            {
                                AppendToList(loe, eo);
                            }
                        }
                    }
                }
            }
        }
        //
        if (loe1.Extent() < 2 || loe2.Extent() < 2)
        {
            return false;
        }
        //
        // find vertices common for the max number of edges in the lists
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> loe = i == 0 ? loe1 : loe2;
            ref Vertex vo = ref (i == 0 ? ref vo1 : ref vo2);

            IndexedMap<Shape, NCollection_List<Shape>> dmveLoc = new(ShapeHasher.Instance);
            for (NCollection_List<Shape>.Iterator itLOE = new(loe); itLOE.More(); itLOE.Next())
            {
                Exploration.MapShapesAndAncestors(itLOE.Value(), ShapeType.Vertex, ShapeType.Edge, dmveLoc);
            }

            int nbEMax = 0;
            for (int j = 0; j < dmveLoc.Count; ++j)
            {
                int nbE = dmveLoc.ValueAt(j).Extent();
                if (nbE > 1 && nbE > nbEMax)
                {
                    vo = ShapeCasts.Vertex(dmveLoc.KeyAt(j));
                    nbEMax = nbE;
                }
            }
            if (vo.IsNull)
            {
                return false;
            }
        }

        if (vo1.IsSame(vo2))
        {
            return false;
        }
        //
        // check positions of the offset and original vertices
        Point3D pi1 = Tool.Pnt(vi1);
        Point3D pi2 = Tool.Pnt(vi2);
        Point3D po1 = Tool.Pnt(vo1);
        Point3D po2 = Tool.Pnt(vo2);
        //
        Vector3D vi = new(pi1, pi2);
        Vector3D vOv = new(po1, po2);
        //
        double angle = vi.AngleTo(vOv);
        bool bInverted = Math.Abs(angle - M_PI) < 1.0e-4;
        if (bInverted)
        {
            NCollection_List<Shape>.Iterator itLEIm = new(leImages);
            for (; itLEIm.More(); itLEIm.Next())
            {
                Shape eInvr = itLEIm.Value();
                _invertedEdges.Add(eInvr);
            }
        }
        return bInverted;
    }

    //=======================================================================
    // function : GetVerticesOnEdges
    // purpose  : Get vertices from the given shape belonging to the given edges
    //=======================================================================
    private static void GetVerticesOnEdges(in Shape cb, IndexedSet<Shape> edges,
                                           HashSet<Shape> verticesOnEdges,
                                           HashSet<Shape> allVertices)
    {
        Explorer exp = new(cb, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Shape e = exp.Current();
            bool isOnGivenEdges = edges.Contains(e);
            for (Iterator itV = new(e); itV.More(); itV.Next())
            {
                allVertices.Add(itV.Value());
                if (isOnGivenEdges)
                {
                    verticesOnEdges.Add(itV.Value());
                }
            }
        }
    }

    //=======================================================================
    // function : CheckInvertedBlock
    // purpose  : Checks if it is possible to remove the block containing inverted edges
    //=======================================================================
    private bool CheckInvertedBlock(in Shape cb, NCollection_List<Shape> lcbf,
                                    Dictionary<Shape, HashSet<Shape>> dmcbvInverted,
                                    Dictionary<Shape, HashSet<Shape>> dmcbvAll)
    {
        // For possible removal of the block:
        // 1. There should be more than just one face in the block
        if (cb.ChildCount < 2)
        {
            return false;
        }
        //
        // 2. The block should at least contain two connected inverted edges with
        //    different origins (not just two images/splits of the same edge)
        HashSet<Shape> mecbInv = new(ShapeHasher.Instance);
        Compound cecbInv = new();
        new BrepBuilder().MakeCompound(ref cecbInv);
        //
        Explorer exp = new(cb, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Shape e = exp.Current();
            if (_invertedEdges.Contains(e))
            {
                if (mecbInv.Add(e))
                {
                    new BrepBuilder().Add(cecbInv, e);
                }
            }
        }
        //
        if (mecbInv.Count < 2)
        {
            return false;
        }
        //
        // check that the edges are connected and different
        NCollection_List<Shape> lcbe = new();
        AlgoTools.MakeConnexityBlocks(cecbInv, ShapeType.Vertex, ShapeType.Edge, lcbe);
        //
        NCollection_List<Shape>.Iterator itLCBE = new(lcbe);
        for (; itLCBE.More(); itLCBE.Next())
        {
            Shape cbe = itLCBE.Value();
            // count the unique edges in the block
            int nbUnique = 0;
            HashSet<Shape> meOrigins = new(ShapeHasher.Instance);
            Iterator itE = new(cbe);
            for (; itE.More(); itE.Next())
            {
                Shape e = itE.Value();
                if (!_oeOrigins.TryGetValue(e, out NCollection_List<Shape>? pLEOr))
                {
                    meOrigins.Add(e);
                    ++nbUnique;
                    continue;
                }
                NCollection_List<Shape>.Iterator itLEOr = new(pLEOr);
                for (; itLEOr.More(); itLEOr.Next())
                {
                    Shape eOr = itLEOr.Value();
                    if (meOrigins.Add(eOr))
                    {
                        ++nbUnique;
                    }
                }
            }
            //
            if (nbUnique >= 2)
            {
                break;
            }
        }
        //
        if (!itLCBE.More())
        {
            return false;
        }
        //
        // 3. the block should not contain inverted edges which vertices
        //    are contained in other blocks
        //
        // collect vertices from inverted edges and compare them with
        // vertices from other blocks
        dmcbvInverted.TryGetValue(cb, out HashSet<Shape>? pMVInverted);
        dmcbvAll.TryGetValue(cb, out HashSet<Shape>? pMVAll);
        if (pMVInverted is null)
        {
            pMVInverted = new HashSet<Shape>(ShapeHasher.Instance);
            dmcbvInverted[cb] = pMVInverted;
            pMVAll = new HashSet<Shape>(ShapeHasher.Instance);
            dmcbvAll[cb] = pMVAll;
            //
            GetVerticesOnEdges(cb, _invertedEdges, pMVInverted, pMVAll);
        }
        //
        NCollection_List<Shape>.Iterator itLCB1 = new(lcbf);
        for (; itLCB1.More(); itLCB1.Next())
        {
            Shape cb1 = itLCB1.Value();
            if (cb1.IsSame(cb))
            {
                continue;
            }
            //
            // collect vertices from inverted edges
            dmcbvInverted.TryGetValue(cb1, out HashSet<Shape>? pMVInverted1);
            dmcbvAll.TryGetValue(cb1, out HashSet<Shape>? pMVAll1);
            if (pMVInverted1 is null)
            {
                pMVInverted1 = new HashSet<Shape>(ShapeHasher.Instance);
                dmcbvInverted[cb1] = pMVInverted1;
                pMVAll1 = new HashSet<Shape>(ShapeHasher.Instance);
                dmcbvAll[cb1] = pMVAll1;
                //
                GetVerticesOnEdges(cb1, _invertedEdges, pMVInverted1, pMVAll1);
            }
            //
            if (NCollection_MapAlgo.HasIntersection(pMVInverted, pMVAll1!))
            {
                return false;
            }
        }
        //
        return true;
    }

    //=======================================================================
    // function : RemoveInvalidSplitsByInvertedEdges
    // purpose  : Looking for the invalid faces containing inverted edges
    //           that can be safely removed
    //=======================================================================
    private void RemoveInvalidSplitsByInvertedEdges(IndexedSet<Shape> meRemoved)
    {
        if (_invertedEdges.Count == 0)
        {
            return;
        }
        //
        // check the faces on regularity, i.e. the splits of the same face
        // should not be connected only by vertex. Such irregular splits
        // will have to be rebuilt and cannot be removed.
        //
        BrepBuilder bb = new();
        IndexedSet<Shape> meAvoid = new(ShapeHasher.Instance);
        Dictionary<Shape, NCollection_List<Shape>> dmvf = new(ShapeHasher.Instance);
        int nb = _ofImages.Count, i;
        for (i = 1; i <= nb; ++i)
        {
            NCollection_List<Shape> lfIm = _ofImages.ValueAt(i - 1);
            //
            Compound cfIm = new();
            bb.MakeCompound(ref cfIm);
            //
            Dictionary<Shape, NCollection_List<Shape>> dmef = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator it = new(lfIm);
            for (; it.More(); it.Next())
            {
                Shape f = it.Value();
                bb.Add(cfIm, f);
                //
                // make a map to use only outer edges
                Explorer exp = new(f, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    //
                    if (!dmef.TryGetValue(e, out NCollection_List<Shape>? pLF))
                    {
                        pLF = new NCollection_List<Shape>();
                        dmef[e] = pLF;
                    }
                    else
                    {
                        // internal edges should not be used
                        meAvoid.Add(e);
                    }
                    AppendToList(pLF, f);
                }
                //
                // fill connection map of the vertices of inverted edges to faces
                exp.Init(f, ShapeType.Vertex);
                for (; exp.More(); exp.Next())
                {
                    Shape v = exp.Current();
                    //
                    if (!dmvf.TryGetValue(v, out NCollection_List<Shape>? pLF))
                    {
                        pLF = new NCollection_List<Shape>();
                        dmvf[v] = pLF;
                    }
                    AppendToList(pLF, f);
                }
            }
            //
            // for the splits to be regular they should form only one block
            NCollection_List<Shape> lcbf = new();
            AlgoTools.MakeConnexityBlocks(cfIm, ShapeType.Edge, ShapeType.Face, lcbf);
            if (lcbf.Extent() == 1)
            {
                continue;
            }
            //
            // check if the inverted edges create the irregularity
            Dictionary<Shape, HashSet<Shape>> dmcbvInverted = new(ShapeHasher.Instance),
                                                                             dmcbvAll = new(ShapeHasher.Instance);
            //
            NCollection_List<Shape>.Iterator itLCB = new(lcbf);
            for (; itLCB.More(); itLCB.Next())
            {
                Shape cb = itLCB.Value();
                //
                // check if it is possible to remove the block
                if (!CheckInvertedBlock(cb, lcbf, dmcbvInverted, dmcbvAll))
                {
                    // non of the edges in this block should be removed
                    Exploration.MapShapes(cb, ShapeType.Edge, meAvoid);
                    continue;
                }
            }
        }
        //
        // all edges not included in aMEAvoid can be removed
        HashSet<Shape> meRem = new(ShapeHasher.Instance);
        for (int iInverted = 0; iInverted < _invertedEdges.Count; ++iInverted)
        {
            Shape e = _invertedEdges[iInverted];
            if (!meAvoid.Contains(e))
            {
                Iterator it = new(e);
                for (; it.More(); it.Next())
                {
                    Shape v = it.Value();
                    if (dmvf.TryGetValue(v, out NCollection_List<Shape>? pLF) && (pLF.Extent() > 3))
                    {
                        meRem.Add(e);
                        break;
                    }
                }
            }
        }
        //
        if (meRem.Count == 0)
        {
            return;
        }
        //
        // all invalid faces containing these edges can be removed
        IndexedMap<Shape, NCollection_List<Shape>> invFaces = new(ShapeHasher.Instance);
        HashSet<Shape> mfRem = new(ShapeHasher.Instance);
        IndexedSet<Shape> mfToUpdate = new(ShapeHasher.Instance);
        nb = _invalidFaces.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape f = _invalidFaces.KeyAt(i - 1);
            NCollection_List<Shape> lfIm = _invalidFaces.ValueAt(i - 1);
            //
            NCollection_List<Shape>.Iterator it = new(lfIm);
            for (; it.More();)
            {
                Shape fIm = it.Value();
                //
                // to be removed the face should have at least two not connected
                // inverted edges
                Compound ceInv = new();
                bb.MakeCompound(ref ceInv);
                Explorer exp = new(fIm, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    if (meRem.Contains(e))
                    {
                        bb.Add(ceInv, e);
                    }
                }
                //
                // check connectivity
                NCollection_List<Shape> lcbe = new();
                AlgoTools.MakeConnexityBlocks(ceInv, ShapeType.Vertex, ShapeType.Edge, lcbe);
                //
                if (lcbe.Extent() >= 2)
                {
                    mfToUpdate.Add(f);
                    mfRem.Add(fIm);
                    lfIm.Remove(it);
                }
                else
                {
                    it.Next();
                }
            }
            //
            if (lfIm.Extent() != 0)
            {
                invFaces.Add(f, new NCollection_List<Shape>(lfIm));
            }
        }
        //
        if (mfRem.Count == 0)
        {
            return;
        }
        //
        _invalidFaces = invFaces;
        // remove from splits
        nb = mfToUpdate.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape f = mfToUpdate[i - 1];
            NCollection_List<Shape> lfIm = _ofImages.GetValue(f);
            //
            NCollection_List<Shape>.Iterator it = new(lfIm);
            for (; it.More();)
            {
                Shape fIm = it.Value();
                if (mfRem.Contains(fIm))
                {
                    Exploration.MapShapes(fIm, ShapeType.Edge, meRemoved);
                    lfIm.Remove(it);
                }
                else
                {
                    it.Next();
                }
            }
        }
    }

    //=======================================================================
    // function : RemoveInvalidSplitsFromValid
    // purpose  : Removing invalid splits of faces from valid
    //=======================================================================
    private void RemoveInvalidSplitsFromValid(Dictionary<Shape, HashSet<Shape>> dmfmvie)
    {
        // Decide whether to remove the found invalid faces or not.
        // The procedure is the following:
        // 1. Make connexity blocks from invalid faces;
        // 2. Find free edges in this blocks;
        // 3. If all free edges are valid for the faces - remove block.
        //
        HashSet<Shape> mFence = new(ShapeHasher.Instance), mfToRem = new(ShapeHasher.Instance);
        Compound cfInv = new();
        BrepBuilder bb = new();
        bb.MakeCompound(ref cfInv);
        NCollection_List<Shape>.Iterator itLF = new();
        //
        // make compound of invalid faces
        Dictionary<Shape, Shape> dmifof = new(ShapeHasher.Instance);
        int i, nb = _invalidFaces.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape f = _invalidFaces.KeyAt(i - 1);
            // artificially invalid faces should not be removed
            if (_artInvalidFaces.ContainsKey(f))
            {
                continue;
            }
            NCollection_List<Shape> lfInv = _invalidFaces.ValueAt(i - 1);
            itLF.Initialize(lfInv);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                if (mFence.Add(fIm))
                {
                    bb.Add(cfInv, fIm);
                    dmifof[fIm] = f;
                }
            }
        }
        //
        // make connexity blocks
        NCollection_List<Shape> lcbInv = new();
        AlgoTools.MakeConnexityBlocks(cfInv, ShapeType.Edge, ShapeType.Face, lcbInv);
        //
        // analyze each block
        itLF.Initialize(lcbInv);
        for (; itLF.More(); itLF.Next())
        {
            Shape cb = itLF.Value();
            //
            // if connexity block contains only one face - it should be removed;
            Explorer exp = new(cb, ShapeType.Face);
            exp.Next();
            if (exp.More())
            {
                // check if there are valid images left
                exp.Init(cb, ShapeType.Face);
                for (; exp.More(); exp.Next())
                {
                    Shape fIm = exp.Current();
                    Shape f = dmifof.GetValue(fIm);
                    //
                    NCollection_List<Shape> lfIm = _ofImages.GetValue(f);
                    NCollection_List<Shape> lfInv = _invalidFaces.GetValue(f);
                    //
                    if (lfIm.Extent() == lfInv.Extent())
                    {
                        break;
                    }
                }
            }
            //
            if (!exp.More())
            {
                exp.Init(cb, ShapeType.Face);
                for (; exp.More(); exp.Next())
                {
                    Shape f = exp.Current();
                    mfToRem.Add(f);
                }
                continue;
            }
            //
            // remove faces connected by inverted edges
            IndexedMap<Shape, NCollection_List<Shape>> dmef = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(cb, ShapeType.Edge, ShapeType.Face, dmef);
            //
            Dictionary<Shape, NCollection_List<Shape>> dmff = new(ShapeHasher.Instance);
            exp.Init(cb, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Shape fcb = exp.Current();
                Shape f = dmifof.GetValue(fcb);
                if (!dmff.TryGetValue(f, out NCollection_List<Shape>? pList))
                {
                    pList = new NCollection_List<Shape>();
                    dmff[f] = pList;
                }
                pList.Append(fcb);
            }

            // Kerf: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itM in dmff)
            {
                Shape f = itM.Key;
                dmfmvie.TryGetValue(f, out HashSet<Shape>? pValidInverted);

                // either remove all of these faces or none.
                NCollection_List<Shape> lfcb = itM.Value;
                NCollection_List<Shape>.Iterator itL = new(lfcb);
                for (; itL.More(); itL.Next())
                {
                    Shape fcb = itL.Value();
                    Explorer expE = new(fcb, ShapeType.Edge);
                    for (; expE.More(); expE.Next())
                    {
                        Shape ecb = expE.Current();
                        if (pValidInverted is not null && pValidInverted.Contains(ecb))
                        {
                            break;
                        }
                        if (dmef.GetValue(ecb).Extent() > 1)
                        {
                            if (!_invertedEdges.Contains(ecb))
                            {
                                break;
                            }
                        }
                    }
                    if (!expE.More())
                    {
                        // if one removed - remove all
                        break;
                    }
                }
                if (itL.More())
                {
                    for (itL.Initialize(lfcb); itL.More(); itL.Next())
                    {
                        mfToRem.Add(itL.Value());
                    }
                }
            }
        }
        //
        if (mfToRem.Count != 0)
        {
            // remove invalid faces from images
            nb = _invalidFaces.Count;
            for (i = 1; i <= nb; ++i)
            {
                Shape f = _invalidFaces.KeyAt(i - 1);
                NCollection_List<Shape> lfImages = _ofImages.GetValue(f);
                itLF.Initialize(lfImages);
                for (; itLF.More();)
                {
                    Shape fIm = itLF.Value();
                    if (mfToRem.Contains(fIm))
                    {
                        lfImages.Remove(itLF);
                    }
                    else
                    {
                        itLF.Next();
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void BuildPairs(IndexedSet<Shape> sMap,
                                   Dictionary<Shape, HashSet<Shape>> intPairs)
    {
        int nbS = sMap.Count;
        if (nbS < 2)
        {
            return;
        }
        for (int it1 = 0; it1 < nbS; ++it1)
        {
            Shape s = sMap[it1];
            if (!intPairs.ContainsKey(s))
            {
                intPairs[s] = new HashSet<Shape>(ShapeHasher.Instance);
            }
        }

        for (int it1 = 1; it1 <= nbS; ++it1)
        {
            Shape s1 = sMap[it1 - 1];
            HashSet<Shape> map1 = intPairs.GetValue(s1);
            for (int it2 = it1 + 1; it2 <= nbS; ++it2)
            {
                Shape s2 = sMap[it2 - 1];
                map1.Add(s2);
                intPairs.GetValue(s2).Add(s1);
            }
        }
    }

    //=================================================================================================

    private static void BuildIntersectionPairs(
        IndexedMap<Shape, NCollection_List<Shape>> myOFImages,
        IndexedMap<Shape, NCollection_List<Shape>> myInvalidFaces, BooleanBuilder builder,
        HashSet<Shape> mfRemoved, Dictionary<Shape, Shape> fOrigins,
        Dictionary<Shape, Dictionary<Shape, HashSet<Shape>>> intPairs)
    {
        ShapeType cType = ShapeType.Vertex;
        // Build connection map from vertices to faces
        IndexedMap<Shape, NCollection_List<Shape>> dmvf = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(builder.InternalShape(), cType, ShapeType.Face, dmvf);

        Dictionary<Shape, NCollection_List<Shape>> images = builder.Images();
        Dictionary<Shape, NCollection_List<Shape>> origins = builder.Origins();

        // Find all faces connected to the not removed faces and build intersection pairs among them.
        // For removed faces intersect only those connected to each other.

        for (int iF = 0; iF < myInvalidFaces.Count; ++iF)
        {
            Shape fInv = myInvalidFaces.KeyAt(iF);

            Compound cf = new(), cfRem = new();
            new BrepBuilder().MakeCompound(ref cf);
            new BrepBuilder().MakeCompound(ref cfRem);

            for (int iC = 0; iC < 2; ++iC)
            {
                NCollection_List<Shape> lf = iC == 0 ? myInvalidFaces.ValueAt(iF) : myOFImages.GetValue(fInv);

                for (NCollection_List<Shape>.Iterator it = new(lf); it.More(); it.Next())
                {
                    NCollection_List<Shape> lfIm = new();
                    TakeModified(it.Value(), images, lfIm);

                    for (NCollection_List<Shape>.Iterator itIm = new(lfIm); itIm.More(); itIm.Next())
                    {
                        Shape fIm = itIm.Value();

                        if (mfRemoved.Contains(fIm))
                        {
                            new BrepBuilder().Add(cfRem, fIm);
                        }
                        else
                        {
                            new BrepBuilder().Add(cf, fIm);
                        }
                    }
                }
            }

            NCollection_List<Shape> lcb = new();
            AlgoTools.MakeConnexityBlocks(cf, ShapeType.Edge, ShapeType.Face, lcb);

            if (lcb.IsEmpty())
            {
                continue;
            }

            Dictionary<Shape, HashSet<Shape>> pFInterMap = new(ShapeHasher.Instance);
            intPairs[fInv] = pFInterMap;

            // build pairs for not removed faces
            for (NCollection_List<Shape>.Iterator itCB = new(lcb); itCB.More(); itCB.Next())
            {
                Shape cb = itCB.Value();

                IndexedSet<Shape> mfInter = new(ShapeHasher.Instance);
                for (Explorer exp = new(cb, cType); exp.More(); exp.Next())
                {
                    Shape cs = exp.Current();
                    if (!dmvf.TryGetValue(cs, out NCollection_List<Shape> pLFV))
                    {
                        continue;
                    }

                    for (NCollection_List<Shape>.Iterator itFV = new(pLFV); itFV.More(); itFV.Next())
                    {
                        Shape fConnected = itFV.Value();

                        NCollection_List<Shape> lfOr = new();
                        TakeModified(fConnected, origins, lfOr);
                        for (NCollection_List<Shape>.Iterator itOr = new(lfOr); itOr.More(); itOr.Next())
                        {
                            if (fOrigins.TryGetValue(itOr.Value(), out Shape pFOr))
                            {
                                mfInter.Add(pFOr);
                            }
                        }
                    }
                }

                // build intersection pairs
                BuildPairs(mfInter, pFInterMap);
            }

            lcb.Clear();
            AlgoTools.MakeConnexityBlocks(cfRem, ShapeType.Edge, ShapeType.Face, lcb);

            if (lcb.IsEmpty())
            {
                continue;
            }

            for (NCollection_List<Shape>.Iterator itCB = new(lcb); itCB.More(); itCB.Next())
            {
                Shape cb = itCB.Value();

                IndexedMap<Shape, NCollection_List<Shape>> dmef = new(ShapeHasher.Instance);
                for (Explorer exp = new(cb, cType); exp.More(); exp.Next())
                {
                    Shape cs = exp.Current();
                    if (!dmvf.TryGetValue(cs, out NCollection_List<Shape> pLFV))
                    {
                        continue;
                    }

                    for (NCollection_List<Shape>.Iterator itFV = new(pLFV); itFV.More(); itFV.Next())
                    {
                        Shape fConnected = itFV.Value();
                        Exploration.MapShapesAndAncestors(fConnected, ShapeType.Edge, ShapeType.Face, dmef);
                    }
                }

                for (int iE = 0; iE < dmef.Count; ++iE)
                {
                    NCollection_List<Shape> lfConnected = dmef.ValueAt(iE);
                    if (lfConnected.Extent() < 2)
                    {
                        continue;
                    }

                    IndexedSet<Shape> mfInter = new(ShapeHasher.Instance);
                    for (NCollection_List<Shape>.Iterator itLF = new(lfConnected); itLF.More(); itLF.Next())
                    {
                        Shape fConnected = itLF.Value();

                        NCollection_List<Shape> lfOr = new();
                        TakeModified(fConnected, origins, lfOr);
                        for (NCollection_List<Shape>.Iterator itOr = new(lfOr); itOr.More(); itOr.Next())
                        {
                            if (fOrigins.TryGetValue(itOr.Value(), out Shape pFOr))
                            {
                                mfInter.Add(pFOr);
                            }
                        }
                    }

                    BuildPairs(mfInter, pFInterMap);
                }
            }
        }
    }

    //=======================================================================
    // function : RemoveInsideFaces
    // purpose  : Looking for the inside faces that can be safely removed
    //=======================================================================
    private void RemoveInsideFaces(NCollection_List<Shape> invertedFaces,
                                   IndexedSet<Shape> mfToCheckInt,
                                   IndexedSet<Shape> mfInvInHole, in Shape fHoles,
                                   IndexedSet<Shape> meRemoved,
                                   IndexedSet<Shape> meInside, ProgressRange range)
    {
        NCollection_List<Shape> ls = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        IndexedSet<Shape> mfInv = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLF = new();
        Dictionary<Shape, Shape> dmfImF = new(ShapeHasher.Instance);
        //
        using ProgressScope ps = new(range, "Looking for inside faces", 10);
        int i, nb = _ofImages.Count;
        for (i = 1; i <= nb; ++i)
        {
            if (!ps.More())
            {
                return;
            }
            Shape f = _ofImages.KeyAt(i - 1);
            // to avoid intersection of the splits of the same
            // offset faces among themselves make compound of the
            // splits and use it as one argument
            Compound cfImi = new();
            new BrepBuilder().MakeCompound(ref cfImi);
            //
            for (int j = 0; j < 2; ++j)
            {
                NCollection_List<Shape>? pLFSp = j == 0
                    ? (_invalidFaces.TryGetValue(f, out NCollection_List<Shape> lInv) ? lInv : null)
                    : _ofImages.ValueAt(i - 1);
                if (pLFSp is null)
                {
                    continue;
                }
                //
                aItLF.Initialize(pLFSp);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape fIm = aItLF.Value();
                    if (mFence.Add(fIm))
                    {
                        new BrepBuilder().Add(cfImi, fIm);
                        dmfImF[fIm] = f;
                        if (j == 0)
                        {
                            mfInv.Add(fIm);
                        }
                    }
                }
            }
            //
            ls.Append(cfImi);
        }
        //
        // to make the solids more complete add for intersection also the faces
        // consisting only of invalid edges and not included into splits
        nb = mfToCheckInt.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape fSp = mfToCheckInt[i - 1];
            if (mFence.Add(fSp))
            {
                ls.Append(fSp);
            }
        }
        //
        MakerVolume mv = new();
        mv.SetArguments(ls);
        mv.SetIntersect(true);
        mv.Perform(ps.Next(9));
        if (mv.HasErrors)
        {
            return;
        }

        //
        // get shapes connection for using in the rebuilding process
        // for the cases in which some of the intersection left undetected
        ShapesConnections(dmfImF, mv);
        //
        // find faces to remove
        Shape sols = mv.InternalShape();
        //
        IndexedMap<Shape, NCollection_List<Shape>> dmfs = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(sols, ShapeType.Face, ShapeType.Solid, dmfs);
        //
        nb = dmfs.Count;
        if (nb == 0)
        {
            return;
        }
        //
        // To use the created solids for classifications, firstly, it is necessary
        // to check them on validity - the created solids should be complete,
        // i.e. all faces should be included.
        //
        HashSet<Shape> mfToRem = new(ShapeHasher.Instance);
        // Check completeness
        if (mv.HasDeleted)
        {
            IndexedSet<Shape> meHoles = new(ShapeHasher.Instance);
            Exploration.MapShapes(fHoles, ShapeType.Edge, meHoles);

            // Map edges of the solids to check the connectivity
            // of the removed invalid splits
            IndexedSet<Shape> meSols = new(ShapeHasher.Instance);
            Exploration.MapShapes(sols, ShapeType.Edge, meSols);

            // perform additional check on faces
            nb = _ofImages.Count;
            for (i = 1; i <= nb; ++i)
            {
                if (!ps.More())
                {
                    return;
                }
                NCollection_List<Shape> lfIm = _ofImages.ValueAt(i - 1);
                if (lfIm.IsEmpty())
                {
                    continue;
                }

                Shape f = _ofImages.KeyAt(i - 1);
                bool bInvalid = _invalidFaces.ContainsKey(f);
                // For invalid faces it is allowed to be at least connected
                // to the solids, otherwise the solids are considered as broken
                bool bConnected = false;

                bool bFaceKept = false;
                aItLF.Initialize(lfIm);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape fIm = aItLF.Value();
                    if (!mv.IsDeleted(fIm))
                    {
                        bFaceKept = true;
                        continue;
                    }
                    //
                    Explorer expE = new(fIm, ShapeType.Edge);
                    for (; expE.More(); expE.Next())
                    {
                        if (meHoles.Contains(expE.Current()))
                        {
                            bFaceKept = true;
                            mfToRem.Add(fIm);
                            break;
                        }
                        if (!bFaceKept && bInvalid && !bConnected)
                        {
                            bConnected = meSols.Contains(expE.Current());
                        }
                    }
                }
                //
                if (!bFaceKept && !bConnected)
                {
                    return;
                }
            }
        }
        //
        IndexedSet<Shape> meBoundary = new(ShapeHasher.Instance);
        nb = dmfs.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape fIm = dmfs.KeyAt(i - 1);
            NCollection_List<Shape> lSol = dmfs.ValueAt(i - 1);
            if (lSol.Extent() > 1)
            {
                mfToRem.Add(fIm);
            }
            else if (fIm.Orientation != Internal)
            {
                Exploration.MapShapes(fIm, ShapeType.Edge, meBoundary);
            }
        }

        // Tool for getting the splits of faces
        Dictionary<Shape, NCollection_List<Shape>> mvIms = mv.Images();

        // update invalid faces with images
        nb = mfInv.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape fInv = mfInv[i - 1];
            TakeModified(fInv, mvIms, mfInv);
        }

        // Take into account the faces invalid by inverted edges
        for (NCollection_List<Shape>.Iterator itLF = new(invertedFaces); itLF.More(); itLF.Next())
        {
            TakeModified(itLF.Value(), mvIms, mfInv);
        }

        // check if the invalid faces inside the holes are really invalid:
        // check its normal direction - if it has changed relatively the
        // original face the offset face is invalid and should be kept for rebuilding
        int nbFH = mfInvInHole.Count;
        for (i = 1; i <= nbFH; ++i)
        {
            if (!ps.More())
            {
                return;
            }
            Shape fInv = mfInvInHole[i - 1];
            NCollection_List<Shape> lfInvIm = ModifiedOf(mv, fInv);
            if (lfInvIm.IsEmpty())
            {
                lfInvIm.Append(fInv);
            }
            //
            if (!dmfImF.TryGetValue(fInv, out Shape pFOffset))
            {
                continue;
            }
            NCollection_List<Shape>.Iterator itLFInv = new(lfInvIm);
            for (; itLFInv.More(); itLFInv.Next())
            {
                Shape fInvIm = itLFInv.Value();
                if (!dmfs.TryGetValue(fInvIm, out NCollection_List<Shape> pLSols) || pLSols.Extent() != 1)
                {
                    continue;
                }
                //
                Shape fSol = pLSols.First();
                //
                Shape fx = new();
                if (!FindShape(fInvIm, fSol, null, ref fx))
                {
                    continue;
                }
                //
                if (OffsetTool.CheckPlanesNormals(ShapeCasts.Face(fx), ShapeCasts.Face(pFOffset)))
                {
                    // the normal direction has not changed, thus the face can be removed
                    mfToRem.Add(fInvIm);
                }
            }
        }
        //
        Compound solids = new();
        new BrepBuilder().MakeCompound(ref solids);
        HashSet<Shape> mfKeep = new(ShapeHasher.Instance);
        //
        Explorer expS = new(sols, ShapeType.Solid);
        for (; expS.More(); expS.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Shape sol = expS.Current();
            //
            bool bAllInv = true, bAllRemoved = true;

            for (Explorer expF = new(sol, ShapeType.Face); expF.More(); expF.Next())
            {
                Shape fs = expF.Current();
                //
                if (fs.Orientation == Internal)
                {
                    mfToRem.Add(fs);
                    continue;
                }

                if (mfToRem.Contains(fs))
                {
                    continue;
                }

                bAllRemoved = false;
                bAllInv &= mfInv.Contains(fs);
            }
            //
            if (bAllInv && !bAllRemoved)
            {
                // remove invalid faces but keep those that have already been marked for removal
                Explorer expF = new(sol, ShapeType.Face);
                for (; expF.More(); expF.Next())
                {
                    Shape fs = expF.Current();
                    //
                    if (mfToRem.Contains(fs))
                    {
                        if (!mfKeep.Add(fs))
                        {
                            mfKeep.Remove(fs);
                        }
                    }
                    else
                    {
                        mfToRem.Add(fs);
                    }
                }
            }
            else
            {
                new BrepBuilder().Add(solids, sol);
                _solids = solids;
            }
        }
        //
        foreach (Shape itM in mfKeep)
        {
            mfToRem.Remove(itM);
        }

        // Remove the invalid hanging parts external to the solids
        RemoveHangingParts(mv, dmfImF, mfInv, mfToRem);

        // Remove newly found internal and hanging faces
        RemoveValidSplits(mfToRem, mv, meRemoved);
        RemoveInvalidSplits(mfToRem, mv, meRemoved);
        //
        // Get inside faces from the removed ones comparing them with boundary edges
        meInside.Clear();
        nb = meRemoved.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape e = meRemoved[i - 1];
            if (!meBoundary.Contains(e))
            {
                meInside.Add(e);
            }
        }

        // build all possible intersection pairs basing on the intersection results
        // taking into account removed faces.
        if (mfToRem.Count != 0)
        {
            BuildIntersectionPairs(_ofImages, _invalidFaces, mv, mfToRem, dmfImF, _intersectionPairs);
        }
    }

    //=======================================================================
    // function : ShapesConnections
    // purpose  : Looking for the connections between faces not to miss
    //           some necessary intersection
    //=======================================================================
    private void ShapesConnections(Dictionary<Shape, Shape> dmfOr, BooleanBuilder builder)
    {
        // Make connexity blocks from invalid edges to use the whole block
        // to which the edge is connected instead of the single edge.
        Compound ceInv = new();
        new BrepBuilder().MakeCompound(ref ceInv);
        for (int i = 0; i < _invalidEdges.Count; ++i)
        {
            AddToContainer(_invalidEdges[i], ceInv);
        }

        NCollection_List<Shape> lcb = new();
        AlgoTools.MakeConnexityBlocks(ceInv, ShapeType.Vertex, ShapeType.Edge, lcb);

        // Binding from the edge to the block
        Dictionary<Shape, Shape> ecbMap = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator itCB = new(lcb); itCB.More(); itCB.Next())
        {
            for (Iterator itE = new(itCB.Value()); itE.More(); itE.Next())
            {
                ecbMap[itE.Value()] = itCB.Value();
            }
        }

        // update invalid edges with images and keep connection to original edge
        Dictionary<Shape, NCollection_List<Shape>> dmeOr = new(ShapeHasher.Instance);
        int nb = _invalidEdges.Count;
        for (int i = 0; i < nb; ++i)
        {
            Shape eInv = _invalidEdges[i];
            NCollection_List<Shape> leIm = ModifiedOf(builder, eInv);
            if (leIm.IsEmpty())
            {
                NCollection_List<Shape> l = new();
                l.Append(eInv);
                dmeOr[eInv] = l;
                continue;
            }
            //
            NCollection_List<Shape>.Iterator itLEIm = new(leIm);
            for (; itLEIm.More(); itLEIm.Next())
            {
                Shape eIm = itLEIm.Value();
                AddToContainer(eIm, eInv, dmeOr);
            }
        }
        //
        // get shapes connections for using in the rebuilding process
        DS pDS = builder.PDS()!;
        // analyze all Face/Face intersections
        List<InterfFF> fFs = pDS.InterfFF();
        int iInt, nbFF = fFs.Count;
        for (iInt = 0; iInt < nbFF; ++iInt)
        {
            InterfFF ff = fFs[iInt];
            List<SectionCurve> vnc = ff.Curves();
            int nbC = vnc.Count;
            if (nbC == 0)
            {
                continue;
            }
            //
            Shape fIm1 = pDS.Shape(ff.Index1());
            Shape fIm2 = pDS.Shape(ff.Index2());
            //
            if (!dmfOr.TryGetValue(fIm1, out Shape pF1) || !dmfOr.TryGetValue(fIm2, out Shape pF2))
            {
                continue;
            }
            //
            if (pF1.IsSame(pF2))
            {
                continue;
            }
            //
            bool bInv1 = _invalidFaces.ContainsKey(pF1);
            bool bInv2 = _invalidFaces.ContainsKey(pF2);
            //
            if (!bInv1 && !bInv2)
            {
                continue;
            }
            //
            // check if it is real Face/Face intersection
            HashSet<Shape> meInt = new(ShapeHasher.Instance);
            for (int iC = 0; iC < nbC; ++iC)
            {
                SectionCurve nc = vnc[iC];
                NCollection_List<PaveBlock> lpb = nc.PaveBlocks();
                NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
                for (; itLPB.More(); itLPB.Next())
                {
                    PaveBlock pb = itLPB.Value();
                    if (pb.HasEdge(out int nEInt))
                    {
                        Shape eInt = pDS.Shape(nEInt);
                        meInt.Add(eInt);
                    }
                }
            }
            //
            if (meInt.Count == 0)
            {
                continue;
            }
            //
            // check if invalid edges of the face are in the same splits with intersection edges
            for (int i = 0; i < 2; ++i)
            {
                if ((i == 0 && !bInv1) || (i != 0 && !bInv2))
                {
                    continue;
                }
                //
                Shape f = i == 0 ? pF1 : pF2;
                Shape fOp = i == 0 ? pF2 : pF1;
                Shape fIm = i == 0 ? fIm1 : fIm2;
                //
                bool bFound = false;
                //
                NCollection_List<Shape> lfIm = ModifiedOf(builder, fIm);
                if (lfIm.IsEmpty())
                {
                    lfIm.Append(fIm);
                }
                //
                NCollection_List<Shape>.Iterator aItLFIm = new(lfIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    Shape fImIm = aItLFIm.Value();
                    //
                    bool bInv = false, bInt = false;
                    Explorer aExpE = new(fImIm, ShapeType.Edge);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        Shape e = aExpE.Current();
                        if (!bInv)
                        {
                            bInv = dmeOr.ContainsKey(e);
                        }
                        if (!bInt)
                        {
                            bInt = meInt.Contains(e);
                        }
                        if (bInv && bInt)
                        {
                            break;
                        }
                    }
                    //
                    if (!bInt || !bInv)
                    {
                        continue;
                    }
                    //
                    bFound = true;
                    //
                    // append opposite face to all invalid edges in the split
                    aExpE.Init(fImIm, ShapeType.Edge);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        Shape e = aExpE.Current();
                        if (!dmeOr.TryGetValue(e, out NCollection_List<Shape>? pLEOr))
                        {
                            continue;
                        }
                        //
                        NCollection_List<Shape>.Iterator itLE = new(pLEOr);
                        for (; itLE.More(); itLE.Next())
                        {
                            Shape eOr = itLE.Value();
                            if (!_ssInterfs.TryGetValue(eOr, out NCollection_List<Shape>? pLFE))
                            {
                                pLFE = new NCollection_List<Shape>();
                                _ssInterfs[eOr] = pLFE;
                            }
                            AppendToList(pLFE, fOp);
                        }
                    }
                }
                if (bFound)
                {
                    // save connection between offset faces
                    if (!_ssInterfs.TryGetValue(f, out NCollection_List<Shape>? pLF))
                    {
                        pLF = new NCollection_List<Shape>();
                        _ssInterfs[f] = pLF;
                    }
                    AppendToList(pLF, fOp);
                }
            }

            // Treatment for the artificial case - check if one of the faces is artificially invalid
            for (int iF = 0; iF < 2; ++iF)
            {
                Shape fArt = iF == 0 ? pF1 : pF2;
                Shape fOpposite = iF == 0 ? pF2 : pF1;

                if (!_artInvalidFaces.ContainsKey(fArt))
                {
                    continue;
                }

                if (_invalidFaces.ContainsKey(fOpposite) && !_artInvalidFaces.ContainsKey(fOpposite))
                {
                    continue;
                }

                // Collect own invalid edges of the face and the invalid edges connected to those
                // own invalid edges to be avoided in the check for intersection.
                IndexedSet<Shape> meAvoid = new(ShapeHasher.Instance);
                if (_artInvalidFaces.TryGetValue(fOpposite, out IndexedSet<Shape>? pFEInv))
                {
                    for (int iE = 0; iE < pFEInv.Count; ++iE)
                    {
                        if (ecbMap.TryGetValue(pFEInv[iE], out Shape pCB))
                        {
                            Exploration.MapShapes(pCB, ShapeType.Edge, meAvoid);
                        }
                    }
                }
                else if (_ofImages.TryGetValue(fOpposite, out NCollection_List<Shape> pLFIm))
                {
                    for (NCollection_List<Shape>.Iterator itLFIm = new(pLFIm); itLFIm.More(); itLFIm.Next())
                    {
                        for (Explorer expE = new(itLFIm.Value(), ShapeType.Edge); expE.More(); expE.Next())
                        {
                            if (ecbMap.TryGetValue(expE.Current(), out Shape pCB))
                            {
                                Exploration.MapShapes(pCB, ShapeType.Edge, meAvoid);
                            }
                        }
                    }
                }

                Shape fArtIm = iF == 0 ? fIm1 : fIm2;
                Shape fOppositeIm = iF == 0 ? fIm2 : fIm1;

                // Check if there are any intersections between edges of artificially
                // invalid face and opposite face
                int nFOp = pDS.Index(fOppositeIm);

                for (Explorer expE = new(fArtIm, ShapeType.Edge); expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (!_invalidEdges.Contains(e) || _invertedEdges.Contains(e) || meAvoid.Contains(e))
                    {
                        continue;
                    }
                    int nE = pDS.Index(e);
                    if (nE < 0)
                    {
                        continue;
                    }

                    if (!pDS.HasInterf(nE, nFOp))
                    {
                        continue;
                    }

                    NCollection_List<Shape> lv = new();
                    List<InterfEF> eFs = pDS.InterfEF();
                    for (int iEF = 0; iEF < eFs.Count; ++iEF)
                    {
                        InterfEF ef = eFs[iEF];
                        if (ef.Contains(nE) && ef.Contains(nFOp))
                        {
                            if (ef.CommonPart().Type() == ShapeType.Vertex)
                            {
                                lv.Append(pDS.Shape(ef.IndexNew()));
                            }
                        }
                    }

                    if (lv.IsEmpty())
                    {
                        continue;
                    }

                    // Make sure that there is an opposite intersection exists, i.e. some of the edges
                    // of the opposite face intersect the artificially invalid face.
                    int nFArt = pDS.Index(fArtIm);
                    Explorer expEOp = new(fOppositeIm, ShapeType.Edge);
                    for (; expEOp.More(); expEOp.Next())
                    {
                        Shape eOp = expEOp.Current();
                        int nEOp = pDS.Index(eOp);
                        if (pDS.HasInterf(nEOp, nFArt))
                        {
                            break;
                        }
                    }
                    if (!expEOp.More())
                    {
                        continue;
                    }

                    // Intersection is present - add connection between offset faces.
                    AddToContainer(fArt, fOpposite, _ssInterfsArt);

                    // Add connection between edge and opposite face
                    AddToContainer(e, fOpposite, _ssInterfsArt);

                    // Along with the opposite face, save the intersection vertices to
                    // be used for trimming the intersection edge in the rebuilding process
                    for (NCollection_List<Shape>.Iterator itLV = new(lv); itLV.More(); itLV.Next())
                    {
                        // Add connection to intersection vertex
                        AddToContainer(e, itLV.Value(), _ssInterfsArt);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : RemoveHangingParts
    // purpose  : Remove isolated invalid hanging parts
    //=======================================================================
    private void RemoveHangingParts(MakerVolume mv, Dictionary<Shape, Shape> dmfImF,
                                    IndexedSet<Shape> mfInv, HashSet<Shape> mfToRem)
    {
        // Map the faces of the result solids to filter them from avoided faces
        IndexedSet<Shape> mfs = new(ShapeHasher.Instance);
        Exploration.MapShapes(mv.InternalShape(), ShapeType.Face, mfs);

        BrepBuilder bb = new();
        // Build compound of all faces not included into solids
        Compound cfHangs = new();
        bb.MakeCompound(ref cfHangs);

        // Tool for getting the splits of faces
        Dictionary<Shape, NCollection_List<Shape>> mvIms = mv.Images();

        NCollection_List<Shape>.Iterator itLArgs = new(mv.Arguments());
        for (; itLArgs.More(); itLArgs.Next())
        {
            Explorer expF0 = new(itLArgs.Value(), ShapeType.Face);
            for (; expF0.More(); expF0.Next())
            {
                Shape f = expF0.Current();
                TakeModified(f, mvIms, cfHangs, mfs);
            }
        }

        // Make connexity blocks of all hanging parts and check that they are isolated
        NCollection_List<Shape> lcbHangs = new();
        AlgoTools.MakeConnexityBlocks(cfHangs, ShapeType.Edge, ShapeType.Face, lcbHangs);
        if (lcbHangs.IsEmpty())
        {
            return;
        }

        // To be removed, the block should contain invalid splits of offset faces and should
        // meet one of the following conditions:
        // 1. The block should not be connected to any invalid parts (Faces or Edges)
        //    contained in solids;
        // 2. The block should be isolated from other faces, i.e. it should consist of
        //    the splits of the single offset face.

        // Map the edges and vertices of the result solids to check connectivity
        // of the hanging blocks to invalid parts contained in solids
        IndexedMap<Shape, NCollection_List<Shape>> dmef = new(ShapeHasher.Instance),
                                                                                 dmve = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(mv.InternalShape(), ShapeType.Edge, ShapeType.Face, dmef);
        Exploration.MapShapesAndAncestors(mv.InternalShape(), ShapeType.Vertex, ShapeType.Edge, dmve);

        // Update invalid edges with intersection results
        HashSet<Shape> meInv = new(ShapeHasher.Instance);
        int i, nbE = _invalidEdges.Count;
        for (i = 1; i <= nbE; ++i)
        {
            TakeModified(_invalidEdges[i - 1], mvIms, meInv);
        }

        // Update inverted edges with intersection results
        HashSet<Shape> meInverted = new(ShapeHasher.Instance);
        for (int iInv = 0; iInv < _invertedEdges.Count; ++iInv)
        {
            TakeModified(_invertedEdges[iInv], mvIms, meInverted);
        }

        // Tool for getting the origins of the splits
        Dictionary<Shape, NCollection_List<Shape>> mvOrs = mv.Origins();

        // Find hanging blocks to remove
        NCollection_List<Shape> blocksToRemove = new();

        NCollection_List<Shape>.Iterator itLCBH = new(lcbHangs);
        for (; itLCBH.More(); itLCBH.Next())
        {
            Shape cbh = itLCBH.Value();

            // Remove the block containing the inverted edges
            bool bHasInverted = false;
            Explorer expE = new(cbh, ShapeType.Edge);
            for (; expE.More() && !bHasInverted; expE.Next())
            {
                Shape e = expE.Current();
                bHasInverted = !dmef.ContainsKey(e) && meInverted.Contains(e);
            }

            if (bHasInverted)
            {
                blocksToRemove.Append(cbh);
                continue;
            }

            // Check the block to contain invalid split
            bool bHasInvalidFace = false;
            // Check connectivity to invalid parts
            bool bIsConnected = false;
            IndexedSet<Shape> blockME = new(ShapeHasher.Instance);
            Exploration.MapShapes(cbh, ShapeType.Edge, blockME);
            // Map to collect all original faces
            HashSet<Shape> mOffsetF = new(ShapeHasher.Instance);

            Explorer expF = new(cbh, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                Shape f = expF.Current();
                // Check block to contain invalid face
                if (!bHasInvalidFace)
                {
                    bHasInvalidFace = mfInv.Contains(f);
                }

                // Check block for connectivity to invalid parts
                if (!bIsConnected)
                {
                    // check edges
                    expE.Init(f, ShapeType.Edge);
                    for (; expE.More() && !bIsConnected; expE.Next())
                    {
                        Shape e = expE.Current();
                        if (dmef.TryGetValue(e, out NCollection_List<Shape> pLF))
                        {
                            NCollection_List<Shape>.Iterator itLF = new(pLF);
                            for (; itLF.More() && !bIsConnected; itLF.Next())
                            {
                                bIsConnected = mfInv.Contains(itLF.Value());
                            }
                        }
                    }
                    // check vertices
                    if (!bIsConnected)
                    {
                        Explorer expV = new(f, ShapeType.Vertex);
                        for (; expV.More() && !bIsConnected; expV.Next())
                        {
                            Shape v = expV.Current();
                            if (dmve.TryGetValue(v, out NCollection_List<Shape> pLE))
                            {
                                NCollection_List<Shape>.Iterator itLE = new(pLE);
                                for (; itLE.More() && !bIsConnected; itLE.Next())
                                {
                                    bIsConnected = !blockME.Contains(itLE.Value()) && meInv.Contains(itLE.Value());
                                }
                            }
                        }
                    }
                }

                // Check block to be isolated
                if (mvOrs.TryGetValue(f, out NCollection_List<Shape>? pLFOr))
                {
                    NCollection_List<Shape>.Iterator itLFOr = new(pLFOr);
                    for (; itLFOr.More(); itLFOr.Next())
                    {
                        if (dmfImF.TryGetValue(itLFOr.Value(), out Shape pFOffset))
                        {
                            mOffsetF.Add(pFOffset);
                        }
                    }
                }
                else
                {
                    if (dmfImF.TryGetValue(f, out Shape pFOffset))
                    {
                        mOffsetF.Add(pFOffset);
                    }
                }
            }

            bool bRemove = bHasInvalidFace && (!bIsConnected || mOffsetF.Count == 1);

            if (bRemove)
            {
                blocksToRemove.Append(cbh);
            }
        }

        // remove the invalidated blocks
        itLCBH.Initialize(blocksToRemove);
        for (; itLCBH.More(); itLCBH.Next())
        {
            Shape cbh = itLCBH.Value();
            Explorer expF = new(cbh, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                mfToRem.Add(expF.Current());
            }
        }
    }

    //=======================================================================
    // function : RemoveValidSplits
    // purpose  : Removing valid splits according to results of intersection
    //=======================================================================
    private void RemoveValidSplits(HashSet<Shape> spRem, BooleanBuilder gf,
                                   IndexedSet<Shape> meRemoved)
    {
        int i, nb = _ofImages.Count;
        if (nb == 0)
        {
            return;
        }
        //
        for (i = 1; i <= nb; ++i)
        {
            NCollection_List<Shape> lsIm = _ofImages.ValueAt(i - 1);
            NCollection_List<Shape>.Iterator it = new(lsIm);
            for (; it.More();)
            {
                Shape sIm = it.Value();
                if (spRem.Contains(sIm))
                {
                    Exploration.MapShapes(sIm, ShapeType.Edge, meRemoved);
                    lsIm.Remove(it);
                    continue;
                }
                //
                // check if all its images are have to be removed
                NCollection_List<Shape> lsImIm = ModifiedOf(gf, sIm);
                if (lsImIm.Extent() != 0)
                {
                    bool bAllRem = true;
                    NCollection_List<Shape>.Iterator it1 = new(lsImIm);
                    for (; it1.More(); it1.Next())
                    {
                        Shape sImIm = it1.Value();
                        if (spRem.Contains(sImIm))
                        {
                            Exploration.MapShapes(sImIm, ShapeType.Edge, meRemoved);
                        }
                        else
                        {
                            bAllRem = false;
                        }
                    }
                    //
                    if (bAllRem)
                    {
                        Exploration.MapShapes(sIm, ShapeType.Edge, meRemoved);
                        lsIm.Remove(it);
                        continue;
                    }
                }
                it.Next();
            }
        }
    }

    //=======================================================================
    // function : RemoveInvalidSplits
    // purpose  : Removing invalid splits according to the results of intersection
    //=======================================================================
    private void RemoveInvalidSplits(HashSet<Shape> spRem, BooleanBuilder gf,
                                     IndexedSet<Shape> meRemoved)
    {
        int i, nb = _invalidFaces.Count;
        if (nb == 0)
        {
            return;
        }
        //
        for (i = 1; i <= nb; ++i)
        {
            Shape s = _invalidFaces.KeyAt(i - 1);
            bool bArt = _artInvalidFaces.ContainsKey(s);
            //
            NCollection_List<Shape> lsIm = _invalidFaces.ValueAt(i - 1);
            NCollection_List<Shape>.Iterator it = new(lsIm);
            for (; it.More();)
            {
                Shape sIm = it.Value();
                if (spRem.Contains(sIm))
                {
                    Exploration.MapShapes(sIm, ShapeType.Edge, meRemoved);
                    lsIm.Remove(it);
                    continue;
                }
                //
                // check if all its images are have to be removed
                NCollection_List<Shape> lsImIm = ModifiedOf(gf, sIm);
                if (lsImIm.IsEmpty())
                {
                    it.Next();
                    continue;
                }
                //
                bool bAllRem = true;
                IndexedSet<Shape> aMERemoved = new(ShapeHasher.Instance);
                NCollection_List<Shape>.Iterator it1 = new(lsImIm);
                for (; it1.More(); it1.Next())
                {
                    Shape sImIm = it1.Value();
                    if (spRem.Contains(sImIm))
                    {
                        Exploration.MapShapes(sImIm, ShapeType.Edge, aMERemoved);
                    }
                    else
                    {
                        bAllRem = false;
                    }
                }
                //
                if (bAllRem)
                {
                    lsIm.Remove(it);
                    continue;
                }
                //
                if (bArt)
                {
                    it.Next();
                    continue;
                }
                //
                // remove the face from invalid if all invalid edges of this face
                // have been marked for removal
                Explorer expE = new(sIm, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape eInv = expE.Current();
                    if (_invalidEdges.Contains(eInv) && !aMERemoved.Contains(eInv))
                    {
                        break;
                    }
                }
                if (!expE.More())
                {
                    Exploration.MapShapes(sIm, ShapeType.Edge, meRemoved);
                    lsIm.Remove(it);
                }
                else
                {
                    it.Next();
                }
            }
        }
    }

    //=======================================================================
    // function : FilterEdgesImages
    // purpose  : Updating the maps of images and origins of the offset edges
    //=======================================================================
    private void FilterEdgesImages(in Shape s)
    {
        // map edges
        IndexedSet<Shape> me = new(ShapeHasher.Instance);
        Exploration.MapShapes(s, ShapeType.Edge, me);
        //
        _oeOrigins.Clear();
        // Kerf: OCCT iterates the map by its edges' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itDM in _oeImages)
        {
            Shape e = itDM.Key;
            NCollection_List<Shape> leIm = itDM.Value;
            //
            NCollection_List<Shape>.Iterator it = new(leIm);
            for (; it.More();)
            {
                Shape eIm = it.Value();
                // filter images
                if (!me.Contains(eIm))
                {
                    // remove the image
                    // edges with no images left should be kept in the map
                    // to avoid their usage when building the splits of faces
                    leIm.Remove(it);
                    continue;
                }
                //
                // save origins
                if (_oeOrigins.ContainsKey(eIm))
                {
                    AppendToList(_oeOrigins.GetValue(eIm), e);
                }
                else
                {
                    NCollection_List<Shape> lOr = new();
                    lOr.Append(e);
                    _oeOrigins[eIm] = lOr;
                }
                //
                it.Next();
            }
        }
    }

    // OCCT's copy of an IndexedDataMap of lists (a value copy: the lists are copied too).
    private static IndexedMap<Shape, NCollection_List<Shape>> CopyOf(
        IndexedMap<Shape, NCollection_List<Shape>> map)
    {
        IndexedMap<Shape, NCollection_List<Shape>> copy = new(ShapeHasher.Instance);
        for (int i = 0; i < map.Count; ++i)
        {
            copy.Add(map.KeyAt(i), new NCollection_List<Shape>(map.ValueAt(i)));
        }
        return copy;
    }

    //=======================================================================
    // function : FilterInvalidFaces
    // purpose  : Filtering of the invalid faces
    //=======================================================================
    private void FilterInvalidFaces(IndexedMap<Shape, NCollection_List<Shape>> dmef,
                                    IndexedSet<Shape> meRemoved)
    {
        //
        // filter invalid faces, considering faces having only valid
        // images left with non-free edges as valid
        // do not remove invalid faces if it creates free edges
        //
        IndexedMap<Shape, NCollection_List<Shape>> reallyInvFaces = new(ShapeHasher.Instance);
        // Edge-Face connexity map of all splits, both invalid and valid
        IndexedMap<Shape, NCollection_List<Shape>> dmefAll = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLF = new();
        //
        int nb = _invalidFaces.Count;
        for (int i = 0; i < nb; ++i)
        {
            Shape f = _invalidFaces.KeyAt(i);
            NCollection_List<Shape> lfInv = _invalidFaces.ValueAt(i);
            //
            if (_artInvalidFaces.ContainsKey(f))
            {
                if (lfInv.IsEmpty())
                {
                    _artInvalidFaces.Remove(f);
                }
                else
                {
                    reallyInvFaces.Add(f, new NCollection_List<Shape>(lfInv));
                }
                continue;
            }
            //
            if (lfInv.IsEmpty())
            {
                continue;
            }
            //
            NCollection_List<Shape> lfIm = _ofImages.GetValue(f);
            bool bInvalid = lfIm.IsEmpty();
            //
            if (!bInvalid)
            {
                // check two lists on common splits
                aItLF.Initialize(lfInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape fInv = aItLF.Value();
                    //
                    NCollection_List<Shape>.Iterator itLFIm = new(lfIm);
                    for (; itLFIm.More(); itLFIm.Next())
                    {
                        Shape fIm = itLFIm.Value();
                        //
                        if (fInv.IsSame(fIm))
                        {
                            break;
                        }
                    }
                    //
                    if (itLFIm.More())
                    {
                        break;
                    }
                }
                //
                bInvalid = aItLF.More();
            }
            //
            if (!bInvalid)
            {
                // check for free edges
                for (int j = 0; !bInvalid && j < 2; ++j)
                {
                    NCollection_List<Shape> li = j == 0 ? lfIm : lfInv;
                    aItLF.Initialize(li);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        Shape fIm = aItLF.Value();
                        //
                        Explorer exp = new(fIm, ShapeType.Edge);
                        for (; exp.More(); exp.Next())
                        {
                            Shape e = exp.Current();
                            if (!meRemoved.Contains(e))
                            {
                                if (dmef.TryGetValue(e, out NCollection_List<Shape> pLEF) && pLEF.Extent() == 1)
                                {
                                    break;
                                }
                            }
                        }
                        //
                        if (exp.More())
                        {
                            break;
                        }
                    }
                    bInvalid = aItLF.More();
                }
            }
            if (bInvalid)
            {
                if (dmefAll.Count == 0)
                {
                    dmefAll = CopyOf(dmef);
                    for (int iF = 0; iF < nb; ++iF)
                    {
                        for (NCollection_List<Shape>.Iterator itLFInv = new(_invalidFaces.ValueAt(iF)); itLFInv.More(); itLFInv.Next())
                        {
                            Exploration.MapShapesAndAncestors(itLFInv.Value(), ShapeType.Edge, ShapeType.Face, dmefAll);
                        }
                    }
                }

                HashSet<Shape> localSplits = new(ShapeHasher.Instance);
                for (int j = 0; j < 2; ++j)
                {
                    for (aItLF.Initialize((j == 0 ? lfIm : lfInv)); aItLF.More(); aItLF.Next())
                    {
                        localSplits.Add(aItLF.Value());
                    }
                }

                // Check if all invalid edges are located inside the split and do not touch
                // any other faces both invalid and valid
                aItLF.Initialize(lfInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape fIm = aItLF.Value();
                    Explorer exp = new(fIm, ShapeType.Edge);
                    for (; exp.More(); exp.Next())
                    {
                        Shape e = exp.Current();
                        if (_invalidEdges.Contains(e) && !meRemoved.Contains(e))
                        {
                            NCollection_List<Shape> lf = dmefAll.GetValue(e);
                            NCollection_List<Shape>.Iterator itLF = new(lf);
                            for (; itLF.More(); itLF.Next())
                            {
                                if (!localSplits.Contains(itLF.Value()))
                                {
                                    break;
                                }
                            }
                            if (itLF.More())
                            {
                                break;
                            }
                        }
                    }
                    if (exp.More())
                    {
                        break;
                    }
                }
                bInvalid = aItLF.More();
                if (!bInvalid)
                {
                    aItLF.Initialize(lfInv);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        AppendToList(lfIm, aItLF.Value());
                    }
                }
            }
            //
            if (bInvalid)
            {
                reallyInvFaces.Add(f, new NCollection_List<Shape>(lfInv));
            }
        }
        //
        _invalidFaces = reallyInvFaces;
    }

    //=======================================================================
    // function : CheckEdgesCreatedByVertex
    // purpose  : Checks additionally the unchecked edges originated from vertices
    //=======================================================================
    private void CheckEdgesCreatedByVertex()
    {
        // Mark the unchecked edges contained in invalid faces as invalid
        int nbF = _invalidFaces.Count;
        for (int i = 0; i < nbF; ++i)
        {
            Shape f = _invalidFaces.KeyAt(i);
            if (_artInvalidFaces.ContainsKey(f))
            {
                continue;
            }

            NCollection_List<Shape> lfIm = _invalidFaces.ValueAt(i);
            for (NCollection_List<Shape>.Iterator it = new(lfIm); it.More(); it.Next())
            {
                Shape fIm = it.Value();
                for (Explorer expE = new(fIm, ShapeType.Edge); expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (_invalidEdges.Contains(e) || _validEdges.Contains(e))
                    {
                        continue;
                    }

                    // check if this edges is not created from vertex and mark it as invalid
                    if (!_edgesOrigins!.TryGetValue(e, out NCollection_List<Shape>? pLEOr))
                    {
                        continue;
                    }
                    NCollection_List<Shape>.Iterator itLEO = new(pLEOr);
                    for (; itLEO.More(); itLEO.Next())
                    {
                        if (itLEO.Value().Type != ShapeType.Vertex)
                        {
                            break;
                        }
                    }
                    if (!itLEO.More())
                    {
                        _invalidEdges.Add(e);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FilterInvalidEdges
    // purpose  : Filtering the invalid edges according to currently invalid faces
    //=======================================================================
    private void FilterInvalidEdges(Dictionary<Shape, IndexedSet<Shape>> dmfmie,
                                    IndexedSet<Shape> meRemoved,
                                    IndexedSet<Shape> meInside,
                                    HashSet<Shape> meUseInRebuild)
    {
        Compound ceInv = new();
        IndexedSet<Shape> meInv = new(ShapeHasher.Instance);
        BrepBuilder bb = new();
        bb.MakeCompound(ref ceInv);
        NCollection_List<Shape>.Iterator itLF = new();
        //
        int i, nb = _invalidFaces.Count;
        for (i = 1; i <= nb; ++i)
        {
            NCollection_List<Shape> lfInv = _invalidFaces.ValueAt(i - 1);
            itLF.Initialize(lfInv);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                Exploration.MapShapes(fIm, ShapeType.Edge, meInv);
                //
                Explorer expE = new(fIm, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (_invalidEdges.Contains(e))
                    {
                        bb.Add(ceInv, e);
                    }
                }
            }
        }
        //
        // remove edges which have been marked for removal
        IndexedSet<Shape> meInvToAvoid = new(ShapeHasher.Instance);
        NCollection_List<Shape> lcbe = new();
        AlgoTools.MakeConnexityBlocks(ceInv, ShapeType.Vertex, ShapeType.Edge, lcbe);
        //
        NCollection_List<Shape>.Iterator itLCBE = new(lcbe);
        for (; itLCBE.More(); itLCBE.Next())
        {
            Shape cbe = itLCBE.Value();
            Explorer expCB = new(cbe, ShapeType.Edge);
            for (; expCB.More(); expCB.Next())
            {
                Shape e = expCB.Current();
                if (!meRemoved.Contains(e))
                {
                    break;
                }
            }
            //
            if (!expCB.More())
            {
                Exploration.MapShapes(cbe, ShapeType.Edge, meInvToAvoid);
            }
        }
        //
        IndexedSet<Shape> reallyInvEdges = new(ShapeHasher.Instance);
        //
        nb = _invalidFaces.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape f = _invalidFaces.KeyAt(i - 1);
            if (_artInvalidFaces.ContainsKey(f))
            {
                if (dmfmie.TryGetValue(f, out IndexedSet<Shape>? mie))
                {
                    int nbIE = mie.Count;
                    for (int iE = 0; iE < nbIE; ++iE)
                    {
                        Shape e = mie[iE];
                        if (meInv.Contains(e) && !meInvToAvoid.Contains(e))
                        {
                            reallyInvEdges.Add(e);
                        }
                    }
                }
            }
            else
            {
                NCollection_List<Shape> lfInv = _invalidFaces.ValueAt(i - 1);
                itLF.Initialize(lfInv);
                for (; itLF.More(); itLF.Next())
                {
                    Shape fIm = itLF.Value();
                    Explorer expE = new(fIm, ShapeType.Edge);
                    for (; expE.More(); expE.Next())
                    {
                        Shape e = expE.Current();
                        if (_invalidEdges.Contains(e) && !meInvToAvoid.Contains(e))
                        {
                            reallyInvEdges.Add(e);
                        }
                    }
                }
            }
        }

        _invalidEdges = reallyInvEdges;

        // Check if any of the currently invalid edges may be used for
        // rebuilding splits of invalid faces.
        // For that the edge should be inside and not connected to invalid
        // boundary edges of the same origin.

        nb = _invalidEdges.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape e = _invalidEdges[i - 1];
            if (!meInside.Contains(e) || !_validEdges.Contains(e))
            {
                continue;
            }

            if (!_oeOrigins.TryGetValue(e, out NCollection_List<Shape>? pEOrigins))
            {
                continue;
            }

            bool bHasInvOutside = false;
            for (NCollection_List<Shape>.Iterator itEOr = new(pEOrigins); !bHasInvOutside && itEOr.More(); itEOr.Next())
            {
                if (_oeImages.TryGetValue(itEOr.Value(), out NCollection_List<Shape>? pEIms))
                {
                    for (NCollection_List<Shape>.Iterator itEIms = new(pEIms); !bHasInvOutside && itEIms.More(); itEIms.Next())
                    {
                        bHasInvOutside = _invalidEdges.Contains(itEIms.Value()) && !meInside.Contains(itEIms.Value());
                    }
                }
            }
            if (!bHasInvOutside)
            {
                meUseInRebuild.Add(e);
            }
        }
    }

    //=======================================================================
    // function : FindFacesToRebuild
    // purpose  : Looking for the faces that have to be rebuilt:
    //           1. Faces close to invalidity
    //           2. Faces containing some invalid parts
    //=======================================================================
    private void FindFacesToRebuild()
    {
        int i, nb = _ofImages.Count;
        if (nb == 0)
        {
            return;
        }
        //
        bool bRebuild;
        NCollection_List<Shape>.Iterator itLF = new();
        NCollection_List<Shape> leValid = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance), meReb = new(ShapeHasher.Instance),
                                      mfReb = new(ShapeHasher.Instance);
        Explorer exp = new();
        //
        Dictionary<Shape, NCollection_List<Shape>> dmflv = new(ShapeHasher.Instance);
        // get edges from invalid faces
        nb = _invalidFaces.Count;
        for (i = 1; i <= nb; i++)
        {
            Shape f = _invalidFaces.KeyAt(i - 1);
            mFence.Clear();
            NCollection_List<Shape> lvAvoid = new();
            NCollection_List<Shape> lfIm = _invalidFaces.ValueAt(i - 1);
            itLF.Initialize(lfIm);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                exp.Init(fIm, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    meReb.Add(e);
                    if (_invalidEdges.Contains(e))
                    {
                        Explorer expV = new(e, ShapeType.Vertex);
                        for (; expV.More(); expV.Next())
                        {
                            Shape v = expV.Current();
                            if (mFence.Add(v))
                            {
                                lvAvoid.Append(v);
                                meReb.Add(v);
                            }
                        }
                    }
                }
            }
            //
            if (lvAvoid.Extent() != 0)
            {
                dmflv[f] = lvAvoid;
            }
            //
            NCollection_List<Shape>? pLF = null;
            if (!_artInvalidFaces.ContainsKey(f))
            {
                _ssInterfs.TryGetValue(f, out pLF);
            }
            else
            {
                _ssInterfsArt.TryGetValue(f, out pLF);
            }
            if (pLF is not null)
            {
                NCollection_List<Shape>.Iterator itLFE = new(pLF);
                for (; itLFE.More(); itLFE.Next())
                {
                    Shape fe = itLFE.Value();
                    mfReb.Add(fe);
                }
            }
        }
        //
        // get face to rebuild
        nb = _ofImages.Count;
        for (i = 1; i <= nb; i++)
        {
            Shape f = _ofImages.KeyAt(i - 1);
            NCollection_List<Shape> lfIm = _ofImages.ValueAt(i - 1);
            HashSet<Shape> mvAvoid = new(ShapeHasher.Instance);
            if (dmflv.ContainsKey(f))
            {
                NCollection_List<Shape> lvAvoid = dmflv.GetValue(f);
                NCollection_List<Shape>.Iterator itLV = new(lvAvoid);
                for (; itLV.More(); itLV.Next())
                {
                    Shape v = itLV.Value();
                    mvAvoid.Add(v);
                }
            }
            //
            bRebuild = mfReb.Contains(f);
            leValid.Clear();
            mFence.Clear();
            //
            itLF.Initialize(lfIm);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                exp.Init(fIm, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Edge eIm = ShapeCasts.Edge(exp.Current());
                    if (!_invalidEdges.Contains(eIm))
                    {
                        if (mFence.Add(eIm))
                        {
                            leValid.Append(eIm);
                        }
                    }
                    //
                    if (!bRebuild)
                    {
                        bRebuild = meReb.Contains(eIm);
                    }
                    //
                    if (!bRebuild)
                    {
                        // check vertices
                        Explorer expV = new(eIm, ShapeType.Vertex);
                        for (; expV.More() && !bRebuild; expV.Next())
                        {
                            Shape v = expV.Current();
                            if (!mvAvoid.Contains(v))
                            {
                                bRebuild = meReb.Contains(v);
                            }
                        }
                    }
                }
            }
            //
            if (!bRebuild)
            {
                bRebuild = lfIm.Extent() != 0 && _invalidFaces.ContainsKey(f);
                if (bRebuild)
                {
                    _fSelfRebAvoid.Add(f);
                }
            }
            //
            if (bRebuild)
            {
                _facesToRebuild.Add(f, new NCollection_List<Shape>(leValid));
            }
        }
    }

    //=======================================================================
    // function : mapShapes
    // purpose  : Collect theVecShapes into theMap with theType set
    //=======================================================================
    private static void MapShapes(NCollection_List<Shape> vecShapes, ShapeType type,
                                  HashSet<Shape> map)
    {
        foreach (Shape shape in vecShapes)
        {
            for (Explorer exp = new(shape, type); exp.More(); exp.Next())
            {
                map.Add(exp.Current());
            }
        }
    }

    //=======================================================================
    // function : IntersectFaces
    // purpose  : Intersection of the faces that should be rebuild to resolve all invalidities
    //=======================================================================
    private void IntersectFaces(HashSet<Shape> vertsToAvoid, ProgressRange range)
    {
        int nbFR = _facesToRebuild.Count;
        if (nbFR == 0)
        {
            return;
        }

        using ProgressScope psOuter = new(range, "Rebuilding invalid faces", 10);
        //
        int i, j, k, nbInv;
        NCollection_List<Shape>.Iterator itLF = new(), itLE = new();
        //
        // get vertices from invalid edges
        HashSet<Shape> mvInv = new(ShapeHasher.Instance), mvInvAll = new(ShapeHasher.Instance);
        nbInv = _invalidEdges.Count;
        for (i = 1; i <= nbInv; ++i)
        {
            Shape eInv = _invalidEdges[i - 1];
            bool bValid = _validEdges.Contains(eInv);
            for (Explorer exp = new(eInv, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Shape v = exp.Current();
                mvInvAll.Add(v);
                if (!bValid)
                {
                    mvInv.Add(v);
                }
            }
        }
        //
        bool bLookVertToAvoid = (mvInv.Count > 0);
        //
        Dictionary<Shape, NCollection_List<Shape>> dmsf = new(ShapeHasher.Instance),
                                                                          mDone = new(ShapeHasher.Instance),
                                                                          meInfETrim = new(ShapeHasher.Instance),
                                                                          dmveFull = new(ShapeHasher.Instance);
        IndexedMap<Shape, NCollection_List<Shape>> fle = new(ShapeHasher.Instance),
                                                                                 dmefInv = new(ShapeHasher.Instance);
        //
        // Add all faces to rebuild to outgoing map <aFLE>,
        // plus link edges and vertices to the faces to
        // define intersection faces
        PrepareFacesForIntersection(bLookVertToAvoid, fle, mDone, dmsf, meInfETrim, dmveFull, dmefInv);

        // Find vertices to avoid while trimming the edges.
        // These vertices are taken from the invalid edges common between
        // splits of different invalid, but not artificially, faces.
        // Additional condition for these vertices is that all
        // edges adjacent to this vertex must be either invalid
        // or contained in invalid faces
        HashSet<Shape> mvrInv = new(vertsToAvoid,vertsToAvoid.Comparer);
        FindVerticesToAvoid(dmefInv, dmveFull, mvrInv);
        //
        psOuter.Next();
        if (!psOuter.More())
        {
            return;
        }

        // The faces should be intersected selectively -
        // intersect only faces neighboring to the same invalid face
        // and connected to its invalid edges;
        // when dealing with artificially invalid faces for intersection to be
        // complete we need to use not only invalid edges, but also the
        // edges connected to invalid ones

        // find blocks of artificially invalid faces
        Dictionary<Shape, Shape> dmfImF = new(ShapeHasher.Instance);
        Compound cfArt = new();
        new BrepBuilder().MakeCompound(ref cfArt);
        // Kerf: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<Shape, IndexedSet<Shape>> itM in _artInvalidFaces)
        {
            Shape f = itM.Key;
            NCollection_List<Shape> lfInv0 = _invalidFaces.GetValue(f);
            itLF.Initialize(lfInv0);
            for (; itLF.More(); itLF.Next())
            {
                new BrepBuilder().Add(cfArt, itLF.Value());
                dmfImF[itLF.Value()] = f;
            }
        }
        //
        // make connexity blocks
        NCollection_List<Shape> lcbArt = new();
        AlgoTools.MakeConnexityBlocks(cfArt, ShapeType.Vertex, ShapeType.Face, lcbArt);
        //
        // alone edges
        HashSet<Shape> meAlone = new(ShapeHasher.Instance), meInvOnArt = new(ShapeHasher.Instance);
        //
        using ProgressScope psArt = new(psOuter.Next(), null, lcbArt.Extent());
        NCollection_List<Shape>.Iterator itLCBArt = new(lcbArt);
        for (; itLCBArt.More(); itLCBArt.Next(), psArt.Next())
        {
            if (!psArt.More())
            {
                return;
            }
            Shape cb = itLCBArt.Value();
            //
            // check if aCB contains splits of only one offset face
            HashSet<Shape> mfArt = new(ShapeHasher.Instance);
            Explorer expF = new(cb, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                mfArt.Add(dmfImF.GetValue(expF.Current()));
            }
            //
            bool bAlone = (mfArt.Count == 1);
            //
            // vertices on invalid edges
            HashSet<Shape> mveInv = new(ShapeHasher.Instance);
            HashSet<Shape> mFence = new(ShapeHasher.Instance);
            // edges that should not be marked as alone - edges having same origins as invalid ones
            HashSet<Shape> meAvoid = new(ShapeHasher.Instance);
            // map to find alone edges by looking for free vertices
            IndexedMap<Shape, NCollection_List<Shape>> dmveVal = new(ShapeHasher.Instance);
            //
            Explorer aExpE = new(cb, ShapeType.Edge);
            for (; aExpE.More(); aExpE.Next())
            {
                Shape e = aExpE.Current();
                if (_invalidEdges.Contains(e))
                {
                    meInvOnArt.Add(e);
                    for (Iterator aItV = new(e); aItV.More(); aItV.Next())
                    {
                        mveInv.Add(aItV.Value());
                    }
                    //
                    if (bAlone)
                    {
                        if (_oeOrigins.TryGetValue(e, out NCollection_List<Shape>? pLEOr0))
                        {
                            NCollection_List<Shape>.Iterator itLEOr0 = new(pLEOr0);
                            for (; itLEOr0.More(); itLEOr0.Next())
                            {
                                NCollection_List<Shape>.Iterator itLEIm = new(_oeImages.GetValue(itLEOr0.Value()));
                                for (; itLEIm.More(); itLEIm.Next())
                                {
                                    meAvoid.Add(itLEIm.Value());
                                }
                            }
                        }
                    }
                    continue;
                }
                //
                if (mFence.Add(e))
                {
                    Exploration.MapShapesAndAncestors(e, ShapeType.Vertex, ShapeType.Edge, dmveVal);
                }
            }
            //
            // find edges with free vertices
            int nbV = dmveVal.Count;
            for (i = 1; i <= nbV; ++i)
            {
                Shape v = dmveVal.KeyAt(i - 1);
                if (!mveInv.Contains(v))
                {
                    continue;
                }
                //
                NCollection_List<Shape> lev = dmveVal.ValueAt(i - 1);
                if (lev.Extent() > 1)
                {
                    continue;
                }
                //
                Shape e = lev.First();
                if (meAvoid.Contains(e))
                {
                    continue;
                }
                //
                meAlone.Add(e);
                //
                // if this alone edge adds nothing to the intersection list
                // it means that the origin of this edge has been split and we need to
                // add the neighboring images of the same origins
                if (dmsf.GetValue(e).Extent() > 1)
                {
                    continue;
                }
                //
                // check also its vertices
                Iterator aItE = new(e);
                for (; aItE.More(); aItE.Next())
                {
                    Shape ve = aItE.Value();
                    if (dmsf.GetValue(ve).Extent() > 2)
                    {
                        break;
                    }
                }
                //
                if (aItE.More())
                {
                    continue;
                }
                //
                // the edge is useless - look for other images
                if (!_oeOrigins.TryGetValue(e, out NCollection_List<Shape>? pLEOr))
                {
                    continue;
                }
                //
                NCollection_List<Shape>.Iterator itLEOr = new(pLEOr);
                for (; itLEOr.More(); itLEOr.Next())
                {
                    Shape eOr = itLEOr.Value();
                    //
                    NCollection_List<Shape> leIm = _oeImages.GetValue(eOr);
                    NCollection_List<Shape>.Iterator itLEIm = new(leIm);
                    for (; itLEIm.More(); itLEIm.Next())
                    {
                        Shape eIm = itLEIm.Value();
                        //
                        if (mFence.Contains(eIm))
                        {
                            meAlone.Add(eIm);
                        }
                    }
                }
            }
        }
        //
        // Get all invalidities from all faces to be used for avoiding
        // repeated usage of the common edges
        HashSet<Shape> mAllInvs = new(ShapeHasher.Instance);
        nbInv = _invalidFaces.Count;
        for (k = 1; k <= nbInv; ++k)
        {
            NCollection_List<Shape>.Iterator aIt = new(_invalidFaces.ValueAt(k - 1));
            for (; aIt.More(); aIt.Next())
            {
                Explorer exp = new(aIt.Value(), ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    if (_invalidEdges.Contains(e) || meAlone.Contains(e))
                    {
                        mAllInvs.Add(e);
                        Iterator aItV = new(e);
                        for (; aItV.More(); aItV.Next())
                        {
                            mAllInvs.Add(aItV.Value());
                        }
                    }
                }
            }
        }
        //
        // Bounding vertices of not trimmed edges
        HashSet<Shape> mvBounds = new(ShapeHasher.Instance);
        //
        HashSet<Shape> meCheckExt = new(ShapeHasher.Instance);
        // Save connections between not trimmed edge and its trimmed parts
        Dictionary<Shape, NCollection_List<Shape>> dmeeTrim = new(ShapeHasher.Instance);
        // Splits of the new edges
        Dictionary<Shape, NCollection_List<Shape>> eImages = new(ShapeHasher.Instance);
        BrepBuilder bb = new();

        // Keep connection between blocks of invalid edges to the lists of
        // found edges to be intersected for its treatment
        IndexedMap<Shape, NCollection_List<Shape>> dmoenEdges = new(ShapeHasher.Instance);

        nbInv = _invalidFaces.Count;
        using ProgressScope psInter = new(psOuter.Next(5), null, nbInv);
        for (k = 1; k <= nbInv; ++k)
        {
            if (!psInter.More())
            {
                return;
            }
            Shape fInv = _invalidFaces.KeyAt(k - 1);
            bool bSelfRebAvoid = _fSelfRebAvoid.Contains(fInv);
            NCollection_List<Shape> lfInv = _invalidFaces.ValueAt(k - 1);
            //
            NCollection_List<Shape> lcb = new();
            if (lfInv.Extent() > 1)
            {
                // make compound of invalid faces
                Compound cfInv = new();
                bb.MakeCompound(ref cfInv);
                //
                NCollection_List<Shape>.Iterator aIt = new(lfInv);
                for (; aIt.More(); aIt.Next())
                {
                    Shape fIm = aIt.Value();
                    bb.Add(cfInv, fIm);
                }
                //
                // make connexity blocks
                AlgoTools.MakeConnexityBlocks(cfInv, ShapeType.Edge, ShapeType.Face, lcb);
            }
            else
            {
                lcb.Assign(lfInv);
            }
            //
            using ProgressScope pscb = new(psInter.Next(), null, lcb.Extent());
            bool bArtificial = _artInvalidFaces.ContainsKey(fInv);
            NCollection_List<Shape>.Iterator itLCB = new(lcb);
            for (; itLCB.More(); itLCB.Next())
            {
                if (!pscb.More())
                {
                    return;
                }
                Shape cbInv = itLCB.Value();
                //
                HashSet<Shape> meFence = new(ShapeHasher.Instance);
                //
                Compound cbe = new();
                bb.MakeCompound(ref cbe);
                //
                // remember inside edges and vertices to further check
                HashSet<Shape> insideEdges = new(ShapeHasher.Instance);
                HashSet<Shape> insideVertices = new(ShapeHasher.Instance);
                Explorer exp = new(cbInv, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    if (_invalidEdges.Contains(e) || (bArtificial && meAlone.Contains(e)))
                    {
                        if (meFence.Add(e))
                        {
                            bb.Add(cbe, e);
                            if (!_edgesToAvoid.Contains(e) && _invalidEdges.Contains(e))
                            {
                                insideEdges.Add(e);
                                Iterator it = new(e);
                                for (; it.More(); it.Next())
                                {
                                    insideVertices.Add(it.Value());
                                }
                            }
                        }
                    }
                }
                //
                // make connexity blocks of edges
                NCollection_List<Shape> lcbe = new();
                AlgoTools.MakeConnexityBlocks(cbe, ShapeType.Vertex, ShapeType.Edge, lcbe);
                //
                using ProgressScope pscbe = new(pscb.Next(), null, lcbe.Extent());
                NCollection_List<Shape>.Iterator itLCBE = new(lcbe);
                for (; itLCBE.More(); itLCBE.Next())
                {
                    if (!pscbe.More())
                    {
                        return;
                    }
                    Shape cbeLoc = itLCBE.Value();
                    //
                    // map of edges and vertices of processing invalidity
                    IndexedSet<Shape> me = new(ShapeHasher.Instance);
                    // map of vertices to trim the new edges
                    Exploration.MapShapes(cbeLoc, ShapeType.Edge, me);
                    IndexedSet<Shape> mecv = new(me);
                    Exploration.MapShapes(cbeLoc, ShapeType.Vertex, me);
                    IndexedSet<Shape> mfInt = new(ShapeHasher.Instance);
                    // additional faces for intersection
                    IndexedSet<Shape> mfIntExt = new(ShapeHasher.Instance);
                    // splits of faces for intersection
                    NCollection_List<Shape> lfInt = new();
                    // faces to avoid intersection
                    IndexedSet<Shape> mfAvoid = new(ShapeHasher.Instance);
                    //
                    FindFacesForIntersection(fInv, me, dmsf, mvInvAll, bArtificial, mfAvoid, mfInt, mfIntExt, lfInt);
                    if (mfInt.Count < 3)
                    {
                        // nothing to intersect
                        pscbe.Next();
                        continue;
                    }
                    //
                    _intersectionPairs.TryGetValue(fInv, out Dictionary<Shape, HashSet<Shape>>? pMFInter);
                    // intersect the faces, but do not intersect the invalid ones
                    // among each other (except for the artificially invalid faces)
                    IndexedSet<Shape> meToInt = new(ShapeHasher.Instance);
                    int nb = mfInt.Count;
                    using ProgressScope psIntPair = new(pscbe.Next(), null, nb);
                    for (i = 1; i <= nb; ++i, psIntPair.Next())
                    {
                        if (!psIntPair.More())
                        {
                            return;
                        }
                        Face fi = ShapeCasts.Face(mfInt[i - 1]);
                        if (bSelfRebAvoid && fi.IsSame(fInv))
                        {
                            continue;
                        }
                        //
                        if (!_ofImages.TryGetValue(fi, out NCollection_List<Shape> lfImi))
                        {
                            continue;
                        }
                        //
                        if (!fle.TryGetValue(fi, out NCollection_List<Shape> lfEi))
                        {
                            continue;
                        }
                        //
                        NCollection_List<Shape> lfDone = mDone.GetValue(fi);
                        //
                        HashSet<Shape>? pInterFi = null;
                        if (pMFInter is not null)
                        {
                            pMFInter.TryGetValue(fi, out pInterFi);
                        }
                        if (pMFInter is not null && pInterFi is null)
                        {
                            continue;
                        }

                        // create map of edges and vertices for aLFImi
                        HashSet<Shape> mevIm = new(ShapeHasher.Instance);
                        MapShapes(lfImi, ShapeType.Edge, mevIm);
                        MapShapes(lfImi, ShapeType.Vertex, mevIm);

                        bool isIContainsE = NCollection_MapAlgo.HasIntersection(mevIm, insideEdges);
                        bool isIContainsV = NCollection_MapAlgo.HasIntersection(mevIm, insideVertices);

                        for (j = i + 1; j <= nb; ++j)
                        {
                            Face fj = ShapeCasts.Face(mfInt[j - 1]);
                            if (bSelfRebAvoid && fj.IsSame(fInv))
                            {
                                continue;
                            }
                            //
                            if (pInterFi is not null && !pInterFi.Contains(fj))
                            {
                                continue;
                            }

                            if (!_ofImages.TryGetValue(fj, out NCollection_List<Shape> lfImj))
                            {
                                continue;
                            }
                            //
                            if (!fle.TryGetValue(fj, out NCollection_List<Shape> lfEj))
                            {
                                continue;
                            }

                            // create map of edges and vertices for aLFImi
                            mevIm.Clear();
                            MapShapes(lfImj, ShapeType.Edge, mevIm);
                            MapShapes(lfImj, ShapeType.Vertex, mevIm);
                            // check images of both faces contain anInsideEdges and anInsideVertices
                            // not process if false and true
                            bool isJContainsE = NCollection_MapAlgo.HasIntersection(mevIm, insideEdges);
                            bool isJContainsV = NCollection_MapAlgo.HasIntersection(mevIm, insideVertices);

                            // Check if one face is connected to inside edge then
                            // the other must be also connected
                            if ((isIContainsE && !isJContainsV) || (isJContainsE && !isIContainsV))
                            {
                                NCollection_List<Shape> lvc = new();
                                // it is necessary to process the images if they already have
                                // common vertices
                                FindCommonParts(lfImi, lfImj, lvc, ShapeType.Vertex);

                                if (lvc.IsEmpty())
                                {
                                    continue;
                                }
                            }
                            //
                            // if there are some common edges between faces
                            // we should use these edges and do not intersect again.
                            NCollection_List<Shape> lec = new();
                            FindCommonParts(lfImi, lfImj, lec);
                            //
                            if (lec.Extent() != 0)
                            {
                                // no need to intersect if we have common edges between faces
                                bool bForceUse = mfIntExt.Contains(fi) || mfIntExt.Contains(fj);
                                ProcessCommonEdges(lec, me, meInfETrim, mAllInvs, bForceUse, mecv, meCheckExt,
                                                   dmeeTrim, lfEi, lfEj, meToInt);

                                // Add common vertices not belonging to the common edges for trimming the intersection
                                // edges
                                IndexedSet<Shape> mvOnCE = new(ShapeHasher.Instance);
                                for (NCollection_List<Shape>.Iterator itE = new(lec); itE.More(); itE.Next())
                                {
                                    Exploration.MapShapes(itE.Value(), ShapeType.Vertex, mvOnCE);
                                }

                                NCollection_List<Shape> lev = new();
                                FindCommonParts(lfImi, lfImj, lev, ShapeType.Vertex);
                                for (NCollection_List<Shape>.Iterator itV = new(lev); itV.More(); itV.Next())
                                {
                                    if (!mvOnCE.Contains(itV.Value()))
                                    {
                                        mecv.Add(itV.Value());
                                    }
                                }
                                continue;
                            }
                            //
                            // check if both these faces are invalid and sharing edges
                            if (_invalidFaces.ContainsKey(fi) && _invalidFaces.ContainsKey(fj)
                                && !_artInvalidFaces.ContainsKey(fi) && !_artInvalidFaces.ContainsKey(fj))
                            {
                                continue;
                            }
                            //
                            // check if these two faces have already been treated
                            itLE.Initialize(lfDone);
                            for (; itLE.More(); itLE.Next())
                            {
                                Shape f = itLE.Value();
                                if (f.IsSame(fj))
                                {
                                    break;
                                }
                            }
                            //
                            if (itLE.More())
                            {
                                // use intersection line obtained on the previous steps
                                // plus, find new origins for these lines
                                UpdateIntersectedFaces(fInv, fi, fj, lfInv, lfImi, lfImj, lfEi, lfEj, meToInt);
                                continue;
                            }
                            //
                            if (mfAvoid.Contains(fi) || mfAvoid.Contains(fj))
                            {
                                continue;
                            }
                            //
                            lfDone.Append(fj);
                            mDone.GetValue(fj).Append(fi);
                            //
                            IntersectFaces(fInv, fi, fj, lfInv, lfImi, lfImj, lfEi, lfEj, mecv, meToInt);
                        }
                    }
                    //
                    // intersect and trim edges for this chain
                    IntersectAndTrimEdges(mfInt, meToInt, dmeeTrim, me, mecv, mvInv, mvrInv, meCheckExt,
                                          bArtificial ? _ssInterfsArt : null, mvBounds, eImages);
                    //
                    int iE, nbEToInt = meToInt.Count;
                    for (iE = 1; iE <= nbEToInt; ++iE)
                    {
                        Shape eInt = meToInt[iE - 1];
                        Explorer expE = new(cbeLoc, ShapeType.Edge);
                        for (; expE.More(); expE.Next())
                        {
                            Shape e = expE.Current();
                            if (!dmoenEdges.TryGetValue(e, out NCollection_List<Shape> pLEToInt))
                            {
                                pLEToInt = dmoenEdges.ValueAt(dmoenEdges.Add(e, new NCollection_List<Shape>()));
                            }
                            AppendToList(pLEToInt, eInt);
                        }
                    }
                }
            }
        }
        //
        // filter the obtained edges
        UpdateValidEdges(fle, dmoenEdges, mvBounds, meInvOnArt, meCheckExt, vertsToAvoid, eImages, dmeeTrim,
                         psOuter.Next(3));
    }

    //=======================================================================
    // function : PrepareFacesForIntersection
    // purpose  : Preparation of the maps for analyzing intersections of the faces
    //=======================================================================
    private void PrepareFacesForIntersection(bool lookVertToAvoid,
                                             IndexedMap<Shape, NCollection_List<Shape>> fle,
                                             Dictionary<Shape, NCollection_List<Shape>> mDone,
                                             Dictionary<Shape, NCollection_List<Shape>> dmsf,
                                             Dictionary<Shape, NCollection_List<Shape>> meInfETrim,
                                             Dictionary<Shape, NCollection_List<Shape>> dmveFull,
                                             IndexedMap<Shape, NCollection_List<Shape>> dmefInv)
    {
        int i, nb = _facesToRebuild.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape f = _facesToRebuild.KeyAt(i - 1);
            //
            // OCCT binds copies of the same empty list.
            fle.Add(f, new NCollection_List<Shape>());
            mDone[f] = new NCollection_List<Shape>();
            //
            NCollection_List<Shape> lfIm = _ofImages.GetValue(f);
            NCollection_List<Shape>.Iterator itLF = new(lfIm);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                Explorer exp = new(fIm, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    // save connection to untrimmed face
                    if (!dmsf.TryGetValue(e, out NCollection_List<Shape>? pLF))
                    {
                        pLF = new NCollection_List<Shape>();
                        dmsf[e] = pLF;
                    }
                    AppendToList(pLF, f);
                    //
                    // save connection to untrimmed edge
                    Shape eInf = _eTrimEInf!.GetValue(e);
                    if (!meInfETrim.TryGetValue(eInf, out NCollection_List<Shape>? pLETrim))
                    {
                        pLETrim = new NCollection_List<Shape>();
                        meInfETrim[eInf] = pLETrim;
                    }
                    AppendToList(pLETrim, e);
                    //
                    Explorer expV = new(e, ShapeType.Vertex);
                    for (; expV.More(); expV.Next())
                    {
                        Shape v = expV.Current();
                        // save connection to face
                        if (!dmsf.TryGetValue(v, out NCollection_List<Shape>? pLFV))
                        {
                            pLFV = new NCollection_List<Shape>();
                            dmsf[v] = pLFV;
                        }
                        AppendToList(pLFV, f);
                        //
                        if (lookVertToAvoid)
                        {
                            // save connection to edges
                            if (!dmveFull.TryGetValue(v, out NCollection_List<Shape>? pLEV))
                            {
                                pLEV = new NCollection_List<Shape>();
                                dmveFull[v] = pLEV;
                            }
                            AppendToList(pLEV, e);
                        }
                    }
                }
            }
            //
            if (lookVertToAvoid)
            {
                // get edges of invalid faces (from invalid splits only)
                if (!_invalidFaces.TryGetValue(f, out NCollection_List<Shape> pLFInv) || _artInvalidFaces.ContainsKey(f))
                {
                    continue;
                }
                //
                itLF.Initialize(pLFInv);
                for (; itLF.More(); itLF.Next())
                {
                    Shape fInv = itLF.Value();
                    Explorer exp = new(fInv, ShapeType.Edge);
                    for (; exp.More(); exp.Next())
                    {
                        Shape e = exp.Current();
                        if (!dmefInv.TryGetValue(e, out NCollection_List<Shape> pLF))
                        {
                            pLF = dmefInv.ValueAt(dmefInv.Add(e, new NCollection_List<Shape>()));
                        }
                        AppendToList(pLF, f);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FindVerticesToAvoid
    // purpose  : Looking for the invalid vertices
    //=======================================================================
    private void FindVerticesToAvoid(IndexedMap<Shape, NCollection_List<Shape>> dmefInv,
                                     Dictionary<Shape, NCollection_List<Shape>> dmveFull,
                                     HashSet<Shape> mvrInv)
    {
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        int i, nb = dmefInv.Count;
        for (i = 1; i <= nb; ++i)
        {
            NCollection_List<Shape> lfInv = dmefInv.ValueAt(i - 1);
            if (lfInv.Extent() == 1)
            {
                continue;
            }
            //
            Shape e = dmefInv.KeyAt(i - 1);
            if (!_invalidEdges.Contains(e) || _validEdges.Contains(e))
            {
                continue;
            }

            if (!mFence.Add(e))
            {
                continue;
            }

            IndexedMap<Shape, NCollection_List<Shape>> mveEdges = new(ShapeHasher.Instance);
            // Do not check the splitting vertices, but check only the ending ones
            if (_oeOrigins.TryGetValue(e, out NCollection_List<Shape>? pLEOr))
            {
                NCollection_List<Shape>.Iterator itLEOr = new(pLEOr);
                for (; itLEOr.More(); itLEOr.Next())
                {
                    NCollection_List<Shape> leIm = _oeImages.GetValue(itLEOr.Value());
                    NCollection_List<Shape>.Iterator itLEIm = new(leIm);
                    for (; itLEIm.More(); itLEIm.Next())
                    {
                        mFence.Add(itLEIm.Value());
                        Exploration.MapShapesAndAncestors(itLEIm.Value(), ShapeType.Vertex, ShapeType.Edge, mveEdges);
                    }
                }
            }
            else
            {
                Exploration.MapShapesAndAncestors(e, ShapeType.Vertex, ShapeType.Edge, mveEdges);
            }

            int j, nbV = mveEdges.Count;
            for (j = 1; j <= nbV; ++j)
            {
                if (mveEdges.ValueAt(j - 1).Extent() != 1)
                {
                    continue;
                }

                Shape v = mveEdges.KeyAt(j - 1);
                if (!mFence.Add(v))
                {
                    continue;
                }
                if (!dmveFull.TryGetValue(v, out NCollection_List<Shape>? pLE))
                {
                    // isolated vertex
                    mvrInv.Add(v);
                    continue;
                }
                //
                // If all edges sharing the vertex are either invalid or
                // the vertex is connected to at least two inverted edges
                // mark the vertex to be avoided in the new splits
                int iNbEInverted = 0;
                bool bAllEdgesInv = true;
                NCollection_List<Shape>.Iterator itLE = new(pLE);
                for (; itLE.More(); itLE.Next())
                {
                    Shape ev = itLE.Value();

                    if (_invertedEdges.Contains(ev))
                    {
                        ++iNbEInverted;
                    }

                    if (bAllEdgesInv)
                    {
                        bAllEdgesInv = _invalidEdges.Contains(ev);
                    }
                }

                if (iNbEInverted > 1 || bAllEdgesInv)
                {
                    mvrInv.Add(v);
                }
            }
        }
    }

    //=======================================================================
    // function : FindFacesForIntersection
    // purpose  : Looking for the faces around each invalidity for intersection
    //=======================================================================
    private void FindFacesForIntersection(in Shape fInv, IndexedSet<Shape> me,
                                          Dictionary<Shape, NCollection_List<Shape>> dmsf,
                                          HashSet<Shape> mvInvAll, bool artCase,
                                          IndexedSet<Shape> mfAvoid,
                                          IndexedSet<Shape> mfInt,
                                          IndexedSet<Shape> mfIntExt,
                                          NCollection_List<Shape> lfImInt)
    {
        int i, nbE = me.Count;
        //
        IndexedSet<Shape> mShapes = new(ShapeHasher.Instance);
        //
        for (i = 1; i <= nbE; ++i)
        {
            Shape s = me[i - 1];
            if (!dmsf.ContainsKey(s))
            {
                continue;
            }
            //
            // in artificial case we intersect the faces which are close to invalidity
            bool bAvoid = artCase ? ((s.Type == ShapeType.Vertex) && !mvInvAll.Contains(s)) : false;
            //
            NCollection_List<Shape> lf0 = dmsf.GetValue(s);
            NCollection_List<Shape>.Iterator itLF0 = new(lf0);
            for (; itLF0.More(); itLF0.Next())
            {
                Shape f = itLF0.Value();
                if (mfInt.Contains(f))
                {
                    continue;
                }
                //
                if (bAvoid && _artInvalidFaces.ContainsKey(f))
                {
                    mfAvoid.Add(f);
                }
                //
                mfInt.Add(f);
                //
                bool bUse = !f.IsSame(fInv);
                //
                NCollection_List<Shape> lfIm = _ofImages.GetValue(f);
                NCollection_List<Shape>.Iterator itLFIm = new(lfIm);
                for (; itLFIm.More(); itLFIm.Next())
                {
                    Shape fIm = itLFIm.Value();
                    lfImInt.Append(fIm);
                    if (bUse)
                    {
                        Exploration.MapShapes(fIm, ShapeType.Edge, mShapes);
                    }
                }
            }
        }
        //
        Dictionary<Shape, NCollection_List<Shape>> ssInterfsMap = artCase ? _ssInterfsArt : _ssInterfs;
        if (!ssInterfsMap.TryGetValue(fInv, out NCollection_List<Shape>? pLFInv))
        {
            return;
        }
        //
        HashSet<Shape> mf = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLF = new(pLFInv);
        for (; itLF.More(); itLF.Next())
        {
            Shape f = itLF.Value();
            mf.Add(f);
        }
        //
        // the faces should be unique in each place
        Compound cf = new();
        new BrepBuilder().MakeCompound(ref cf);
        //
        IndexedSet<Shape> mfToAdd = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> dmfOr = new(ShapeHasher.Instance);
        //
        for (i = 1; i <= nbE; ++i)
        {
            Shape s = me[i - 1];
            if (!ssInterfsMap.TryGetValue(s, out NCollection_List<Shape>? pLF))
            {
                continue;
            }
            //
            itLF.Initialize(pLF);
            for (; itLF.More(); itLF.Next())
            {
                Shape f = itLF.Value();
                if (mfInt.Contains(f) || mfToAdd.Contains(f) || !mf.Contains(f))
                {
                    continue;
                }
                //
                // check if the face has some connection to already added for intersection faces
                NCollection_List<Shape> lfIm = _ofImages.GetValue(f);
                if (!artCase)
                {
                    NCollection_List<Shape>.Iterator itLFIm0 = new(lfIm);
                    for (; itLFIm0.More(); itLFIm0.Next())
                    {
                        Shape fIm = itLFIm0.Value();
                        Explorer exp = new(fIm, ShapeType.Edge);
                        for (; exp.More(); exp.Next())
                        {
                            if (mShapes.Contains(exp.Current()))
                            {
                                break;
                            }
                        }
                        if (exp.More())
                        {
                            break;
                        }
                    }
                    if (!itLFIm0.More())
                    {
                        continue;
                    }
                }
                //
                mfToAdd.Add(f);
                NCollection_List<Shape>.Iterator itLFIm = new(lfIm);
                for (; itLFIm.More(); itLFIm.Next())
                {
                    Shape fIm = itLFIm.Value();
                    dmfOr[fIm] = f;
                    new BrepBuilder().Add(cf, fIm);
                }
            }
        }
        //
        if (mfToAdd.Count == 0)
        {
            return;
        }
        //
        NCollection_List<Shape> lcb = new();
        AlgoTools.MakeConnexityBlocks(cf, ShapeType.Edge, ShapeType.Face, lcb);
        //
        if ((lcb.Extent() == 1) && (mfToAdd.Count > 1))
        {
            return;
        }
        //
        NCollection_List<Shape>.Iterator itLCB = new(lcb);
        for (; itLCB.More(); itLCB.Next())
        {
            Shape cb = itLCB.Value();
            mfToAdd.Clear();
            Explorer expF = new(cb, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                Shape fIm = expF.Current();
                mfToAdd.Add(dmfOr.GetValue(fIm));
            }
            //
            if (mfToAdd.Count == 1)
            {
                Shape f = mfToAdd[0];
                //
                mfInt.Add(f);
                mfIntExt.Add(f);
                //
                NCollection_List<Shape> lfIm = _ofImages.GetValue(f);
                NCollection_List<Shape>.Iterator itLFIm = new(lfIm);
                for (; itLFIm.More(); itLFIm.Next())
                {
                    Shape fIm = itLFIm.Value();
                    lfImInt.Append(fIm);
                }
            }
        }
    }

    //=======================================================================
    // function : ProcessCommonEdges
    // purpose  : Analyzing the common edges between splits of offset faces
    //=======================================================================
    private void ProcessCommonEdges(NCollection_List<Shape> lec, IndexedSet<Shape> me,
                                    Dictionary<Shape, NCollection_List<Shape>> meInfETrim,
                                    HashSet<Shape> allInvs, bool forceUse,
                                    IndexedSet<Shape> mecv, HashSet<Shape> meCheckExt,
                                    Dictionary<Shape, NCollection_List<Shape>> dmeeTrim,
                                    NCollection_List<Shape> lfEi, NCollection_List<Shape> lfEj,
                                    IndexedSet<Shape> meToInt)
    {
        NCollection_List<Shape> aLEC = new();
        // process common edges
        NCollection_List<Shape>.Iterator itLE = new(lec);
        for (; itLE.More(); itLE.Next())
        {
            Shape ec = itLE.Value();
            //
            // check first if common edges are valid
            if (_invalidEdges.Contains(ec) && !_validEdges.Contains(ec))
            {
                continue;
            }
            //
            // common edge should have connection to current invalidity
            if (me.Contains(ec))
            {
                aLEC.Append(ec);
                continue;
            }
            //
            Iterator itV = new(ec);
            for (; itV.More(); itV.Next())
            {
                Shape ve = itV.Value();
                if (me.Contains(ve))
                {
                    aLEC.Append(ec);
                    break;
                }
            }
        }
        //
        bool bUseOnlyInf = aLEC.IsEmpty();
        if (bUseOnlyInf)
        {
            if (forceUse)
            {
                aLEC.Assign(lec);
            }
            else
            {
                itLE.Initialize(lec);
                for (; itLE.More(); itLE.Next())
                {
                    Shape ec = itLE.Value();
                    // check if all images of the origin of this edge
                    // are not connected to any invalidity
                    Shape eInt0 = _eTrimEInf!.GetValue(ec);
                    NCollection_List<Shape> lve = meInfETrim.GetValue(eInt0);
                    NCollection_List<Shape>.Iterator itLVE = new(lve);
                    for (; itLVE.More(); itLVE.Next())
                    {
                        Shape eCx = itLVE.Value();
                        if (allInvs.Contains(eCx) || _invalidEdges.Contains(eCx))
                        {
                            return;
                        }
                        //
                        Iterator itV = new(eCx);
                        for (; itV.More(); itV.Next())
                        {
                            if (allInvs.Contains(itV.Value()))
                            {
                                return;
                            }
                        }
                    }
                    // use only one element
                    if (aLEC.IsEmpty())
                    {
                        aLEC.Append(ec);
                    }
                }
            }
        }
        //
        itLE.Initialize(aLEC);
        for (; itLE.More(); itLE.Next())
        {
            Shape ec = itLE.Value();
            //
            Shape eInt = _eTrimEInf!.GetValue(ec);
            if (!bUseOnlyInf)
            {
                // find the edges of the same original edge
                // and take their vertices as well
                NCollection_List<Shape> lve = meInfETrim.GetValue(eInt);
                NCollection_List<Shape>.Iterator itLVE = new(lve);
                for (; itLVE.More(); itLVE.Next())
                {
                    Shape eCx = itLVE.Value();
                    //
                    if (!_oeOrigins.TryGetValue(eCx, out NCollection_List<Shape>? pLEOr) || (pLEOr.Extent() == 1))
                    {
                        Exploration.MapShapes(eCx, ShapeType.Vertex, mecv);
                    }
                }
                //
                // bind unlimited edge to its trimmed part in face to update maps of
                // images and origins in the future
                if (!dmeeTrim.TryGetValue(eInt, out NCollection_List<Shape>? pLTAdded))
                {
                    pLTAdded = new NCollection_List<Shape>();
                    dmeeTrim[eInt] = pLTAdded;
                }
                AppendToList(pLTAdded, ec);
            }
            else if (!forceUse)
            {
                meCheckExt.Add(eInt);
            }
            //
            AppendToList(lfEi, eInt);
            AppendToList(lfEj, eInt);
            meToInt.Add(eInt);
        }
    }

    //=================================================================================================

    private static void FindOrigins(NCollection_List<Shape> lfIm1, NCollection_List<Shape> lfIm2,
                                    IndexedSet<Shape> me,
                                    Dictionary<Shape, NCollection_List<Shape>> origins,
                                    NCollection_List<Shape> leOr)
    {
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> lf = i == 0 ? lfIm1 : lfIm2;
            NCollection_List<Shape>.Iterator it = new(lf);
            for (; it.More(); it.Next())
            {
                Shape f = it.Value();
                //
                Explorer exp = new(f, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    //
                    if (me.Contains(e) && origins.ContainsKey(e))
                    {
                        NCollection_List<Shape> aLEOr = origins.GetValue(e);
                        NCollection_List<Shape>.Iterator itLE = new(aLEOr);
                        for (; itLE.More(); itLE.Next())
                        {
                            Shape eOr = itLE.Value();
                            //
                            if (mFence.Add(eOr) && (eOr.Type == ShapeType.Edge))
                            {
                                leOr.Append(eOr);
                            }
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : UpdateIntersectedFaces
    // purpose  : Updating the already interfered faces
    //=======================================================================
    private void UpdateIntersectedFaces(in Shape fInv, in Shape fi, in Shape fj,
                                        NCollection_List<Shape> lfInv, NCollection_List<Shape> lfImi,
                                        NCollection_List<Shape> lfImj, NCollection_List<Shape> lfEi,
                                        NCollection_List<Shape> lfEj, IndexedSet<Shape> meToInt)
    {
        // Find common edges in these two lists
        HashSet<Shape> mEi = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLE = new(lfEi);
        for (; itLE.More(); itLE.Next())
        {
            Shape e = itLE.Value();
            mEi.Add(e);
        }
        //
        // find origins
        IndexedSet<Shape> meToFindOrigins = new(ShapeHasher.Instance);
        NCollection_List<Shape> leToFindOrigins = new();
        if (!fi.IsSame(fInv))
        {
            FindCommonParts(lfImi, lfInv, leToFindOrigins);
        }
        if (!fj.IsSame(fInv))
        {
            FindCommonParts(lfImj, lfInv, leToFindOrigins);
        }
        //
        NCollection_List<Shape> leOrInit = new();
        itLE.Initialize(leToFindOrigins);
        for (; itLE.More(); itLE.Next())
        {
            Shape ec = itLE.Value();
            meToFindOrigins.Add(ec);
        }
        //
        FindOrigins(lfImi, lfImj, meToFindOrigins, _edgesOrigins!, leOrInit);
        //
        itLE.Initialize(lfEj);
        for (; itLE.More(); itLE.Next())
        {
            Shape e = itLE.Value();
            if (mEi.Contains(e))
            {
                meToInt.Add(e);
                if (leOrInit.Extent() != 0)
                {
                    if (_edgesOrigins!.ContainsKey(e))
                    {
                        NCollection_List<Shape> leOr = _edgesOrigins.GetValue(e);
                        NCollection_List<Shape>.Iterator itLEOr = new(leOrInit);
                        for (; itLEOr.More(); itLEOr.Next())
                        {
                            Shape eOr = itLEOr.Value();
                            AppendToList(leOr, eOr);
                        }
                    }
                    else
                    {
                        _edgesOrigins[e] = new NCollection_List<Shape>(leOrInit);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : IntersectFaces
    // purpose  : Intersection of the pair of faces
    //=======================================================================
    private void IntersectFaces(in Shape fInv, in Shape fi, in Shape fj,
                                NCollection_List<Shape> lfInv, NCollection_List<Shape> lfImi,
                                NCollection_List<Shape> lfImj, NCollection_List<Shape> lfEi,
                                NCollection_List<Shape> lfEj, IndexedSet<Shape> mecv,
                                IndexedSet<Shape> meToInt)
    {
        // intersect faces
        Containment side = Out;
        NCollection_List<Shape> lInt1 = new(), lInt2 = new();
        Edge nullEdge = new();
        Face nullFace = new();
        OffsetTool.Inter3D(ShapeCasts.Face(fi), ShapeCasts.Face(fj), lInt1, lInt2, side, nullEdge, nullFace,
                                nullFace);
        //
        if (lInt1.IsEmpty())
        {
            return;
        }
        //
        // find common vertices for trimming edges
        NCollection_List<Shape> lcv = new();
        NCollection_List<Shape>.Iterator itLE = new();
        FindCommonParts(lfImi, lfImj, lcv, ShapeType.Vertex);
        if (lcv.Extent() > 1)
        {
            itLE.Initialize(lcv);
            for (; itLE.More(); itLE.Next())
            {
                Shape cv = itLE.Value();
                mecv.Add(cv);
            }
        }
        //
        // find origins
        IndexedSet<Shape> meToFindOrigins = new(ShapeHasher.Instance);
        NCollection_List<Shape> leToFindOrigins = new();
        if (!fi.IsSame(fInv))
        {
            FindCommonParts(lfImi, lfInv, leToFindOrigins);
        }
        if (!fj.IsSame(fInv))
        {
            FindCommonParts(lfImj, lfInv, leToFindOrigins);
        }
        NCollection_List<Shape> leOrInit = new();
        itLE.Initialize(leToFindOrigins);
        for (; itLE.More(); itLE.Next())
        {
            Shape ec = itLE.Value();
            meToFindOrigins.Add(ec);
        }
        //
        FindOrigins(lfImi, lfImj, meToFindOrigins, _edgesOrigins!, leOrInit);
        //
        itLE.Initialize(lInt1);
        for (; itLE.More(); itLE.Next())
        {
            Shape eInt = itLE.Value();
            lfEi.Append(eInt);
            lfEj.Append(eInt);
            //
            if (leOrInit.Extent() != 0)
            {
                _edgesOrigins![eInt] = new NCollection_List<Shape>(leOrInit);
            }
            //
            meToInt.Add(eInt);
        }
    }

    //=======================================================================
    // function : IntersectAndTrimEdges
    // purpose  : Intersection of the new intersection edges among themselves
    //=======================================================================
    private void IntersectAndTrimEdges(IndexedSet<Shape> mfInt, IndexedSet<Shape> meInt,
                                       Dictionary<Shape, NCollection_List<Shape>> dmeeTrim,
                                       IndexedSet<Shape> msInv, IndexedSet<Shape> mve,
                                       HashSet<Shape> vertsToAvoid,
                                       HashSet<Shape> newVertsToAvoid,
                                       HashSet<Shape> meCheckExt,
                                       Dictionary<Shape, NCollection_List<Shape>>? ssInterfs,
                                       HashSet<Shape> mvBounds,
                                       Dictionary<Shape, NCollection_List<Shape>> eImages)
    {
        int i, nb = meInt.Count;
        if (nb == 0)
        {
            return;
        }
        //
        NCollection_List<Shape> lArgs = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new(), it1 = new();
        Explorer exp = new();
        //
        // get vertices from the splits of intersected faces.
        // vertices are taken from the edges close to invalidity
        //
        IndexedMap<Shape, NCollection_List<Shape>> dmve = new(ShapeHasher.Instance);
        nb = mfInt.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape f = mfInt[i - 1];
            NCollection_List<Shape> le = _facesToRebuild.GetValue(f);
            //
            it.Initialize(le);
            for (; it.More(); it.Next())
            {
                Shape e = it.Value();
                Exploration.MapShapesAndAncestors(e, ShapeType.Vertex, ShapeType.Edge, dmve);
                //
                exp.Init(e, ShapeType.Vertex);
                for (; exp.More(); exp.Next())
                {
                    Shape v1 = exp.Current();
                    if (!vertsToAvoid.Contains(v1) && mve.Contains(v1) && mFence.Add(v1))
                    {
                        lArgs.Append(v1);
                    }
                }
            }
        }
        //
        nb = msInv.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape s = msInv[i - 1];
            // edge case
            if (ssInterfs is not null)
            {
                if (ssInterfs.TryGetValue(s, out NCollection_List<Shape>? pLV))
                {
                    // Add vertices from intersection info to trim section edges of artificial faces
                    for (NCollection_List<Shape>.Iterator itLV = new(pLV); itLV.More(); itLV.Next())
                    {
                        if (itLV.Value().Type == ShapeType.Vertex)
                        {
                            lArgs.Append(itLV.Value());
                        }
                    }
                }
            }

            // vertex case
            if (dmve.TryGetValue(s, out NCollection_List<Shape> pLVE))
            {
                it.Initialize(pLVE);
                for (; it.More(); it.Next())
                {
                    Shape e = it.Value();
                    //
                    exp.Init(e, ShapeType.Vertex);
                    for (; exp.More(); exp.Next())
                    {
                        Shape v1 = exp.Current();
                        if (!vertsToAvoid.Contains(v1) && mFence.Add(v1))
                        {
                            lArgs.Append(v1);
                        }
                    }
                }
            }
        }
        //
        // bounding vertices of untrimmed edges
        NCollection_List<Shape> lvBounds = new();
        // new intersection edges
        NCollection_List<Shape> leNew = new();
        // get edges to intersect
        NCollection_List<Shape> leInt = new();
        // Common intersection edges. Should be intersected separately
        NCollection_List<Shape> lce = new();
        //
        nb = meInt.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape e = meInt[i - 1];
            if (meCheckExt.Contains(e))
            {
                // avoid trimming of the intersection edges by additional common edges
                lce.Append(e);
                continue;
            }
            //
            if (!dmeeTrim.ContainsKey(e))
            {
                leNew.Append(e);
            }
            //
            leInt.Append(e);
            lArgs.Append(e);
            //
            exp.Init(e, ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Shape v = exp.Current();
                lvBounds.Append(v);
            }
        }
        //
        // Intersect Edges
        BooleanBuilder gf = new();
        gf.SetArguments(lArgs);
        gf.Perform();
        if (gf.HasErrors)
        {
            return;
        }
        //
        // update vertices to avoid with SD vertices
        it.Initialize(lvBounds);
        for (; it.More(); it.Next())
        {
            Shape v = it.Value();
            NCollection_List<Shape> lvIm = ModifiedOf(gf, v);
            if (lvIm.IsEmpty())
            {
                mvBounds.Add(v);
            }
            else
            {
                Shape vIm = lvIm.First();
                mvBounds.Add(vIm);
            }
        }
        //
        // find invalid splits of edges
        HashSet<Shape> meInv = new(ShapeHasher.Instance);
        GetInvalidEdges(newVertsToAvoid, mvBounds, gf, meInv);
        //
        BrepBuilder bb = new();
        // get valid splits to intersect with the commons
        Compound ceIm = new();
        bb.MakeCompound(ref ceIm);
        //
        // remove the splits containing vertices from invalid edges
        it.Initialize(leInt);
        for (; it.More(); it.Next())
        {
            Shape e = it.Value();
            //
            NCollection_List<Shape> leIm = ModifiedOf(gf, e);
            if (leIm.IsEmpty())
            {
                continue;
            }
            //
            it1.Initialize(leIm);
            for (; it1.More();)
            {
                Shape eIm = it1.Value();
                //
                if (meInv.Contains(eIm))
                {
                    leIm.Remove(it1);
                }
                else
                {
                    bb.Add(ceIm, eIm);
                    it1.Next();
                }
            }
            //
            if (leIm.Extent() != 0)
            {
                if (!eImages.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
                {
                    pLEIm = new NCollection_List<Shape>();
                    eImages[e] = pLEIm;
                }
                pLEIm.Append(leIm);
            }
        }
        //
        if (lce.Extent() == 0)
        {
            return;
        }
        //
        // trim common edges by other intersection edges
        BooleanBuilder gfce = new();
        gfce.SetArguments(lce);
        gfce.AddArgument(ceIm);
        gfce.Perform();
        //
        if (gfce.HasErrors)
        {
            return;
        }
        //
        DS pDS = gfce.PDS()!;
        NCollection_List<Shape>.Iterator itLCE = new(lce);
        for (; itLCE.More(); itLCE.Next())
        {
            Shape e = itLCE.Value();
            NCollection_List<Shape> leIm = ModifiedOf(gfce, e);
            if (leIm.IsEmpty())
            {
                continue;
            }
            //
            // check if it's not coincide with some intersection edge
            NCollection_List<PaveBlock>.Iterator itLPB = new(pDS.PaveBlocks(pDS.Index(e)));
            for (; itLPB.More(); itLPB.Next())
            {
                if (pDS.IsCommonBlock(itLPB.Value()))
                {
                    // find with what it is a common
                    NCollection_List<PaveBlock> lpbc = pDS.CommonBlock(itLPB.Value())!.PaveBlocks();
                    NCollection_List<PaveBlock>.Iterator itLPBC = new(lpbc);
                    for (; itLPBC.More(); itLPBC.Next())
                    {
                        Shape ec = pDS.Shape(itLPBC.Value().OriginalEdge());
                        if (!meCheckExt.Contains(ec))
                        {
                            break;
                        }
                    }
                    if (itLPBC.More())
                    {
                        break;
                    }
                }
            }
            if (itLPB.More())
            {
                // avoid creation of unnecessary splits from commons which
                // coincide with intersection edges
                continue;
            }
            //
            // save the images
            if (!eImages.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
            {
                pLEIm = new NCollection_List<Shape>();
                eImages[e] = pLEIm;
            }
            pLEIm.Append(leIm);
            //
            // save bounding vertices
            for (Iterator itV = new(e); itV.More(); itV.Next())
            {
                Shape v = itV.Value();
                NCollection_List<Shape> lvIm = ModifiedOf(gfce, v);
                mvBounds.Add(lvIm.IsEmpty() ? v : lvIm.First());
            }
        }
    }

    //=======================================================================
    // function : GetInvalidEdges
    // purpose  : Looking for the invalid edges by intersecting with invalid vertices
    //=======================================================================
    private void GetInvalidEdges(HashSet<Shape> vertsToAvoid, HashSet<Shape> mvBounds,
                                 BooleanBuilder gf, HashSet<Shape> meInv)
    {
        if (vertsToAvoid.Count == 0)
        {
            return;
        }
        //
        NCollection_List<Shape>.Iterator it = new();
        // get vertices created with intersection edges
        Shape res = gf.InternalShape();
        IndexedMap<Shape, NCollection_List<Shape>> dmve = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(res, ShapeType.Vertex, ShapeType.Edge, dmve);
        //
        DS pDS = gf.PDS()!;
        //
        // find invalid splits of edges
        // check if the vertex is invalid:
        // a. it may be the vertex SD with the vertices to avoid
        // b. or it may be the vertex which is created by the intersection
        //    of only existing edges, i.e. no new intersection edges goes
        //    through this vertex
        //
        HashSet<Shape> mvInv = new(ShapeHasher.Instance);
        int i, nb = dmve.Count;
        for (i = 1; i <= nb; ++i)
        {
            Vertex v = ShapeCasts.Vertex(dmve.KeyAt(i - 1));
            if (mvBounds.Contains(v))
            {
                continue;
            }
            //
            int nV = pDS.Index(v);
            if ((nV >= 0) && !pDS.IsNewShape(nV))
            {
                continue;
            }
            //
            // Kerf: OCCT iterates the map by its vertices' addresses; the port in insertion order (decision 1032).
            bool bFound = false;
            foreach (Shape itM in vertsToAvoid)
            {
                Vertex vInv = ShapeCasts.Vertex(itM);
                int iFlag = AlgoTools.ComputeVV(v, vInv);
                if (iFlag == 0)
                {
                    mvInv.Add(v);
                    bFound = true;
                    break;
                }
            }
            //
            if (bFound)
            {
                NCollection_List<Shape> lve = dmve.GetValue(v);
                it.Initialize(lve);
                for (; it.More(); it.Next())
                {
                    Shape e = it.Value();
                    meInv.Add(e);
                }
            }
        }
    }

    //=======================================================================
    // function : UpdateValidEdges
    // purpose  : Making the new splits and updating the maps
    //=======================================================================
    private void UpdateValidEdges(IndexedMap<Shape, NCollection_List<Shape>> fle,
                                  IndexedMap<Shape, NCollection_List<Shape>> oenEdges,
                                  HashSet<Shape> mvBounds, HashSet<Shape> meInvOnArt,
                                  HashSet<Shape> meCheckExt, HashSet<Shape> vertsToAvoid,
                                  Dictionary<Shape, NCollection_List<Shape>> eImages,
                                  Dictionary<Shape, NCollection_List<Shape>> eeTrim,
                                  ProgressRange range)
    {
        using ProgressScope psOuter = new(range, "Updating edges", 10);
        // update images and origins of edges, plus update AsDes
        //
        // new edges
        NCollection_List<Shape> le = new();
        // back connection from edges to faces
        Dictionary<Shape, NCollection_List<Shape>> melf = new(ShapeHasher.Instance);
        //
        HashSet<Shape> meTmp = new(ShapeHasher.Instance);
        int i, nb = fle.Count;
        for (i = 1; i <= nb; ++i)
        {
            Face f = ShapeCasts.Face(fle.KeyAt(i - 1));
            //
            NCollection_List<Shape> leInt = fle.ValueAt(i - 1);
            NCollection_List<Shape>.Iterator itLE0 = new(leInt);
            for (; itLE0.More(); itLE0.Next())
            {
                Shape e = itLE0.Value();
                if ((meCheckExt.Contains(e) || meTmp.Contains(e)) && !eImages.ContainsKey(e))
                {
                    meCheckExt.Remove(e);
                    meTmp.Add(e);
                    continue;
                }
                if (!melf.TryGetValue(e, out NCollection_List<Shape>? pLF))
                {
                    pLF = new NCollection_List<Shape>();
                    melf[e] = pLF;
                    le.Append(e);
                }
                pLF.Append(f);
            }
        }
        //
        if (le.IsEmpty())
        {
            return;
        }
        //
        // bounding edges, that are going to be replaced
        HashSet<Shape> meb = new(ShapeHasher.Instance);
        //
        // new intersection edges
        HashSet<Shape> meNew = new(ShapeHasher.Instance);
        // map of old vertices
        HashSet<Shape> mvOld = new(ShapeHasher.Instance);
        // back connection to untrimmed edges
        Dictionary<Shape, NCollection_List<Shape>> dmeOr = new(ShapeHasher.Instance);
        //
        // trim the new intersection edges
        TrimNewIntersectionEdges(le, eeTrim, mvBounds, meCheckExt, eImages, meb, mvOld, meNew, dmeOr, melf);
        //
        if (eImages.Count == 0)
        {
            // No new splits is preserved
            // update intersection edges and exit
            UpdateNewIntersectionEdges(le, melf, eImages, eeTrim);
            return;
        }

        psOuter.Next();
        if (!psOuter.More())
        {
            return;
        }

        BrepBuilder bb = new();

        // Make connexity blocks of the invalid edges
        // and treat each block separately

        // Compound of all invalid edges to make the blocks
        Compound ceAll = new();
        bb.MakeCompound(ref ceAll);

        int nbE = oenEdges.Count;
        for (i = 1; i <= nbE; ++i)
        {
            bb.Add(ceAll, oenEdges.KeyAt(i - 1));
        }

        // Separate the edges into blocks
        NCollection_List<Shape> lBlocks = new();
        AlgoTools.MakeConnexityBlocks(ceAll, ShapeType.Vertex, ShapeType.Edge, lBlocks);

        // Perform intersection of the new splits for each block

        // Intersected splits
        IndexedMap<Shape, NCollection_List<Shape>> mBlocksSp = new(ShapeHasher.Instance);

        using ProgressScope psb = new(psOuter.Next(), null, lBlocks.Extent());
        NCollection_List<Shape>.Iterator itLB = new(lBlocks);
        for (; itLB.More(); itLB.Next(), psb.Next())
        {
            if (!psb.More())
            {
                return;
            }
            Shape block = itLB.Value();

            // Get the list of new edges for the block
            NCollection_List<Shape> blockLENew = new();
            {
                // Fence map
                HashSet<Shape> meFence = new(ShapeHasher.Instance);
                Explorer expE = new(block, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    NCollection_List<Shape> leInt = oenEdges.GetValue(e);
                    NCollection_List<Shape>.Iterator itLEInt = new(leInt);
                    for (; itLEInt.More(); itLEInt.Next())
                    {
                        if (meFence.Add(itLEInt.Value()))
                        {
                            blockLENew.Append(itLEInt.Value());
                        }
                    }
                }
            }

            if (blockLENew.IsEmpty())
            {
                continue;
            }

            // Get the splits of new edges to intersect
            NCollection_List<Shape> lSplits = new();

            NCollection_List<Shape>.Iterator itLE = new(blockLENew);
            for (; itLE.More(); itLE.Next())
            {
                Shape e = itLE.Value();
                if (!eImages.TryGetValue(e, out NCollection_List<Shape>? pLEIm) || pLEIm.IsEmpty())
                {
                    continue;
                }

                NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    lSplits.Append(itLEIm.Value());
                }
            }

            if (lSplits.IsEmpty())
            {
                continue;
            }

            Shape ce = new();
            if (lSplits.Extent() > 1)
            {
                // Intersect the new splits among themselves to avoid self-intersections
                IntersectEdges(lSplits, blockLENew, mvBounds, vertsToAvoid, meNew, meCheckExt, eImages,
                               dmeOr, melf, ref ce);
            }
            else
            {
                ce = lSplits.First();
            }

            mBlocksSp.Add(ce, blockLENew);
        }

        // Perform filtering of the edges in two steps:
        // - Check each block separately using localized bounds
        //   taken only from the splits of faces of the current block;
        // - Intersect all splits together and filter the splits by all bounds.

        // FIRST STAGE - separate treatment of the blocks

        // Valid splits to be preserved on the first stage
        HashSet<Shape> meVal = new(ShapeHasher.Instance);

        // Blocks of valid edges on the first stage
        NCollection_List<Shape> lValBlocks = new();

        int nbB = mBlocksSp.Count;
        using ProgressScope psbSp = new(psOuter.Next(), null, nbB);
        for (i = 1; i <= nbB; ++i, psbSp.Next())
        {
            if (!psbSp.More())
            {
                return;
            }
            Shape ce = mBlocksSp.KeyAt(i - 1);
            NCollection_List<Shape> blockLENew = mBlocksSp.ValueAt(i - 1);

            // Get all participating faces to get the bounds
            NCollection_List<Shape> lFaces0 = new();
            NCollection_List<Shape>.Iterator itLE = new(blockLENew);
            for (; itLE.More(); itLE.Next())
            {
                if (!melf.TryGetValue(itLE.Value(), out NCollection_List<Shape>? pLF))
                {
                    continue;
                }
                NCollection_List<Shape>.Iterator itLF = new(pLF);
                for (; itLF.More(); itLF.Next())
                {
                    AppendToList(lFaces0, itLF.Value());
                }
            }

            // Localized bounds of the splits of the offset faces
            // to filter the new splits of the current block
            Shape filterBounds0 = new();
            GetBounds(lFaces0, meb, ref filterBounds0);

            // Filter the splits by bounds
            HashSet<Shape> meInvLoc = new(ShapeHasher.Instance);
            GetInvalidEdgesByBounds(ce, filterBounds0, mvOld, meNew, dmeOr, melf, eImages, meCheckExt, meInvOnArt,
                                    vertsToAvoid, meInvLoc);

            // Keep only valid edges of the block
            Compound ceVal = new();
            bb.MakeCompound(ref ceVal);

            bool bKept = false;

            Explorer expE = new(ce, ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                Shape eSp = expE.Current();
                if (!meInvLoc.Contains(eSp) && meVal.Add(eSp))
                {
                    bb.Add(ceVal, eSp);
                    bKept = true;
                }
            }

            if (bKept)
            {
                lValBlocks.Append(ceVal);
            }
        }

        // Filter the images of edges after the first filtering stage
        Shape splits1 = new();
        FilterSplits(le, meVal, false, eImages, ref splits1);

        if (lValBlocks.IsEmpty())
        {
            // update intersection edges
            UpdateNewIntersectionEdges(le, melf, eImages, eeTrim);
            return;
        }

        psOuter.Next();
        if (!psOuter.More())
        {
            return;
        }

        // SECOND STAGE - Filter the remaining splits together

        // Add for intersection already removed new edges using them
        // as markers for other invalid edges
        nbB = mBlocksSp.Count;
        for (i = 1; i <= nbB; ++i)
        {
            Shape ce = mBlocksSp.KeyAt(i - 1);
            for (Explorer exp = new(ce, ShapeType.Edge); exp.More(); exp.Next())
            {
                Shape eIm = exp.Current();
                if (meNew.Contains(eIm) && !meVal.Contains(eIm))
                {
                    lValBlocks.Append(eIm);
                }
            }
        }

        if (lValBlocks.Extent() > 1)
        {
            // intersect the new splits among themselves to avoid self-intersections
            IntersectEdges(lValBlocks, le, mvBounds, vertsToAvoid, meNew, meCheckExt, eImages, dmeOr, melf,
                           ref splits1);
        }
        else
        {
            splits1 = lValBlocks.First();
        }

        psOuter.Next();
        if (!psOuter.More())
        {
            return;
        }

        // Get all faces to get the bounds from their splits
        NCollection_List<Shape> lFaces = new();
        for (i = 1; i <= _ofImages.Count; ++i)
        {
            lFaces.Append(_ofImages.KeyAt(i - 1));
        }

        // Bounds of the splits of the offset faces to filter the new splits
        Shape filterBounds = new();
        GetBounds(lFaces, meb, ref filterBounds);

        // Filter the splits by intersection with bounds
        HashSet<Shape> meInv = new(ShapeHasher.Instance);
        GetInvalidEdgesByBounds(splits1, filterBounds, mvOld, meNew, dmeOr, melf, eImages, meCheckExt, meInvOnArt,
                                vertsToAvoid, meInv);

        // Filter the images of edges after the second filtering stage
        // and combine all valid edges into a single compound
        Shape splits = new();
        FilterSplits(le, meInv, true, eImages, ref splits);

        psOuter.Next();
        if (!psOuter.More())
        {
            return;
        }

        // get bounds to update
        // we need to update the edges of all the affected faces
        NCollection_List<Shape> lf = new();
        // prepare the vertices from new splits of edges
        IndexedSet<Shape> mvSp = new(ShapeHasher.Instance);
        Exploration.MapShapes(splits, ShapeType.Vertex, mvSp);
        //
        int nbF = _ofImages.Count;
        for (i = 1; i <= nbF; ++i)
        {
            Shape f = _ofImages.KeyAt(i - 1);
            if (fle.ContainsKey(f))
            {
                lf.Append(f);
                continue;
            }
            //
            // check the splits of faces to have vertices from splits
            NCollection_List<Shape> lfIm = _ofImages.ValueAt(i - 1);
            NCollection_List<Shape>.Iterator itLFIm = new(lfIm);
            for (; itLFIm.More(); itLFIm.Next())
            {
                Shape fIm = itLFIm.Value();
                //
                Explorer expV = new(fIm, ShapeType.Vertex);
                for (; expV.More(); expV.Next())
                {
                    Shape v = expV.Current();
                    if (mvSp.Contains(v))
                    {
                        break;
                    }
                }
                //
                if (expV.More())
                {
                    break;
                }
            }
            //
            if (itLFIm.More())
            {
                lf.Append(f);
            }
        }
        //
        // get bounds from splits of faces of aLF
        Shape bounds = new();
        NCollection_List<Shape> laValid = new(), laBounds = new();
        GetBoundsToUpdate(lf, meb, laBounds, laValid, ref bounds);
        //
        // Intersect valid splits with bounds and update both
        BooleanBuilder gf = new();
        gf.AddArgument(bounds);
        gf.AddArgument(splits);
        gf.Perform(psOuter.Next(3));
        //
        // update splits
        UpdateImages(le, eImages, gf, _modifiedEdges);
        //
        // update new intersection edges
        UpdateNewIntersectionEdges(le, melf, eImages, eeTrim);
        //
        // update bounds
        UpdateImages(laValid, _oeImages, gf, _modifiedEdges);
        UpdateOrigins(laBounds, _oeOrigins, gf);
        UpdateOrigins(laBounds, _edgesOrigins!, gf);
        UpdateIntersectedEdges(laBounds, gf);
        //
        // update the EdgesToAvoid with the splits
        IndexedSet<Shape> newEdges = new(ShapeHasher.Instance);
        if (gf.Images().TryGetValue(splits, out NCollection_List<Shape>? pSplitsIm))
        {
            NCollection_List<Shape>.Iterator itSpIm = new(pSplitsIm);
            for (; itSpIm.More(); itSpIm.Next())
            {
                Exploration.MapShapes(itSpIm.Value(), ShapeType.Edge, newEdges);
            }
        }

        Compound insideEdges = new();
        new BrepBuilder().MakeCompound(ref insideEdges);
        for (int iE = 0; iE < _insideEdges.Count; ++iE)
        {
            new BrepBuilder().Add(insideEdges, _insideEdges[iE]);
        }

        //
        // Rebuild the map of edges to avoid, using the intersection results
        IndexedSet<Shape> meAvoid = new(ShapeHasher.Instance);
        Compound ceAvoid = new();
        new BrepBuilder().MakeCompound(ref ceAvoid);
        // GF's data structure
        DS pDS = gf.PDS()!;

        nbE = _edgesToAvoid.Count;
        for (i = 1; i <= nbE; ++i)
        {
            Shape e = _edgesToAvoid[i - 1];
            NCollection_List<Shape> leIm = ModifiedOf(gf, e);

            // Only untouched and fully coinciding edges should be kept in the avoid map
            bool bKeep = leIm.IsEmpty();
            if (leIm.Extent() == 1 && e.IsSame(leIm.First()))
            {
                NCollection_List<PaveBlock> lpb = pDS.PaveBlocks(pDS.Index(e));
                if (lpb.Extent() == 1)
                {
                    PaveBlock pb = lpb.First();
                    CommonBlock? cb = pDS.CommonBlock(pb);
                    if (cb is not null)
                    {
                        NCollection_List<PaveBlock> lpbcb = cb.PaveBlocks();
                        NCollection_List<PaveBlock>.Iterator itLPB = new(lpbcb);
                        for (; itLPB.More(); itLPB.Next())
                        {
                            if (pDS.PaveBlocks(itLPB.Value().OriginalEdge()).Extent() > 1)
                            {
                                break;
                            }
                        }
                        bKeep = !itLPB.More();
                    }
                }
            }

            if (bKeep)
            {
                // keep the original edge
                AddToContainer(e, meAvoid);
                continue;
            }

            NCollection_List<Shape>.Iterator itLEIm = new(leIm);
            for (; itLEIm.More(); itLEIm.Next())
            {
                Shape eIm = itLEIm.Value();
                if (!newEdges.Contains(eIm))
                {
                    AddToContainer(eIm, ceAvoid);
                }
            }
        }

        bool isCut = false;
        if (ceAvoid.ChildCount > 0)
        {
            // Perform intersection with the small subset of the edges to make
            // it possible to use the inside edges for building new splits.
            Bop bop = new();
            bop.AddArgument(ceAvoid);
            bop.AddTool(insideEdges);
            bop.SetOperation(Operation.Cut);
            bop.Perform();
            isCut = !bop.HasErrors;

            if (isCut)
            {
                for (Iterator itCE = new(ceAvoid); itCE.More(); itCE.Next())
                {
                    if (!bop.IsDeleted(itCE.Value()))
                    {
                        meAvoid.Add(itCE.Value());
                    }
                }
            }
        }

        if (!isCut)
        {
            Exploration.MapShapes(ceAvoid, ShapeType.Edge, meAvoid);
        }

        _edgesToAvoid = meAvoid;
    }

    //=================================================================================================

    //! Trims intersection edges
    private void TrimNewIntersectionEdges(NCollection_List<Shape> le,
                                          Dictionary<Shape, NCollection_List<Shape>> eeTrim,
                                          HashSet<Shape> mvBounds, HashSet<Shape> meCheckExt,
                                          Dictionary<Shape, NCollection_List<Shape>> eImages,
                                          HashSet<Shape> meb, HashSet<Shape> mvOld,
                                          HashSet<Shape> meNew,
                                          Dictionary<Shape, NCollection_List<Shape>> dmeOr,
                                          Dictionary<Shape, NCollection_List<Shape>> melf)
    {
        NCollection_List<Shape>.Iterator it = new(), it1 = new();
        it.Initialize(le);
        for (; it.More(); it.Next())
        {
            Shape e = it.Value();
            //
            bool bCheckExt = meCheckExt.Remove(e);
            //
            bool bOld = eeTrim.ContainsKey(e);
            if (bOld)
            {
                NCollection_List<Shape> aLET = eeTrim.GetValue(e);
                it1.Initialize(aLET);
                for (; it1.More(); it1.Next())
                {
                    Shape et = it1.Value();
                    meb.Add(et);
                    Explorer expV = new(et, ShapeType.Vertex);
                    for (; expV.More(); expV.Next())
                    {
                        Shape v = expV.Current();
                        mvOld.Add(v);
                    }
                }
            }
            //
            if (!eImages.ContainsKey(e))
            {
                continue;
            }
            //
            NCollection_List<Shape> leIm = eImages.GetValue(e);
            if (leIm.IsEmpty())
            {
                eImages.Remove(e);
                continue;
            }
            //
            Shape ceIm = new();
            HashSet<Shape> mevBounds = new(ShapeHasher.Instance);
            //
            if (leIm.Extent() > 1)
            {
                IndexedSet<Shape> mv = new(ShapeHasher.Instance);
                // fuse these parts
                BooleanBuilder gfe = new();
                NCollection_List<Shape>.Iterator itLEIm0 = new(leIm);
                for (; itLEIm0.More(); itLEIm0.Next())
                {
                    Shape eIm = itLEIm0.Value();
                    gfe.AddArgument(eIm);
                    Exploration.MapShapes(eIm, ShapeType.Vertex, mv);
                }
                //
                // add two bounding vertices of this edge to the operation
                Exploration.Vertices(ShapeCasts.Edge(e), out Vertex v1, out Vertex v2);
                //
                gfe.AddArgument(v1);
                gfe.AddArgument(v2);
                mv.Add(v1);
                mv.Add(v2);
                //
                gfe.Perform();
                if (!gfe.HasErrors)
                {
                    // get images of bounding vertices to remove splits containing them
                    // in case some of the bounding edges has been interfered
                    // during operation it is necessary to update their images as well
                    int iV, nbV = mv.Count;
                    for (iV = 1; iV <= nbV; ++iV)
                    {
                        Shape v = mv[iV - 1];
                        if (mvBounds.Contains(v) || v.IsSame(v1) || v.IsSame(v2))
                        {
                            NCollection_List<Shape> lvIm = ModifiedOf(gfe, v);
                            mevBounds.Add(lvIm.IsEmpty() ? v : lvIm.First());
                        }
                    }
                    //
                    ceIm = gfe.InternalShape();
                }
            }
            else
            {
                ceIm = leIm.First();
            }
            //
            leIm.Clear();
            //
            Explorer exp = new(ceIm, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Shape eIm = exp.Current();
                //
                // check the split not to contain bounding vertices
                Iterator itV = new(eIm);
                for (; itV.More(); itV.Next())
                {
                    Shape v = itV.Value();
                    if (mevBounds.Contains(v) || mvBounds.Contains(v))
                    {
                        break;
                    }
                }
                //
                if (!itV.More())
                {
                    leIm.Append(eIm);
                    //
                    // OCCT's Bound replaces any list already bound.
                    NCollection_List<Shape> lOr = new();
                    lOr.Append(e);
                    dmeOr[eIm] = lOr;
                }
            }
            //
            if (leIm.IsEmpty())
            {
                eImages.Remove(e);
            }
            else
            {
                NCollection_List<Shape> lfe = melf.GetValue(e);
                NCollection_List<Shape>.Iterator itLEIm = new(leIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    Shape eIm = itLEIm.Value();
                    if (!melf.TryGetValue(eIm, out NCollection_List<Shape>? pLFEIm))
                    {
                        pLFEIm = new NCollection_List<Shape>();
                        melf[eIm] = pLFEIm;
                    }
                    NCollection_List<Shape>.Iterator itLF = new(lfe);
                    for (; itLF.More(); itLF.Next())
                    {
                        AppendToList(pLFEIm, itLF.Value());
                    }
                    //
                    if (bCheckExt)
                    {
                        meCheckExt.Add(eIm);
                    }
                    else if (!bOld)
                    {
                        meNew.Add(eIm);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : IntersectEdges
    // purpose  : Intersecting the trimmed edges to avoid self-intersections
    //=======================================================================
    private void IntersectEdges(NCollection_List<Shape> la, NCollection_List<Shape> le,
                                HashSet<Shape> mvBounds, HashSet<Shape> vertsToAvoid,
                                HashSet<Shape> meNew, HashSet<Shape> meCheckExt,
                                Dictionary<Shape, NCollection_List<Shape>> eImages,
                                Dictionary<Shape, NCollection_List<Shape>> dmeOr,
                                Dictionary<Shape, NCollection_List<Shape>> melf,
                                ref Shape splits)
    {
        BooleanBuilder gfa = new();
        gfa.SetArguments(la);
        gfa.Perform();
        if (gfa.HasErrors)
        {
            // just copy input to the result
            Compound sp0 = new();
            BrepBuilder bb0 = new();
            bb0.MakeCompound(ref sp0);
            NCollection_List<Shape>.Iterator anIt = new(la);
            for (; anIt.More(); anIt.Next())
            {
                Shape e = anIt.Value();
                bb0.Add(sp0, e);
            }
            splits = sp0;
            return;
        }
        //
        UpdateImages(le, eImages, gfa, _modifiedEdges);
        //
        // compound of valid splits
        splits = gfa.InternalShape();
        //
        NCollection_List<Shape>.Iterator it = new(), it1 = new();

        // prepare list of edges to update
        NCollection_List<Shape> leInput = new();
        for (it.Initialize(la); it.More(); it.Next())
        {
            Explorer expE = new(it.Value(), ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                leInput.Append(expE.Current());
            }
        }

        // update new edges
        it.Initialize(leInput);
        for (; it.More(); it.Next())
        {
            Shape e = it.Value();
            if (!meNew.Contains(e))
            {
                continue;
            }

            NCollection_List<Shape> leIm = ModifiedOf(gfa, e);
            if (leIm.IsEmpty())
            {
                continue;
            }

            meNew.Remove(e);
            it1.Initialize(leIm);
            for (; it1.More(); it1.Next())
            {
                meNew.Add(it1.Value());
            }
        }
        //
        // update edges after intersection for extended checking
        it.Initialize(leInput);
        for (; it.More(); it.Next())
        {
            Shape e = it.Value();
            NCollection_List<Shape> leIm = ModifiedOf(gfa, e);
            if (leIm.IsEmpty())
            {
                continue;
            }
            //
            if (meCheckExt.Contains(e))
            {
                it1.Initialize(leIm);
                for (; it1.More(); it1.Next())
                {
                    meCheckExt.Add(it1.Value());
                }
                meCheckExt.Remove(e);
            }
            //
            NCollection_List<Shape> lfe = melf.GetValue(e);
            it1.Initialize(leIm);
            for (; it1.More(); it1.Next())
            {
                Shape eIm = it1.Value();
                if (!melf.TryGetValue(eIm, out NCollection_List<Shape>? pLFEIm))
                {
                    pLFEIm = new NCollection_List<Shape>();
                    melf[eIm] = pLFEIm;
                }
                NCollection_List<Shape>.Iterator itLF = new(lfe);
                for (; itLF.More(); itLF.Next())
                {
                    AppendToList(pLFEIm, itLF.Value());
                }
            }
        }
        //
        HashSet<Shape> meInv = new(ShapeHasher.Instance);
        GetInvalidEdges(vertsToAvoid, mvBounds, gfa, meInv);
        if (meInv.Count != 0)
        {
            // update shape
            Compound sp = new();
            BrepBuilder bb = new();
            bb.MakeCompound(ref sp);
            Explorer exp = new(splits, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Shape e = exp.Current();
                if (!meInv.Contains(e))
                {
                    bb.Add(sp, e);
                }
            }
            splits = sp;
        }
        //
        // update origins
        UpdateOrigins(leInput, dmeOr, gfa);
    }

    //=======================================================================
    // function : GetBounds
    // purpose  : Getting edges from the splits of offset faces
    //=======================================================================
    private void GetBounds(NCollection_List<Shape> lFaces, HashSet<Shape> meb,
                           ref Shape bounds)
    {
        BrepBuilder bb = new();
        // Make compound of edges contained in the splits of faces
        Compound aBounds = new();
        bb.MakeCompound(ref aBounds);
        // Fence map
        HashSet<Shape> mFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itLF = new(lFaces);
        for (; itLF.More(); itLF.Next())
        {
            if (!_ofImages.TryGetValue(itLF.Value(), out NCollection_List<Shape> pLFIm))
            {
                continue;
            }
            NCollection_List<Shape>.Iterator it = new(pLFIm);
            for (; it.More(); it.Next())
            {
                Shape fIm = it.Value();
                //
                Explorer expE = new(fIm, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape eIm = expE.Current();
                    if (!meb.Contains(eIm) && mFence.Add(eIm))
                    {
                        bb.Add(aBounds, eIm);
                    }
                }
            }
        }
        bounds = aBounds;
    }

    //=======================================================================
    // function : GetBoundsToUpdate
    // purpose  : Get bounding edges that should be updated
    //=======================================================================
    private void GetBoundsToUpdate(NCollection_List<Shape> lf, HashSet<Shape> meb,
                                   NCollection_List<Shape> laBounds, NCollection_List<Shape> laValid,
                                   ref Shape bounds)
    {
        // get all edges
        Compound aBounds = new();
        BrepBuilder bb = new();
        bb.MakeCompound(ref aBounds);
        //
        HashSet<Shape> maValid = new(ShapeHasher.Instance), mFence = new(ShapeHasher.Instance);
        //
        NCollection_List<Shape>.Iterator itLF = new(lf);
        for (; itLF.More(); itLF.Next())
        {
            Shape f = itLF.Value();
            //
            IndexedSet<Shape> mde = new(ShapeHasher.Instance);
            NCollection_List<Shape> lfDes = _asDes!.Descendant(f);
            NCollection_List<Shape>.Iterator itLFDes = new(lfDes);
            for (; itLFDes.More(); itLFDes.Next())
            {
                Shape ed = itLFDes.Value();
                if (!_oeImages.TryGetValue(ed, out NCollection_List<Shape>? pLEDIm))
                {
                    mde.Add(ed);
                    continue;
                }
                //
                NCollection_List<Shape>.Iterator itLEDIm = new(pLEDIm);
                for (; itLEDIm.More(); itLEDIm.Next())
                {
                    Shape edIm = itLEDIm.Value();
                    mde.Add(edIm);
                }
            }
            //
            int j, nbE = mde.Count;
            for (j = 1; j <= nbE; ++j)
            {
                Edge eIm = ShapeCasts.Edge(mde[j - 1]);
                //
                if (!meb.Contains(eIm) && mFence.Add(eIm))
                {
                    bb.Add(aBounds, eIm);
                    laBounds.Append(eIm);
                }
                //
                if (_oeOrigins.TryGetValue(eIm, out NCollection_List<Shape>? pLO))
                {
                    NCollection_List<Shape>.Iterator itLO = new(pLO);
                    for (; itLO.More(); itLO.Next())
                    {
                        Shape eo = itLO.Value();
                        //
                        if (maValid.Add(eo))
                        {
                            laValid.Append(eo);
                        }
                    }
                }
                else
                {
                    if (maValid.Add(eIm))
                    {
                        laValid.Append(eIm);
                    }
                }
            }
        }
        bounds = aBounds;
    }

    //=======================================================================
    // function : GetInvalidEdgesByBounds
    // purpose  : Filter new splits by intersection with bounds
    //=======================================================================
    private void GetInvalidEdgesByBounds(in Shape splits, in Shape bounds,
                                         HashSet<Shape> mvOld, HashSet<Shape> meNew,
                                         Dictionary<Shape, NCollection_List<Shape>> dmeOr,
                                         Dictionary<Shape, NCollection_List<Shape>> melf,
                                         Dictionary<Shape, NCollection_List<Shape>> eImages,
                                         HashSet<Shape> meCheckExt, HashSet<Shape> meInvOnArt,
                                         HashSet<Shape> vertsToAvoid, HashSet<Shape> meInv)
    {
        // map splits to check the vertices of edges
        IndexedMap<Shape, NCollection_List<Shape>> dmve = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(splits, ShapeType.Vertex, ShapeType.Edge, dmve);
        //
        Section sec = new();
        sec.AddArgument(splits);
        sec.AddArgument(bounds);
        //
        sec.Perform();
        //
        // invalid vertices
        IndexedSet<Shape> mvInv = new(ShapeHasher.Instance);
        // vertices to check additionally by classification relatively to solid
        HashSet<Shape> mvCheckAdd = new(ShapeHasher.Instance);
        // collect parts for removal
        DS pDS = sec.PDS()!;
        //
        // check edge/edge intersections
        List<InterfEE> eEs = pDS.InterfEE();
        int i, nb = eEs.Count;
        for (i = 0; i < nb; ++i)
        {
            InterfEE ee = eEs[i];
            //
            Shape e1 = pDS.Shape(ee.Index1());
            Shape e2 = pDS.Shape(ee.Index2());
            //
            if (!ee.HasIndexNew())
            {
                if (meCheckExt.Contains(e1) && (ee.CommonPart().Type() == ShapeType.Edge))
                {
                    meInv.Add(e1);
                }
                continue;
            }
            //
            if (_invalidEdges.Contains(e2))
            {
                meInv.Add(e1);
            }
            //
            if (meInvOnArt.Contains(e2))
            {
                // avoid checking of the vertices of the split edge intersected by
                // the invalid edge from artificial face
                Exploration.Vertices(ShapeCasts.Edge(e2), out Vertex v1, out Vertex v2);
                if (dmve.ContainsKey(v1) && dmve.ContainsKey(v2))
                {
                    continue;
                }
            }
            //
            // add vertices of all images of the edge from splits for checking
            NCollection_List<Shape> leOr = dmeOr.GetValue(e1);
            NCollection_List<Shape>.Iterator itLEOr = new(leOr);
            for (; itLEOr.More(); itLEOr.Next())
            {
                Shape eOr = itLEOr.Value();
                //
                if (!eImages.TryGetValue(eOr, out NCollection_List<Shape>? pLEIm))
                {
                    continue;
                }
                NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    Shape eIm = itLEIm.Value();
                    //
                    Iterator itV = new(eIm);
                    for (; itV.More(); itV.Next())
                    {
                        Shape aV = itV.Value();
                        if (!mvOld.Contains(aV))
                        {
                            mvInv.Add(aV);
                            mvCheckAdd.Add(aV);
                        }
                    }
                }
            }
        }
        //
        // to avoid unnecessary filling of parts due to extra trim of the edges
        // process Edge/Edge interferences of type EDGE, i.e. common blocks and check
        // not the bounding vertices of the edges, but check the edge itself
        // to be lying on some face
        //
        // all common blocks are contained in the result of SECTION operation
        // between sets of edges
        Shape secR = sec.InternalShape();
        //
        IndexedSet<Shape> msSec = new(ShapeHasher.Instance);
        Exploration.MapShapes(secR, msSec);
        //
        Dictionary<Shape, NCollection_List<Shape>> im = sec.Images();
        for (Explorer exp = new(splits, ShapeType.Edge); exp.More(); exp.Next())
        {
            Shape e = exp.Current();
            if (sec.IsDeleted(e))
            {
                // no common blocks for this edge
                continue;
            }
            //
            if (!im.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
            {
                // no splits, i.e. completely coincides with some edge from boundary
                continue;
            }
            //
            NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
            for (; itLEIm.More(); itLEIm.Next())
            {
                Shape eIm = itLEIm.Value();
                if (!msSec.Contains(eIm))
                {
                    // the edge included in section only partially.
                    // the part not included in section may be excessive
                    //
                    // check vertices of this edge - if one of them is new
                    // the edge might be removed
                    Exploration.Vertices(ShapeCasts.Edge(eIm), out Vertex v1, out Vertex v2);
                    if (!mvOld.Contains(v1) || !mvOld.Contains(v2))
                    {
                        // add this edge for checking by making new vertex in the middle of the edge
                        Vertex aV = new();
                        Curve3D c = Tool.Curve(ShapeCasts.Edge(eIm), out double f, out double l)!;
                        new BrepBuilder().MakeVertex(ref aV, c.Value((f + l) * 0.5), global::Kerf.Engine.Foundation.Precision.Confusion());
                        // and adding this vertex for checking
                        dmve.ValueAt(dmve.Add(aV, new NCollection_List<Shape>())).Append(e);
                        mvInv.Add(aV);
                        break;
                    }
                }
            }
        }
        //
        // Add for check also the edges created from common between splits
        // of offset faces edges not connected to any invalidity.
        // These edges may also accidentally fill some part.
        // Kerf: OCCT iterates the map by its edges' addresses; the port in insertion order (decision 1032).
        foreach (Shape e in meCheckExt)
        {
            //
            // make new vertex in the middle of the edge
            Vertex aV = new();
            Curve3D c = Tool.Curve(ShapeCasts.Edge(e), out double f, out double l)!;
            new BrepBuilder().MakeVertex(ref aV, c.Value((f + l) * 0.5), global::Kerf.Engine.Foundation.Precision.Confusion());
            // add this vertex for checking
            dmve.ValueAt(dmve.Add(aV, new NCollection_List<Shape>())).Append(e);
            mvInv.Add(aV);
        }
        //
        // add for check also the vertices connected only to new or old edges
        nb = dmve.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape aV = dmve.KeyAt(i - 1);
            if (mvOld.Contains(aV))
            {
                continue;
            }
            //
            bool bNew = false, bOld = false;
            NCollection_List<Shape> lEx = dmve.ValueAt(i - 1);
            NCollection_List<Shape>.Iterator it = new(lEx);
            for (; it.More(); it.Next())
            {
                Shape e = it.Value();
                if (meCheckExt.Contains(e))
                {
                    continue;
                }
                //
                if (meNew.Contains(e))
                {
                    bNew = true;
                }
                else
                {
                    bOld = true;
                }
                //
                if (bNew && bOld)
                {
                    break;
                }
            }
            //
            if (!bNew || !bOld)
            {
                mvInv.Add(aV);
                mvCheckAdd.Remove(aV);
            }
        }
        //
        // perform the checking of the vertices
        int iv, nbIV = mvInv.Count;
        for (iv = 1; iv <= nbIV; ++iv)
        {
            Vertex aV = ShapeCasts.Vertex(mvInv[iv - 1]);
            if (mvOld.Contains(aV))
            {
                continue;
            }
            //
            if (!dmve.TryGetValue(aV, out NCollection_List<Shape> pLEInv))
            {
                continue;
            }
            // find faces by the edges to check the vertex
            IndexedSet<Shape> mf = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator itLE = new(pLEInv);
            for (; itLE.More(); itLE.Next())
            {
                Shape e = itLE.Value();
                NCollection_List<Shape> lf = melf.GetValue(e);
                NCollection_List<Shape>.Iterator itLF0 = new(lf);
                for (; itLF0.More(); itLF0.Next())
                {
                    mf.Add(itLF0.Value());
                }
            }
            //
            // check the vertex to belong to some split of the faces
            bool bInvalid = true;
            //
            int nbF = mf.Count;
            for (i = 1; i <= nbF && bInvalid; ++i)
            {
                Face aF = ShapeCasts.Face(mf[i - 1]);
                NCollection_List<Shape> lfIm = _ofImages.GetValue(aF);
                //
                NCollection_List<Shape>.Iterator itLF = new(lfIm);
                for (; itLF.More() && bInvalid; itLF.Next())
                {
                    Face fIm = ShapeCasts.Face(itLF.Value());
                    Explorer exp = new(fIm, ShapeType.Vertex);
                    for (; exp.More() && bInvalid; exp.Next())
                    {
                        Shape vf = exp.Current();
                        bInvalid = !vf.IsSame(aV);
                    }
                }
                //
                if (bInvalid)
                {
                    double u = 0.0, v = 0.0, tol = 0.0;
                    int iStatus = _context.ComputeVF(aV, aF, ref u, ref v, ref tol);
                    if (iStatus == 0)
                    {
                        // classify the point relatively faces
                        Point2D p2d = new(u, v);
                        itLF.Initialize(lfIm);
                        for (; itLF.More() && bInvalid; itLF.Next())
                        {
                            Face fIm = ShapeCasts.Face(itLF.Value());
                            bInvalid = !_context.IsPointInOnFace(fIm, p2d);
                        }
                    }
                }
            }
            //
            if (bInvalid && mvCheckAdd.Contains(aV))
            {
                // the vertex is invalid for all faces
                // check the same vertex for the solids
                Point3D p = Tool.Pnt(aV);
                double tolV = Tool.Tolerance(aV);
                //
                Explorer expS = new(_solids, ShapeType.Solid);
                for (; expS.More() && bInvalid; expS.Next())
                {
                    Solid sol = ShapeCasts.Solid(expS.Current());
                    SolidClassifier sc = _context.SolidClassifier(sol);
                    sc.Perform(p, tolV);
                    bInvalid = (sc.State() == Out);
                }
            }
            //
            if (bInvalid)
            {
                vertsToAvoid.Add(aV);
                itLE.Initialize(pLEInv);
                for (; itLE.More(); itLE.Next())
                {
                    meInv.Add(itLE.Value());
                }
            }
        }
    }

    //=======================================================================
    // function : FilterSplits
    // purpose  : Filter the images of edges from the invalid edges
    //=======================================================================
    private void FilterSplits(NCollection_List<Shape> le, HashSet<Shape> meFilter, bool isInv,
                              Dictionary<Shape, NCollection_List<Shape>> eImages,
                              ref Shape splits)
    {
        Compound aSplits = new();
        new BrepBuilder().MakeCompound(ref aSplits);
        HashSet<Shape> mFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itLE = new(le);
        for (; itLE.More(); itLE.Next())
        {
            Shape e = itLE.Value();
            if (!eImages.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
            {
                continue;
            }

            NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
            for (; itLEIm.More();)
            {
                Shape eIm = itLEIm.Value();
                if (meFilter.Contains(eIm) == isInv)
                {
                    pLEIm.Remove(itLEIm);
                    continue;
                }

                if (mFence.Add(eIm))
                {
                    new BrepBuilder().Add(aSplits, eIm);
                }
                itLEIm.Next();
            }

            if (pLEIm.IsEmpty())
            {
                eImages.Remove(e);
            }
        }
        splits = aSplits;
    }

    //=======================================================================
    // function : UpdateNewIntersectionEdges
    // purpose  : Updating the maps of images and origins of the offset edges
    //=======================================================================
    private void UpdateNewIntersectionEdges(NCollection_List<Shape> le,
                                            Dictionary<Shape, NCollection_List<Shape>> melf,
                                            Dictionary<Shape, NCollection_List<Shape>> eImages,
                                            Dictionary<Shape, NCollection_List<Shape>> eeTrim)
    {
        NCollection_List<Shape> leImEmpty = new();
        NCollection_List<Shape>.Iterator it = new(), it1 = new();
        // update global maps of images and origins with new splits
        it.Initialize(le);
        for (; it.More(); it.Next())
        {
            Shape e = it.Value();
            //
            if (!eImages.ContainsKey(e))
            {
                if (!eeTrim.TryGetValue(e, out NCollection_List<Shape>? pLET))
                {
                    continue;
                }
                //
                NCollection_List<Shape>.Iterator itLET = new(pLET);
                for (; itLET.More();)
                {
                    Shape et = itLET.Value();
                    if (!_invalidEdges.Contains(et) && !_invertedEdges.Contains(et))
                    {
                        pLET.Remove(itLET);
                    }
                    else
                    {
                        itLET.Next();
                    }
                }
                //
                if (pLET.IsEmpty())
                {
                    continue;
                }
            }
            // new images
            NCollection_List<Shape> leNew = eImages.ContainsKey(e) ? eImages.GetValue(e) : leImEmpty;
            //
            // save connection to untrimmed edge for the next steps
            it1.Initialize(leNew);
            for (; it1.More(); it1.Next())
            {
                Shape et = it1.Value();
                _eTrimEInf![et] = e;
                _modifiedEdges.Add(et);
            }
            //
            // check if it is existing edge
            if (!eeTrim.ContainsKey(e))
            {
                NCollection_List<Shape> lf = melf.GetValue(e);
                // the edge is new
                // add this edge to AsDes
                it1.Initialize(lf);
                for (; it1.More(); it1.Next())
                {
                    Shape f = it1.Value();
                    _asDes!.Add(f, e);
                }
                //
                // add aE to the images
                _oeImages[e] = new NCollection_List<Shape>(leNew);
                _modifiedEdges.Add(e);
                //
                // add to origins
                NCollection_List<Shape>.Iterator itNew = new(leNew);
                for (; itNew.More(); itNew.Next())
                {
                    Shape eNew = itNew.Value();
                    if (_oeOrigins.ContainsKey(eNew))
                    {
                        NCollection_List<Shape> eOrigins = _oeOrigins.GetValue(eNew);
                        AppendToList(eOrigins, e);
                    }
                    else
                    {
                        NCollection_List<Shape> eOrigins = new();
                        eOrigins.Append(e);
                        _oeOrigins[eNew] = eOrigins;
                    }
                }
                //
                // update connection to initial origins
                if (_edgesOrigins!.ContainsKey(e))
                {
                    NCollection_List<Shape> leOrInit = _edgesOrigins.GetValue(e);
                    it1.Initialize(leNew);
                    for (; it1.More(); it1.Next())
                    {
                        Shape eNew = it1.Value();
                        if (_edgesOrigins.ContainsKey(eNew))
                        {
                            NCollection_List<Shape> leNewOr = _edgesOrigins.GetValue(eNew);
                            NCollection_List<Shape>.Iterator itOrInit = new(leOrInit);
                            for (; itOrInit.More(); itOrInit.Next())
                            {
                                Shape eOr = itOrInit.Value();
                                AppendToList(leNewOr, eOr);
                            }
                        }
                        else
                        {
                            _edgesOrigins[eNew] = new NCollection_List<Shape>(leOrInit);
                        }
                    }
                }
                //
                continue;
            }
            //
            // old images
            NCollection_List<Shape> leOld = eeTrim.GetValue(e);
            //
            // list of initial origins
            NCollection_List<Shape> initOrigins = new();
            //
            // it is necessary to replace the old edges with new ones
            it1.Initialize(leOld);
            for (; it1.More(); it1.Next())
            {
                Shape eOld = it1.Value();
                //
                if (_oeOrigins.ContainsKey(eOld))
                {
                    // get its origins
                    NCollection_List<Shape> eOrigins = _oeOrigins.GetValue(eOld);
                    //
                    NCollection_List<Shape>.Iterator itOr = new(eOrigins);
                    for (; itOr.More(); itOr.Next())
                    {
                        Shape eOr = itOr.Value();
                        //
                        _modifiedEdges.Add(eOr);
                        //
                        NCollection_List<Shape> aEImages = _oeImages.GetValue(eOr);
                        //
                        // remove old edge from images
                        NCollection_List<Shape>.Iterator itIm = new(aEImages);
                        for (; itIm.More();)
                        {
                            Shape eIm = itIm.Value();
                            if (eIm.IsSame(eOld))
                            {
                                aEImages.Remove(itIm);
                            }
                            else
                            {
                                itIm.Next();
                            }
                        }
                        //
                        // add new images
                        NCollection_List<Shape>.Iterator itNew = new(leNew);
                        for (; itNew.More(); itNew.Next())
                        {
                            Shape eNew = itNew.Value();
                            AppendToList(aEImages, eNew);
                            if (_oeOrigins.ContainsKey(eNew))
                            {
                                NCollection_List<Shape> eNewOrigins = _oeOrigins.GetValue(eNew);
                                AppendToList(eNewOrigins, eOr);
                            }
                            else
                            {
                                NCollection_List<Shape> eNewOrigins = new();
                                eNewOrigins.Append(eOr);
                                _oeOrigins[eNew] = eNewOrigins;
                            }
                        }
                    }
                }
                else
                {
                    // add to images
                    _oeImages[eOld] = new NCollection_List<Shape>(leNew);
                    //
                    _modifiedEdges.Add(eOld);
                    //
                    // add to origins
                    NCollection_List<Shape>.Iterator itNew = new(leNew);
                    for (; itNew.More(); itNew.Next())
                    {
                        Shape eNew = itNew.Value();
                        if (_oeOrigins.ContainsKey(eNew))
                        {
                            NCollection_List<Shape> eOrigins = _oeOrigins.GetValue(eNew);
                            AppendToList(eOrigins, eOld);
                        }
                        else
                        {
                            NCollection_List<Shape> eOrigins = new();
                            eOrigins.Append(eOld);
                            _oeOrigins[eNew] = eOrigins;
                        }
                    }
                }
                //
                // update connection to initial shape
                if (_edgesOrigins!.ContainsKey(eOld))
                {
                    NCollection_List<Shape> leOrInit = _edgesOrigins.GetValue(eOld);
                    NCollection_List<Shape>.Iterator itEOrInit = new(leOrInit);
                    for (; itEOrInit.More(); itEOrInit.Next())
                    {
                        Shape eOrInit = itEOrInit.Value();
                        AppendToList(initOrigins, eOrInit);
                    }
                }
            }
            //
            if (initOrigins.Extent() != 0)
            {
                NCollection_List<Shape>.Iterator itNew = new(leNew);
                for (; itNew.More(); itNew.Next())
                {
                    Shape eNew = itNew.Value();
                    if (_edgesOrigins!.ContainsKey(eNew))
                    {
                        NCollection_List<Shape> leNewOr = _edgesOrigins.GetValue(eNew);
                        NCollection_List<Shape>.Iterator itOrInit = new(initOrigins);
                        for (; itOrInit.More(); itOrInit.Next())
                        {
                            Shape eOr = itOrInit.Value();
                            AppendToList(leNewOr, eOr);
                        }
                    }
                    else
                    {
                        _edgesOrigins[eNew] = new NCollection_List<Shape>(initOrigins);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FillGaps
    // purpose  : Fill possible gaps (holes) in the splits of the offset faces
    //=======================================================================
    private void FillGaps(ProgressRange range)
    {
        int nbF = _ofImages.Count;
        if (nbF == 0)
        {
            return;
        }

        using ProgressScope ps = new(range, "Filling gaps", 2 * nbF);

        // Check the splits of offset faces on the free edges and fill the gaps (holes)
        // in created splits, otherwise the closed volume will not be possible to create.

        // Map the splits of faces to find free edges
        IndexedMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance);
        for (int i = 1; i <= nbF; ++i, ps.Next())
        {
            if (!ps.More())
            {
                return;
            }

            NCollection_List<Shape>.Iterator itLF = new(_ofImages.ValueAt(i - 1));
            for (; itLF.More(); itLF.Next())
            {
                Exploration.MapShapesAndAncestors(itLF.Value(), ShapeType.Edge, ShapeType.Face, efMap);
            }
        }

        // Analyze images of each offset face on the presence of free edges
        // and try to fill the holes
        for (int i = 1; i <= nbF; ++i, ps.Next())
        {
            if (!ps.More())
            {
                return;
            }

            NCollection_List<Shape> lfImages = _ofImages.ValueAt(i - 1);
            if (lfImages.IsEmpty())
            {
                continue;
            }

            // Collect all edges from the splits
            Compound edges = new();
            new BrepBuilder().MakeCompound(ref edges);

            // Collect all free edges into a map with reverted orientation
            // (OCCT's default hasher: the orientation counts).
            HashSet<Shape> freeEdgesMap = new(EqualityComparer<Shape>.Default);
            NCollection_List<Shape>.Iterator itLF = new(lfImages);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                Explorer expE = new(fIm, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (e.Orientation != Forward && e.Orientation != Reversed)
                    {
                        // Skip internals
                        continue;
                    }

                    NCollection_List<Shape> lf = efMap.GetValue(e);
                    if (lf.Extent() == 1)
                    {
                        freeEdgesMap.Add(e.Reversed());
                    }

                    new BrepBuilder().Add(edges, e);
                }
            }

            if (freeEdgesMap.Count == 0)
            {
                // No free edges
                continue;
            }

            // Free edges are found - fill the gaps by creating new splits
            // of the face using these free edges
            Shape f = _ofImages.KeyAt(i - 1);

            // Build new splits using all kept edges and among new splits
            // find those containing free edges
            NCollection_List<Shape> lfNew = new();
            Dictionary<Shape, Shape> dummy = new(ShapeHasher.Instance);

            BuildSplitsOfFace(ShapeCasts.Face(f), edges, dummy, lfNew);

            // Find faces filling holes
            itLF.Initialize(lfNew);
            for (; itLF.More(); itLF.Next())
            {
                Shape fNew = itLF.Value();
                Explorer expE = new(fNew, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (freeEdgesMap.Contains(e))
                    {
                        // Add face to splits
                        lfImages.Append(fNew);
                        break;
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FillHistory
    // purpose  : Saving obtained results in history tools
    //=======================================================================
    private void FillHistory()
    {
        int nbF = _ofImages.Count;
        if (nbF == 0)
        {
            return;
        }

        // OFFSET_DEBUG: the compound of the preliminary result is left out.

        // Map of kept edges
        IndexedSet<Shape> edgesMap = new(ShapeHasher.Instance);

        // Fill history for faces
        for (int i = 0; i < nbF; ++i)
        {
            NCollection_List<Shape> lfImages = _ofImages.ValueAt(i);
            if (lfImages.IsEmpty())
            {
                continue;
            }

            // Add the splits to history map
            Shape f = _ofImages.KeyAt(i);
            if (_image.HasImage(f))
            {
                _image.Add(f, lfImages);
            }
            else
            {
                _image.Bind(f, lfImages);
            }

            // Collect edges from splits
            NCollection_List<Shape>.Iterator itLF = new(lfImages);
            for (; itLF.More(); itLF.Next())
            {
                Shape fIm = itLF.Value();
                Exploration.MapShapes(fIm, ShapeType.Edge, edgesMap);
            }
        }

        // Fill history for edges (iteration by the map is safe because the
        // order is not important here)
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itEIm in _oeImages)
        {
            Shape e = itEIm.Key;
            NCollection_List<Shape> leIm = itEIm.Value;

            bool bHasImage = _image.HasImage(e);
            NCollection_List<Shape>.Iterator itLE = new(leIm);
            for (; itLE.More(); itLE.Next())
            {
                Shape eIm = itLE.Value();
                if (edgesMap.Contains(eIm))
                {
                    if (bHasImage)
                    {
                        _image.Add(e, eIm);
                    }
                    else
                    {
                        _image.Bind(e, eIm);
                        bHasImage = true;
                    }
                }
            }
        }
    }
}

internal sealed partial class MakeOffset
{
    //=======================================================================
    // function : BuildSplitsOfTrimmedFaces
    // purpose  : Building splits of already trimmed faces
    //=======================================================================

    /// <summary>
    /// Builds the splits of the already trimmed offset faces, for the Complete mode with the Intersection join.
    /// </summary>
    private void BuildSplitsOfTrimmedFaces(NCollection_List<Shape> lf, AsDes asDes,
                                           ShapeImage image, ProgressRange range)
    {
        BuildOffsetFaces bfTool = new(image);
        bfTool.SetFaces(lf);
        bfTool.SetAsDesInfo(asDes);
        bfTool.BuildSplitsOfTrimmedFaces(range);
    }

    //=======================================================================
    // function : BuildSplitsOfExtendedFaces
    // purpose  : Building splits of not-trimmed offset faces.
    //           For the cases in which invalidity will be found,
    //           these invalidities will be rebuilt.
    //=======================================================================

    /// <summary>
    /// Builds the splits of the extended (not trimmed) offset faces, for the Complete mode with the Intersection join
    /// (an alternative to BRepOffset_MakeLoops::Build, limited to planar cases); invalidities found are rebuilt.
    /// </summary>
    private void BuildSplitsOfExtendedFaces(NCollection_List<Shape> lf, Analyse analyse,
                                            AsDes asDes,
                                            Dictionary<Shape, NCollection_List<Shape>> edgesOrigins,
                                            Dictionary<Shape, Shape> facesOrigins,
                                            Dictionary<Shape, Shape> eTrimEInf,
                                            ShapeImage image, ProgressRange range)
    {
        BuildOffsetFaces bfTool = new(image);
        bfTool.SetFaces(lf);
        bfTool.SetAsDesInfo(asDes);
        bfTool.SetAnalysis(analyse);
        bfTool.SetEdgesOrigins(edgesOrigins);
        bfTool.SetFacesOrigins(facesOrigins);
        bfTool.SetInfEdges(eTrimEInf);
        bfTool.BuildSplitsOfExtendedFaces(range);
    }
}
