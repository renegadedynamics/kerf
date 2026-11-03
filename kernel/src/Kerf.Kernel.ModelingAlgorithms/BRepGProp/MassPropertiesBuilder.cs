// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Engine.Measurement.MeshProps.BRepGProp_MeshObjType;
using static Kerf.Geometry.TransformForm;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The global properties of a shape: of its edges (length, <see cref="LinearProperties(in Shape, Properties, bool, bool)"/>),
/// of its faces (area, <see cref="SurfaceProperties(in Shape, Properties, bool, bool)"/>) or of the volume
/// they bound (<see cref="VolumeProperties(in Shape, Properties, bool, bool, bool)"/>): mass, centre of mass
/// and inertia, at density 1.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's forms fill the caller's <see cref="Properties"/>, which is reset first, its reference point at the
/// shape's location. Each also has a functional overload that returns a new system.
/// </para>
/// <para>
/// With <c>SkipShared</c>, an edge or face met again (the same TShape at the same location) is counted once. With
/// <c>UseTriangulation</c>, faces with a triangulation and edges with a polygon are measured on the mesh; faces without
/// a surface and edges without a curve are always measured on their mesh. The forms with <c>Eps</c> integrate
/// adaptively and return the largest relative error reached by a face.
/// </para>
/// </remarks>
public static class MassPropertiesBuilder
{
    /// <summary>Transforms GProp_GProps through its protected members, for the solid-level cache of volume properties.</summary>
    /// <remarks>
    /// Properties are computed once per unique solid and transformed for the duplicate instances at other locations.
    /// </remarks>
    private sealed class GPropTransformer : Properties
    {
        public GPropTransformer(Properties source)
            : base(source)
        {
        }

        //! Apply a rigid transform to the stored properties.
        //! Transforms reference point (loc) as absolute point, gravity center offset (g)
        //! as relative vector (rotation only, no translation), and rotates inertia tensor.
        public void ApplyTransform(in Transform3D trsf)
        {
            _loc.Transform(trsf);
            TransformForm form = trsf.Form();
            if (form != Identity && form != Translation)
            {
                // g is a relative offset from loc to center of mass - rotate only.
                Matrix3x3 r = trsf.VectorialPart();
                Xyz gxyz = _g.XYZ();
                gxyz.Multiply(r);
                _g.SetXYZ(gxyz);
                Matrix3x3 rt = r.Transposed();
                _inertia = r.Multiplied(_inertia).Multiplied(rt);
            }
        }

        //! Negate volume contribution (for reversed solid orientation).
        public void Negate()
        {
            _dim = -_dim;
            _inertia.Multiply(-1.0);
        }
    }

    private static Point3D RoughBaryCenter(in Shape s)
    {
        int i;
        var ex = new Explorer();
        Xyz xyz = new(0, 0, 0);
        for (ex.Init(s, ShapeType.Vertex), i = 0; ex.More(); ex.Next(), i++)
        {
            xyz += Tool.Pnt(ShapeCasts.Vertex(ex.Current())).XYZ();
        }

        if (i > 0)
        {
            xyz /= i;
        }
        else
        {
            // Try using triangulation
            ex.Init(s, ShapeType.Face);
            for (; ex.More(); ex.Next())
            {
                Shape f = ex.Current();
                Triangulation? tri = Tool.Triangulation(ShapeCasts.Face(f), out Placement locDummy);
                if (tri is not null && tri.NodeCount > 0)
                {
                    xyz = tri.Node(1).XYZ();
                    if (!locDummy.IsIdentity)
                    {
                        locDummy.Transformation().Transforms(ref xyz);
                    }

                    break;
                }
            }
        }

        return new Point3D(xyz);
    }

    /// <summary>
    /// The linear properties of the edges of <paramref name="s"/> into <paramref name="lProps"/>: total length, centre
    /// of mass and inertia.
    /// </summary>
    /// <param name="s">The shape.</param>
    /// <param name="lProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="skipShared">Count an edge met again (IsSame) once.</param>
    /// <param name="useTriangulation">Measure edges with a polygon on the polygon.</param>
    public static void LinearProperties(in Shape s,
                                        Properties lProps,
                                        bool skipShared = false,
                                        bool useTriangulation = false)
    {
        // find the origin
        Point3D p = new(0, 0, 0);
        p.Transform(s.Location().Transformation());
        lProps.Assign(new Properties(p));

        var bac = new EdgeCurveAdaptor();
        var eMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var ex = new Explorer();
        for (ex.Init(s, ShapeType.Edge); ex.More(); ex.Next())
        {
            Edge e = ShapeCasts.Edge(ex.Current());
            if (skipShared && !eMap.Add(e))
            {
                continue;
            }

            NCollection_HArray1<Point3D>? theNodes = null;
            bool isGeom = Tool.IsGeometric(e);
            if (useTriangulation || !isGeom)
            {
                theNodes = MeshCinert.PreparePolygon(e);
            }

            if (theNodes is not null)
            {
                var mg = new MeshCinert();
                mg.SetLocation(p);
                mg.Perform(theNodes.Array1());
                lProps.Add(mg);
            }
            else
            {
                if (isGeom)
                {
                    bac.Initialize(e);
                    var cg = new Cinert(bac, p);
                    lProps.Add(cg);
                }
            }
        }
    }

    /// <summary>The linear properties of the edges of <paramref name="s"/>, in a new system (see the other form).</summary>
    public static Properties LinearProperties(in Shape s, bool skipShared = false, bool useTriangulation = false)
    {
        var props = new Properties();
        LinearProperties(s, props, skipShared, useTriangulation);
        return props;
    }

    private static double SurfacePropertiesCore(in Shape s,
                                            Properties props,
                                            double eps,
                                            bool skipShared,
                                            bool useTriangulation)
    {
        int i;
        double errorMax = 0.0, error;
        var ex = new Explorer();
        var p = RoughBaryCenter(s);
        var g = new global::Kerf.Engine.Measurement.Sinert();
        g.SetLocation(p);
        var mg = new MeshProps(global::Kerf.Engine.Measurement.MeshProps.BRepGProp_MeshObjType.Sinert);
        mg.SetLocation(p);

        var bf = new FaceIntegrand();
        var bd = new Domain();
        var fMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Placement locDummy;

        for (ex.Init(s, ShapeType.Face), i = 1; ex.More(); ex.Next(), i++)
        {
            Face f = ShapeCasts.Face(ex.Current());
            if (skipShared && !fMap.Add(f))
            {
                continue;
            }

            bool noSurf = false, noTri = false;
            {
                Surface? surf = Tool.Surface(f, out locDummy);
                if (surf is null)
                {
                    noSurf = true;
                }

                Triangulation? tri = Tool.Triangulation(f, out locDummy);
                if (tri is null || tri.NodeCount == 0 || tri.TriangleCount == 0)
                {
                    noTri = true;
                }

                if (noTri && noSurf)
                {
                    continue;
                }
            }

            if ((useTriangulation && !noTri) || (noSurf && !noTri))
            {
                Orientation ori = f.Orientation();
                Triangulation? tri = Tool.Triangulation(f, out locDummy);
                mg.Perform(tri, locDummy, ori);
                props.Add(mg);
            }
            else
            {
                bf.Load(f);
                bool isNatRestr = (f.ChildCount == 0);
                if (!isNatRestr)
                {
                    bd.Init(f);
                }

                if (eps < 1.0)
                {
                    g.Perform(bf, bd, eps);
                    error = g.GetEpsilon();
                    if (errorMax < error)
                    {
                        errorMax = error;
                    }
                }
                else
                {
                    if (isNatRestr)
                    {
                        g.Perform(bf);
                    }
                    else
                    {
                        g.Perform(bf, bd);
                    }
                }

                props.Add(g);
            }
        }

        return errorMax;
    }

    /// <summary>
    /// The surface properties of the faces of <paramref name="s"/> into <paramref name="sProps"/>: total area, centre
    /// of mass and inertia, by Gauss integration with numbers of points chosen by the types of the surfaces and curves.
    /// </summary>
    /// <param name="s">The shape.</param>
    /// <param name="sProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="skipShared">Count a face met again (IsSame) once.</param>
    /// <param name="useTriangulation">Measure faces with a triangulation on the triangulation.</param>
    public static void SurfaceProperties(in Shape s,
                                         Properties sProps,
                                         bool skipShared = false,
                                         bool useTriangulation = false)
    {
        // find the origin
        Point3D p = new(0, 0, 0);
        p.Transform(s.Location().Transformation());
        sProps.Assign(new Properties(p));
        SurfacePropertiesCore(s, sProps, 1.0, skipShared, useTriangulation);
    }

    /// <summary>The surface properties of the faces of <paramref name="s"/>, in a new system (see the other form).</summary>
    public static Properties SurfaceProperties(in Shape s, bool skipShared = false, bool useTriangulation = false)
    {
        var props = new Properties();
        SurfaceProperties(s, props, skipShared, useTriangulation);
        return props;
    }

    /// <summary>
    /// The surface properties of the faces of <paramref name="s"/> into <paramref name="sProps"/>, by adaptive Gauss
    /// integration to the relative error <paramref name="eps"/> of each face's area.
    /// </summary>
    /// <returns>The largest relative error reached by a face.</returns>
    public static double SurfaceProperties(in Shape s, Properties sProps, double eps, bool skipShared = false)
    {
        // find the origin
        Point3D p = new(0, 0, 0);
        p.Transform(s.Location().Transformation());
        sProps.Assign(new Properties(p));
        double errorMax = SurfacePropertiesCore(s, sProps, eps, skipShared, false);
        return errorMax;
    }

    //! Process faces of a shape for volume properties computation.
    //! This is the core face-level integration loop used both for unique solids
    //! and for shapes without shared solids.
    private static double VolumePropertiesFaces(in Shape s,
                                                Properties props,
                                                in Point3D refPnt,
                                                double eps,
                                                bool skipShared,
                                                bool useTriangulation)
    {
        int i;
        double errorMax = 0.0, error = 0.0;
        var ex = new Explorer();
        var g = new global::Kerf.Engine.Measurement.Vinert();
        g.SetLocation(refPnt);
        var mg = new MeshProps(global::Kerf.Engine.Measurement.MeshProps.BRepGProp_MeshObjType.Vinert);
        mg.SetLocation(refPnt);

        var bf = new FaceIntegrand();
        var bd = new Domain();
        var fwdFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var rvsFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Placement locDummy;

        for (ex.Init(s, ShapeType.Face), i = 1; ex.More(); ex.Next(), i++)
        {
            Face f = ShapeCasts.Face(ex.Current());
            Orientation ori = f.Orientation();
            bool isFwd = ori == Forward;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = ori == Reversed;
            }

            if (skipShared)
            {
                if ((isFwd && !fwdFMap.Add(f)) || (isRvs && !rvsFMap.Add(f)))
                {
                    continue;
                }
            }

            bool noSurf = false, noTri = false;
            {
                Surface? surf = Tool.Surface(f, out locDummy);
                if (surf is null)
                {
                    noSurf = true;
                }

                Triangulation? tri = Tool.Triangulation(f, out locDummy);
                if (tri is null || tri.NodeCount == 0 || tri.TriangleCount == 0)
                {
                    noTri = true;
                }

                if (noTri && noSurf)
                {
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                if ((useTriangulation && !noTri) || (noSurf && !noTri))
                {
                    Triangulation? tri = Tool.Triangulation(f, out locDummy);
                    mg.Perform(tri, locDummy, ori);
                    props.Add(mg);
                }
                else
                {
                    bf.Load(f);
                    bool isNatRestr = (f.ChildCount == 0);
                    if (!isNatRestr)
                    {
                        bd.Init(f);
                    }

                    if (eps < 1.0)
                    {
                        g.Perform(bf, bd, eps);
                        error = g.GetEpsilon();
                        if (errorMax < error)
                        {
                            errorMax = error;
                        }
                    }
                    else
                    {
                        if (isNatRestr)
                        {
                            g.Perform(bf);
                        }
                        else
                        {
                            g.Perform(bf, bd);
                        }
                    }

                    props.Add(g);
                }
            }
        }

        return errorMax;
    }

    private sealed class SolidCacheEntry
    {
        public Properties Props = new();
        public Placement Location = new();
        public Orientation Orientation;
    }

    private static double VolumePropertiesCore(in Shape s,
                                           Properties props,
                                           double eps,
                                           bool skipShared,
                                           bool useTriangulation)
    {
        Point3D p = RoughBaryCenter(s);

        // Check for shared solids: if the same TShape appears multiple times
        // (via compound nesting with different locations), we compute properties
        // once per unique solid and reuse via rigid transform for duplicates.
        bool hasSharedSolids = false;
        {
            var seenSolids = new NCollection_Map<TShape>(ReferenceEqualityComparer.Instance);
            for (var exS = new Explorer(s, ShapeType.Solid); exS.More(); exS.Next())
            {
                if (!seenSolids.Add(exS.Current().TShape()!))
                {
                    hasSharedSolids = true;
                    break;
                }
            }
        }

        if (!hasSharedSolids)
        {
            // No shared solids: use direct face-level iteration (original path).
            return VolumePropertiesFaces(s, props, p, eps, skipShared, useTriangulation);
        }

        // Shared solids detected: iterate at solid level with caching.
        // For each unique TShape, compute properties once via face integration.
        // For duplicate instances, transform the cached result by the relative
        // location and add to Props (using Huygens theorem via GProp_GProps::Add).
        var solidCache = new NCollection_DataMap<TShape, SolidCacheEntry>(ReferenceEqualityComparer.Instance);

        double errorMax = 0.0;

        for (var exS = new Explorer(s, ShapeType.Solid); exS.More(); exS.Next())
        {
            Shape solid = exS.Current();
            TShape ts = solid.TShape()!;

            if (solidCache.IsBound(ts))
            {
                // Duplicate instance: transform cached properties.
                SolidCacheEntry cached = solidCache.Find(ts);
                Placement relLoc = solid.Location().Multiplied(cached.Location.Inverted());

                // When SkipShared is enabled, skip exact duplicate instances
                // (same placement and orientation) - matches original face-level dedup behavior.
                if (skipShared && relLoc.IsIdentity && solid.Orientation() == cached.Orientation)
                {
                    continue;
                }

                // Cache reuse is only valid for rigid (isometric) transforms.
                // Shape locations may contain scaling or negative determinant
                // (TopoDS_Shape::Location() does not enforce rigidity by default),
                // which would produce incorrect volume/inertia with rotation-only transform.
                Transform3D relTrsf = relLoc.Transformation();
                if (Math.Abs(Math.Abs(relTrsf.ScaleFactor()) - 1.0) > Placement.ScalePrec()
                    || relTrsf.IsNegative)
                {
                    // Non-rigid relative transform: fall back to direct face-level computation.
                    var solidProps = new Properties(p);
                    double error = VolumePropertiesFaces(solid, solidProps, p, eps, skipShared, useTriangulation);
                    if (errorMax < error)
                    {
                        errorMax = error;
                    }

                    props.Add(solidProps);
                    continue;
                }

                var transformed = new GPropTransformer(cached.Props);
                if (!relLoc.IsIdentity)
                {
                    transformed.ApplyTransform(relTrsf);
                }

                if (solid.Orientation() != cached.Orientation)
                {
                    transformed.Negate();
                }

                props.Add(transformed);
            }
            else
            {
                // First instance of this TShape: compute via face integration.
                var solidProps = new Properties(p);
                double error = VolumePropertiesFaces(solid, solidProps, p, eps, skipShared, useTriangulation);
                if (errorMax < error)
                {
                    errorMax = error;
                }

                var entry = new SolidCacheEntry
                {
                    Props = new Properties(solidProps),
                    Location = solid.Location(),
                    Orientation = solid.Orientation(),
                };
                solidCache.Bind(ts, entry);

                props.Add(solidProps);
            }
        }

        // Handle faces not belonging to any solid (free shells/faces in the compound).
        {
            bool hasFree = false;
            Compound freeComp = new();
            var bld = new BrepBuilder();
            bld.MakeCompound(ref freeComp);
            for (var exF = new Explorer(s, ShapeType.Face, ShapeType.Solid); exF.More(); exF.Next())
            {
                bld.Add(freeComp, exF.Current());
                hasFree = true;
            }

            if (hasFree)
            {
                var freeProps = new Properties(p);
                double freeError = VolumePropertiesFaces(freeComp, freeProps, p, eps, skipShared, useTriangulation);
                if (errorMax < freeError)
                {
                    errorMax = freeError;
                }

                props.Add(freeProps);
            }
        }

        return errorMax;
    }

    /// <summary>
    /// The volume properties of <paramref name="s"/> into <paramref name="vProps"/>: the volume bounded by its faces,
    /// centre of mass and inertia, by Gauss integration with numbers of points chosen by the types of the surfaces and
    /// curves. The faces must bound a region of space (a closed shell, a solid, ...); a solid met again at another
    /// rigid placement is computed once and moved.
    /// </summary>
    /// <param name="s">The shape.</param>
    /// <param name="vProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="onlyClosed">Count only the closed shells (BRep_Tool.IsClosed).</param>
    /// <param name="skipShared">Count a face met again (IsSame, with the same orientation) or a shell met again once.</param>
    /// <param name="useTriangulation">Measure faces with a triangulation on the triangulation.</param>
    public static void VolumeProperties(in Shape s,
                                        Properties vProps,
                                        bool onlyClosed = false,
                                        bool skipShared = false,
                                        bool useTriangulation = false)
    {
        // find the origin
        Point3D p = new(0, 0, 0);
        p.Transform(s.Location().Transformation());
        vProps.Assign(new Properties(p));
        if (onlyClosed)
        {
            var shMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
            var ex = new Explorer(s, ShapeType.Shell);
            for (; ex.More(); ex.Next())
            {
                Shape sh = ex.Current();
                if (skipShared && !shMap.Add(sh))
                {
                    continue;
                }

                if (Tool.IsClosed(sh))
                {
                    VolumePropertiesCore(sh, vProps, 1.0, skipShared, useTriangulation);
                }
            }
        }
        else
        {
            VolumePropertiesCore(s, vProps, 1.0, skipShared, useTriangulation);
        }
    }

    /// <summary>The volume properties of <paramref name="s"/>, in a new system (see the other form).</summary>
    public static Properties VolumeProperties(in Shape s,
                                                bool onlyClosed = false,
                                                bool skipShared = false,
                                                bool useTriangulation = false)
    {
        var props = new Properties();
        VolumeProperties(s, props, onlyClosed, skipShared, useTriangulation);
        return props;
    }

    /// <summary>
    /// The volume properties of <paramref name="s"/> into <paramref name="vProps"/>, by adaptive Gauss integration to
    /// the relative error <paramref name="eps"/> of each face's contribution.
    /// </summary>
    /// <returns>The largest relative error reached by a face.</returns>
    public static double VolumeProperties(in Shape s,
                                          Properties vProps,
                                          double eps,
                                          bool onlyClosed = false,
                                          bool skipShared = false)
    {
        // find the origin
        Point3D p = new(0, 0, 0);
        p.Transform(s.Location().Transformation());
        vProps.Assign(new Properties(p));
        int i;
        double errorMax = 0.0, error = 0.0;
        if (onlyClosed)
        {
            var shMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
            var ex = new Explorer(s, ShapeType.Shell);
            for (i = 1; ex.More(); ex.Next(), i++)
            {
                Shape sh = ex.Current();
                if (skipShared && !shMap.Add(sh))
                {
                    continue;
                }

                if (Tool.IsClosed(sh))
                {
                    error = VolumePropertiesCore(sh, vProps, eps, skipShared, false);
                    if (errorMax < error)
                    {
                        errorMax = error;
                    }
                }
            }
        }
        else
        {
            errorMax = VolumePropertiesCore(s, vProps, eps, skipShared, false);
        }

        return errorMax;
    }

    //===========================================================================================//
    // Volume properties by Gauss-Kronrod integration
    //===========================================================================================//

    private static double VolumePropertiesGKCore(in Shape shape,
                                             Properties props,
                                             double tol,
                                             bool isUseSpan,
                                             bool cgFlag,
                                             bool iFlag,
                                             bool skipShared)
    {
        var exp = new Explorer();
        exp.Init(shape, ShapeType.Face);

        double aTol = tol;

        // Compute properties.
        Point3D loc = RoughBaryCenter(shape);
        var vProps = new VinertGK();
        var propFace = new FaceIntegrand(isUseSpan);
        var propDomain = new Domain();
        double localError;
        double error = 0.0;
        var fwdFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var rvsFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

        vProps.SetLocation(loc);

        for (; exp.More(); exp.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            Orientation ori = face.Orientation();
            bool isFwd = ori == Forward;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = ori == Reversed;
            }

            if (skipShared)
            {
                if ((isFwd && !fwdFMap.Add(face)) || (isRvs && !rvsFMap.Add(face)))
                {
                    continue;
                }
            }

            {
                Surface? surf = Tool.Surface(face, out _);
                if (surf is null)
                {
                    // skip faces without geometry
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                propFace.Load(face);

                bool isNatRestr = (face.ChildCount == 0);
                if (isNatRestr)
                {
                    localError = vProps.Perform(propFace, aTol, cgFlag, iFlag);
                }
                else
                {
                    propDomain.Init(face);
                    localError = vProps.Perform(propFace, propDomain, aTol, cgFlag, iFlag);
                }

                if (localError < 0.0)
                {
                    return localError;
                }

                error += localError;
                props.Add(vProps);
            }
        }

        return error;
    }

    /// <summary>
    /// The volume properties of <paramref name="s"/> into <paramref name="vProps"/>, by adaptive Gauss-Kronrod
    /// integration (<see cref="VinertGK"/>) to the tolerance <paramref name="eps"/>.
    /// </summary>
    /// <param name="s">The shape.</param>
    /// <param name="vProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="eps">The tolerance of the integration.</param>
    /// <param name="onlyClosed">Count only the shells that BRepCheck_Shell finds closed.</param>
    /// <param name="isUseSpan">Integrate span by span on B-spline faces.</param>
    /// <param name="cgFlag">Compute the centre of mass.</param>
    /// <param name="iFlag">Compute the centre of mass and the inertia.</param>
    /// <param name="skipShared">Count a face met again (IsSame, with the same orientation) or a shell met again once.</param>
    /// <returns>The absolute error reached divided by the volume (above Epsilon(1)), or a negative value on failure.</returns>
    public static double VolumePropertiesGK(in Shape s,
                                            Properties vProps,
                                            double eps = 0.001,
                                            bool onlyClosed = false,
                                            bool isUseSpan = false,
                                            bool cgFlag = false,
                                            bool iFlag = false,
                                            bool skipShared = false)
    {
        Point3D p = new(0, 0, 0);
        double error = 0.0;

        p.Transform(s.Location().Transformation());
        vProps.Assign(new Properties(p));

        if (onlyClosed)
        {
            // To select closed shells.
            var exp = new Explorer();
            var closedShells = new NCollection_List<Shape>();
            var shMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

            exp.Init(s, ShapeType.Shell);

            for (; exp.More(); exp.Next())
            {
                Shape shell = exp.Current();
                if (skipShared && !shMap.Add(shell))
                {
                    continue;
                }

                var checker = new ShellCheck(ShapeCasts.Shell(shell));
                StatusCheck status = checker.Closed(false);

                if (status == StatusCheck.NoError)
                {
                    closedShells.Append(shell);
                }
            }

            if (closedShells.IsEmpty())
            {
                return -1.0;
            }

            // Compute the properties for each closed shell.
            double tol = eps;
            double localError;
            var iter = new NCollection_List<Shape>.Iterator(closedShells);

            for (; iter.More(); iter.Next())
            {
                Shape shell = iter.Value();

                localError = VolumePropertiesGKCore(shell, vProps, tol, isUseSpan, cgFlag, iFlag, skipShared);

                if (localError < 0)
                {
                    return localError;
                }

                error += localError;
            }
        }
        else
        {
            error = VolumePropertiesGKCore(s, vProps, eps, isUseSpan, cgFlag, iFlag, skipShared);
        }

        double vol = vProps.Mass();
        if (vol > Epsilon(1.0))
        {
            error /= vol;
        }

        return error;
    }

    private static double VolumePropertiesGKCore(in Shape shape,
                                             Properties props,
                                             in Plane3D pln,
                                             double tol,
                                             bool isUseSpan,
                                             bool cgFlag,
                                             bool iFlag,
                                             bool skipShared)
    {
        var exp = new Explorer();
        exp.Init(shape, ShapeType.Face);

        double aTol = tol;

        // Compute properties.
        Point3D loc = RoughBaryCenter(shape);
        var vProps = new VinertGK();
        var propFace = new FaceIntegrand(isUseSpan);
        var propDomain = new Domain();
        double localError;
        double error = 0.0;
        var fwdFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var rvsFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

        vProps.SetLocation(loc);

        for (; exp.More(); exp.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            Orientation ori = face.Orientation();
            bool isFwd = ori == Forward;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = ori == Reversed;
            }

            if (skipShared)
            {
                if ((isFwd && !fwdFMap.Add(face)) || (isRvs && !rvsFMap.Add(face)))
                {
                    continue;
                }
            }

            {
                Surface? surf = Tool.Surface(face, out _);
                if (surf is null)
                {
                    // skip faces without geometry
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                propFace.Load(face);

                bool isNatRestr = (face.ChildCount == 0);
                if (isNatRestr)
                {
                    localError = vProps.Perform(propFace, pln, aTol, cgFlag, iFlag);
                }
                else
                {
                    propDomain.Init(face);
                    localError = vProps.Perform(propFace, propDomain, pln, aTol, cgFlag, iFlag);
                }

                if (localError < 0.0)
                {
                    return localError;
                }

                error += localError;
                props.Add(vProps);
            }
        }

        return error;
    }

    /// <summary>
    /// The volume properties of the region between the faces of <paramref name="s"/> and the plane
    /// <paramref name="pln"/> into <paramref name="vProps"/>, by adaptive Gauss-Kronrod integration; the parameters
    /// are those of the other form.
    /// </summary>
    public static double VolumePropertiesGK(in Shape s,
                                            Properties vProps,
                                            in Plane3D pln,
                                            double eps = 0.001,
                                            bool onlyClosed = false,
                                            bool isUseSpan = false,
                                            bool cgFlag = false,
                                            bool iFlag = false,
                                            bool skipShared = false)
    {
        Point3D p = new(0, 0, 0);
        double error = 0.0;

        p.Transform(s.Location().Transformation());
        vProps.Assign(new Properties(p));

        if (onlyClosed)
        {
            // To select closed shells.
            var exp = new Explorer();
            var closedShells = new NCollection_List<Shape>();
            var shMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

            exp.Init(s, ShapeType.Shell);

            for (; exp.More(); exp.Next())
            {
                Shape shell = exp.Current();
                if (skipShared && !shMap.Add(shell))
                {
                    continue;
                }

                var checker = new ShellCheck(ShapeCasts.Shell(shell));
                StatusCheck status = checker.Closed(false);

                if (status == StatusCheck.NoError)
                {
                    closedShells.Append(shell);
                }
            }

            if (closedShells.IsEmpty())
            {
                return -1.0;
            }

            // Compute the properties for each closed shell.
            double tol = eps;
            double localError;
            var iter = new NCollection_List<Shape>.Iterator(closedShells);

            for (; iter.More(); iter.Next())
            {
                Shape shell = iter.Value();

                localError = VolumePropertiesGKCore(shell, vProps, pln, tol, isUseSpan, cgFlag, iFlag, skipShared);

                if (localError < 0)
                {
                    return localError;
                }

                error += localError;
            }
        }
        else
        {
            error = VolumePropertiesGKCore(s, vProps, pln, eps, isUseSpan, cgFlag, iFlag, skipShared);
        }

        double vol = vProps.Mass();
        if (vol > Epsilon(1.0))
        {
            error /= vol;
        }

        return error;
    }
}
