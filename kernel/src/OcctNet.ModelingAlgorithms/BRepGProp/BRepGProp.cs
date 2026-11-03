// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepCheck_Status;
using static Occt.BRepGProp_MeshProps.BRepGProp_MeshObjType;
using static Kerf.Geometry.TransformForm;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

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
public static class BRepGProp
{
    /// <summary>Transforms GProp_GProps through its protected members, for the solid-level cache of volume properties.</summary>
    /// <remarks>
    /// Properties are computed once per unique solid and transformed for the duplicate instances at other locations.
    /// </remarks>
    private sealed class GPropTransformer : Properties
    {
        public GPropTransformer(Properties theSource)
            : base(theSource)
        {
        }

        //! Apply a rigid transform to the stored properties.
        //! Transforms reference point (loc) as absolute point, gravity center offset (g)
        //! as relative vector (rotation only, no translation), and rotates inertia tensor.
        public void ApplyTransform(in Transform3D theTrsf)
        {
            _loc.Transform(theTrsf);
            TransformForm aForm = theTrsf.Form();
            if (aForm != Identity && aForm != Translation)
            {
                // g is a relative offset from loc to center of mass - rotate only.
                Matrix3x3 aR = theTrsf.VectorialPart();
                Xyz aGxyz = _g.XYZ();
                aGxyz.Multiply(aR);
                _g.SetXYZ(aGxyz);
                Matrix3x3 aRt = aR.Transposed();
                _inertia = aR.Multiplied(_inertia).Multiplied(aRt);
            }
        }

        //! Negate volume contribution (for reversed solid orientation).
        public void Negate()
        {
            _dim = -_dim;
            _inertia.Multiply(-1.0);
        }
    }

    private static Point3D roughBaryCenter(in Shape S)
    {
        int i;
        var ex = new Explorer();
        Xyz xyz = new(0, 0, 0);
        for (ex.Init(S, ShapeType.Vertex), i = 0; ex.More(); ex.Next(), i++)
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
            ex.Init(S, ShapeType.Face);
            for (; ex.More(); ex.Next())
            {
                Shape aF = ex.Current();
                Triangulation? aTri = Tool.Triangulation(ShapeCasts.Face(aF), out Placement aLocDummy);
                if (aTri is not null && aTri.NodeCount > 0)
                {
                    xyz = aTri.Node(1).XYZ();
                    if (!aLocDummy.IsIdentity)
                    {
                        aLocDummy.Transformation().Transforms(ref xyz);
                    }

                    break;
                }
            }
        }

        return new Point3D(xyz);
    }

    /// <summary>
    /// The linear properties of the edges of <paramref name="S"/> into <paramref name="LProps"/>: total length, centre
    /// of mass and inertia.
    /// </summary>
    /// <param name="S">The shape.</param>
    /// <param name="LProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="SkipShared">Count an edge met again (IsSame) once.</param>
    /// <param name="UseTriangulation">Measure edges with a polygon on the polygon.</param>
    public static void LinearProperties(in Shape S,
                                        Properties LProps,
                                        bool SkipShared = false,
                                        bool UseTriangulation = false)
    {
        // find the origin
        Point3D P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        LProps.Assign(new Properties(P));

        var BAC = new EdgeCurveAdaptor();
        var anEMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var ex = new Explorer();
        for (ex.Init(S, ShapeType.Edge); ex.More(); ex.Next())
        {
            Edge aE = ShapeCasts.Edge(ex.Current());
            if (SkipShared && !anEMap.Add(aE))
            {
                continue;
            }

            NCollection_HArray1<Point3D>? theNodes = null;
            bool IsGeom = Tool.IsGeometric(aE);
            if (UseTriangulation || !IsGeom)
            {
                theNodes = BRepGProp_MeshCinert.PreparePolygon(aE);
            }

            if (theNodes is not null)
            {
                var MG = new BRepGProp_MeshCinert();
                MG.SetLocation(P);
                MG.Perform(theNodes.Array1());
                LProps.Add(MG);
            }
            else
            {
                if (IsGeom)
                {
                    BAC.Initialize(aE);
                    var CG = new BRepGProp_Cinert(BAC, P);
                    LProps.Add(CG);
                }
            }
        }
    }

    /// <summary>The linear properties of the edges of <paramref name="S"/>, in a new system (see the other form).</summary>
    public static Properties LinearProperties(in Shape S, bool SkipShared = false, bool UseTriangulation = false)
    {
        var aProps = new Properties();
        LinearProperties(S, aProps, SkipShared, UseTriangulation);
        return aProps;
    }

    private static double surfaceProperties(in Shape S,
                                            Properties Props,
                                            double Eps,
                                            bool SkipShared,
                                            bool UseTriangulation)
    {
        int i;
        double ErrorMax = 0.0, Error;
        var ex = new Explorer();
        var P = roughBaryCenter(S);
        var G = new BRepGProp_Sinert();
        G.SetLocation(P);
        var MG = new BRepGProp_MeshProps(Sinert);
        MG.SetLocation(P);

        var BF = new BRepGProp_Face();
        var BD = new BRepGProp_Domain();
        var aFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Placement aLocDummy;

        for (ex.Init(S, ShapeType.Face), i = 1; ex.More(); ex.Next(), i++)
        {
            Face F = ShapeCasts.Face(ex.Current());
            if (SkipShared && !aFMap.Add(F))
            {
                continue;
            }

            bool NoSurf = false, NoTri = false;
            {
                Surface? aSurf = Tool.Surface(F, out aLocDummy);
                if (aSurf is null)
                {
                    NoSurf = true;
                }

                Triangulation? aTri = Tool.Triangulation(F, out aLocDummy);
                if (aTri is null || aTri.NodeCount == 0 || aTri.TriangleCount == 0)
                {
                    NoTri = true;
                }

                if (NoTri && NoSurf)
                {
                    continue;
                }
            }

            if ((UseTriangulation && !NoTri) || (NoSurf && !NoTri))
            {
                Orientation anOri = F.Orientation();
                Triangulation? aTri = Tool.Triangulation(F, out aLocDummy);
                MG.Perform(aTri, aLocDummy, anOri);
                Props.Add(MG);
            }
            else
            {
                BF.Load(F);
                bool IsNatRestr = (F.ChildCount == 0);
                if (!IsNatRestr)
                {
                    BD.Init(F);
                }

                if (Eps < 1.0)
                {
                    G.Perform(BF, BD, Eps);
                    Error = G.GetEpsilon();
                    if (ErrorMax < Error)
                    {
                        ErrorMax = Error;
                    }
                }
                else
                {
                    if (IsNatRestr)
                    {
                        G.Perform(BF);
                    }
                    else
                    {
                        G.Perform(BF, BD);
                    }
                }

                Props.Add(G);
            }
        }

        return ErrorMax;
    }

    /// <summary>
    /// The surface properties of the faces of <paramref name="S"/> into <paramref name="SProps"/>: total area, centre
    /// of mass and inertia, by Gauss integration with numbers of points chosen by the types of the surfaces and curves.
    /// </summary>
    /// <param name="S">The shape.</param>
    /// <param name="SProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="SkipShared">Count a face met again (IsSame) once.</param>
    /// <param name="UseTriangulation">Measure faces with a triangulation on the triangulation.</param>
    public static void SurfaceProperties(in Shape S,
                                         Properties SProps,
                                         bool SkipShared = false,
                                         bool UseTriangulation = false)
    {
        // find the origin
        Point3D P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        SProps.Assign(new Properties(P));
        surfaceProperties(S, SProps, 1.0, SkipShared, UseTriangulation);
    }

    /// <summary>The surface properties of the faces of <paramref name="S"/>, in a new system (see the other form).</summary>
    public static Properties SurfaceProperties(in Shape S, bool SkipShared = false, bool UseTriangulation = false)
    {
        var aProps = new Properties();
        SurfaceProperties(S, aProps, SkipShared, UseTriangulation);
        return aProps;
    }

    /// <summary>
    /// The surface properties of the faces of <paramref name="S"/> into <paramref name="SProps"/>, by adaptive Gauss
    /// integration to the relative error <paramref name="Eps"/> of each face's area.
    /// </summary>
    /// <returns>The largest relative error reached by a face.</returns>
    public static double SurfaceProperties(in Shape S, Properties SProps, double Eps, bool SkipShared = false)
    {
        // find the origin
        Point3D P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        SProps.Assign(new Properties(P));
        double ErrorMax = surfaceProperties(S, SProps, Eps, SkipShared, false);
        return ErrorMax;
    }

    //! Process faces of a shape for volume properties computation.
    //! This is the core face-level integration loop used both for unique solids
    //! and for shapes without shared solids.
    private static double volumePropertiesFaces(in Shape S,
                                                Properties Props,
                                                in Point3D theRefPnt,
                                                double Eps,
                                                bool SkipShared,
                                                bool UseTriangulation)
    {
        int i;
        double ErrorMax = 0.0, Error = 0.0;
        var ex = new Explorer();
        var G = new BRepGProp_Vinert();
        G.SetLocation(theRefPnt);
        var MG = new BRepGProp_MeshProps(Vinert);
        MG.SetLocation(theRefPnt);

        var BF = new BRepGProp_Face();
        var BD = new BRepGProp_Domain();
        var aFwdFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aRvsFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Placement aLocDummy;

        for (ex.Init(S, ShapeType.Face), i = 1; ex.More(); ex.Next(), i++)
        {
            Face F = ShapeCasts.Face(ex.Current());
            Orientation anOri = F.Orientation();
            bool isFwd = anOri == Forward;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = anOri == Reversed;
            }

            if (SkipShared)
            {
                if ((isFwd && !aFwdFMap.Add(F)) || (isRvs && !aRvsFMap.Add(F)))
                {
                    continue;
                }
            }

            bool NoSurf = false, NoTri = false;
            {
                Surface? aSurf = Tool.Surface(F, out aLocDummy);
                if (aSurf is null)
                {
                    NoSurf = true;
                }

                Triangulation? aTri = Tool.Triangulation(F, out aLocDummy);
                if (aTri is null || aTri.NodeCount == 0 || aTri.TriangleCount == 0)
                {
                    NoTri = true;
                }

                if (NoTri && NoSurf)
                {
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                if ((UseTriangulation && !NoTri) || (NoSurf && !NoTri))
                {
                    Triangulation? aTri = Tool.Triangulation(F, out aLocDummy);
                    MG.Perform(aTri, aLocDummy, anOri);
                    Props.Add(MG);
                }
                else
                {
                    BF.Load(F);
                    bool IsNatRestr = (F.ChildCount == 0);
                    if (!IsNatRestr)
                    {
                        BD.Init(F);
                    }

                    if (Eps < 1.0)
                    {
                        G.Perform(BF, BD, Eps);
                        Error = G.GetEpsilon();
                        if (ErrorMax < Error)
                        {
                            ErrorMax = Error;
                        }
                    }
                    else
                    {
                        if (IsNatRestr)
                        {
                            G.Perform(BF);
                        }
                        else
                        {
                            G.Perform(BF, BD);
                        }
                    }

                    Props.Add(G);
                }
            }
        }

        return ErrorMax;
    }

    private sealed class SolidCacheEntry
    {
        public Properties Props = new();
        public Placement Location = new();
        public Orientation Orientation;
    }

    private static double volumeProperties(in Shape S,
                                           Properties Props,
                                           double Eps,
                                           bool SkipShared,
                                           bool UseTriangulation)
    {
        Point3D P = roughBaryCenter(S);

        // Check for shared solids: if the same TShape appears multiple times
        // (via compound nesting with different locations), we compute properties
        // once per unique solid and reuse via rigid transform for duplicates.
        bool hasSharedSolids = false;
        {
            var aSeenSolids = new NCollection_Map<TShape>(ReferenceEqualityComparer.Instance);
            for (var exS = new Explorer(S, ShapeType.Solid); exS.More(); exS.Next())
            {
                if (!aSeenSolids.Add(exS.Current().TShape()!))
                {
                    hasSharedSolids = true;
                    break;
                }
            }
        }

        if (!hasSharedSolids)
        {
            // No shared solids: use direct face-level iteration (original path).
            return volumePropertiesFaces(S, Props, P, Eps, SkipShared, UseTriangulation);
        }

        // Shared solids detected: iterate at solid level with caching.
        // For each unique TShape, compute properties once via face integration.
        // For duplicate instances, transform the cached result by the relative
        // location and add to Props (using Huygens theorem via GProp_GProps::Add).
        var aSolidCache = new NCollection_DataMap<TShape, SolidCacheEntry>(ReferenceEqualityComparer.Instance);

        double ErrorMax = 0.0;

        for (var exS = new Explorer(S, ShapeType.Solid); exS.More(); exS.Next())
        {
            Shape aSolid = exS.Current();
            TShape aTS = aSolid.TShape()!;

            if (aSolidCache.IsBound(aTS))
            {
                // Duplicate instance: transform cached properties.
                SolidCacheEntry aCached = aSolidCache.Find(aTS);
                Placement aRelLoc = aSolid.Location().Multiplied(aCached.Location.Inverted());

                // When SkipShared is enabled, skip exact duplicate instances
                // (same placement and orientation) - matches original face-level dedup behavior.
                if (SkipShared && aRelLoc.IsIdentity && aSolid.Orientation() == aCached.Orientation)
                {
                    continue;
                }

                // Cache reuse is only valid for rigid (isometric) transforms.
                // Shape locations may contain scaling or negative determinant
                // (TopoDS_Shape::Location() does not enforce rigidity by default),
                // which would produce incorrect volume/inertia with rotation-only transform.
                Transform3D aRelTrsf = aRelLoc.Transformation();
                if (Math.Abs(Math.Abs(aRelTrsf.ScaleFactor()) - 1.0) > Placement.ScalePrec()
                    || aRelTrsf.IsNegative)
                {
                    // Non-rigid relative transform: fall back to direct face-level computation.
                    var aSolidProps = new Properties(P);
                    double anError = volumePropertiesFaces(aSolid, aSolidProps, P, Eps, SkipShared, UseTriangulation);
                    if (ErrorMax < anError)
                    {
                        ErrorMax = anError;
                    }

                    Props.Add(aSolidProps);
                    continue;
                }

                var aTransformed = new GPropTransformer(aCached.Props);
                if (!aRelLoc.IsIdentity)
                {
                    aTransformed.ApplyTransform(aRelTrsf);
                }

                if (aSolid.Orientation() != aCached.Orientation)
                {
                    aTransformed.Negate();
                }

                Props.Add(aTransformed);
            }
            else
            {
                // First instance of this TShape: compute via face integration.
                var aSolidProps = new Properties(P);
                double anError = volumePropertiesFaces(aSolid, aSolidProps, P, Eps, SkipShared, UseTriangulation);
                if (ErrorMax < anError)
                {
                    ErrorMax = anError;
                }

                var anEntry = new SolidCacheEntry
                {
                    Props = new Properties(aSolidProps),
                    Location = aSolid.Location(),
                    Orientation = aSolid.Orientation(),
                };
                aSolidCache.Bind(aTS, anEntry);

                Props.Add(aSolidProps);
            }
        }

        // Handle faces not belonging to any solid (free shells/faces in the compound).
        {
            bool hasFree = false;
            Compound aFreeComp = new();
            var aBld = new BrepBuilder();
            aBld.MakeCompound(ref aFreeComp);
            for (var exF = new Explorer(S, ShapeType.Face, ShapeType.Solid); exF.More(); exF.Next())
            {
                aBld.Add(aFreeComp, exF.Current());
                hasFree = true;
            }

            if (hasFree)
            {
                var aFreeProps = new Properties(P);
                double aFreeError = volumePropertiesFaces(aFreeComp, aFreeProps, P, Eps, SkipShared, UseTriangulation);
                if (ErrorMax < aFreeError)
                {
                    ErrorMax = aFreeError;
                }

                Props.Add(aFreeProps);
            }
        }

        return ErrorMax;
    }

    /// <summary>
    /// The volume properties of <paramref name="S"/> into <paramref name="VProps"/>: the volume bounded by its faces,
    /// centre of mass and inertia, by Gauss integration with numbers of points chosen by the types of the surfaces and
    /// curves. The faces must bound a region of space (a closed shell, a solid, ...); a solid met again at another
    /// rigid placement is computed once and moved.
    /// </summary>
    /// <param name="S">The shape.</param>
    /// <param name="VProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="OnlyClosed">Count only the closed shells (BRep_Tool.IsClosed).</param>
    /// <param name="SkipShared">Count a face met again (IsSame, with the same orientation) or a shell met again once.</param>
    /// <param name="UseTriangulation">Measure faces with a triangulation on the triangulation.</param>
    public static void VolumeProperties(in Shape S,
                                        Properties VProps,
                                        bool OnlyClosed = false,
                                        bool SkipShared = false,
                                        bool UseTriangulation = false)
    {
        // find the origin
        Point3D P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        VProps.Assign(new Properties(P));
        if (OnlyClosed)
        {
            var aShMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
            var ex = new Explorer(S, ShapeType.Shell);
            for (; ex.More(); ex.Next())
            {
                Shape Sh = ex.Current();
                if (SkipShared && !aShMap.Add(Sh))
                {
                    continue;
                }

                if (Tool.IsClosed(Sh))
                {
                    volumeProperties(Sh, VProps, 1.0, SkipShared, UseTriangulation);
                }
            }
        }
        else
        {
            volumeProperties(S, VProps, 1.0, SkipShared, UseTriangulation);
        }
    }

    /// <summary>The volume properties of <paramref name="S"/>, in a new system (see the other form).</summary>
    public static Properties VolumeProperties(in Shape S,
                                                bool OnlyClosed = false,
                                                bool SkipShared = false,
                                                bool UseTriangulation = false)
    {
        var aProps = new Properties();
        VolumeProperties(S, aProps, OnlyClosed, SkipShared, UseTriangulation);
        return aProps;
    }

    /// <summary>
    /// The volume properties of <paramref name="S"/> into <paramref name="VProps"/>, by adaptive Gauss integration to
    /// the relative error <paramref name="Eps"/> of each face's contribution.
    /// </summary>
    /// <returns>The largest relative error reached by a face.</returns>
    public static double VolumeProperties(in Shape S,
                                          Properties VProps,
                                          double Eps,
                                          bool OnlyClosed = false,
                                          bool SkipShared = false)
    {
        // find the origin
        Point3D P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        VProps.Assign(new Properties(P));
        int i;
        double ErrorMax = 0.0, Error = 0.0;
        if (OnlyClosed)
        {
            var aShMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
            var ex = new Explorer(S, ShapeType.Shell);
            for (i = 1; ex.More(); ex.Next(), i++)
            {
                Shape Sh = ex.Current();
                if (SkipShared && !aShMap.Add(Sh))
                {
                    continue;
                }

                if (Tool.IsClosed(Sh))
                {
                    Error = volumeProperties(Sh, VProps, Eps, SkipShared, false);
                    if (ErrorMax < Error)
                    {
                        ErrorMax = Error;
                    }
                }
            }
        }
        else
        {
            ErrorMax = volumeProperties(S, VProps, Eps, SkipShared, false);
        }

        return ErrorMax;
    }

    //===========================================================================================//
    // Volume properties by Gauss-Kronrod integration
    //===========================================================================================//

    private static double volumePropertiesGK(in Shape theShape,
                                             Properties theProps,
                                             double theTol,
                                             bool IsUseSpan,
                                             bool CGFlag,
                                             bool IFlag,
                                             bool SkipShared)
    {
        var anExp = new Explorer();
        anExp.Init(theShape, ShapeType.Face);

        double aTol = theTol;

        // Compute properties.
        Point3D aLoc = roughBaryCenter(theShape);
        var aVProps = new BRepGProp_VinertGK();
        var aPropFace = new BRepGProp_Face(IsUseSpan);
        var aPropDomain = new BRepGProp_Domain();
        double aLocalError;
        double anError = 0.0;
        var aFwdFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aRvsFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

        aVProps.SetLocation(aLoc);

        for (; anExp.More(); anExp.Next())
        {
            Face aFace = ShapeCasts.Face(anExp.Current());
            Orientation anOri = aFace.Orientation();
            bool isFwd = anOri == Forward;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = anOri == Reversed;
            }

            if (SkipShared)
            {
                if ((isFwd && !aFwdFMap.Add(aFace)) || (isRvs && !aRvsFMap.Add(aFace)))
                {
                    continue;
                }
            }

            {
                Surface? aSurf = Tool.Surface(aFace, out _);
                if (aSurf is null)
                {
                    // skip faces without geometry
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                aPropFace.Load(aFace);

                bool IsNatRestr = (aFace.ChildCount == 0);
                if (IsNatRestr)
                {
                    aLocalError = aVProps.Perform(aPropFace, aTol, CGFlag, IFlag);
                }
                else
                {
                    aPropDomain.Init(aFace);
                    aLocalError = aVProps.Perform(aPropFace, aPropDomain, aTol, CGFlag, IFlag);
                }

                if (aLocalError < 0.0)
                {
                    return aLocalError;
                }

                anError += aLocalError;
                theProps.Add(aVProps);
            }
        }

        return anError;
    }

    /// <summary>
    /// The volume properties of <paramref name="S"/> into <paramref name="VProps"/>, by adaptive Gauss-Kronrod
    /// integration (<see cref="BRepGProp_VinertGK"/>) to the tolerance <paramref name="Eps"/>.
    /// </summary>
    /// <param name="S">The shape.</param>
    /// <param name="VProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="Eps">The tolerance of the integration.</param>
    /// <param name="OnlyClosed">Count only the shells that BRepCheck_Shell finds closed.</param>
    /// <param name="IsUseSpan">Integrate span by span on B-spline faces.</param>
    /// <param name="CGFlag">Compute the centre of mass.</param>
    /// <param name="IFlag">Compute the centre of mass and the inertia.</param>
    /// <param name="SkipShared">Count a face met again (IsSame, with the same orientation) or a shell met again once.</param>
    /// <returns>The absolute error reached divided by the volume (above Epsilon(1)), or a negative value on failure.</returns>
    public static double VolumePropertiesGK(in Shape S,
                                            Properties VProps,
                                            double Eps = 0.001,
                                            bool OnlyClosed = false,
                                            bool IsUseSpan = false,
                                            bool CGFlag = false,
                                            bool IFlag = false,
                                            bool SkipShared = false)
    {
        Point3D P = new(0, 0, 0);
        double anError = 0.0;

        P.Transform(S.Location().Transformation());
        VProps.Assign(new Properties(P));

        if (OnlyClosed)
        {
            // To select closed shells.
            var anExp = new Explorer();
            var aClosedShells = new NCollection_List<Shape>();
            var aShMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

            anExp.Init(S, ShapeType.Shell);

            for (; anExp.More(); anExp.Next())
            {
                Shape aShell = anExp.Current();
                if (SkipShared && !aShMap.Add(aShell))
                {
                    continue;
                }

                var aChecker = new BRepCheck_Shell(ShapeCasts.Shell(aShell));
                BRepCheck_Status aStatus = aChecker.Closed(false);

                if (aStatus == BRepCheck_NoError)
                {
                    aClosedShells.Append(aShell);
                }
            }

            if (aClosedShells.IsEmpty())
            {
                return -1.0;
            }

            // Compute the properties for each closed shell.
            double aTol = Eps;
            double aLocalError;
            var anIter = new NCollection_List<Shape>.Iterator(aClosedShells);

            for (; anIter.More(); anIter.Next())
            {
                Shape aShell = anIter.Value();

                aLocalError = volumePropertiesGK(aShell, VProps, aTol, IsUseSpan, CGFlag, IFlag, SkipShared);

                if (aLocalError < 0)
                {
                    return aLocalError;
                }

                anError += aLocalError;
            }
        }
        else
        {
            anError = volumePropertiesGK(S, VProps, Eps, IsUseSpan, CGFlag, IFlag, SkipShared);
        }

        double vol = VProps.Mass();
        if (vol > Epsilon(1.0))
        {
            anError /= vol;
        }

        return anError;
    }

    private static double volumePropertiesGK(in Shape theShape,
                                             Properties theProps,
                                             in Plane3D thePln,
                                             double theTol,
                                             bool IsUseSpan,
                                             bool CGFlag,
                                             bool IFlag,
                                             bool SkipShared)
    {
        var anExp = new Explorer();
        anExp.Init(theShape, ShapeType.Face);

        double aTol = theTol;

        // Compute properties.
        Point3D aLoc = roughBaryCenter(theShape);
        var aVProps = new BRepGProp_VinertGK();
        var aPropFace = new BRepGProp_Face(IsUseSpan);
        var aPropDomain = new BRepGProp_Domain();
        double aLocalError;
        double anError = 0.0;
        var aFwdFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aRvsFMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

        aVProps.SetLocation(aLoc);

        for (; anExp.More(); anExp.Next())
        {
            Face aFace = ShapeCasts.Face(anExp.Current());
            Orientation anOri = aFace.Orientation();
            bool isFwd = anOri == Forward;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = anOri == Reversed;
            }

            if (SkipShared)
            {
                if ((isFwd && !aFwdFMap.Add(aFace)) || (isRvs && !aRvsFMap.Add(aFace)))
                {
                    continue;
                }
            }

            {
                Surface? aSurf = Tool.Surface(aFace, out _);
                if (aSurf is null)
                {
                    // skip faces without geometry
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                aPropFace.Load(aFace);

                bool IsNatRestr = (aFace.ChildCount == 0);
                if (IsNatRestr)
                {
                    aLocalError = aVProps.Perform(aPropFace, thePln, aTol, CGFlag, IFlag);
                }
                else
                {
                    aPropDomain.Init(aFace);
                    aLocalError = aVProps.Perform(aPropFace, aPropDomain, thePln, aTol, CGFlag, IFlag);
                }

                if (aLocalError < 0.0)
                {
                    return aLocalError;
                }

                anError += aLocalError;
                theProps.Add(aVProps);
            }
        }

        return anError;
    }

    /// <summary>
    /// The volume properties of the region between the faces of <paramref name="S"/> and the plane
    /// <paramref name="thePln"/> into <paramref name="VProps"/>, by adaptive Gauss-Kronrod integration; the parameters
    /// are those of the other form.
    /// </summary>
    public static double VolumePropertiesGK(in Shape S,
                                            Properties VProps,
                                            in Plane3D thePln,
                                            double Eps = 0.001,
                                            bool OnlyClosed = false,
                                            bool IsUseSpan = false,
                                            bool CGFlag = false,
                                            bool IFlag = false,
                                            bool SkipShared = false)
    {
        Point3D P = new(0, 0, 0);
        double anError = 0.0;

        P.Transform(S.Location().Transformation());
        VProps.Assign(new Properties(P));

        if (OnlyClosed)
        {
            // To select closed shells.
            var anExp = new Explorer();
            var aClosedShells = new NCollection_List<Shape>();
            var aShMap = new NCollection_Map<Shape>(ShapeHasher.Instance);

            anExp.Init(S, ShapeType.Shell);

            for (; anExp.More(); anExp.Next())
            {
                Shape aShell = anExp.Current();
                if (SkipShared && !aShMap.Add(aShell))
                {
                    continue;
                }

                var aChecker = new BRepCheck_Shell(ShapeCasts.Shell(aShell));
                BRepCheck_Status aStatus = aChecker.Closed(false);

                if (aStatus == BRepCheck_NoError)
                {
                    aClosedShells.Append(aShell);
                }
            }

            if (aClosedShells.IsEmpty())
            {
                return -1.0;
            }

            // Compute the properties for each closed shell.
            double aTol = Eps;
            double aLocalError;
            var anIter = new NCollection_List<Shape>.Iterator(aClosedShells);

            for (; anIter.More(); anIter.Next())
            {
                Shape aShell = anIter.Value();

                aLocalError = volumePropertiesGK(aShell, VProps, thePln, aTol, IsUseSpan, CGFlag, IFlag, SkipShared);

                if (aLocalError < 0)
                {
                    return aLocalError;
                }

                anError += aLocalError;
            }
        }
        else
        {
            anError = volumePropertiesGK(S, VProps, thePln, Eps, IsUseSpan, CGFlag, IFlag, SkipShared);
        }

        double vol = VProps.Mass();
        if (vol > Epsilon(1.0))
        {
            anError /= vol;
        }

        return anError;
    }
}
