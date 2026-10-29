// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;
using static Occt.BRepGProp_MeshProps.BRepGProp_MeshObjType;
using static Occt.gp_TrsfForm;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The global properties of a shape: of its edges (length, <see cref="LinearProperties(in TopoDS_Shape, GProp_GProps, bool, bool)"/>),
/// of its faces (area, <see cref="SurfaceProperties(in TopoDS_Shape, GProp_GProps, bool, bool)"/>) or of the volume
/// they bound (<see cref="VolumeProperties(in TopoDS_Shape, GProp_GProps, bool, bool, bool)"/>): mass, centre of mass
/// and inertia, at density 1.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's forms fill the caller's <see cref="GProp_GProps"/>, which is reset first, its reference point at the
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
    private sealed class GPropTransformer : GProp_GProps
    {
        public GPropTransformer(GProp_GProps theSource)
            : base(theSource)
        {
        }

        //! Apply a rigid transform to the stored properties.
        //! Transforms reference point (loc) as absolute point, gravity center offset (g)
        //! as relative vector (rotation only, no translation), and rotates inertia tensor.
        public void ApplyTransform(in gp_Trsf theTrsf)
        {
            loc.Transform(theTrsf);
            gp_TrsfForm aForm = theTrsf.Form();
            if (aForm != gp_Identity && aForm != gp_Translation)
            {
                // g is a relative offset from loc to center of mass - rotate only.
                gp_Mat aR = theTrsf.VectorialPart();
                gp_XYZ aGxyz = g.XYZ();
                aGxyz.Multiply(aR);
                g.SetXYZ(aGxyz);
                gp_Mat aRt = aR.Transposed();
                inertia = aR.Multiplied(inertia).Multiplied(aRt);
            }
        }

        //! Negate volume contribution (for reversed solid orientation).
        public void Negate()
        {
            dim = -dim;
            inertia.Multiply(-1.0);
        }
    }

    private static gp_Pnt roughBaryCenter(in TopoDS_Shape S)
    {
        int i;
        var ex = new TopExp_Explorer();
        gp_XYZ xyz = new(0, 0, 0);
        for (ex.Init(S, TopAbs_VERTEX), i = 0; ex.More(); ex.Next(), i++)
        {
            xyz += BRep_Tool.Pnt(TopoDS.Vertex(ex.Current())).XYZ();
        }

        if (i > 0)
        {
            xyz /= i;
        }
        else
        {
            // Try using triangulation
            ex.Init(S, TopAbs_FACE);
            for (; ex.More(); ex.Next())
            {
                TopoDS_Shape aF = ex.Current();
                Poly_Triangulation? aTri = BRep_Tool.Triangulation(TopoDS.Face(aF), out TopLoc_Location aLocDummy);
                if (aTri is not null && aTri.NbNodes() > 0)
                {
                    xyz = aTri.Node(1).XYZ();
                    if (!aLocDummy.IsIdentity())
                    {
                        aLocDummy.Transformation().Transforms(ref xyz);
                    }

                    break;
                }
            }
        }

        return new gp_Pnt(xyz);
    }

    /// <summary>
    /// The linear properties of the edges of <paramref name="S"/> into <paramref name="LProps"/>: total length, centre
    /// of mass and inertia.
    /// </summary>
    /// <param name="S">The shape.</param>
    /// <param name="LProps">The system to fill; reset first, its reference point at the shape's location.</param>
    /// <param name="SkipShared">Count an edge met again (IsSame) once.</param>
    /// <param name="UseTriangulation">Measure edges with a polygon on the polygon.</param>
    public static void LinearProperties(in TopoDS_Shape S,
                                        GProp_GProps LProps,
                                        bool SkipShared = false,
                                        bool UseTriangulation = false)
    {
        // find the origin
        gp_Pnt P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        LProps.Assign(new GProp_GProps(P));

        GeomAdaptor_TransformedCurve BAC; // TODO(3B): BRepAdaptor_Curve
        var anEMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var ex = new TopExp_Explorer();
        for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(ex.Current());
            if (SkipShared && !anEMap.Add(aE))
            {
                continue;
            }

            NCollection_HArray1<gp_Pnt>? theNodes = null;
            bool IsGeom = BRep_Tool.IsGeometric(aE);
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
                    BAC = BRepLib_CheckCurveOnSurface.BRepAdaptorCurve(aE); // TODO(3B): BAC.Initialize(aE)
                    var CG = new BRepGProp_Cinert(BAC, P);
                    LProps.Add(CG);
                }
            }
        }
    }

    /// <summary>The linear properties of the edges of <paramref name="S"/>, in a new system (see the other form).</summary>
    public static GProp_GProps LinearProperties(in TopoDS_Shape S, bool SkipShared = false, bool UseTriangulation = false)
    {
        var aProps = new GProp_GProps();
        LinearProperties(S, aProps, SkipShared, UseTriangulation);
        return aProps;
    }

    private static double surfaceProperties(in TopoDS_Shape S,
                                            GProp_GProps Props,
                                            double Eps,
                                            bool SkipShared,
                                            bool UseTriangulation)
    {
        int i;
        double ErrorMax = 0.0, Error;
        var ex = new TopExp_Explorer();
        var P = roughBaryCenter(S);
        var G = new BRepGProp_Sinert();
        G.SetLocation(P);
        var MG = new BRepGProp_MeshProps(Sinert);
        MG.SetLocation(P);

        var BF = new BRepGProp_Face();
        var BD = new BRepGProp_Domain();
        var aFMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopLoc_Location aLocDummy;

        for (ex.Init(S, TopAbs_FACE), i = 1; ex.More(); ex.Next(), i++)
        {
            TopoDS_Face F = TopoDS.Face(ex.Current());
            if (SkipShared && !aFMap.Add(F))
            {
                continue;
            }

            bool NoSurf = false, NoTri = false;
            {
                Geom_Surface? aSurf = BRep_Tool.Surface(F, out aLocDummy);
                if (aSurf is null)
                {
                    NoSurf = true;
                }

                Poly_Triangulation? aTri = BRep_Tool.Triangulation(F, out aLocDummy);
                if (aTri is null || aTri.NbNodes() == 0 || aTri.NbTriangles() == 0)
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
                TopAbs_Orientation anOri = F.Orientation();
                Poly_Triangulation? aTri = BRep_Tool.Triangulation(F, out aLocDummy);
                MG.Perform(aTri, aLocDummy, anOri);
                Props.Add(MG);
            }
            else
            {
                BF.Load(F);
                bool IsNatRestr = (F.NbChildren() == 0);
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
    public static void SurfaceProperties(in TopoDS_Shape S,
                                         GProp_GProps SProps,
                                         bool SkipShared = false,
                                         bool UseTriangulation = false)
    {
        // find the origin
        gp_Pnt P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        SProps.Assign(new GProp_GProps(P));
        surfaceProperties(S, SProps, 1.0, SkipShared, UseTriangulation);
    }

    /// <summary>The surface properties of the faces of <paramref name="S"/>, in a new system (see the other form).</summary>
    public static GProp_GProps SurfaceProperties(in TopoDS_Shape S, bool SkipShared = false, bool UseTriangulation = false)
    {
        var aProps = new GProp_GProps();
        SurfaceProperties(S, aProps, SkipShared, UseTriangulation);
        return aProps;
    }

    /// <summary>
    /// The surface properties of the faces of <paramref name="S"/> into <paramref name="SProps"/>, by adaptive Gauss
    /// integration to the relative error <paramref name="Eps"/> of each face's area.
    /// </summary>
    /// <returns>The largest relative error reached by a face.</returns>
    public static double SurfaceProperties(in TopoDS_Shape S, GProp_GProps SProps, double Eps, bool SkipShared = false)
    {
        // find the origin
        gp_Pnt P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        SProps.Assign(new GProp_GProps(P));
        double ErrorMax = surfaceProperties(S, SProps, Eps, SkipShared, false);
        return ErrorMax;
    }

    //! Process faces of a shape for volume properties computation.
    //! This is the core face-level integration loop used both for unique solids
    //! and for shapes without shared solids.
    private static double volumePropertiesFaces(in TopoDS_Shape S,
                                                GProp_GProps Props,
                                                in gp_Pnt theRefPnt,
                                                double Eps,
                                                bool SkipShared,
                                                bool UseTriangulation)
    {
        int i;
        double ErrorMax = 0.0, Error = 0.0;
        var ex = new TopExp_Explorer();
        var G = new BRepGProp_Vinert();
        G.SetLocation(theRefPnt);
        var MG = new BRepGProp_MeshProps(Vinert);
        MG.SetLocation(theRefPnt);

        var BF = new BRepGProp_Face();
        var BD = new BRepGProp_Domain();
        var aFwdFMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aRvsFMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopLoc_Location aLocDummy;

        for (ex.Init(S, TopAbs_FACE), i = 1; ex.More(); ex.Next(), i++)
        {
            TopoDS_Face F = TopoDS.Face(ex.Current());
            TopAbs_Orientation anOri = F.Orientation();
            bool isFwd = anOri == TopAbs_FORWARD;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = anOri == TopAbs_REVERSED;
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
                Geom_Surface? aSurf = BRep_Tool.Surface(F, out aLocDummy);
                if (aSurf is null)
                {
                    NoSurf = true;
                }

                Poly_Triangulation? aTri = BRep_Tool.Triangulation(F, out aLocDummy);
                if (aTri is null || aTri.NbNodes() == 0 || aTri.NbTriangles() == 0)
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
                    Poly_Triangulation? aTri = BRep_Tool.Triangulation(F, out aLocDummy);
                    MG.Perform(aTri, aLocDummy, anOri);
                    Props.Add(MG);
                }
                else
                {
                    BF.Load(F);
                    bool IsNatRestr = (F.NbChildren() == 0);
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
        public GProp_GProps Props = new();
        public TopLoc_Location Location = new();
        public TopAbs_Orientation Orientation;
    }

    private static double volumeProperties(in TopoDS_Shape S,
                                           GProp_GProps Props,
                                           double Eps,
                                           bool SkipShared,
                                           bool UseTriangulation)
    {
        gp_Pnt P = roughBaryCenter(S);

        // Check for shared solids: if the same TShape appears multiple times
        // (via compound nesting with different locations), we compute properties
        // once per unique solid and reuse via rigid transform for duplicates.
        bool hasSharedSolids = false;
        {
            var aSeenSolids = new NCollection_Map<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
            for (var exS = new TopExp_Explorer(S, TopAbs_SOLID); exS.More(); exS.Next())
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
        var aSolidCache = new NCollection_DataMap<TopoDS_TShape, SolidCacheEntry>(ReferenceEqualityComparer.Instance);

        double ErrorMax = 0.0;

        for (var exS = new TopExp_Explorer(S, TopAbs_SOLID); exS.More(); exS.Next())
        {
            TopoDS_Shape aSolid = exS.Current();
            TopoDS_TShape aTS = aSolid.TShape()!;

            if (aSolidCache.IsBound(aTS))
            {
                // Duplicate instance: transform cached properties.
                SolidCacheEntry aCached = aSolidCache.Find(aTS);
                TopLoc_Location aRelLoc = aSolid.Location().Multiplied(aCached.Location.Inverted());

                // When SkipShared is enabled, skip exact duplicate instances
                // (same placement and orientation) - matches original face-level dedup behavior.
                if (SkipShared && aRelLoc.IsIdentity() && aSolid.Orientation() == aCached.Orientation)
                {
                    continue;
                }

                // Cache reuse is only valid for rigid (isometric) transforms.
                // Shape locations may contain scaling or negative determinant
                // (TopoDS_Shape::Location() does not enforce rigidity by default),
                // which would produce incorrect volume/inertia with rotation-only transform.
                gp_Trsf aRelTrsf = aRelLoc.Transformation();
                if (Math.Abs(Math.Abs(aRelTrsf.ScaleFactor()) - 1.0) > TopLoc_Location.ScalePrec()
                    || aRelTrsf.IsNegative())
                {
                    // Non-rigid relative transform: fall back to direct face-level computation.
                    var aSolidProps = new GProp_GProps(P);
                    double anError = volumePropertiesFaces(aSolid, aSolidProps, P, Eps, SkipShared, UseTriangulation);
                    if (ErrorMax < anError)
                    {
                        ErrorMax = anError;
                    }

                    Props.Add(aSolidProps);
                    continue;
                }

                var aTransformed = new GPropTransformer(aCached.Props);
                if (!aRelLoc.IsIdentity())
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
                var aSolidProps = new GProp_GProps(P);
                double anError = volumePropertiesFaces(aSolid, aSolidProps, P, Eps, SkipShared, UseTriangulation);
                if (ErrorMax < anError)
                {
                    ErrorMax = anError;
                }

                var anEntry = new SolidCacheEntry
                {
                    Props = new GProp_GProps(aSolidProps),
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
            TopoDS_Compound aFreeComp = new();
            var aBld = new BRep_Builder();
            aBld.MakeCompound(ref aFreeComp);
            for (var exF = new TopExp_Explorer(S, TopAbs_FACE, TopAbs_SOLID); exF.More(); exF.Next())
            {
                aBld.Add(aFreeComp, exF.Current());
                hasFree = true;
            }

            if (hasFree)
            {
                var aFreeProps = new GProp_GProps(P);
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
    public static void VolumeProperties(in TopoDS_Shape S,
                                        GProp_GProps VProps,
                                        bool OnlyClosed = false,
                                        bool SkipShared = false,
                                        bool UseTriangulation = false)
    {
        // find the origin
        gp_Pnt P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        VProps.Assign(new GProp_GProps(P));
        if (OnlyClosed)
        {
            var aShMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var ex = new TopExp_Explorer(S, TopAbs_SHELL);
            for (; ex.More(); ex.Next())
            {
                TopoDS_Shape Sh = ex.Current();
                if (SkipShared && !aShMap.Add(Sh))
                {
                    continue;
                }

                if (BRep_Tool.IsClosed(Sh))
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
    public static GProp_GProps VolumeProperties(in TopoDS_Shape S,
                                                bool OnlyClosed = false,
                                                bool SkipShared = false,
                                                bool UseTriangulation = false)
    {
        var aProps = new GProp_GProps();
        VolumeProperties(S, aProps, OnlyClosed, SkipShared, UseTriangulation);
        return aProps;
    }

    /// <summary>
    /// The volume properties of <paramref name="S"/> into <paramref name="VProps"/>, by adaptive Gauss integration to
    /// the relative error <paramref name="Eps"/> of each face's contribution.
    /// </summary>
    /// <returns>The largest relative error reached by a face.</returns>
    public static double VolumeProperties(in TopoDS_Shape S,
                                          GProp_GProps VProps,
                                          double Eps,
                                          bool OnlyClosed = false,
                                          bool SkipShared = false)
    {
        // find the origin
        gp_Pnt P = new(0, 0, 0);
        P.Transform(S.Location().Transformation());
        VProps.Assign(new GProp_GProps(P));
        int i;
        double ErrorMax = 0.0, Error = 0.0;
        if (OnlyClosed)
        {
            var aShMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            var ex = new TopExp_Explorer(S, TopAbs_SHELL);
            for (i = 1; ex.More(); ex.Next(), i++)
            {
                TopoDS_Shape Sh = ex.Current();
                if (SkipShared && !aShMap.Add(Sh))
                {
                    continue;
                }

                if (BRep_Tool.IsClosed(Sh))
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

    private static double volumePropertiesGK(in TopoDS_Shape theShape,
                                             GProp_GProps theProps,
                                             double theTol,
                                             bool IsUseSpan,
                                             bool CGFlag,
                                             bool IFlag,
                                             bool SkipShared)
    {
        var anExp = new TopExp_Explorer();
        anExp.Init(theShape, TopAbs_FACE);

        double aTol = theTol;

        // Compute properties.
        gp_Pnt aLoc = roughBaryCenter(theShape);
        var aVProps = new BRepGProp_VinertGK();
        var aPropFace = new BRepGProp_Face(IsUseSpan);
        var aPropDomain = new BRepGProp_Domain();
        double aLocalError;
        double anError = 0.0;
        var aFwdFMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aRvsFMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        aVProps.SetLocation(aLoc);

        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExp.Current());
            TopAbs_Orientation anOri = aFace.Orientation();
            bool isFwd = anOri == TopAbs_FORWARD;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = anOri == TopAbs_REVERSED;
            }

            if (SkipShared)
            {
                if ((isFwd && !aFwdFMap.Add(aFace)) || (isRvs && !aRvsFMap.Add(aFace)))
                {
                    continue;
                }
            }

            {
                Geom_Surface? aSurf = BRep_Tool.Surface(aFace, out _);
                if (aSurf is null)
                {
                    // skip faces without geometry
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                aPropFace.Load(aFace);

                bool IsNatRestr = (aFace.NbChildren() == 0);
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
    public static double VolumePropertiesGK(in TopoDS_Shape S,
                                            GProp_GProps VProps,
                                            double Eps = 0.001,
                                            bool OnlyClosed = false,
                                            bool IsUseSpan = false,
                                            bool CGFlag = false,
                                            bool IFlag = false,
                                            bool SkipShared = false)
    {
        gp_Pnt P = new(0, 0, 0);
        double anError = 0.0;

        P.Transform(S.Location().Transformation());
        VProps.Assign(new GProp_GProps(P));

        if (OnlyClosed)
        {
            // To select closed shells.
            var anExp = new TopExp_Explorer();
            var aClosedShells = new NCollection_List<TopoDS_Shape>();
            var aShMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

            anExp.Init(S, TopAbs_SHELL);

            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Shape aShell = anExp.Current();
                if (SkipShared && !aShMap.Add(aShell))
                {
                    continue;
                }

                var aChecker = new BRepCheck_Shell(TopoDS.Shell(aShell));
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
            var anIter = new NCollection_List<TopoDS_Shape>.Iterator(aClosedShells);

            for (; anIter.More(); anIter.Next())
            {
                TopoDS_Shape aShell = anIter.Value();

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

    private static double volumePropertiesGK(in TopoDS_Shape theShape,
                                             GProp_GProps theProps,
                                             in gp_Pln thePln,
                                             double theTol,
                                             bool IsUseSpan,
                                             bool CGFlag,
                                             bool IFlag,
                                             bool SkipShared)
    {
        var anExp = new TopExp_Explorer();
        anExp.Init(theShape, TopAbs_FACE);

        double aTol = theTol;

        // Compute properties.
        gp_Pnt aLoc = roughBaryCenter(theShape);
        var aVProps = new BRepGProp_VinertGK();
        var aPropFace = new BRepGProp_Face(IsUseSpan);
        var aPropDomain = new BRepGProp_Domain();
        double aLocalError;
        double anError = 0.0;
        var aFwdFMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aRvsFMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        aVProps.SetLocation(aLoc);

        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anExp.Current());
            TopAbs_Orientation anOri = aFace.Orientation();
            bool isFwd = anOri == TopAbs_FORWARD;
            bool isRvs = false;
            if (!isFwd)
            {
                isRvs = anOri == TopAbs_REVERSED;
            }

            if (SkipShared)
            {
                if ((isFwd && !aFwdFMap.Add(aFace)) || (isRvs && !aRvsFMap.Add(aFace)))
                {
                    continue;
                }
            }

            {
                Geom_Surface? aSurf = BRep_Tool.Surface(aFace, out _);
                if (aSurf is null)
                {
                    // skip faces without geometry
                    continue;
                }
            }

            if (isFwd || isRvs)
            {
                aPropFace.Load(aFace);

                bool IsNatRestr = (aFace.NbChildren() == 0);
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
    public static double VolumePropertiesGK(in TopoDS_Shape S,
                                            GProp_GProps VProps,
                                            in gp_Pln thePln,
                                            double Eps = 0.001,
                                            bool OnlyClosed = false,
                                            bool IsUseSpan = false,
                                            bool CGFlag = false,
                                            bool IFlag = false,
                                            bool SkipShared = false)
    {
        gp_Pnt P = new(0, 0, 0);
        double anError = 0.0;

        P.Transform(S.Location().Transformation());
        VProps.Assign(new GProp_GProps(P));

        if (OnlyClosed)
        {
            // To select closed shells.
            var anExp = new TopExp_Explorer();
            var aClosedShells = new NCollection_List<TopoDS_Shape>();
            var aShMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

            anExp.Init(S, TopAbs_SHELL);

            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Shape aShell = anExp.Current();
                if (SkipShared && !aShMap.Add(aShell))
                {
                    continue;
                }

                var aChecker = new BRepCheck_Shell(TopoDS.Shell(aShell));
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
            var anIter = new NCollection_List<TopoDS_Shape>.Iterator(aClosedShells);

            for (; anIter.More(); anIter.Next())
            {
                TopoDS_Shape aShell = anIter.Value();

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
