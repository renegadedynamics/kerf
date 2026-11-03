// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.Draft_ErrorStatus;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The draft angle modification of a shape for <see cref="BRepTools_Modifier"/>: faces turned by an angle around their
/// intersection with a neutral plane, with their tangent neighbours, and the new edges and vertices between them.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape it is given: the new geometry it computes is its own, and
/// <see cref="NewCurve2d"/> writes only the range of the modifier's new edge. OCCT's maps keyed by faces, edges and
/// vertices with TopTools_ShapeMapHasher are keyed by TopoDS_Shape in the port (the same IsSame comparison).
/// </remarks>
public sealed partial class Draft_Modification : BRepTools_Modification
{
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, Draft_FaceInfo> myFMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, Draft_EdgeInfo> myEMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, Draft_VertexInfo> myVMap = new(TopTools_ShapeMapHasher.Instance);
    private bool myComp;
    private TopoDS_Shape myShape = new();
    private TopoDS_Shape badShape = new();
    private Draft_ErrorStatus errStat;
    private TopoDS_Face curFace = new();
    private readonly NCollection_List<TopoDS_Shape> conneF = new();
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myEFMap =
        new(TopTools_ShapeMapHasher.Instance);

    //=================================================================================================

    /// <summary>The modification of the shape <paramref name="S"/>.</summary>
    public Draft_Modification(in TopoDS_Shape S)
    {
        myComp = false;
        myShape = S;
        TopExp.MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, myEFMap);
    }

    //=================================================================================================

    /// <summary>Resets on the same shape.</summary>
    public void Clear()
    {
        myComp = false;
        myFMap.Clear();
        myEMap.Clear();
        myVMap.Clear();
        myEFMap.Clear();
        badShape = new TopoDS_Shape();
        errStat = Draft_NoError;
    }

    //=================================================================================================

    /// <summary>Changes the basis shape and resets.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
        Clear();
        TopExp.MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, myEFMap);
    }

    //=================================================================================================

    /// <summary>
    /// Adds the face <paramref name="F"/> and propagates the draft to its tangent neighbours. Returns false on an
    /// error; <see cref="ProblematicShape"/> is then the bad shape.
    /// </summary>
    /// <exception cref="ConstructionErrorException">An earlier Add failed and the bad face wasn't removed.</exception>
    public bool Add(in TopoDS_Face F, in Direction3D Direction, double Angle, in Plane3D NeutralPlane, bool Flag = true)
    {
        if (!badShape.IsNull())
        {
            throw new ConstructionErrorException();
        }

        if (myComp)
        {
            Clear();
        }
        curFace = F;
        return InternalAdd(F, Direction, Angle, NeutralPlane, Flag);
    }

    //=================================================================================================

    /// <summary>Removes the face <paramref name="F"/> and the faces added with it.</summary>
    /// <exception cref="NoSuchObjectException">The face wasn't added, or the modification was performed.</exception>
    public void Remove(in TopoDS_Face F)
    {
        if (!myFMap.Contains(F) || myComp)
        {
            throw new NoSuchObjectException();
        }

        conneF.Clear();

        curFace = myFMap.FindFromKey(F).RootFace();
        for (int i = 1; i <= myFMap.Extent(); i++)
        {
            TopoDS_Face theF = TopoDS.Face(myFMap.FindKey(i));
            if (myFMap.FindFromKey(theF).RootFace().IsSame(curFace))
            {
                conneF.Append(theF);
                if (theF.IsSame(badShape))
                {
                    badShape = new TopoDS_Shape();
                }
            }
        }

        for (NCollection_List<TopoDS_Shape>.Iterator ltod = new(conneF); ltod.More(); ltod.Next())
        {
            myFMap.RemoveKey(TopoDS.Face(ltod.Value()));
        }

        conneF.Clear();
        for (int i = 1; i <= myEMap.Extent(); i++)
        {
            TopoDS_Edge theE = TopoDS.Edge(myEMap.FindKey(i));
            if (myEMap.FindFromKey(theE).RootFace().IsSame(curFace))
            {
                conneF.Append(theE);
            }
        }
        for (NCollection_List<TopoDS_Shape>.Iterator ltod = new(conneF); ltod.More(); ltod.Next())
        {
            myEMap.RemoveKey(TopoDS.Edge(ltod.Value()));
        }
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> succeeded.</summary>
    public bool IsDone() => myComp && badShape.IsNull();

    //=================================================================================================

    /// <summary>The error status.</summary>
    public Draft_ErrorStatus Error() => errStat;

    //=================================================================================================

    /// <summary>The face, edge or vertex on which an error occurred (null if none).</summary>
    public TopoDS_Shape ProblematicShape() => badShape;

    //=================================================================================================

    /// <summary>The faces added together with the face <paramref name="F"/>.</summary>
    /// <exception cref="NoSuchObjectException">The face wasn't added.</exception>
    /// <exception cref="NotDoneException">The modification isn't done.</exception>
    public NCollection_List<TopoDS_Shape> ConnectedFaces(in TopoDS_Face F)
    {
        if (!myFMap.Contains(F))
        {
            throw new NoSuchObjectException();
        }
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        conneF.Clear();
        curFace = myFMap.FindFromKey(F).RootFace();

        for (int i = 1; i <= myFMap.Extent(); i++)
        {
            TopoDS_Face theF = TopoDS.Face(myFMap.FindKey(i));
            if (myFMap.FindFromKey(theF).RootFace().IsSame(curFace))
            {
                conneF.Append(theF);
            }
        }

        return conneF;
    }

    //=================================================================================================

    /// <summary>The faces on which a modification has been given.</summary>
    /// <exception cref="NotDoneException">An Add failed.</exception>
    public NCollection_List<TopoDS_Shape> ModifiedFaces()
    {
        if (!badShape.IsNull())
        {
            throw new NotDoneException();
        }
        conneF.Clear();

        for (int i = 1; i <= myFMap.Extent(); i++)
        {
            TopoDS_Face theF = TopoDS.Face(myFMap.FindKey(i));
            if (!myFMap.FindFromKey(theF).RootFace().IsNull())
            {
                conneF.Append(theF);
            }
        }

        return conneF;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        S = null;
        L = new Placement();
        Tol = 0.0;
        RevWires = false;
        RevFace = false;
        if (!IsDone())
        {
            throw new DomainErrorException();
        }

        if (!myFMap.Contains(F) || !myFMap.FindFromKey(F).NewGeometry())
        {
            return false;
        }

        RevWires = false;
        RevFace = false;
        Tol = BRep_Tool.Tolerance(F);

        S = BRep_Tool.Surface(F, out L);

        L = new Placement(); // L.Identity()

        S = myFMap.FindFromKey(F).Geometry();

        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out Placement L, out double Tol)
    {
        C = null;
        L = new Placement();
        Tol = 0.0;
        if (!IsDone())
        {
            throw new DomainErrorException();
        }

        if (!myEMap.Contains(E))
        {
            return false;
        }

        Draft_EdgeInfo Einf = myEMap.FindFromKey(E);
        if (!myEMap.FindFromKey(E).NewGeometry())
        {
            return false;
        }

        Tol = Einf.Tolerance();
        Tol = Max(Tol, BRep_Tool.Tolerance(E));
        L = new Placement(); // L.Identity()
        C = myEMap.FindFromKey(E).Geometry();

        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewPoint(in TopoDS_Vertex V, out Point3D P, out double Tol)
    {
        P = new Point3D();
        Tol = 0.0;
        if (!IsDone())
        {
            throw new DomainErrorException();
        }

        if (!myVMap.Contains(V))
        {
            return false;
        }

        Tol = BRep_Tool.Tolerance(V);
        P = myVMap.FindFromKey(V).Geometry();
        return true;
    }

    //=================================================================================================

    /// <remarks>Writes the range of <paramref name="NewE"/>, the modifier's new edge.</remarks>
    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        if (!IsDone())
        {
            throw new DomainErrorException();
        }

        if (!myEMap.Contains(E))
        {
            return false;
        }

        BRep_Tool.Range(NewE, out double Fp, out double Lp);

        Geom_Surface SB = myFMap.FindFromKey(F).Geometry()!;

        Tol = BRep_Tool.Tolerance(E);

        Draft_EdgeInfo Einf = myEMap.FindFromKey(E);
        if (Einf.FirstFace().IsSame(F) && Einf.FirstPC() is not null)
        {
            C = Einf.FirstPC();
        }
        else if (Einf.SecondFace().IsSame(F) && Einf.SecondPC() is not null)
        {
            C = Einf.SecondPC();
        }
        else
        {
            if (!myEMap.FindFromKey(E).NewGeometry())
            {
                BRep_Tool.Range(E, out double Fpi, out double Lpi);
                if (Fpi <= Fp && Fp <= Lpi && Fpi <= Lp && Lp <= Lpi)
                {
                    return false;
                }
            }

            //  if (!BRep_Tool::IsClosed(E,F)) {
            BRep_Tool.Range(NewE, out Fp, out Lp);
            Geom_TrimmedCurve TC = new(myEMap.FindFromKey(E).Geometry()!, Fp, Lp);
            Fp = TC.FirstParameter();
            Lp = TC.LastParameter();
            BRep_Builder B = new();
            B.Range(NewE, Fp, Lp);
            C = GeomProjLib.Curve2d(TC, Fp, Lp, SB, ref Tol);
        }

        Type typs = SB.GetType();
        if (typs == typeof(Geom_RectangularTrimmedSurface))
        {
            SB = ((Geom_RectangularTrimmedSurface)SB).BasisSurface();
            typs = SB.GetType();
        }

        bool JeRecadre = false;
        if (typs == typeof(Geom_SurfaceOfLinearExtrusion))
        {
            Geom_Curve aC = ((Geom_SurfaceOfLinearExtrusion)SB).BasisCurve();
            Type typc = aC.GetType();
            if (typc == typeof(Geom_Circle))
            {
                JeRecadre = true;
            }
        }

        JeRecadre = JeRecadre || (typs == typeof(Geom_CylindricalSurface))
                    || (typs == typeof(Geom_SphericalSurface))
                    || (typs == typeof(Geom_ConicalSurface));

        if (JeRecadre)
        {
            bool bTranslate;
            double aD2;
            Point2D PF, NewPF;
            Vector2D aV2DT = new(), vectra = new(2.0 * M_PI, 0.0);
            Geom2d_Curve aC2DE;
            //
            aC2DE = BRep_Tool.CurveOnSurface(E, F, out double aT1, out double aT2)!;
            //
            PF = aC2DE.Value(0.5 * (aT1 + aT2));
            //
            NewPF = C!.Value(0.5 * (Fp + Lp));
            //
            aD2 = NewPF.SquareDistance(PF);
            //
            bTranslate = false;
            if (NewPF.Translated(vectra).SquareDistance(PF) < aD2)
            {
                aV2DT = vectra;
                bTranslate = !bTranslate; // True
            }
            else if (NewPF.Translated(-vectra).SquareDistance(PF) < aD2)
            {
                aV2DT = -vectra;
                bTranslate = !bTranslate; // True
            }
            //
            if (bTranslate)
            {
                // OCCT.NET: OCCT translates the curve in place, which may be the one kept for the edge (FirstPC or
                // SecondPC); a frozen one (already given to a modifier's edge) is translated on a copy.
                if (C.IsFrozen)
                {
                    C = (Geom2d_Curve)C.Copy();
                }
                C.Translate(aV2DT);
            }
        }
        //
        Geom_Curve? aC3d = BRep_Tool.Curve(NewE, out Fp, out Lp);
        Tol = BRepTools.EvalAndUpdateTol(NewE, aC3d!, C!, SB, Fp, Lp);
        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        if (!IsDone())
        {
            throw new DomainErrorException();
        }

        if (!myVMap.Contains(V))
        {
            return false;
        }

        P = myVMap.ChangeFromKey(V).Parameter(E);
        Geom_Curve GC = myEMap.FindFromKey(E).Geometry()!;
        Type typc = GC.GetType();
        if (typc == typeof(Geom_TrimmedCurve))
        {
            // OCCT down-casts to Geom_TrimmedCurve and keeps the trimmed curve.
            typc = GC.GetType();
        }

        if (GC.IsClosed())
        {
            TopoDS_Vertex FV = TopExp.FirstVertex(E);
            double paramf;
            if (myVMap.Contains(FV))
            {
                paramf = myVMap.ChangeFromKey(FV).Parameter(E);
            }
            else
            {
                paramf = BRep_Tool.Parameter(FV, E);
            }

            // Patch
            double FirstPar = GC.FirstParameter(), LastPar = GC.LastParameter();
            double pconf = global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (Math.Abs(paramf - LastPar) <= pconf)
            {
                paramf = FirstPar;
                FV.Orientation(E.Orientation());
                if (V.IsEqual(FV))
                {
                    P = paramf;
                }
            }

            FV.Orientation(E.Orientation());
            if (!V.IsEqual(FV) && P <= paramf)
            {
                if (GC.IsPeriodic())
                {
                    P += GC.Period();
                }
                else
                {
                    P = GC.LastParameter();
                }
            }
        }

        Tol = Max(BRep_Tool.Tolerance(V), BRep_Tool.Tolerance(E));
        return true;
    }

    //=================================================================================================

    public override Continuity Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }
}
