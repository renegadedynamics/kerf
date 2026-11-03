// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.Draft_ErrorStatus;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The draft angle modification of a shape for <see cref="Modifier"/>: faces turned by an angle around their
/// intersection with a neutral plane, with their tangent neighbours, and the new edges and vertices between them.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape it is given: the new geometry it computes is its own, and
/// <see cref="NewCurve2d"/> writes only the range of the modifier's new edge. OCCT's maps keyed by faces, edges and
/// vertices with TopTools_ShapeMapHasher are keyed by TopoDS_Shape in the port (the same IsSame comparison).
/// </remarks>
public sealed partial class Draft_Modification : Modification
{
    private readonly NCollection_IndexedDataMap<Shape, Draft_FaceInfo> myFMap = new(ShapeHasher.Instance);
    private readonly NCollection_IndexedDataMap<Shape, Draft_EdgeInfo> myEMap = new(ShapeHasher.Instance);
    private readonly NCollection_IndexedDataMap<Shape, Draft_VertexInfo> myVMap = new(ShapeHasher.Instance);
    private bool myComp;
    private Shape myShape = new();
    private Shape badShape = new();
    private Draft_ErrorStatus errStat;
    private Face curFace = new();
    private readonly NCollection_List<Shape> conneF = new();
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myEFMap =
        new(ShapeHasher.Instance);

    //=================================================================================================

    /// <summary>The modification of the shape <paramref name="S"/>.</summary>
    public Draft_Modification(in Shape S)
    {
        myComp = false;
        myShape = S;
        Exploration.MapShapesAndAncestors(myShape, ShapeType.Edge, ShapeType.Face, myEFMap);
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
        badShape = new Shape();
        errStat = Draft_NoError;
    }

    //=================================================================================================

    /// <summary>Changes the basis shape and resets.</summary>
    public void Init(in Shape S)
    {
        myShape = S;
        Clear();
        Exploration.MapShapesAndAncestors(myShape, ShapeType.Edge, ShapeType.Face, myEFMap);
    }

    //=================================================================================================

    /// <summary>
    /// Adds the face <paramref name="F"/> and propagates the draft to its tangent neighbours. Returns false on an
    /// error; <see cref="ProblematicShape"/> is then the bad shape.
    /// </summary>
    /// <exception cref="ConstructionErrorException">An earlier Add failed and the bad face wasn't removed.</exception>
    public bool Add(in Face F, in Direction3D Direction, double Angle, in Plane3D NeutralPlane, bool Flag = true)
    {
        if (!badShape.IsNull)
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
    public void Remove(in Face F)
    {
        if (!myFMap.Contains(F) || myComp)
        {
            throw new NoSuchObjectException();
        }

        conneF.Clear();

        curFace = myFMap.FindFromKey(F).RootFace();
        for (int i = 1; i <= myFMap.Extent(); i++)
        {
            Face theF = ShapeCasts.Face(myFMap.FindKey(i));
            if (myFMap.FindFromKey(theF).RootFace().IsSame(curFace))
            {
                conneF.Append(theF);
                if (theF.IsSame(badShape))
                {
                    badShape = new Shape();
                }
            }
        }

        for (NCollection_List<Shape>.Iterator ltod = new(conneF); ltod.More(); ltod.Next())
        {
            myFMap.RemoveKey(ShapeCasts.Face(ltod.Value()));
        }

        conneF.Clear();
        for (int i = 1; i <= myEMap.Extent(); i++)
        {
            Edge theE = ShapeCasts.Edge(myEMap.FindKey(i));
            if (myEMap.FindFromKey(theE).RootFace().IsSame(curFace))
            {
                conneF.Append(theE);
            }
        }
        for (NCollection_List<Shape>.Iterator ltod = new(conneF); ltod.More(); ltod.Next())
        {
            myEMap.RemoveKey(ShapeCasts.Edge(ltod.Value()));
        }
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> succeeded.</summary>
    public bool IsDone() => myComp && badShape.IsNull;

    //=================================================================================================

    /// <summary>The error status.</summary>
    public Draft_ErrorStatus Error() => errStat;

    //=================================================================================================

    /// <summary>The face, edge or vertex on which an error occurred (null if none).</summary>
    public Shape ProblematicShape() => badShape;

    //=================================================================================================

    /// <summary>The faces added together with the face <paramref name="F"/>.</summary>
    /// <exception cref="NoSuchObjectException">The face wasn't added.</exception>
    /// <exception cref="NotDoneException">The modification isn't done.</exception>
    public NCollection_List<Shape> ConnectedFaces(in Face F)
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
            Face theF = ShapeCasts.Face(myFMap.FindKey(i));
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
    public NCollection_List<Shape> ModifiedFaces()
    {
        if (!badShape.IsNull)
        {
            throw new NotDoneException();
        }
        conneF.Clear();

        for (int i = 1; i <= myFMap.Extent(); i++)
        {
            Face theF = ShapeCasts.Face(myFMap.FindKey(i));
            if (!myFMap.FindFromKey(theF).RootFace().IsNull)
            {
                conneF.Append(theF);
            }
        }

        return conneF;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewSurface(in Face F,
                                    out Surface? S,
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
        Tol = Tool.Tolerance(F);

        S = Tool.Surface(F, out L);

        L = new Placement(); // L.Identity()

        S = myFMap.FindFromKey(F).Geometry();

        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewCurve(in Edge E, out Curve3D? C, out Placement L, out double Tol)
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
        Tol = Max(Tol, Tool.Tolerance(E));
        L = new Placement(); // L.Identity()
        C = myEMap.FindFromKey(E).Geometry();

        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewPoint(in Vertex V, out Point3D P, out double Tol)
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

        Tol = Tool.Tolerance(V);
        P = myVMap.FindFromKey(V).Geometry();
        return true;
    }

    //=================================================================================================

    /// <remarks>Writes the range of <paramref name="NewE"/>, the modifier's new edge.</remarks>
    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewCurve2d(in Edge E,
                                    in Face F,
                                    in Edge NewE,
                                    in Face NewF,
                                    out Curve2D? C,
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

        Tool.Range(NewE, out double Fp, out double Lp);

        Surface SB = myFMap.FindFromKey(F).Geometry()!;

        Tol = Tool.Tolerance(E);

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
                Tool.Range(E, out double Fpi, out double Lpi);
                if (Fpi <= Fp && Fp <= Lpi && Fpi <= Lp && Lp <= Lpi)
                {
                    return false;
                }
            }

            //  if (!BRep_Tool::IsClosed(E,F)) {
            Tool.Range(NewE, out Fp, out Lp);
            TrimmedCurve3D TC = new(myEMap.FindFromKey(E).Geometry()!, Fp, Lp);
            Fp = TC.FirstParameter();
            Lp = TC.LastParameter();
            BrepBuilder B = new();
            B.Range(NewE, Fp, Lp);
            C = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(TC, Fp, Lp, SB, ref Tol);
        }

        Type typs = SB.GetType();
        if (typs == typeof(TrimmedSurface))
        {
            SB = ((TrimmedSurface)SB).BasisSurface();
            typs = SB.GetType();
        }

        bool JeRecadre = false;
        if (typs == typeof(ExtrusionSurface))
        {
            Curve3D aC = ((ExtrusionSurface)SB).BasisCurve();
            Type typc = aC.GetType();
            if (typc == typeof(CircleCurve3D))
            {
                JeRecadre = true;
            }
        }

        JeRecadre = JeRecadre || (typs == typeof(CylindricalSurface))
                    || (typs == typeof(SphericalSurface))
                    || (typs == typeof(ConicalSurface));

        if (JeRecadre)
        {
            bool bTranslate;
            double aD2;
            Point2D PF, NewPF;
            Vector2D aV2DT = new(), vectra = new(2.0 * M_PI, 0.0);
            Curve2D aC2DE;
            //
            aC2DE = Tool.CurveOnSurface(E, F, out double aT1, out double aT2)!;
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
                    C = (Curve2D)C.Copy();
                }
                C.Translate(aV2DT);
            }
        }
        //
        Curve3D? aC3d = Tool.Curve(NewE, out Fp, out Lp);
        Tol = BrepTools.EvalAndUpdateTol(NewE, aC3d!, C!, SB, Fp, Lp);
        return true;
    }

    //=================================================================================================

    /// <exception cref="DomainErrorException">The modification isn't done.</exception>
    public override bool NewParameter(in Vertex V, in Edge E, ref double P, ref double Tol)
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
        Curve3D GC = myEMap.FindFromKey(E).Geometry()!;
        Type typc = GC.GetType();
        if (typc == typeof(TrimmedCurve3D))
        {
            // OCCT down-casts to Geom_TrimmedCurve and keeps the trimmed curve.
            typc = GC.GetType();
        }

        if (GC.IsClosed)
        {
            Vertex FV = Exploration.FirstVertex(E);
            double paramf;
            if (myVMap.Contains(FV))
            {
                paramf = myVMap.ChangeFromKey(FV).Parameter(E);
            }
            else
            {
                paramf = Tool.Parameter(FV, E);
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
                if (GC.IsPeriodic)
                {
                    P += GC.Period();
                }
                else
                {
                    P = GC.LastParameter();
                }
            }
        }

        Tol = Max(Tool.Tolerance(V), Tool.Tolerance(E));
        return true;
    }

    //=================================================================================================

    public override Continuity Continuity(in Edge E,
                                             in Face F1,
                                             in Face F2,
                                             in Edge NewE,
                                             in Face NewF1,
                                             in Face NewF2)
    {
        return Tool.Continuity(E, F1, F2);
    }
}
