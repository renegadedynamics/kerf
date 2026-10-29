// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevol.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds the revolved topology of a shape around an axis, by an angle or a whole turn: vertex to edge, edge to face,
/// wire to shell, face to solid, shell to compsolid. A degenerated edge shared by several faces is copied for each.
/// </summary>
/// <remarks>
/// <para>Without copy, OCCT shares the generating shape's sub-shapes in the result and writes into them. A shape
/// holding frozen TShapes is swept as an exact copy, and the result holds its own sub-shapes wherever the sweep left
/// them unchanged (<see cref="BRepPrimAPI_MakeSweep"/>); the history answers for the caller's sub-shapes.</para>
/// <para>If an edge of the shape crosses the axis, the result is null (<see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// returns a null shape, as OCCT does).</para>
/// </remarks>
public class BRepPrimAPI_MakeRevol : BRepPrimAPI_MakeSweep
{
    private readonly BRepSweep_Revol myRevol;
    private readonly NCollection_List<TopoDS_Shape> myDegenerated = new();
    private BRepTools_History? myHist;
    private bool myIsBuild;

    // perform checks on the argument
    private static TopoDS_Shape check(in TopoDS_Shape S)
    {
        BRepLib.BuildCurves3d(S);
        return S;
    }

    /// <summary>
    /// The revolution of <paramref name="S"/> around <paramref name="A"/> by the angle <paramref name="D"/> (radians,
    /// reduced to [0, 2π]). With <paramref name="Copy"/>, S's sub-shapes are copied.
    /// </summary>
    public BRepPrimAPI_MakeRevol(in TopoDS_Shape S, in gp_Ax1 A, double D, bool Copy = false)
    {
        TopoDS_Shape aS = GeneratingShapeForWrite(S);
        myRevol = new BRepSweep_Revol(check(aS), A, D, Copy);
        myIsBuild = false;
        if (!CheckValidity(check(aS), A))
        {
            myShape.Nullify();
            myIsBuild = true;
        }
        else
        {
            Build();
        }
    }

    /// <summary>The whole revolution of <paramref name="S"/> around <paramref name="A"/>.</summary>
    public BRepPrimAPI_MakeRevol(in TopoDS_Shape S, in gp_Ax1 A, bool Copy = false)
    {
        TopoDS_Shape aS = GeneratingShapeForWrite(S);
        myRevol = new BRepSweep_Revol(check(aS), A, 2.0 * Math.PI, Copy);
        myIsBuild = false;
        if (!CheckValidity(check(aS), A))
        {
            myShape.Nullify();
            myIsBuild = true;
        }
        else
        {
            Build();
        }
    }

    /// <summary>The internal sweeping algorithm (on the copy of a frozen shape).</summary>
    internal BRepSweep_Revol Revol() => myRevol;

    /// <summary>Builds the resulting shape (redefined from MakeShape).</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        if (myIsBuild)
        {
            return;
        }
        myShape = myRevol.Shape();
        BRepLib.UpdateInnerTolerances(myShape);

        Done();
        myIsBuild = true;

        myHist = null;
        myDegenerated.Clear();
        var aDegE = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        var aBB = new BRep_Builder();

        var anExp = new TopExp_Explorer(myShape, TopAbs_EDGE);
        // Problem is that some degenerated edges can be shared by different faces.
        // It is not valid for correct shape.
        // To solve problem it is possible to copy shared degenerated edge for each face, which has it,
        // and replace shared edge by its copy
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Shape anEdge = anExp.Current();
            var aTEdge = (BRep_TEdge)anEdge.TShape()!;
            if (aTEdge.Degenerated())
            {
                if (aDegE.Find(anEdge, out NCollection_List<TopoDS_Shape> anL))
                {
                    // Make the copy if degenerated edge occurs more then once
                    TopoDS_Shape aCopyE = anEdge.EmptyCopied();
                    aCopyE.Orientation(TopAbs_FORWARD);
                    var aVIter = new TopoDS_Iterator(anEdge.Oriented(TopAbs_FORWARD), false);
                    for (; aVIter.More(); aVIter.Next())
                    {
                        aBB.Add(aCopyE, aVIter.Value());
                    }
                    aCopyE.Orientation(anEdge.Orientation());
                    anL.Append(aCopyE);
                    myDegenerated.Append(aCopyE);
                }
                else
                {
                    anL = aDegE.Bound(anEdge, new NCollection_List<TopoDS_Shape>());
                    anL.Append(anEdge);
                    myDegenerated.Append(anEdge);
                }
            }
        }
        if (!myDegenerated.IsEmpty())
        {
            var aSubs = new BRepTools_ReShape();
            var aDegF = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            bool isReplaced = false;
            anExp.Init(myShape, TopAbs_FACE);
            // Replace degenerated edge by its copies for different faces
            // First, for each face list of d.e. is created
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Shape aF = anExp.Current();
                var anExpE = new TopExp_Explorer(aF, TopAbs_EDGE);
                for (; anExpE.More(); anExpE.Next())
                {
                    TopoDS_Shape anE = anExpE.Current();
                    if (BRep_Tool.Degenerated(TopoDS.Edge(anE)))
                    {
                        if (!aDegF.Find(aF, out NCollection_List<TopoDS_Shape> anL))
                        {
                            anL = aDegF.Bound(aF, new NCollection_List<TopoDS_Shape>());
                        }
                        anL.Append(anE);
                    }
                }
            }
            //
            // Second, replace edges by copies using ReShape
            var aSubsF = new BRepTools_ReShape();
            // OCCT iterates its map in the hash order of the faces' TShape addresses; the port's map iterates in the
            // order the faces were bound (decision 18). Which face keeps the original degenerated edge may differ.
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aFIter in aDegF)
            {
                aSubs.Clear();
                isReplaced = false;
                TopoDS_Shape aF = aFIter.Key;
                NCollection_List<TopoDS_Shape> aDEL = aFIter.Value;
                var anEIter = new NCollection_List<TopoDS_Shape>.Iterator(aDEL);
                for (; anEIter.More(); anEIter.Next())
                {
                    TopoDS_Shape anE = anEIter.Value();
                    if (aDegE.IsBound(anE))
                    {
                        NCollection_List<TopoDS_Shape> aCEL = aDegE.ChangeFind(anE);
                        var anIt = new NCollection_List<TopoDS_Shape>.Iterator(aCEL);
                        for (; anIt.More(); anIt.Next())
                        {
                            if (anIt.Value().IsEqual(anE))
                            {
                                // First occurrence of initial deg. edge is not replaced
                                aCEL.Remove(anIt);
                                break;
                            }
                            if (anIt.Value().Orientation() == anE.Orientation())
                            {
                                // All other occurrences of anE are replaced by any copy
                                // with suitable orientation
                                isReplaced = true;
                                aSubs.Replace(anE, anIt.Value());
                                aCEL.Remove(anIt);
                                break;
                            }
                        }
                    }
                }
                if (isReplaced)
                {
                    TopoDS_Shape aNF = aSubs.InternalApply(aF);
                    aSubsF.Replace(aF, aNF);
                    if (myHist is null)
                    {
                        myHist = aSubs.InternalHistory();
                    }
                    else
                    {
                        myHist.Merge(aSubs.InternalHistory());
                    }
                    myShape = aSubsF.InternalApply(myShape);
                    myHist.Merge(aSubsF.InternalHistory());
                    // Pair aF->aNF is in history after first replacing of edge by aNF = aSubs.Apply(aF)
                    // After merging history for replacing faces, modified list for aF contains two exemplar of
                    // aNF So, using ReplaceModified clears modified list for aF and leaves only one exemplar of
                    // aNF
                    myHist.ReplaceModified(aF, aNF);
                    aSubsF.Clear();
                }
            }
        }

        KeepUnchangedGenerating(ref myShape);
    }

    // used in CheckValidity to find out is there
    // intersection between curve and axe of revolution
    private static bool IsIntersect(Adaptor3d_Curve theC, in gp_Ax1 theAxe)
    {
        var anAxis = new gp_Lin(theAxe);
        // Quick test for circle
        if (theC.GetType() == GeomAbs_Circle)
        {
            gp_Circ aCirc = theC.Circle();
            gp_Pnt aCentr = aCirc.Location();
            double anR2 = aCirc.Radius();
            anR2 -= Precision.Confusion();
            anR2 *= anR2;
            if (anAxis.SquareDistance(aCentr) > anR2)
            {
                return false;
            }
        }
        var aL = new Geom_Line(anAxis);
        var aLin = new GeomAdaptor_Curve(aL);
        double aParTol = theC.Resolution(Precision.Confusion());
        double aParF = theC.FirstParameter() + aParTol, aParL = theC.LastParameter() - aParTol;

        var anExtr = new Extrema_ExtCC(theC, aLin);
        anExtr.Perform();
        if (anExtr.IsDone() && anExtr.NbExt() > 0)
        {
            for (int i = 1; i <= anExtr.NbExt(); i++)
            {
                if (anExtr.SquareDistance(i) > Precision.SquareConfusion())
                {
                    continue;
                }
                anExtr.Points(i, out Extrema_POnCurv aP1, out Extrema_POnCurv _);
                if ((aParF < aP1.Parameter()) && (aP1.Parameter() < aParL))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>False if an edge of <paramref name="theShape"/> crosses the axis (edges lying on it are valid).</summary>
    protected bool CheckValidity(in TopoDS_Shape theShape, in gp_Ax1 theA)
    {
        var anExp = new TopExp_Explorer(theShape, TopAbs_EDGE);
        bool IsValid = true;
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            if (BRep_Tool.Degenerated(anE))
            {
                continue;
            }

            Geom_Curve C = BRep_Tool.Curve(anE, out TopLoc_Location L, out double First, out double Last)!;
            gp_Trsf Tr = L.Transformation();
            C = (Geom_Curve)C.Copy();
            C = new Geom_TrimmedCurve(C, First, Last);
            C.Transform(Tr);

            var HC = new GeomAdaptor_Curve();
            HC.Load(C, First, Last);

            // Checking coincidence axe of revolution and basis curve
            // This code is taken directly from GeomAdaptor_SurfaceOfRevolution
            int Ratio = 1;
            double Dist;
            gp_Pnt PP;
            do
            {
                PP = HC.Value(First + (Last - First) / Ratio);
                Dist = new gp_Lin(theA).Distance(PP);
                Ratio++;
            } while (Dist < Precision.Confusion() && Ratio < 100);
            //
            if (Ratio >= 100) // edge coincides with axes
            {
                IsValid = true; // Such edges are allowed by revol algo and treated
                                // by special way, so they must be considered as valid
            }
            else
            {
                IsValid = !IsIntersect(HC, theA);
            }
            if (!IsValid)
            {
                break;
            }
        }
        //
        return IsValid;
    }

    internal override TopoDS_Shape InternalFirstShape() => FromSwept(myRevol.FirstShape());

    internal override TopoDS_Shape InternalLastShape() => FromSwept(myRevol.LastShape());

    /// <summary>The list of shapes generated from the shape <paramref name="S"/>.</summary>
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        myGenerated.Clear();
        TopoDS_Shape aSS = ToSwept(S);

        if (!myRevol.IsUsed(aSS))
        {
            return myGenerated;
        }

        TopoDS_Shape aGS = myRevol.Shape(aSS);
        if (!aGS.IsNull())
        {
            if (BRepTools_History.IsSupportedType(aGS))
            {
                if (aGS.ShapeType() == TopAbs_EDGE)
                {
                    bool isDeg = BRep_Tool.Degenerated(TopoDS.Edge(aGS));
                    if (isDeg)
                    {
                        var anIt = new NCollection_List<TopoDS_Shape>.Iterator(myDegenerated);
                        for (; anIt.More(); anIt.Next())
                        {
                            if (aGS.IsSame(anIt.Value()))
                            {
                                myGenerated.Append(FromSwept(aGS));
                                if (myHist is not null)
                                {
                                    foreach (TopoDS_Shape aM in myHist.Modified(aGS))
                                    {
                                        myGenerated.Append(FromSwept(aM));
                                    }
                                    return myGenerated;
                                }
                            }
                        }
                        return myGenerated;
                    }
                }
                //
                if (myHist is null)
                {
                    myGenerated.Append(FromSwept(aGS));
                    return myGenerated;
                }
                //
                IBentoList<TopoDS_Shape> aModified = myHist.Modified(aGS);
                if (aModified.Count == 0)
                {
                    myGenerated.Append(FromSwept(aGS));
                    return myGenerated;
                }
                //
                foreach (TopoDS_Shape aM in aModified)
                {
                    myGenerated.Append(FromSwept(aM));
                }
            }
        }
        return myGenerated;
    }

    /// <summary>True if the shape <paramref name="S"/> has been deleted.</summary>
    public override bool IsDeleted(in TopoDS_Shape S)
    {
        return !myRevol.IsUsed(ToSwept(S));
    }

    /// <summary>The shape at the beginning of the revolution generated with <paramref name="theShape"/>, a sub-shape of the generating shape (frozen).</summary>
    public TopoDS_Shape FirstShape(in TopoDS_Shape theShape) => Published(InternalFirstShape(theShape));

    /// <summary><see cref="FirstShape(in TopoDS_Shape)"/> without freezing.</summary>
    internal TopoDS_Shape InternalFirstShape(in TopoDS_Shape theShape) => FromSwept(myRevol.FirstShape(ToSwept(theShape)));

    /// <summary>The shape at the end of the revolution generated with <paramref name="theShape"/>, a sub-shape of the generating shape (frozen).</summary>
    public TopoDS_Shape LastShape(in TopoDS_Shape theShape) => Published(InternalLastShape(theShape));

    /// <summary><see cref="LastShape(in TopoDS_Shape)"/> without freezing.</summary>
    internal TopoDS_Shape InternalLastShape(in TopoDS_Shape theShape) => FromSwept(myRevol.LastShape(ToSwept(theShape)));

    /// <summary>True if the result has degenerated edges.</summary>
    public bool HasDegenerated() => !myDegenerated.IsEmpty();

    /// <summary>The degenerated edges of the result (frozen).</summary>
    public IBentoList<TopoDS_Shape> Degenerated() => PublishedList(myDegenerated);

    /// <summary>The list of degenerated edges, unfrozen (OCCT's const reference).</summary>
    internal NCollection_List<TopoDS_Shape> InternalDegenerated() => myDegenerated;
}
