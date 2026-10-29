// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Extrema_ExtFlag;
using static Occt.IntCurveSurface_TransitionOnCurve;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>Classifies a point in a solid explored by a <see cref="BRepClass3d_SolidExplorer"/>.</summary>
/// <remarks>A working object (decision 21). It only reads the shape.</remarks>
public class BRepClass3d_SClassifier
{
    private TopoDS_Face myFace = new();

    //! This variable stores information about algorithm internal state.
    //! Type of this variable differs from TopAbs_State since it contains
    //! additional information about error status.
    //! 1 - Error inside of the algorithm.
    //! 2 - ON.
    //! 3 - IN.
    //! 4 - OUT.
    private int myState;

    /// <summary>An empty classifier.</summary>
    public BRepClass3d_SClassifier()
    {
        myState = 0;
    }

    /// <summary>Classifies the point <paramref name="P"/> with the tolerance <paramref name="Tol"/> in the solid of <paramref name="S"/>.</summary>
    public BRepClass3d_SClassifier(BRepClass3d_SolidExplorer S, in gp_Pnt P, double Tol)
    {
        if (S.Reject(P))
        {
            myState = 3; //-- in ds solid case without face
        }
        else
        {
            Perform(S, P, Tol);
        }
    }

    /// <summary>
    /// Classifies a point at infinity in the solid of <paramref name="aSE"/> (OUT for a finite solid, IN for a hole in
    /// space): the reversed normal at random points of the faces is intersected with the solid, until the nearest
    /// intersection is inside a face and not tangent. The tolerance is unused.
    /// </summary>
    public void PerformInfinitePoint(BRepClass3d_SolidExplorer aSE, double Tol)
    {
        // Take a normal to the first extracted face in its random inner point
        // and intersect this reversed normal with the faces of the solid.
        // If the min.par.-intersection point is
        //  a) inner point of a face
        //  b) transition is not TANGENT
        //     (the line does not touch the face but pierces it)
        // then set <myState> to IN or OUT according to transition
        // else take the next random point inside the min.par.-intersected face
        // and continue

        if (aSE.Reject(new gp_Pnt(0, 0, 0)))
        {
            myState = 3; //-- in ds solid case without face
            return;
        }

        //
        //------------------------------------------------------------
        // 1
        bool bFound;
        double aParam, aU = 0.0, aV = 0.0;
        gp_Pnt aPoint = new();
        gp_Dir aDN = new();

        var aRandomGenerator = new math_BullardGenerator();
        myFace.Nullify();
        myState = 2;

        // Collect faces in sequence to iterate
        var aFaces = new List<TopoDS_Face>();
        for (aSE.InitShell(); aSE.MoreShell(); aSE.NextShell())
        {
            for (aSE.InitFace(); aSE.MoreFace(); aSE.NextFace())
            {
                aFaces.Add(aSE.CurrentFace());
            }
        }

        // iteratively try up to 10 probing points from each face
        const int NB_MAX_POINTS_PER_FACE = 10;
        for (int itry = 0; itry < NB_MAX_POINTS_PER_FACE; itry++)
        {
            for (int iFace = 0; iFace < aFaces.Count; ++iFace)
            {
                TopoDS_Face aF = aFaces[iFace];

                TopAbs_State aState = TopAbs_OUT;
                IntCurveSurface_TransitionOnCurve aTransition = IntCurveSurface_Tangent;

                aParam = 0.1 + 0.8 * aRandomGenerator.NextReal(); // random number in range [0.1, 0.9]
                bFound = BRepClass3d_SolidExplorer.FindAPointInTheFace(aF, ref aPoint, ref aU, ref aV, ref aParam);
                if (!bFound || !FaceNormal(aF, aU, aV, ref aDN))
                {
                    continue;
                }

                var aLin = new gp_Lin(aPoint, -aDN);
                double parmin = RealLast();
                for (aSE.InitShell(); aSE.MoreShell(); aSE.NextShell())
                {
                    if (!aSE.RejectShell(aLin))
                    {
                        for (aSE.InitFace(); aSE.MoreFace(); aSE.NextFace())
                        {
                            if (!aSE.RejectFace(aLin))
                            {
                                TopoDS_Shape aLocalShape = aSE.CurrentFace();
                                TopoDS_Face CurFace = TopoDS.Face(aLocalShape);
                                IntCurvesFace_Intersector Intersector3d = aSE.Intersector(CurFace);
                                Intersector3d.Perform(aLin, -RealLast(), parmin);

                                if (Intersector3d.IsDone())
                                {
                                    if (Intersector3d.NbPnt() != 0)
                                    {
                                        int imin = 1;
                                        for (int i = 2; i <= Intersector3d.NbPnt(); i++)
                                        {
                                            if (Intersector3d.WParameter(i) < Intersector3d.WParameter(imin))
                                            {
                                                imin = i;
                                            }
                                        }

                                        parmin = Intersector3d.WParameter(imin);
                                        aState = Intersector3d.State(imin);
                                        aTransition = Intersector3d.Transition(imin);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        myState = 1;
                    }
                } // end of loop on the whole solid

                if (aState == TopAbs_IN)
                {
                    if (aTransition == IntCurveSurface_Out)
                    {
                        //-- The line is going from inside the solid to outside
                        //-- the solid.
                        myState = 3; //-- IN --
                        return;
                    }
                    else if (aTransition == IntCurveSurface_In)
                    {
                        myState = 4; //-- OUT --
                        return;
                    }
                }
            } // iteration by faces
        } // iteration by points
    }

    /// <summary>Classifies the point <paramref name="P"/> with the tolerance <paramref name="Tol"/> in the solid of <paramref name="SolidExplorer"/>.</summary>
    public void Perform(BRepClass3d_SolidExplorer SolidExplorer, in gp_Pnt P, double Tol)
    {
        if (SolidExplorer.Reject(P))
        {
            // Solid without faces => the whole space. Always in.
            myState = 3; // IN
            return;
        }

        NCollection_UBTree<int, Bnd_Box> aTree = SolidExplorer.GetTree();
        NCollection_IndexedMap<TopoDS_Shape> aMapEV = SolidExplorer.GetMapEV();

        // Vertices/Edges vs Point.
        var aSelectorPoint = new BRepClass3d_BndBoxTreeSelectorPoint(aMapEV);
        aSelectorPoint.SetCurrentPoint(P);

        int SelsVE = 0;
        SelsVE = aTree.Select(aSelectorPoint);

        if (SelsVE > 0)
        {
            // The point P lays inside the tolerance area of vertices/edges => return ON state.
            myState = 2; // ON.
            return;
        }

        var mapEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(SolidExplorer.GetShape(), TopAbs_EDGE, TopAbs_FACE, mapEF);

        var aSelectorLine = new BRepClass3d_BndBoxTreeSelectorLine(aMapEV);

        myFace.Nullify();
        myState = 0;

        gp_Lin L = new();
        double Par = 0.0;
        // We compute the intersection between the line built in the Solid Explorer and the shape.
        //-- --------------------------------------------------------------------------------
        // Calculate intersection with the face closest to the direction of bounding boxes
        // by priority so that to have the smallest possible parmin.
        // Optimization to produce as much as possible rejections with other faces.
        int iFlag;

        // If found line passes through a bound of any face, it means that the line
        // is not found properly and it is necessary to repeat whole procedure.
        // That's why the main loop while is added.
        bool isFaultyLine = true;
        int anIndFace = 0;
        double parmin = 0.0;
        while (isFaultyLine)
        {
            if (anIndFace == 0)
            {
                iFlag = SolidExplorer.Segment(P, ref L, ref Par);
            }
            else
            {
                iFlag = SolidExplorer.OtherSegment(P, ref L, ref Par);
            }

            int aCurInd = SolidExplorer.GetFaceSegmentIndex();

            if (aCurInd > anIndFace)
            {
                anIndFace = aCurInd;
            }
            else
            {
                myState = 1; // Faulty.
                return;
            }

            if (iFlag == 1)
            {
                // IsOnFace iFlag==1 i.e face is Infinite
                myState = 2; // ON.
                return;
            }

            if (iFlag == 2)
            {
                myState = 4; // OUT.
                return;
            }

            // Check if the point is ON surface but OUT of the face.
            // Just skip this face because it is bad for classification.
            if (iFlag == 3)
            {
                continue;
            }

            isFaultyLine = false;
            parmin = RealLast();

            double NearFaultPar = RealLast(); // Parameter on line.
            aSelectorLine.ClearResults();
            aSelectorLine.SetCurrentLine(L, Par);
            int SelsEVL = 0;
            SelsEVL = aTree.Select(aSelectorLine); // SelsEE > 0 => Line/Edges & Line/Vertex intersection
            if (SelsEVL > 0)
            {
                // Line and edges / vertices interference.
                int VLInterNb = aSelectorLine.GetNbVertParam();
                var LVInts = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                for (int i = 1; i <= VLInterNb; i++)
                {
                    // Line and vertex.
                    aSelectorLine.GetVertParam(i, out TopoDS_Vertex V, out double LP);

                    LVInts.Add(V);
                    if (Math.Abs(LP) < Math.Abs(NearFaultPar))
                    {
                        NearFaultPar = LP;
                    }
                }

                double param = 0.0;
                TopoDS_Edge EE;
                double Lpar = RealLast();
                for (int i = 1; i <= aSelectorLine.GetNbEdgeParam(); i++)
                {
                    // Line and edge.
                    aSelectorLine.GetEdgeParam(i, out EE, out param, out Lpar);
                    NCollection_List<TopoDS_Shape> ffs = mapEF.FindFromKey(EE); // ffs size == 2
                    if (ffs.Extent() != 2)
                    {
                        continue;
                    }

                    TopoDS_Face f1 = TopoDS.Face(ffs.First());
                    TopoDS_Face f2 = TopoDS.Face(ffs.Last());
                    TopExp.Vertices(EE, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    if (LVInts.Contains(V1) || LVInts.Contains(V2))
                    {
                        continue;
                    }

                    IntCurveSurface_TransitionOnCurve tran = IntCurveSurface_Tangent;
                    int Tst = GetTransi(f1, f2, EE, param, L, ref tran);
                    if (Tst == 1 && Math.Abs(Lpar) < Math.Abs(parmin))
                    {
                        parmin = Lpar;
                        Trans(parmin, ref tran, ref myState);
                    }
                    else if (Math.Abs(Lpar) < Math.Abs(NearFaultPar))
                    {
                        NearFaultPar = Lpar;
                    }
                }
            }

            for (SolidExplorer.InitShell(); SolidExplorer.MoreShell() && !isFaultyLine;
                 SolidExplorer.NextShell())
            {
                if (!SolidExplorer.RejectShell(L))
                {
                    for (SolidExplorer.InitFace(); SolidExplorer.MoreFace() && !isFaultyLine;
                         SolidExplorer.NextFace())
                    {
                        if (!SolidExplorer.RejectFace(L))
                        {
                            TopoDS_Shape aLocalShape = SolidExplorer.CurrentFace();
                            TopoDS_Face f = TopoDS.Face(aLocalShape);
                            IntCurvesFace_Intersector Intersector3d = SolidExplorer.Intersector(f);

                            // Prolong segment, since there are cases when
                            // the intersector does not find intersection points with the original
                            // segment due to rough triangulation of a parameterized surface.
                            double addW = Max(10 * Tol, 0.01 * Par);
                            double AddW = addW;

                            Bnd_Box aBoxF = Intersector3d.Bounding();

                            // The box must be finite in order to correctly prolong the segment to its bounds.
                            if (!aBoxF.IsVoid() && !aBoxF.IsWhole())
                            {
                                aBoxF.Get(out double _, out double _, out double _, out double _, out double _, out double _);

                                double boxaddW = GetAddToParam(L, Par, aBoxF);
                                addW = Max(addW, boxaddW);
                            }

                            double minW = -AddW;
                            double maxW = Min(Par * 10, Par + addW);
                            Intersector3d.Perform(L, minW, maxW);
                            if (Intersector3d.IsDone())
                            {
                                if (Intersector3d.NbPnt() == 0)
                                {
                                    if (Intersector3d.IsParallel())
                                    {
                                        // Check distance between surface and point
                                        var aBAS = new BRepAdaptor_Surface(f, false);
                                        var aProj = new Extrema_ExtPS(P,
                                                                      aBAS,
                                                                      Precision.PConfusion(),
                                                                      Precision.PConfusion(),
                                                                      Extrema_ExtFlag_MIN);
                                        if (aProj.IsDone() && aProj.NbExt() > 0)
                                        {
                                            int i, indmin = 0;
                                            double d = RealLast();
                                            for (i = 1; i <= aProj.NbExt(); ++i)
                                            {
                                                if (aProj.SquareDistance(i) < d)
                                                {
                                                    d = aProj.SquareDistance(i);
                                                    indmin = i;
                                                }
                                            }

                                            if (indmin > 0)
                                            {
                                                if (d <= Tol * Tol)
                                                {
                                                    Extrema_POnSurf aPonS = aProj.Point(indmin);
                                                    aPonS.Parameter(out double anU, out double anV);
                                                    gp_Pnt2d aP2d = new(anU, anV);
                                                    TopAbs_State aSt = Intersector3d.ClassifyUVPoint(aP2d);
                                                    if (aSt == TopAbs_IN || aSt == TopAbs_ON)
                                                    {
                                                        myState = 2;
                                                        myFace = f;
                                                        parmin = 0.0;
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                for (int i = 1; i <= Intersector3d.NbPnt(); i++)
                                {
                                    if (Math.Abs(Intersector3d.WParameter(i))
                                        < Math.Abs(parmin) - Precision.PConfusion())
                                    {
                                        parmin = Intersector3d.WParameter(i);
                                        TopAbs_State aState = Intersector3d.State(i);
                                        if (Math.Abs(parmin) <= Tol)
                                        {
                                            myState = 2;
                                            myFace = f;
                                            break;
                                        }
                                        //  Treatment of case TopAbs_ON separately.
                                        else if (aState == TopAbs_IN)
                                        {
                                            //-- The intersection point between the line and a face F
                                            // -- of the solid is in the face F

                                            IntCurveSurface_TransitionOnCurve tran = Intersector3d.Transition(i);
                                            if (tran == IntCurveSurface_Tangent)
                                            {
                                                continue; // ignore this point
                                            }

                                            Trans(parmin, ref tran, ref myState);
                                            myFace = f;
                                        }
                                        // If the state is TopAbs_ON, it is necessary to chose
                                        // another line and to repeat the whole procedure.
                                        else if (aState == TopAbs_ON)
                                        {
                                            isFaultyLine = true;
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        //-- No point has been found by the Intersector3d.
                                        //-- Or a Point has been found with a greater parameter.
                                    }
                                } //-- loop by intersection points

                                if (myState == 2)
                                {
                                    break;
                                }
                            } //-- Face has not been rejected
                            else
                            {
                                myState = 1;
                            }
                        }
                    } //-- Exploration of the faces

                    if (myState == 2)
                    {
                        break;
                    }
                } //-- Shell has not been rejected
                else
                {
                    myState = 1;
                }
            } //-- Exploration of the shells

            if (NearFaultPar != RealLast()
                && Math.Abs(parmin) >= Math.Abs(NearFaultPar) - Precision.PConfusion())
            {
                isFaultyLine = true;
            }
        }
    }

    /// <summary>The result of the classification (OUT also when the algorithm failed).</summary>
    public TopAbs_State State()
    {
        if (myState == 2)
        {
            return (TopAbs_ON);
        }
        else if (myState == 3)
        {
            return (TopAbs_IN);
        }
        else if (myState == 4)
        {
            return (TopAbs_OUT);
        }

        // return OUT state when there is an error during execution.
        return (TopAbs_OUT);
    }

    /// <summary>The face that decided the classification; when the state is ON, the face holding the point.</summary>
    public TopoDS_Face Face() => (myFace);

    /// <summary>True if the classification failed (OCCT's name: the state is then OUT).</summary>
    public bool Rejected() => (myState == 1);

    /// <summary>True when the point is on a face.</summary>
    public bool IsOnAFace() => (myState == 2);

    /// <summary>Sets the state to IN.</summary>
    protected void ForceIn() => myState = 3;

    /// <summary>Sets the state to OUT.</summary>
    protected void ForceOut() => myState = 4;

    private static double GetAddToParam(in gp_Lin L, double P, in Bnd_Box B)
    {
        B.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        Span<double> x = [aXmin, aXmax];
        Span<double> y = [aYmin, aYmax];
        Span<double> z = [aZmin, aZmax];
        int i = 0, j = 0, k = 0;
        double Par = P;
        for (i = 0; i < 2; i++)
        {
            for (j = 0; j < 2; j++)
            {
                for (k = 0; k < 2; k++)
                {
                    double X = Math.Abs(x[i] - L.Location().X());
                    double Y = Math.Abs(y[j] - L.Location().Y());
                    double Z = Math.Abs(z[k] - L.Location().Z());
                    if (X < 1.0e+20 && Y < 1.0e+20 && Z < 1.0e+20)
                    {
                        gp_Pnt aP = new(x[i], y[j], z[k]);
                        double par = ElCLib.Parameter(L, aP);
                        if (par > Par)
                        {
                            Par = par;
                        }
                    }
                    else
                    {
                        return 1.0e+20;
                    }
                }
            }
        }

        return Par - P;
    }

    // modified by NIZHNY-MKK  Mon Jun 21 15:13:40 2004
    private static bool FaceNormal(in TopoDS_Face aF, double U, double V, ref gp_Dir aDN)
    {
        Geom_Surface aS = BRep_Tool.Surface(aF)!;
        aS.D1(U, V, out gp_Pnt _, out gp_Vec aD1U, out gp_Vec aD1V);
        gp_Vec aN = aD1U.Crossed(aD1V);
        if (aN.Magnitude() <= gp.Resolution())
        {
            return false;
        }

        aN.Normalize();
        aDN.SetXYZ(aN.XYZ());
        if (aF.Orientation() == TopAbs_REVERSED)
        {
            aDN.Reverse();
        }

        return true;
    }

    private static bool GetNormalOnFaceBound(in TopoDS_Edge E, in TopoDS_Face F, double param, ref gp_Dir OutDir)
    {
        Geom2d_Curve? c2d = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
        if (c2d is null)
        {
            return false;
        }

        if (param < f || param > l)
        {
            return false;
        }

        c2d.D0(param, out gp_Pnt2d P2d);
        return FaceNormal(F, P2d.X(), P2d.Y(), ref OutDir);
    }

    // gets transition of line <L> passing through/near the edge <e> of faces <f1>, <f2>. <param> is
    //  a parameter on the edge where the minimum distance between <l> and <e> was found
    private static int GetTransi(in TopoDS_Face f1,
                                 in TopoDS_Face f2,
                                 in TopoDS_Edge e,
                                 double param,
                                 in gp_Lin L,
                                 ref IntCurveSurface_TransitionOnCurve trans)
    {
        // return statuses:
        // 1 => OK
        // 0 => skip
        //-1 => probably a faulty line
        gp_Dir nf1 = new(), nf2 = new();
        if (!GetNormalOnFaceBound(e, f1, param, ref nf1))
        {
            return -1;
        }

        if (!GetNormalOnFaceBound(e, f2, param, ref nf2))
        {
            return -1;
        }

        gp_Dir LDir = L.Direction();

        if (Math.Abs(LDir.Dot(nf1)) < Precision.Angular()
            || Math.Abs(LDir.Dot(nf2)) < Precision.Angular())
        {
            // line is orthogonal to normal(s)
            // trans = IntCurveSurface_Tangent;
            return -1;
        }

        if (nf1.IsParallel(nf2, Precision.Angular()))
        {
            double angD = nf1.Dot(LDir);
            if (Math.Abs(angD) < Precision.Angular())
            {
                return -1;
            }
            else if (angD > 0)
            {
                trans = IntCurveSurface_Out;
            }
            else
            {
                // angD < -Precision::Angular())
                trans = IntCurveSurface_In;
            }

            return 1;
        }

        gp_Vec N = new(nf1 ^ nf2);

        //proj LDir on the plane defined by nf1/nf2 directions
        gp_Dir ProjL = new((N.XYZ() ^ LDir.XYZ()) ^ N.XYZ());

        double fAD = nf1.Dot(ProjL);
        double sAD = nf2.Dot(ProjL);

        if (fAD < -Precision.Angular() && sAD < -Precision.Angular())
        {
            trans = IntCurveSurface_In;
        }
        else if (fAD > Precision.Angular() && sAD > Precision.Angular())
        {
            trans = IntCurveSurface_Out;
        }
        else
        {
            return 0;
        }

        return 1;
    }

    private static void Trans(double parmin, ref IntCurveSurface_TransitionOnCurve tran, ref int state)
    {
        // if parmin is negative we should reverse transition
        if (parmin < 0)
        {
            tran = (tran == IntCurveSurface_Out ? IntCurveSurface_In : IntCurveSurface_Out);
        }

        if (tran == IntCurveSurface_Out)
        {
            //-- The line is going from inside the solid to outside
            //-- the solid.
            state = 3; // IN
        }
        else
        {
            state = 4; // OUT
        }
    }
}
