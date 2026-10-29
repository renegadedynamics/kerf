// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtPS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenExtPS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Extrema_ElementType;
using static Occt.Extrema_ExtAlgo;
using static Occt.Extrema_ExtFlag;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// All the extrema of the distance between a point and a surface, found by sampling: starting from the extrema of the
/// distance to a grid of points (a mesh of nodes, edges and faces, or a tree of spheres), each is refined on the
/// surface with math_FunctionSetRoot.
/// </summary>
/// <remarks>
/// The sampling must be fine enough that if there are N extrema on the surface, there are N extrema on the grid. The
/// surface adaptor is kept, as OCCT keeps a pointer: it must outlive the algorithm. The grid (or the tree) is built
/// once and reused for the next points, until <see cref="Initialize(Adaptor3d_Surface, int, int, double, double)"/>
/// or a change of algorithm.
/// </remarks>
public class Extrema_GenExtPS
{
    // The selectors of the sphere tree (Extrema_GenExtPS.cxx). OCCT keeps references to the sphere array and to the
    // caller's solution; the port keeps the array and the solution itself, which Sphere() returns.
    private abstract class Bnd_SphereUBTreeSelector : NCollection_UBTree<int, Bnd_Sphere>.Selector
    {
        protected gp_Pnt myXYZ;
        protected readonly NCollection_HArray1<Bnd_Sphere> mySphereArray;
        protected Bnd_Sphere mySol;

        protected Bnd_SphereUBTreeSelector(NCollection_HArray1<Bnd_Sphere> theSphereArray, in Bnd_Sphere theSol)
        {
            myXYZ = new gp_Pnt(0, 0, 0);
            mySphereArray = theSphereArray;
            mySol = theSol;
        }

        public void DefineCheckPoint(in gp_Pnt theXYZ) => myXYZ = theXYZ;

        public Bnd_Sphere Sphere() => mySol;
    }

    private sealed class Bnd_SphereUBTreeSelectorMin : Bnd_SphereUBTreeSelector
    {
        private double myMinDist;

        public Bnd_SphereUBTreeSelectorMin(NCollection_HArray1<Bnd_Sphere> theSphereArray, in Bnd_Sphere theSol)
            : base(theSphereArray, theSol)
        {
            myMinDist = RealLast();
        }

        public void SetMinDist(double theMinDist) => myMinDist = theMinDist;

        public double MinDist() => myMinDist;

        public override bool Reject(in Bnd_Sphere theBnd)
        {
            // myMinDist is decreased each time a nearer object is found
            return theBnd.IsOut(myXYZ.XYZ(), ref myMinDist);
        }

        public override bool Accept(in int theInd)
        {
            Bnd_Sphere aSph = mySphereArray.Value(theInd);
            double aCurDist;

            //    if ( (aCurDist = aSph.SquareDistance(myXYZ.XYZ())) < mySol.SquareDistance(myXYZ.XYZ()) )
            if ((aCurDist = aSph.Distance(myXYZ.XYZ())) < mySol.Distance(myXYZ.XYZ()))
            {
                mySol = aSph;
                if (aCurDist < myMinDist)
                {
                    myMinDist = aCurDist;
                }

                return true;
            }

            return false;
        }
    }

    private sealed class Bnd_SphereUBTreeSelectorMax : Bnd_SphereUBTreeSelector
    {
        private double myMaxDist;

        public Bnd_SphereUBTreeSelectorMax(NCollection_HArray1<Bnd_Sphere> theSphereArray, in Bnd_Sphere theSol)
            : base(theSphereArray, theSol)
        {
            // OCCT: Reject reads this as a maximal distance (Bnd_Sphere::IsOut), so with 0 every sphere is rejected
            // and the search for the maximum starts from the first grid node
            myMaxDist = 0;
        }

        public void SetMaxDist(double theMaxDist) => myMaxDist = theMaxDist;

        public double MaxDist() => myMaxDist;

        public override bool Reject(in Bnd_Sphere theBnd)
        {
            // myMaxDist is decreased each time a nearer object is found
            return theBnd.IsOut(myXYZ.XYZ(), ref myMaxDist);
        }

        public override bool Accept(in int theInd)
        {
            Bnd_Sphere aSph = mySphereArray.Value(theInd);
            double aCurDist;

            //    if ( (aCurDist = aSph.SquareDistance(myXYZ.XYZ())) > mySol.SquareDistance(myXYZ.XYZ()) )
            if ((aCurDist = aSph.Distance(myXYZ.XYZ())) > mySol.Distance(myXYZ.XYZ()))
            {
                mySol = aSph;
                if (aCurDist > myMaxDist)
                {
                    myMaxDist = aCurDist;
                }

                return true;
            }

            return false;
        }
    }

    private bool myDone;
    private bool myInit;
    private double myumin;
    private double myusup;
    private double myvmin;
    private double myvsup;
    private int myusample;
    private int myvsample;
    private double mytolu;
    private double mytolv;

    private readonly NCollection_Array2<Extrema_POnSurfParams> myPoints = new();
    private NCollection_UBTree<int, Bnd_Sphere>? mySphereUBTree;
    private NCollection_HArray1<Bnd_Sphere>? mySphereArray;
    private readonly Extrema_FuncPSNorm myF;
    private Adaptor3d_Surface? myS;
    private Extrema_ExtFlag myFlag;
    private Extrema_ExtAlgo myAlgo;
    private NCollection_HArray1<double>? myUParams;
    private NCollection_HArray1<double>? myVParams;
    private readonly NCollection_Array2<Extrema_POnSurfParams> myFacePntParams = new();
    private readonly NCollection_Array2<Extrema_POnSurfParams> myUEdgePntParams = new();
    private readonly NCollection_Array2<Extrema_POnSurfParams> myVEdgePntParams = new();
    private Extrema_POnSurfParams myGridParam = new();

    /*-----------------------------------------------------------------------------
    Function:
       Find all extremum distances between point P and surface
      S using sampling (NbU,NbV).

    Method:
       The algorithm bases on the hypothesis that sampling is precise enough,
      if there exist N extreme distances between the point and the surface,
      so there also exist N extrema between the point and the grid.
      So, the algorithm consists in starting from extrema of the grid to find the
      extrema of the surface.
      The extrema are calculated by the algorithm math_FunctionSetRoot with the
      following arguments:
      - F: Extrema_FuncExtPS created from P and S,
      - UV: math_Vector the components which of are parameters of the extremum on the
        grid,
      - Tol: std::min(TolU,TolV), (Prov.:math_FunctionSetRoot does not authorize a vector)
      - UVinf: math_Vector the components which of are lower limits of u and v,
      - UVsup: math_Vector the components which of are upper limits of u and v.
    -----------------------------------------------------------------------------*/

    /// <summary>An empty algorithm (<see cref="Initialize(Adaptor3d_Surface, int, int, double, double)"/> sets the surface).</summary>
    public Extrema_GenExtPS()
    {
        myumin = 0.0;
        myusup = 0.0;
        myvmin = 0.0;
        myvsup = 0.0;
        myusample = 0;
        myvsample = 0;
        mytolu = 0.0;
        mytolv = 0.0;
        myS = null;
        myF = new Extrema_FuncPSNorm();
        myDone = false;
        myInit = false;
        myFlag = Extrema_ExtFlag_MINMAX;
        myAlgo = Extrema_ExtAlgo_Grad;
    }

    /// <summary>
    /// Computes all the extrema of the distance between P and S on S's whole range, starting from a sampling NbU x NbV;
    /// TolU and TolV stop the iterations.
    /// </summary>
    public Extrema_GenExtPS(in gp_Pnt P, Adaptor3d_Surface S, int NbU, int NbV, double TolU, double TolV,
        Extrema_ExtFlag F = Extrema_ExtFlag_MINMAX, Extrema_ExtAlgo A = Extrema_ExtAlgo_Grad)
    {
        myF = new Extrema_FuncPSNorm(P, S);
        myFlag = F;
        myAlgo = A;
        Initialize(S, NbU, NbV, TolU, TolV);
        Perform(P);
    }

    /// <summary>
    /// Computes all the extrema of the distance between P and S on [Umin, Usup] x [Vmin, Vsup], starting from a
    /// sampling NbU x NbV; TolU and TolV stop the iterations.
    /// </summary>
    public Extrema_GenExtPS(in gp_Pnt P, Adaptor3d_Surface S, int NbU, int NbV, double Umin, double Usup, double Vmin,
        double Vsup, double TolU, double TolV, Extrema_ExtFlag F = Extrema_ExtFlag_MINMAX,
        Extrema_ExtAlgo A = Extrema_ExtAlgo_Grad)
    {
        myF = new Extrema_FuncPSNorm(P, S);
        myFlag = F;
        myAlgo = A;
        Initialize(S, NbU, NbV, Umin, Usup, Vmin, Vsup, TolU, TolV);
        Perform(P);
    }

    public void Initialize(Adaptor3d_Surface S, int NbU, int NbV, double TolU, double TolV)
    {
        myumin = S.FirstUParameter();
        myusup = S.LastUParameter();
        myvmin = S.FirstVParameter();
        myvsup = S.LastVParameter();
        Initialize(S, NbU, NbV, myumin, myusup, myvmin, myvsup, TolU, TolV);
    }

    public void Initialize(Adaptor3d_Surface S, int NbU, int NbV, double Umin, double Usup, double Vmin, double Vsup,
        double TolU, double TolV)
    {
        myS = S;
        myusample = NbU;
        myvsample = NbV;
        mytolu = TolU;
        mytolv = TolV;
        myumin = Umin;
        myusup = Usup;
        myvmin = Vmin;
        myvsup = Vsup;

        if ((myusample < 2) || (myvsample < 2))
        {
            throw new Standard_OutOfRange();
        }

        myF.Initialize(S);

        mySphereUBTree = null;
        myUParams = null;
        myVParams = null;
        myInit = false;
    }

    private static void fillParams(NCollection_Array1<double> theKnots, int theDegree, double theParMin,
        double theParMax, ref NCollection_HArray1<double>? theParams, int theSample)
    {
        NCollection_DynamicArray<double> aParams = new();
        int i = 1;
        double aPrevPar = theParMin;
        aParams.Append(aPrevPar);
        // calculation the array of parametric points depending on the knots array variation and degree of
        // given surface
        for (; i < theKnots.Length() && theKnots[i] < (theParMax - Precision.PConfusion()); i++)
        {
            if (theKnots[i + 1] < theParMin + Precision.PConfusion())
            {
                continue;
            }

            double aStep = (theKnots[i + 1] - theKnots[i]) / Max(theDegree, 2);
            int k = 1;
            for (; k <= theDegree; k++)
            {
                double aPar = theKnots[i] + k * aStep;
                if (aPar > theParMax - Precision.PConfusion())
                {
                    break;
                }
                if (aPar > aPrevPar + Precision.PConfusion())
                {
                    aParams.Append(aPar);
                    aPrevPar = aPar;
                }
            }
        }
        aParams.Append(theParMax);
        int nbPar = aParams.Length();
        // in case of an insufficient number of points the grid will be built later
        if (nbPar < theSample)
        {
            return;
        }
        theParams = new NCollection_HArray1<double>(1, nbPar);
        for (i = 0; i < nbPar; i++)
        {
            theParams.SetValue(i + 1, aParams[i]);
        }
    }

    // Selection of points to build the grid, depending on the type of surface.
    private void GetGridPoints(Adaptor3d_Surface theSurf)
    {
        // creation parametric points for BSpline and Bezier surfaces
        // with taking into account of Degree and NbKnots of BSpline or Bezier geometry
        if (theSurf.GetType() == GeomAbs_OffsetSurface)
        {
            GetGridPoints(theSurf.BasisSurface());
        }
        // parametric points for BSpline surfaces
        else if (theSurf.GetType() == GeomAbs_BSplineSurface)
        {
            Geom_BSplineSurface? aBspl = theSurf.BSpline();
            if (aBspl is not null)
            {
                NCollection_Array1<double> aUKnots = aBspl.InternalUKnots();
                NCollection_Array1<double> aVKnots = aBspl.InternalVKnots();
                fillParams(aUKnots, aBspl.UDegree(), myumin, myusup, ref myUParams, myusample);
                fillParams(aVKnots, aBspl.VDegree(), myvmin, myvsup, ref myVParams, myvsample);
            }
        }
        // calculation parametric points for Bezier surfaces
        else if (theSurf.GetType() == GeomAbs_BezierSurface)
        {
            Geom_BezierSurface? aBezier = theSurf.Bezier();
            if (aBezier is null)
            {
                return;
            }

            NCollection_Array1<double> aUKnots = new(1, 2);
            NCollection_Array1<double> aVKnots = new(1, 2);
            aBezier.Bounds(out aUKnots[1], out aUKnots[2], out aVKnots[1], out aVKnots[2]);
            fillParams(aUKnots, aBezier.UDegree(), myumin, myusup, ref myUParams, myusample);
            fillParams(aVKnots, aBezier.VDegree(), myvmin, myvsup, ref myVParams, myvsample);
        }
        // creation points for surfaces based on BSpline or Bezier curves
        else if (theSurf.GetType() == GeomAbs_SurfaceOfRevolution || theSurf.GetType() == GeomAbs_SurfaceOfExtrusion)
        {
            NCollection_HArray1<double>? anArrKnots = null;
            int aDegree = 0;
            if (theSurf.BasisCurve().GetType() == GeomAbs_BSplineCurve)
            {
                Geom_BSplineCurve? aBspl = theSurf.BasisCurve().BSpline();
                if (aBspl is not null)
                {
                    anArrKnots = new NCollection_HArray1<double>(aBspl.InternalKnots());
                    aDegree = aBspl.Degree();
                }
            }
            if (theSurf.BasisCurve().GetType() == GeomAbs_BezierCurve)
            {
                Geom_BezierCurve? aBez = theSurf.BasisCurve().Bezier();
                if (aBez is not null)
                {
                    anArrKnots = new NCollection_HArray1<double>(1, 2);
                    anArrKnots.SetValue(1, aBez.FirstParameter());
                    anArrKnots.SetValue(2, aBez.LastParameter());
                    aDegree = aBez.Degree();
                }
            }
            if (anArrKnots is null)
            {
                return;
            }
            if (theSurf.GetType() == GeomAbs_SurfaceOfRevolution)
            {
                fillParams(anArrKnots.Array1(), aDegree, myvmin, myvsup, ref myVParams, myvsample);
            }
            else
            {
                fillParams(anArrKnots.Array1(), aDegree, myumin, myusup, ref myUParams, myusample);
            }
        }
        // update the number of points in sample
        if (myUParams is not null)
        {
            myusample = myUParams.Length();
        }
        if (myVParams is not null)
        {
            myvsample = myVParams.Length();
        }
    }

    // Computes the point on surface parameters on an edge. If it coincides with theParam0 or theParam1, that one is
    // returned. (OCCT returns a reference to theParam0, theParam1 or myGridParam; the callers copy it.)
    private Extrema_POnSurfParams ComputeEdgeParameters(bool IsUEdge, in Extrema_POnSurfParams theParam0,
        in Extrema_POnSurfParams theParam1, in gp_Pnt thePoint, double theDiffTol)
    {
        double aSqrDist01 = theParam0.Value().SquareDistance(theParam1.Value());

        if (aSqrDist01 <= theDiffTol)
        {
            // The points are confused. Get the first point and change its type.
            return theParam0;
        }
        else
        {
            double aDiffDist = Math.Abs(theParam0.GetSqrDistance() - theParam1.GetSqrDistance());

            if (aDiffDist >= aSqrDist01 - theDiffTol)
            {
                // The shortest distance is one of the nodes.
                if (theParam0.GetSqrDistance() > theParam1.GetSqrDistance())
                {
                    // The shortest distance is the point 1.
                    return theParam1;
                }
                else
                {
                    // The shortest distance is the point 0.
                    return theParam0;
                }
            }
            else
            {
                // The shortest distance is inside the edge.
                gp_XYZ aPoP = thePoint.XYZ().Subtracted(theParam0.Value().XYZ());
                gp_XYZ aPoP1 = theParam1.Value().XYZ().Subtracted(theParam0.Value().XYZ());
                double aRatio = aPoP.Dot(aPoP1) / aSqrDist01;
                Span<double> aU = stackalloc double[2];
                Span<double> aV = stackalloc double[2];

                theParam0.Parameter(out aU[0], out aV[0]);
                theParam1.Parameter(out aU[1], out aV[1]);

                double aUPar = aU[0];
                double aVPar = aV[0];

                if (IsUEdge)
                {
                    aUPar += aRatio * (aU[1] - aU[0]);
                }
                else
                {
                    aVPar += aRatio * (aV[1] - aV[0]);
                }

                myGridParam.SetParameters(aUPar, aVPar, myS!.Value(aUPar, aVPar));
                Span<int> anIndices = stackalloc int[2];

                theParam0.GetIndices(out anIndices[0], out anIndices[1]);
                myGridParam.SetElementType(IsUEdge ? Extrema_UIsoEdge : Extrema_VIsoEdge);
                myGridParam.SetSqrDistance(thePoint.SquareDistance(myGridParam.Value()));
                myGridParam.SetIndices(anIndices[0], anIndices[1]);
                return myGridParam;
            }
        }
    }

    // Creation of the grid of parametric points.
    private void BuildGrid(in gp_Pnt thePoint)
    {
        // if grid was already built skip its creation
        if (!myInit)
        {
            // build parametric grid in case of a complex surface geometry (BSpline and Bezier surfaces)
            GetGridPoints(myS!);

            // build grid in other cases
            if (myUParams is null)
            {
                double PasU = myusup - myumin;
                double U0 = PasU / myusample / 100.0;
                PasU = (PasU - U0) / (myusample - 1);
                U0 = U0 / 2.0 + myumin;
                myUParams = new NCollection_HArray1<double>(1, myusample);
                double U = U0;
                for (int NoU = 1; NoU <= myusample; NoU++, U += PasU)
                {
                    myUParams.SetValue(NoU, U);
                }
            }

            if (myVParams is null)
            {
                double PasV = myvsup - myvmin;
                double V0 = PasV / myvsample / 100.0;
                PasV = (PasV - V0) / (myvsample - 1);
                V0 = V0 / 2.0 + myvmin;

                myVParams = new NCollection_HArray1<double>(1, myvsample);
                double V = V0;
                for (int NoV = 1; NoV <= myvsample; NoV++, V += PasV)
                {
                    myVParams.SetValue(NoV, V);
                }
            }

            // If flag was changed and extrema not reinitialized Extrema would fail
            myPoints.Resize(0, myusample + 1, 0, myvsample + 1, false);

            GeomGridEval_Surface aGridEval = new(myS!);

            NCollection_Array2<gp_Pnt> aGridPoints = aGridEval.EvaluateGrid(myUParams.Array1(), myVParams.Array1());

            if (aGridPoints.IsEmpty())
            {
                return;
            }

            for (int NoU = 1; NoU <= myusample; NoU++)
            {
                for (int NoV = 1; NoV <= myvsample; NoV++)
                {
                    gp_Pnt aP1 = aGridPoints.Value(NoU, NoV);
                    Extrema_POnSurfParams aParam = new(myUParams.Value(NoU), myVParams.Value(NoV), aP1);
                    aParam.SetElementType(Extrema_Node);
                    aParam.SetIndices(NoU, NoV);
                    myPoints.SetValue(NoU, NoV, aParam);
                }
            }

            myFacePntParams.Resize(0, myusample, 0, myvsample, false);
            myUEdgePntParams.Resize(1, myusample - 1, 1, myvsample, false);
            myVEdgePntParams.Resize(1, myusample, 1, myvsample - 1, false);

            // Fill boundary with negative square distance.
            // It is used for computation of Maximum.
            for (int NoV = 0; NoV <= myvsample + 1; NoV++)
            {
                myPoints.ChangeValue(0, NoV).SetSqrDistance(-1.0);
                myPoints.ChangeValue(myusample + 1, NoV).SetSqrDistance(-1.0);
            }

            for (int NoU = 1; NoU <= myusample; NoU++)
            {
                myPoints.ChangeValue(NoU, 0).SetSqrDistance(-1.0);
                myPoints.ChangeValue(NoU, myvsample + 1).SetSqrDistance(-1.0);
            }

            myInit = true;
        }

        // Compute distances to mesh.
        // Step 1. Compute distances to nodes.
        for (int NoU = 1; NoU <= myusample; NoU++)
        {
            for (int NoV = 1; NoV <= myvsample; NoV++)
            {
                ref Extrema_POnSurfParams aParam = ref myPoints.ChangeValue(NoU, NoV);

                aParam.SetSqrDistance(thePoint.SquareDistance(aParam.Value()));
            }
        }

        // For search of minimum compute distances to mesh.
        if (myFlag == Extrema_ExtFlag_MIN || myFlag == Extrema_ExtFlag_MINMAX)
        {
            // This is the tolerance of difference of squared values.
            // No need to set it too small.
            double aDiffTol = mytolu + mytolv;

            // Step 2. Compute distances to edges.
            // Assume UEdge(i, j) = { Point(i, j); Point(i + 1, j    ) }
            // Assume VEdge(i, j) = { Point(i, j); Point(i,     j + 1) }
            for (int NoU = 1; NoU <= myusample; NoU++)
            {
                for (int NoV = 1; NoV <= myvsample; NoV++)
                {
                    Extrema_POnSurfParams aParam0 = myPoints.Value(NoU, NoV);

                    if (NoU < myusample)
                    {
                        // Compute parameters to UEdge.
                        Extrema_POnSurfParams aParam1 = myPoints.Value(NoU + 1, NoV);
                        Extrema_POnSurfParams anEdgeParam =
                            ComputeEdgeParameters(true, aParam0, aParam1, thePoint, aDiffTol);

                        myUEdgePntParams.SetValue(NoU, NoV, anEdgeParam);
                    }

                    if (NoV < myvsample)
                    {
                        // Compute parameters to VEdge.
                        Extrema_POnSurfParams aParam1 = myPoints.Value(NoU, NoV + 1);
                        Extrema_POnSurfParams anEdgeParam =
                            ComputeEdgeParameters(false, aParam0, aParam1, thePoint, aDiffTol);

                        myVEdgePntParams.SetValue(NoU, NoV, anEdgeParam);
                    }
                }
            }

            // Step 3. Compute distances to faces.
            // Assume myFacePntParams(i, j) =
            // { Point(i, j); Point(i + 1, j); Point(i + 1, j + 1); Point(i, j + 1) }
            //   Or
            // { UEdge(i, j); VEdge(i + 1, j); UEdge(i, j + 1); VEdge(i, j) }
            double aSqrDist01;
            double aDiffDist;
            bool isOut;
            Span<double> aU = stackalloc double[2]; // declared in the loop in OCCT
            Span<double> aV = stackalloc double[2];

            for (int NoU = 1; NoU < myusample; NoU++)
            {
                for (int NoV = 1; NoV < myvsample; NoV++)
                {
                    Extrema_POnSurfParams aUE0 = myUEdgePntParams.Value(NoU, NoV);
                    Extrema_POnSurfParams aUE1 = myUEdgePntParams.Value(NoU, NoV + 1);
                    Extrema_POnSurfParams aVE0 = myVEdgePntParams.Value(NoU, NoV);
                    Extrema_POnSurfParams aVE1 = myVEdgePntParams.Value(NoU + 1, NoV);

                    aSqrDist01 = aUE0.Value().SquareDistance(aUE1.Value());
                    aDiffDist = Math.Abs(aUE0.GetSqrDistance() - aUE1.GetSqrDistance());
                    isOut = false;

                    if (aDiffDist >= aSqrDist01 - aDiffTol)
                    {
                        // The projection is outside the face.
                        isOut = true;
                    }
                    else
                    {
                        aSqrDist01 = aVE0.Value().SquareDistance(aVE1.Value());
                        aDiffDist = Math.Abs(aVE0.GetSqrDistance() - aVE1.GetSqrDistance());

                        if (aDiffDist >= aSqrDist01 - aDiffTol)
                        {
                            // The projection is outside the face.
                            isOut = true;
                        }
                    }

                    if (isOut)
                    {
                        // Get the closest point on an edge.
                        Extrema_POnSurfParams aUEMin = aUE0.GetSqrDistance() < aUE1.GetSqrDistance() ? aUE0 : aUE1;
                        Extrema_POnSurfParams aVEMin = aVE0.GetSqrDistance() < aVE1.GetSqrDistance() ? aVE0 : aVE1;
                        Extrema_POnSurfParams aEMin =
                            aUEMin.GetSqrDistance() < aVEMin.GetSqrDistance() ? aUEMin : aVEMin;

                        myFacePntParams.SetValue(NoU, NoV, aEMin);
                    }
                    else
                    {
                        // Find closest point inside the face.
                        double aUPar;
                        double aVPar;

                        // Compute U parameter.
                        aUE0.Parameter(out aU[0], out aV[0]);
                        aUE1.Parameter(out aU[1], out aV[1]);
                        aUPar = 0.5 * (aU[0] + aU[1]);
                        // Compute V parameter.
                        aVE0.Parameter(out aU[0], out aV[0]);
                        aVE1.Parameter(out aU[1], out aV[1]);
                        aVPar = 0.5 * (aV[0] + aV[1]);

                        Extrema_POnSurfParams aParam = new(aUPar, aVPar, myS!.Value(aUPar, aVPar));

                        aParam.SetElementType(Extrema_Face);
                        aParam.SetSqrDistance(thePoint.SquareDistance(aParam.Value()));
                        aParam.SetIndices(NoU, NoV);
                        myFacePntParams.SetValue(NoU, NoV, aParam);
                    }
                }
            }

            // Fill boundary with RealLast square distance.
            for (int NoV = 0; NoV <= myvsample; NoV++)
            {
                myFacePntParams.ChangeValue(0, NoV).SetSqrDistance(RealLast());
                myFacePntParams.ChangeValue(myusample, NoV).SetSqrDistance(RealLast());
            }

            for (int NoU = 1; NoU < myusample; NoU++)
            {
                myFacePntParams.ChangeValue(NoU, 0).SetSqrDistance(RealLast());
                myFacePntParams.ChangeValue(NoU, myvsample).SetSqrDistance(RealLast());
            }
        }
    }

    private static double LengthOfIso(Adaptor3d_Surface theS, GeomAbs_IsoType theIso, double thePar1, double thePar2,
        int theNbPnts, double thePar)
    {
        double aLen = 0.0;
        int i;
        double dPar = (thePar2 - thePar1) / (theNbPnts - 1);
        gp_Pnt aP1, aP2;
        double aPar = thePar1 + dPar;
        if (theIso == GeomAbs_IsoU)
        {
            aP1 = theS.Value(thePar, thePar1);
        }
        else
        {
            aP1 = theS.Value(thePar1, thePar);
        }

        for (i = 2; i <= theNbPnts; ++i)
        {
            if (theIso == GeomAbs_IsoU)
            {
                aP2 = theS.Value(thePar, aPar);
            }
            else
            {
                aP2 = theS.Value(aPar, thePar);
            }
            aLen += aP1.Distance(aP2);
            aP1 = aP2;
            aPar += dPar;
        }
        return aLen;
    }

    private static void CorrectNbSamples(Adaptor3d_Surface theS, double theU1, double theU2, ref int theNbU,
        double theV1, double theV2, ref int theNbV)
    {
        double aMinLen = 1.0e-3;
        int nbp = Min(23, theNbV);
        double aLenU1 = LengthOfIso(theS, GeomAbs_IsoU, theV1, theV2, nbp, theU1);
        if (aLenU1 <= aMinLen)
        {
            double aL = LengthOfIso(theS, GeomAbs_IsoU, theV1, theV2, nbp, .7 * theU1 + 0.3 * theU2);
            aLenU1 = Max(aL, aLenU1);
        }
        double aLenU2 = LengthOfIso(theS, GeomAbs_IsoU, theV1, theV2, nbp, theU2);
        if (aLenU2 <= aMinLen)
        {
            double aL = LengthOfIso(theS, GeomAbs_IsoU, theV1, theV2, nbp, .3 * theU1 + 0.7 * theU2);
            aLenU2 = Max(aL, aLenU2);
        }
        nbp = Min(23, theNbV);
        double aLenV1 = LengthOfIso(theS, GeomAbs_IsoV, theU1, theU2, nbp, theV1);
        if (aLenV1 <= aMinLen)
        {
            double aL = LengthOfIso(theS, GeomAbs_IsoV, theU1, theU2, nbp, .7 * theV1 + 0.3 * theV2);
            aLenV1 = Max(aL, aLenV1);
        }
        double aLenV2 = LengthOfIso(theS, GeomAbs_IsoV, theU1, theU2, nbp, theV2);
        if (aLenV2 <= aMinLen)
        {
            double aL = LengthOfIso(theS, GeomAbs_IsoV, theU1, theU2, nbp, .3 * theV1 + 0.7 * theV2);
            aLenV2 = Max(aL, aLenV2);
        }
        //
        double aStepV1 = aLenU1 / theNbV;
        double aStepV2 = aLenU2 / theNbV;
        double aStepU1 = aLenV1 / theNbU;
        double aStepU2 = aLenV2 / theNbU;

        double aMaxStepV = Max(aStepV1, aStepV2);
        double aMaxStepU = Max(aStepU1, aStepU2);
        //
        double aRatio = aMaxStepV / aMaxStepU;
        if (aRatio > 10.0)
        {
            int aMult = RealToInt(Math.Log(aRatio));
            if (aMult > 1)
            {
                theNbV *= aMult;
            }
        }
        else if (aRatio < 0.1)
        {
            int aMult = RealToInt(-Math.Log(aRatio));
            if (aMult > 1)
            {
                theNbV *= aMult; // OCCT: multiplies NbV again where NbU was presumably meant
            }
        }
    }

    private void BuildTree()
    {
        // if tree already exists, assume it is already correctly filled
        if (mySphereUBTree is not null)
        {
            return;
        }

        if (myS!.GetType() == GeomAbs_BSplineSurface)
        {
            Geom_BSplineSurface aBspl = myS.BSpline();
            int aUValue = aBspl.UDegree() * aBspl.NbUKnots();
            int aVValue = aBspl.VDegree() * aBspl.NbVKnots();
            // 300 is value, which is used for singular points (see Extrema_ExtPS.cxx::Initialize(...))
            if (aUValue > myusample)
            {
                myusample = Min(aUValue, 300);
            }
            if (aVValue > myvsample)
            {
                myvsample = Min(aVValue, 300);
            }
        }
        //
        CorrectNbSamples(myS, myumin, myusup, ref myusample, myvmin, myvsup, ref myvsample);
        //
        double PasU = myusup - myumin;
        double PasV = myvsup - myvmin;
        double U0 = PasU / myusample / 100.0;
        double V0 = PasV / myvsample / 100.0;
        PasU = (PasU - U0) / (myusample - 1);
        PasV = (PasV - V0) / (myvsample - 1);
        U0 = U0 / 2.0 + myumin;
        V0 = V0 / 2.0 + myvmin;

        // build grid of parametric points
        myUParams = new NCollection_HArray1<double>(1, myusample);
        myVParams = new NCollection_HArray1<double>(1, myvsample);
        int NoU, NoV;
        double U = U0, V = V0;
        for (NoU = 1; NoU <= myusample; NoU++, U += PasU)
        {
            myUParams.SetValue(NoU, U);
        }
        for (NoV = 1; NoV <= myvsample; NoV++, V += PasV)
        {
            myVParams.SetValue(NoV, V);
        }

        // Build UB-tree with surface points for fast proximity search.
        // Use optimized grid evaluator with span-based caching for B-spline surfaces.
        mySphereUBTree = new NCollection_UBTree<int, Bnd_Sphere>();
        NCollection_UBTreeFiller<int, Bnd_Sphere> aFiller = new(mySphereUBTree);
        int i = 0;

        mySphereArray = new NCollection_HArray1<Bnd_Sphere>(0, myusample * myvsample);

        // Use unified grid evaluator for all surface types (optimized for BSpline, Bezier, etc.)
        GeomGridEval_Surface aGridEval = new(myS);

        NCollection_Array2<gp_Pnt> aGridPoints = aGridEval.EvaluateGrid(myUParams.Array1(), myVParams.Array1());

        for (NoU = 1; NoU <= myusample; NoU++)
        {
            for (NoV = 1; NoV <= myvsample; NoV++)
            {
                gp_Pnt aP1 = aGridPoints.Value(NoU, NoV);
                Bnd_Sphere aSph = new(aP1.XYZ(), 0 /*mytolu < mytolv ? mytolu : mytolv*/, NoU, NoV);
                aFiller.Add(i, aSph);
                mySphereArray.SetValue(i, aSph);
                i++;
            }
        }
        aFiller.Fill();
    }

    private void FindSolution(in gp_Pnt P, in Extrema_POnSurfParams theParams)
    {
        _ = P;
        math_Vector Tol = new(1, 2);
        Tol[1] = mytolu;
        Tol[2] = mytolv;

        math_Vector UV = new(1, 2);
        theParams.Parameter(out UV[1], out UV[2]);

        math_Vector UVinf = new(1, 2), UVsup = new(1, 2);
        UVinf[1] = myumin;
        UVinf[2] = myvmin;
        UVsup[1] = myusup;
        UVsup[2] = myvsup;

        math_FunctionSetRoot S = new(myF, Tol);
        S.Perform(myF, UV, UVinf, UVsup);

        myDone = true;
    }

    public void SetFlag(Extrema_ExtFlag F) => myFlag = F;

    public void SetAlgo(Extrema_ExtAlgo A)
    {
        if (myAlgo != A)
        {
            myInit = false;
        }
        myAlgo = A;
    }

    /// <summary>Computes the extrema for the point P. Throws if the surface has not been initialized.</summary>
    public void Perform(in gp_Pnt P)
    {
        myDone = false;
        myF.SetPoint(P);

        if (myAlgo == Extrema_ExtAlgo_Grad)
        {
            BuildGrid(P);
            int NoU, NoV;

            if (myFlag == Extrema_ExtFlag_MIN || myFlag == Extrema_ExtFlag_MINMAX)
            {
                Extrema_ElementType anElemType;
                int iU;
                int iV;
                int iU2;
                int iV2;
                bool isMin;
                int i;

                for (NoU = 1; NoU < myusample; NoU++)
                {
                    for (NoV = 1; NoV < myvsample; NoV++)
                    {
                        Extrema_POnSurfParams aParam = myFacePntParams.Value(NoU, NoV);

                        isMin = false;
                        anElemType = aParam.GetElementType();

                        if (anElemType == Extrema_Face)
                        {
                            isMin = true;
                        }
                        else
                        {
                            // Check if it is a boundary edge or corner vertex.
                            aParam.GetIndices(out iU, out iV);

                            if (anElemType == Extrema_UIsoEdge)
                            {
                                isMin = (iV == 1 || iV == myvsample);
                            }
                            else if (anElemType == Extrema_VIsoEdge)
                            {
                                isMin = (iU == 1 || iU == myusample);
                            }
                            else if (anElemType == Extrema_Node)
                            {
                                isMin = (iU == 1 || iU == myusample) && (iV == 1 || iV == myvsample);
                            }

                            if (!isMin)
                            {
                                // This is a middle element.
                                if (anElemType == Extrema_UIsoEdge
                                    || (anElemType == Extrema_Node && (iU == 1 || iU == myusample)))
                                {
                                    // Check the down face.
                                    Extrema_POnSurfParams aDownParam = myFacePntParams.Value(NoU, NoV - 1);

                                    if (aDownParam.GetElementType() == anElemType)
                                    {
                                        aDownParam.GetIndices(out iU2, out iV2);
                                        isMin = (iU == iU2 && iV == iV2);
                                    }
                                }
                                else if (anElemType == Extrema_VIsoEdge
                                         || (anElemType == Extrema_Node && (iV == 1 || iV == myvsample)))
                                {
                                    // Check the right face.
                                    Extrema_POnSurfParams aRightParam = myFacePntParams.Value(NoU - 1, NoV);

                                    if (aRightParam.GetElementType() == anElemType)
                                    {
                                        aRightParam.GetIndices(out iU2, out iV2);
                                        isMin = (iU == iU2 && iV == iV2);
                                    }
                                }
                                else if (iU == NoU && iV == NoV)
                                {
                                    // Check the lower-left node. For this purpose it is necessary
                                    // to check lower-left, lower and left faces.
                                    isMin = true;

                                    Span<Extrema_POnSurfParams> anOtherParam =
                                    [
                                        myFacePntParams.Value(NoU, NoV - 1), // Down
                                        myFacePntParams.Value(NoU - 1, NoV - 1), // Lower-left
                                        myFacePntParams.Value(NoU - 1, NoV), // Left
                                    ];

                                    for (i = 0; i < 3 && isMin; i++)
                                    {
                                        if (anOtherParam[i].GetElementType() == Extrema_Node)
                                        {
                                            anOtherParam[i].GetIndices(out iU2, out iV2);
                                            isMin = (iU == iU2 && iV == iV2);
                                        }
                                        else
                                        {
                                            isMin = false;
                                        }
                                    }
                                }
                            }
                        }

                        if (isMin)
                        {
                            FindSolution(P, aParam);
                        }
                    }
                }
            }

            if (myFlag == Extrema_ExtFlag_MAX || myFlag == Extrema_ExtFlag_MINMAX)
            {
                double Dist;

                for (NoU = 1; NoU <= myusample; NoU++)
                {
                    for (NoV = 1; NoV <= myvsample; NoV++)
                    {
                        Extrema_POnSurfParams aParamMain = myPoints.Value(NoU, NoV);
                        Extrema_POnSurfParams aParam1 = myPoints.Value(NoU - 1, NoV - 1);
                        Extrema_POnSurfParams aParam2 = myPoints.Value(NoU - 1, NoV);
                        Extrema_POnSurfParams aParam3 = myPoints.Value(NoU - 1, NoV + 1);
                        Extrema_POnSurfParams aParam4 = myPoints.Value(NoU, NoV - 1);
                        Extrema_POnSurfParams aParam5 = myPoints.Value(NoU, NoV + 1);
                        Extrema_POnSurfParams aParam6 = myPoints.Value(NoU + 1, NoV - 1);
                        Extrema_POnSurfParams aParam7 = myPoints.Value(NoU + 1, NoV);
                        Extrema_POnSurfParams aParam8 = myPoints.Value(NoU + 1, NoV + 1);

                        Dist = aParamMain.GetSqrDistance();

                        if ((aParam1.GetSqrDistance() <= Dist) && (aParam2.GetSqrDistance() <= Dist)
                            && (aParam3.GetSqrDistance() <= Dist) && (aParam4.GetSqrDistance() <= Dist)
                            && (aParam5.GetSqrDistance() <= Dist) && (aParam6.GetSqrDistance() <= Dist)
                            && (aParam7.GetSqrDistance() <= Dist) && (aParam8.GetSqrDistance() <= Dist))
                        {
                            // Find maximum.
                            FindSolution(P, myPoints.Value(NoU, NoV));
                        }
                    }
                }
            }
        }
        else
        {
            BuildTree();
            if (myFlag == Extrema_ExtFlag_MIN || myFlag == Extrema_ExtFlag_MINMAX)
            {
                Bnd_Sphere aSol = mySphereArray!.Value(0);
                Bnd_SphereUBTreeSelectorMin aSelector = new(mySphereArray, aSol);
                // aSelector.SetMaxDist( RealLast() );
                aSelector.DefineCheckPoint(P);
                mySphereUBTree!.Select(aSelector);
                // TODO: check if no solution in binary tree
                Bnd_Sphere aSph = aSelector.Sphere();
                double aU = myUParams!.Value(aSph.U());
                double aV = myVParams!.Value(aSph.V());
                Extrema_POnSurfParams aParams = new(aU, aV, myS!.Value(aU, aV));

                aParams.SetSqrDistance(P.SquareDistance(aParams.Value()));
                aParams.SetIndices(aSph.U(), aSph.V());
                FindSolution(P, aParams);
            }
            if (myFlag == Extrema_ExtFlag_MAX || myFlag == Extrema_ExtFlag_MINMAX)
            {
                Bnd_Sphere aSol = mySphereArray!.Value(0);
                Bnd_SphereUBTreeSelectorMax aSelector = new(mySphereArray, aSol);
                // aSelector.SetMaxDist( RealLast() );
                aSelector.DefineCheckPoint(P);
                mySphereUBTree!.Select(aSelector);
                // TODO: check if no solution in binary tree
                Bnd_Sphere aSph = aSelector.Sphere();
                double aU = myUParams!.Value(aSph.U());
                double aV = myVParams!.Value(aSph.V());
                Extrema_POnSurfParams aParams = new(aU, aV, myS!.Value(aU, aV));
                aParams.SetSqrDistance(P.SquareDistance(aParams.Value()));
                aParams.SetIndices(aSph.U(), aSph.V());

                FindSolution(P, aParams);
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myF.NbExt();
    }

    /// <summary>The Nth square distance.</summary>
    public double SquareDistance(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }

        return myF.SquareDistance(N);
    }

    /// <summary>The point of the Nth distance.</summary>
    public Extrema_POnSurf Point(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }

        return myF.Point(N);
    }
}
