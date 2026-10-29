// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Face.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Face.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The surface tool of BRepGProp's surface and volume integrations: a face (its surface, placed and forward) and the
/// current arc of its boundary (a pcurve, or a side of the UV rectangle), with the integration orders and knots
/// chosen for each type of surface and curve.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: the Load methods change the current arc. OCCT copies it by value into
/// BRepGProp_TFunction and BRepGProp_UFunction; the port's copy is the internal copy constructor.
/// </remarks>
public sealed class BRepGProp_Face
{
    private static readonly double Epsilon1 = Epsilon(1.0);

    private readonly BRepAdaptor_Surface mySurface;
    private readonly Geom2dAdaptor_Curve myCurve;
    private Geom_Surface? myFaceSurface;
    private TopLoc_Location myFaceLocation;
    private bool myIsFaceContextReady;
    private bool mySReverse;
    private readonly bool myIsUseSpan;

    /// <summary>
    /// An empty face tool. <paramref name="IsUseSpan"/> asks for the spans of a B-spline face
    /// (<see cref="GetUKnots"/>, <see cref="GetTKnots"/>); it has no effect on other faces.
    /// </summary>
    public BRepGProp_Face(bool IsUseSpan = false)
    {
        mySurface = new BRepAdaptor_Surface();
        myCurve = new Geom2dAdaptor_Curve();
        myFaceLocation = new TopLoc_Location();
        myIsFaceContextReady = false;
        mySReverse = false;
        myIsUseSpan = IsUseSpan;
    }

    /// <summary>The face tool of <paramref name="F"/>; see <see cref="BRepGProp_Face(bool)"/> for <paramref name="IsUseSpan"/>.</summary>
    public BRepGProp_Face(in TopoDS_Face F, bool IsUseSpan = false)
        : this(IsUseSpan)
    {
        Load(F);
    }

    /// <summary>C++ copy construction: the adaptors are shallow-copied, so the copy can be reloaded on its own.</summary>
    internal BRepGProp_Face(BRepGProp_Face theOther)
    {
        mySurface = (BRepAdaptor_Surface)theOther.mySurface.ShallowCopy();
        myCurve = (Geom2dAdaptor_Curve)theOther.myCurve.ShallowCopy();
        myFaceSurface = theOther.myFaceSurface;
        myFaceLocation = theOther.myFaceLocation;
        myIsFaceContextReady = theOther.myIsFaceContextReady;
        mySReverse = theOther.mySReverse;
        myIsUseSpan = theOther.myIsUseSpan;
    }

    /// <summary>Loads the face: its surface, forward, and whether it is reversed.</summary>
    public void Load(in TopoDS_Face F)
    {
        TopoDS_Shape aLocalShape = F.Oriented(TopAbs_FORWARD);
        mySurface.Initialize(TopoDS.Face(aLocalShape));
        myFaceSurface = BRep_Tool.Surface(mySurface.Face(), out myFaceLocation);
        myIsFaceContextReady = myFaceSurface is not null;
        //  mySurface.Initialize(TopoDS::Face(F.Oriented(TopAbs_FORWARD)));
        mySReverse = (F.Orientation() == TopAbs_REVERSED);
    }

    /// <summary>The number of points for an accurate integration in the U direction.</summary>
    public int UIntegrationOrder()
    {
        int Nu;
        switch (mySurface.GetType())
        {
            case GeomAbs_Plane:
                Nu = 4;
                break;

            case GeomAbs_BezierSurface:
            {
                Nu = ((Geom_BezierSurface)mySurface.GeomSurfaceOriginal()!).UDegree() + 1;
                Nu = Max(4, Nu);
            }

                break;
            case GeomAbs_BSplineSurface:
            {
                int a = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).UDegree() + 1;
                int b = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).NbUKnots() - 1;
                Nu = Max(4, a * b);
            }

                break;
            default:
                Nu = 9;
                break;
        }

        return Max(8, 2 * Nu);
    }

    /// <summary>The number of points for an accurate integration in the V direction.</summary>
    public int VIntegrationOrder()
    {
        int Nv;
        switch (mySurface.GetType())
        {
            case GeomAbs_Plane:
                Nv = 4;
                break;

            case GeomAbs_BezierSurface:
            {
                Nv = ((Geom_BezierSurface)mySurface.GeomSurfaceOriginal()!).VDegree() + 1;
                Nv = Max(4, Nv);
            }

                break;
            case GeomAbs_BSplineSurface:
            {
                int a = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).VDegree() + 1;
                int b = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).NbVKnots() - 1;
                Nv = Max(4, a * b);
            }

                break;
            default:
                Nv = 9;
                break;
        }

        return Max(8, 2 * Nv);
    }

    /// <summary>True if the face is not trimmed.</summary>
    public bool NaturalRestriction() => BRep_Tool.NaturalRestriction(mySurface.Face());

    /// <summary>The face, forward.</summary>
    public TopoDS_Face GetFace() => mySurface.Face();

    /// <summary>The point of parameter U on the current boundary curve.</summary>
    public gp_Pnt2d Value2d(double U) => myCurve.Value(U);

    /// <summary>The number of points along the current boundary curve for an accurate integration.</summary>
    public int IntegrationOrder()
    {
        int N;

        switch (myCurve.GetType())
        {
            case GeomAbs_Line:
                N = 2;
                break;

            case GeomAbs_Circle:
            case GeomAbs_Ellipse:
            case GeomAbs_Hyperbola:
                N = 9;
                break;

            case GeomAbs_Parabola:
                N = 9;
                break;

            case GeomAbs_BezierCurve:
            {
                N = ((Geom2d_BezierCurve)myCurve.Curve()!).Degree() + 1;
            }

                break;
            case GeomAbs_BSplineCurve:
            {
                int a = ((Geom2d_BSplineCurve)myCurve.Curve()!).Degree() + 1;
                int b = ((Geom2d_BSplineCurve)myCurve.Curve()!).NbKnots() - 1;
                N = a * b;
            }

                break;
            default:
                N = 9;
                break;
        }

        return Max(4, 2 * N);
    }

    /// <summary>The parametric bounds of the face.</summary>
    public void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = mySurface.FirstUParameter();
        U2 = mySurface.LastUParameter();
        V1 = mySurface.FirstVParameter();
        V2 = mySurface.LastVParameter();
    }

    /// <summary>Loads the boundary arc of the edge; false if the edge has no pcurve on the face.</summary>
    public bool Load(in TopoDS_Edge E)
    {
        double a, b;
        if (!myIsFaceContextReady)
        {
            return false;
        }

        Geom2d_Curve? C = BRep_Tool.CurveOnSurface(E, myFaceSurface, myFaceLocation, out a, out b);
        if (C is null)
        {
            return false;
        }

        if (E.Orientation() == TopAbs_REVERSED)
        {
            double x = a;
            a = C.ReversedParameter(b);
            b = C.ReversedParameter(x);
            C = C.Reversed();
        }

        myCurve.Load(C, a, b);
        return true;
    }

    /// <summary>The point of parameters (U, V) on the face and the normal there (D1U ^ D1V, reversed with the face).</summary>
    public void Normal(double U, double V, out gp_Pnt P, out gp_Vec VNor)
    {
        mySurface.D1(U, V, out P, out gp_Vec D1U, out gp_Vec D1V);
        VNor = D1U.Crossed(D1V);
        if (mySReverse)
        {
            VNor.Reverse();
        }
    }

    //  APO 17.04.2002 (OCC104)
    // This is functions that calculate coeff. to optimize "integration order".
    // They had been produced experimentally for some hard example.
    private const double AS = -0.15, AL = -0.50, B = 1.0, C = 0.75, D = 0.25;

    private static double SCoeff(double Eps) => Eps < 0.1 ? AS * (B + Math.Log10(Eps)) + C : C;

    private static double LCoeff(double Eps) => Eps < 0.1 ? AL * (B + Math.Log10(Eps)) + D : D;

    /// <summary>The number of Gauss points over the surface for the relative precision <paramref name="Eps"/>.</summary>
    public int SIntOrder(double Eps)
    {
        int Nv, Nu;
        switch (mySurface.GetType())
        {
            case GeomAbs_Plane:
                Nu = 1;
                Nv = 1;
                break;
            case GeomAbs_Cylinder:
                Nu = 2;
                Nv = 1;
                break;
            case GeomAbs_Cone:
                Nu = 2;
                Nv = 1;
                break;
            case GeomAbs_Sphere:
                Nu = 2;
                Nv = 2;
                break;
            case GeomAbs_Torus:
                Nu = 2;
                Nv = 2;
                break;
            case GeomAbs_BezierSurface:
                Nv = ((Geom_BezierSurface)mySurface.GeomSurfaceOriginal()!).VDegree();
                Nu = ((Geom_BezierSurface)mySurface.GeomSurfaceOriginal()!).UDegree();
                break;
            case GeomAbs_BSplineSurface:
                Nv = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).VDegree();
                Nu = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).UDegree();
                break;
            default:
                Nu = 2;
                Nv = 2;
                break;
        }

        return Min(RealToInt(Math.Ceiling(SCoeff(Eps) * Max((Nu + 1), (Nv + 1)))), math.GaussPointsMax());
    }

    /// <summary>The number of subintervals in U over the surface.</summary>
    public int SUIntSubs()
    {
        int N;
        switch (mySurface.GetType())
        {
            case GeomAbs_Plane:
                N = 2;
                break;
            case GeomAbs_Cylinder:
                N = 4;
                break;
            case GeomAbs_Cone:
                N = 4;
                break;
            case GeomAbs_Sphere:
                N = 4;
                break;
            case GeomAbs_Torus:
                N = 4;
                break;
            case GeomAbs_BezierSurface:
                N = 2;
                break;
            case GeomAbs_BSplineSurface:
                N = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).NbUKnots();
                break;
            default:
                N = 2;
                break;
        }

        return N - 1;
    }

    /// <summary>The number of subintervals in V over the surface.</summary>
    public int SVIntSubs()
    {
        int N;
        switch (mySurface.GetType())
        {
            case GeomAbs_Plane:
                N = 2;
                break;
            case GeomAbs_Cylinder:
                N = 2;
                break;
            case GeomAbs_Cone:
                N = 2;
                break;
            case GeomAbs_Sphere:
                N = 3;
                break;
            case GeomAbs_Torus:
                N = 4;
                break;
            case GeomAbs_BezierSurface:
                N = 2;
                break;
            case GeomAbs_BSplineSurface:
                N = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).NbVKnots();
                break;
            default:
                N = 2;
                break;
        }

        return N - 1;
    }

    /// <summary>Fills <paramref name="Knots"/> (SUIntSubs() + 1 values) with the U bounds of the subintervals.</summary>
    public void UKnots(NCollection_Array1<double> Knots)
    {
        switch (mySurface.GetType())
        {
            case GeomAbs_Plane:
                Knots[1] = mySurface.FirstUParameter();
                Knots[2] = mySurface.LastUParameter();
                break;
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
                Knots[1] = 0.0;
                Knots[2] = M_PI * 2.0 / 3.0;
                Knots[3] = M_PI * 4.0 / 3.0;
                Knots[4] = M_PI * 6.0 / 3.0;
                break;
            case GeomAbs_BSplineSurface:
            {
                NCollection_Array1<double> aSrcKnots = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).InternalUKnots();
                for (int i = Knots.Lower(); i <= Knots.Upper(); i++)
                {
                    Knots[i] = aSrcKnots[i];
                }
            }

                break;
            default:
                Knots[1] = mySurface.FirstUParameter();
                Knots[2] = mySurface.LastUParameter();
                break;
        }
    }

    /// <summary>Fills <paramref name="Knots"/> (SVIntSubs() + 1 values) with the V bounds of the subintervals.</summary>
    public void VKnots(NCollection_Array1<double> Knots)
    {
        switch (mySurface.GetType())
        {
            case GeomAbs_Plane:
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
                // OCCT takes the U bounds for the V knots of these surfaces (occt-issues.md)
                Knots[1] = mySurface.FirstUParameter();
                Knots[2] = mySurface.LastUParameter();
                break;
            case GeomAbs_Sphere:
                Knots[1] = -M_PI / 2.0;
                Knots[2] = 0.0;
                Knots[3] = +M_PI / 2.0;
                break;
            case GeomAbs_Torus:
                Knots[1] = 0.0;
                Knots[2] = M_PI * 2.0 / 3.0;
                Knots[3] = M_PI * 4.0 / 3.0;
                Knots[4] = M_PI * 6.0 / 3.0;
                break;
            case GeomAbs_BSplineSurface:
            {
                NCollection_Array1<double> aSrcKnots = ((Geom_BSplineSurface)mySurface.GeomSurfaceOriginal()!).InternalVKnots();
                for (int i = Knots.Lower(); i <= Knots.Upper(); i++)
                {
                    Knots[i] = aSrcKnots[i];
                }
            }

                break;
            default:
                Knots[1] = mySurface.FirstUParameter();
                Knots[2] = mySurface.LastUParameter();
                break;
        }
    }

    /// <summary>The number of Gauss points along the current boundary curve for the relative precision <paramref name="Eps"/>.</summary>
    public int LIntOrder(double Eps)
    {
        Bnd_Box2d aBox = new();

        BndLib_Add2dCurve.Add(myCurve, 1.0e-7, ref aBox);
        double aXmin, aXmax, aYmin, aYmax;
        aBox.Get(out aXmin, out aYmin, out aXmax, out aYmax);
        double aVmin = mySurface.FirstVParameter();
        double aVmax = mySurface.LastVParameter();

        double dv = (aVmax - aVmin);
        double anR = (dv > Epsilon1 ? Min((aYmax - aYmin) / dv, 1.0) : 1.0);

        int anRInt = RealToInt(Math.Ceiling(SVIntSubs() * anR));
        int aLSubs = LIntSubs();

        //  double NL, NS = std::max(SIntOrder(1.0)*anRInt/LIntSubs(), 1);
        double NL, NS = Max(SIntOrder(1.0) * anRInt / aLSubs, 1);
        switch (myCurve.GetType())
        {
            case GeomAbs_Line:
                NL = 1;
                break;
            case GeomAbs_Circle:
                NL = 2 * 3;
                break; // correction for the spans of converted curve
            case GeomAbs_Ellipse:
                NL = 2 * 3;
                break; //
            case GeomAbs_Parabola:
                NL = 2 * 3;
                break;
            case GeomAbs_Hyperbola:
                NL = 3 * 3;
                break;
            case GeomAbs_BezierCurve:
                NL = ((Geom2d_BezierCurve)myCurve.Curve()!).Degree();
                break;
            case GeomAbs_BSplineCurve:
                NL = ((Geom2d_BSplineCurve)myCurve.Curve()!).Degree();
                break;
            default:
                NL = 3 * 3;
                break;
        }

        NL = Max(NL, NS);

        int nn = RealToInt(aLSubs <= 4 ? Math.Ceiling(LCoeff(Eps) * (NL + 1)) : NL + 1);

        return Min(nn, math.GaussPointsMax());
    }

    /// <summary>The number of subintervals of the current boundary curve.</summary>
    public int LIntSubs()
    {
        int N;
        switch (myCurve.GetType())
        {
            case GeomAbs_Line:
                N = 2;
                break;
            case GeomAbs_Circle:
            case GeomAbs_Ellipse:
                N = 4;
                break;
            case GeomAbs_Parabola:
            case GeomAbs_Hyperbola:
                N = 2;
                break;
            case GeomAbs_BSplineCurve:
                N = ((Geom2d_BSplineCurve)myCurve.Curve()!).NbKnots();
                break;
            default:
                N = 2;
                break;
        }

        return N - 1;
    }

    /// <summary>Fills <paramref name="Knots"/> (LIntSubs() + 1 values) with the bounds of the subintervals of the current curve.</summary>
    public void LKnots(NCollection_Array1<double> Knots)
    {
        switch (myCurve.GetType())
        {
            case GeomAbs_Line:
                Knots[1] = myCurve.FirstParameter();
                Knots[2] = myCurve.LastParameter();
                break;
            case GeomAbs_Circle:
            case GeomAbs_Ellipse:
                Knots[1] = 0.0;
                Knots[2] = M_PI * 2.0 / 3.0;
                Knots[3] = M_PI * 4.0 / 3.0;
                Knots[4] = M_PI * 6.0 / 3.0;
                break;
            case GeomAbs_Parabola:
            case GeomAbs_Hyperbola:
                Knots[1] = myCurve.FirstParameter();
                Knots[2] = myCurve.LastParameter();
                break;
            case GeomAbs_BSplineCurve:
            {
                NCollection_Array1<double> aSrcKnots = ((Geom2d_BSplineCurve)myCurve.Curve()!).InternalKnots();
                for (int i = Knots.Lower(); i <= Knots.Upper(); i++)
                {
                    Knots[i] = aSrcKnots[i];
                }
            }

                break;
            default:
                Knots[1] = myCurve.FirstParameter();
                Knots[2] = myCurve.LastParameter();
                break;
        }
    }

    /// <summary>The parameter of the start point of the current arc.</summary>
    public double FirstParameter() => myCurve.FirstParameter();

    /// <summary>The parameter of the end point of the current arc.</summary>
    public double LastParameter() => myCurve.LastParameter();

    /// <summary>The point of parameter U of the current boundary curve and the first derivative there.</summary>
    public void D12d(double U, out gp_Pnt2d P, out gp_Vec2d V1) => myCurve.D1(U, out P, out V1);

    /// <summary>
    /// Loads a side of the UV rectangle of the face as the current arc: with <see cref="GeomAbs_IsoU"/> the left
    /// (<paramref name="IsFirstParam"/>) or right side, with <see cref="GeomAbs_IsoV"/> the bottom or top one.
    /// </summary>
    public void Load(bool IsFirstParam, GeomAbs_IsoType theIsoType)
    {
        double aLen;
        double aU1;
        double aU2;
        double aV1;
        double aV2;
        gp_Pnt2d aLoc = new();
        gp_Dir2d aDir = new();

        Bounds(out aU1, out aU2, out aV1, out aV2);

        if (theIsoType == GeomAbs_IsoU)
        {
            aLen = aV2 - aV1;

            if (IsFirstParam)
            {
                aLoc.SetCoord(aU1, aV2);
                aDir.SetCoord(0.0, -1.0);
            }
            else
            {
                aLoc.SetCoord(aU2, aV1);
                aDir.SetCoord(0.0, 1.0);
            }
        }
        else if (theIsoType == GeomAbs_IsoV)
        {
            aLen = aU2 - aU1;

            if (IsFirstParam)
            {
                aLoc.SetCoord(aU1, aV1);
                aDir.SetCoord(1.0, 0.0);
            }
            else
            {
                aLoc.SetCoord(aU2, aV2);
                aDir.SetCoord(-1.0, 0.0);
            }
        }
        else
        {
            return;
        }

        Geom2d_Curve aLin = new Geom2d_Line(aLoc, aDir);

        myCurve.Load(aLin, 0.0, aLen);
    }

    private static void GetRealKnots(double theMin,
                                     double theMax,
                                     NCollection_HArray1<double> theKnots,
                                     out NCollection_HArray1<double> theRealKnots)
    {
        int i = theKnots.Lower() - 1;
        int iU = theKnots.Upper();
        int aStartI = 0;
        int aEndI = 0;
        double aTol = Precision.Confusion();

        while (++i < iU)
        {
            if (aStartI == 0 && theKnots.Value(i) > theMin + aTol)
            {
                aStartI = i;
            }

            if (aEndI == 0 && theKnots.Value(i + 1) > theMax - aTol)
            {
                aEndI = i;
            }

            if (aStartI != 0 && aEndI != 0)
            {
                break;
            }
        }

        if (aStartI == 0)
        {
            aStartI = iU;
        }

        int aNbNode = Max(0, aEndI - aStartI + 1) + 2;
        int j;

        theRealKnots = new NCollection_HArray1<double>(1, aNbNode);
        theRealKnots.SetValue(1, theMin);
        theRealKnots.SetValue(aNbNode, theMax);

        for (i = 2, j = aStartI; j <= aEndI; i++, j++)
        {
            theRealKnots.SetValue(i, theKnots.Value(j));
        }
    }

    private static void GetCurveKnots(double theMin,
                                      double theMax,
                                      Geom2dAdaptor_Curve theCurve,
                                      out NCollection_HArray1<double> theKnots)
    {
        bool isSBSpline = theCurve.GetType() == GeomAbs_BSplineCurve;

        if (isSBSpline)
        {
            Geom2d_BSplineCurve aCrv;
            NCollection_HArray1<double> aCrvKnots;

            aCrv = (Geom2d_BSplineCurve)theCurve.Curve()!;
            aCrvKnots = new NCollection_HArray1<double>(aCrv.InternalKnots());
            GetRealKnots(theMin, theMax, aCrvKnots, out theKnots);
        }
        else
        {
            theKnots = new NCollection_HArray1<double>(1, 2);
            theKnots.SetValue(1, theMin);
            theKnots.SetValue(2, theMax);
        }
    }

    /// <summary>
    /// The U knots of the face in [theUMin, theUMax]: theUMin, the knots of a B-spline face (or of the B-spline basis
    /// curve of an extrusion) strictly inside, in increasing order, if the face uses spans, then theUMax.
    /// </summary>
    public NCollection_HArray1<double> GetUKnots(double theUMin, double theUMax)
    {
        NCollection_HArray1<double> theUKnots;
        bool isSBSpline = mySurface.GetType() == GeomAbs_BSplineSurface;
        bool isCBSpline = false;

        if (!isSBSpline)
        {
            if (mySurface.GetType() == GeomAbs_SurfaceOfExtrusion)
            {
                var aCurve = new GeomAdaptor_Curve();
                Geom_Surface aSurf = mySurface.GeomSurfaceOriginal()!;

                aCurve.Load(((Geom_SurfaceOfLinearExtrusion)aSurf).BasisCurve());
                isCBSpline = aCurve.GetType() == GeomAbs_BSplineCurve;
            }
        }

        if (myIsUseSpan && (isSBSpline || isCBSpline))
        {
            NCollection_HArray1<double> aKnots;

            if (isSBSpline)
            {
                Geom_Surface aSurf = mySurface.GeomSurfaceOriginal()!;
                Geom_BSplineSurface aBSplSurf;

                aBSplSurf = (Geom_BSplineSurface)aSurf;
                aKnots = new NCollection_HArray1<double>(aBSplSurf.InternalUKnots());
            }
            else
            {
                var aCurve = new GeomAdaptor_Curve();
                Geom_Surface aSurf = mySurface.GeomSurfaceOriginal()!;
                Geom_BSplineCurve aBSplCurve;

                aCurve.Load(((Geom_SurfaceOfLinearExtrusion)aSurf).BasisCurve());
                aBSplCurve = aCurve.BSpline();
                aKnots = new NCollection_HArray1<double>(aBSplCurve.InternalKnots());
            }

            GetRealKnots(theUMin, theUMax, aKnots, out theUKnots);
        }
        else
        {
            theUKnots = new NCollection_HArray1<double>(1, 2);
            theUKnots.SetValue(1, theUMin);
            theUKnots.SetValue(2, theUMax);
        }

        return theUKnots;
    }

    /// <summary>
    /// The T knots of the current arc in [theTMin, theTMax]: theTMin, the knots of a B-spline arc strictly inside, in
    /// increasing order, if the face is a B-spline using spans, then theTMax.
    /// </summary>
    public NCollection_HArray1<double> GetTKnots(double theTMin, double theTMax)
    {
        NCollection_HArray1<double> theTKnots;
        bool isBSpline = mySurface.GetType() == GeomAbs_BSplineSurface;

        if (myIsUseSpan && isBSpline)
        {
            // OCCT copies the V knots of the face here and doesn't use them
            NCollection_HArray1<double> aSurfKnots;
            Geom_Surface aSurf = mySurface.GeomSurfaceOriginal()!;
            Geom_BSplineSurface aBSplSurf;

            aBSplSurf = (Geom_BSplineSurface)aSurf;
            aSurfKnots = new NCollection_HArray1<double>(aBSplSurf.InternalVKnots());
            _ = aSurfKnots;
            GetCurveKnots(theTMin, theTMax, myCurve, out theTKnots);
        }
        else
        {
            theTKnots = new NCollection_HArray1<double>(1, 2);
            theTKnots.SetValue(1, theTMin);
            theTKnots.SetValue(2, theTMax);
        }

        return theTKnots;
    }
}
