// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Surface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Surface.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// Complements Geom_Surface with the detection of its singularities, the check of its spatial closure and the
/// projection of 3D points onto it.
/// </summary>
/// <remarks>
/// <para>
/// Singularities: each stores the precision with which the corresponding iso-line is considered degenerated. Their
/// number, for a given precision, is never more than 4.
/// </para>
/// <para>
/// Closure: checked with a given precision (Precision::Confusion() by default). A surface Geom_Surface says is closed
/// is closed; otherwise more analysis is done.
/// </para>
/// <para>
/// Projection: done with a given precision, taking the singularities into account; a variant searches from an
/// already computed solution.
/// </para>
/// <para>
/// A working object: it computes most of this once and caches it. The bound iso-curves it computes are frozen
/// before they are cached or handed out.
/// </para>
/// </remarks>
public class ShapeAnalysis_Surface : Transient
{
    protected Surface?         mySurf;
    protected SurfaceAdaptor?  myAdSur;
    protected ExtPS         myExtPS = new ExtPS();
    protected bool                  myExtOK;
    protected int                   myNbDeg;
    protected double[]              myPreci    = new double[4];
    protected Point3D[]              myP3d      = new Point3D[4];
    protected Point2D[]            myFirstP2d = new Point2D[4];
    protected Point2D[]            myLastP2d  = new Point2D[4];
    protected double[]              myFirstPar = new double[4];
    protected double[]              myLastPar  = new double[4];
    protected bool[]                myUIsoDeg  = new bool[4];
    protected bool                  myIsos;
    protected double                myUF;
    protected double                myUL;
    protected double                myVF;
    protected double                myVL;
    protected Curve3D?           myIsoUF;
    protected Curve3D?           myIsoUL;
    protected Curve3D?           myIsoVF;
    protected Curve3D?           myIsoVL;
    protected bool                  myIsoBoxes;
    protected Box               myBndUF = new Box();
    protected Box               myBndUL = new Box();
    protected Box               myBndVF = new Box();
    protected Box               myBndVL = new Box();
    protected double                myGap;
    protected double                myUDelt;
    protected double                myVDelt;
    protected double                myUCloseVal;
    protected double                myVCloseVal;

    private static void RestrictBounds(ref double theFirst, ref double theLast)
    {
        bool isFInf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theFirst);
        bool isLInf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theLast);
        if (isFInf || isLInf)
        {
            if (isFInf && isLInf)
            {
                theFirst = -1000;
                theLast  = 1000;
            }
            else if (isFInf)
            {
                theFirst = theLast - 2000;
            }
            else
            {
                theLast = theFirst + 2000;
            }
        }
    }

    private static void RestrictBounds(ref double theUf, ref double theUl, ref double theVf, ref double theVl)
    {
        RestrictBounds(ref theUf, ref theUl);
        RestrictBounds(ref theVf, ref theVl);
    }

    /// <summary>Creates an analyzer of an existing surface.</summary>
    /// <remarks>
    /// A null surface leaves the analyzer empty, as in OCCT (which sends a warning through Message, whose default
    /// messenger sends nothing in the port).
    /// </remarks>
    public ShapeAnalysis_Surface(Surface? S)
    {
        mySurf      = S;
        myExtOK     = false; //: 30
        myNbDeg     = -1;
        myIsos      = false;
        myIsoBoxes  = false;
        myGap       = 0.0;
        myUDelt     = 0.01;
        myVDelt     = 0.01;
        myUCloseVal = -1;
        myVCloseVal = -1;
        // Bug 33895: Prevent crash when surface is null
        if (mySurf is null)
        {
            // TODO(4A): Message.SendWarning("ShapeAnalysis_Surface: Cannot create with null surface");
            return;
        }

        mySurf.Bounds(out myUF, out myUL, out myVF, out myVL);
        myAdSur = new SurfaceAdaptor(mySurf);
    }

    /// <summary>Loads an existing surface.</summary>
    public void Init(Surface? theSurface)
    {
        if (ReferenceEquals(mySurf, theSurface))
        {
            return;
        }

        // Bug 33895: Prevent crash when surface is null
        if (theSurface is null)
        {
            // TODO(4A): Message.SendWarning("ShapeAnalysis_Surface: Cannot initialize with null surface");
            return;
        }

        myExtOK     = false; //: 30
        mySurf      = theSurface;
        myNbDeg     = -1;
        myUCloseVal = myVCloseVal = -1;
        myGap       = 0.0;
        mySurf.Bounds(out myUF, out myUL, out myVF, out myVL);
        myAdSur    = new SurfaceAdaptor(mySurf);
        myIsos     = false;
        myIsoBoxes = false;
        myIsoUF    = null;
        myIsoUL    = null;
        myIsoVF    = null;
        myIsoVL    = null;
    }

    /// <summary>Reads all the data from another analyzer, without recomputing (it shares the other's adaptor).</summary>
    public void Init(ShapeAnalysis_Surface other)
    {
        Init(other.Surface());
        myAdSur = other.TrueAdaptor3d();
        myNbDeg = other.myNbDeg; // rln S4135 direct transmission (to avoid computation in <other>)
        for (int i = 0; i < myNbDeg; i++)
        {
            other.Singularity(i + 1, ref myPreci[i], ref myP3d[i], ref myFirstP2d[i], ref myLastP2d[i],
                              ref myFirstPar[i], ref myLastPar[i], ref myUIsoDeg[i]);
        }
    }

    /// <summary>Sets the parametric domain analysed (by default the surface's bounds).</summary>
    public void SetDomain(double U1, double U2, double V1, double V2)
    {
        myUF = U1;
        myUL = U2;
        myVF = V1;
        myVL = V2;
    }

    /// <summary>Returns the surface being analysed.</summary>
    public Surface? Surface() => mySurf;

    /// <summary>Returns the adaptor of the surface.</summary>
    public SurfaceAdaptor? Adaptor3d() => myAdSur;

    /// <summary>Returns the adaptor (null when the surface is null).</summary>
    public SurfaceAdaptor? TrueAdaptor3d() => myAdSur;

    /// <summary>
    /// Returns the 3D distance found by the last of IsDegenerated, DegeneratedValues, ProjectDegenerated (to the found
    /// or last singularity), IsUClosed, IsVClosed (the smallest precision for which the surface is closed) and
    /// ValueOfUV (to the solution).
    /// </summary>
    public double Gap() => myGap;

    /// <summary>Returns the point of parameters (u, v).</summary>
    public Point3D Value(double u, double v) => myAdSur!.Value(u, v);

    /// <summary>Returns the point of parameters p2d.</summary>
    public Point3D Value(in Point2D p2d) => myAdSur!.Value(p2d.X(), p2d.Y());

    private void ComputeSingularities()
    {
        // rln S4135
        if (myNbDeg >= 0)
        {
            return;
        }

        //: 51 abv 22 Dec 97: allow forcing:  if (myNbDeg >= 0) return;
        // CKY 27-FEV-98 : en appel direct on peut forcer. En appel interne on optimise
        if (mySurf is null)
        {
            return;
        }

        Bounds(out double su1, out double su2, out double sv1, out double sv2); // modified by rln on 12/11/97 mySurf-> is deleted

        myNbDeg = 0; //: r3

        if (mySurf is ConicalSurface conicS)
        {
            double vApex = -conicS.RefRadius() / Math.Sin(conicS.SemiAngle());
            myPreci[0] = 0;
            myP3d[0]   = conicS.Apex();
            myFirstP2d[0].SetCoord(su1, vApex);
            myLastP2d[0].SetCoord(su2, vApex);
            myFirstPar[0] = su1;
            myLastPar[0]  = su2;
            myUIsoDeg[0]  = false;
            myNbDeg       = 1;
        }
        else if (mySurf is ToroidalSurface toroidS)
        {
            double minorR = toroidS.MinorRadius();
            double majorR = toroidS.MajorRadius();
            // szv#4:S4163:12Mar99 warning - possible div by zero
            double Ang = Math.Acos(Min(1.0, majorR / minorR));
            myPreci[0] = myPreci[1] = Max(0.0, majorR - minorR);
            myP3d[0]   = mySurf.Value(0.0, M_PI - Ang);
            myFirstP2d[0].SetCoord(su1, M_PI - Ang);
            myLastP2d[0].SetCoord(su2, M_PI - Ang);
            myP3d[1] = mySurf.Value(0.0, M_PI + Ang);
            myFirstP2d[1].SetCoord(su2, M_PI + Ang);
            myLastP2d[1].SetCoord(su1, M_PI + Ang);
            myFirstPar[0] = myFirstPar[1] = su1;
            myLastPar[0]  = myLastPar[1]  = su2;
            myUIsoDeg[0]  = myUIsoDeg[1]  = false;
            myNbDeg       = (majorR > minorR ? 1 : 2);
        }
        else if (mySurf is SphericalSurface)
        {
            myPreci[0] = myPreci[1] = 0;
            myP3d[0]   = mySurf.Value(su1, sv2); // Northern pole is first
            myP3d[1]   = mySurf.Value(su1, sv1);
            myFirstP2d[0].SetCoord(su2, sv2);
            myLastP2d[0].SetCoord(su1, sv2);
            myFirstP2d[1].SetCoord(su1, sv1);
            myLastP2d[1].SetCoord(su2, sv1);
            myFirstPar[0] = myFirstPar[1] = su1;
            myLastPar[0]  = myLastPar[1]  = su2;
            myUIsoDeg[0]  = myUIsoDeg[1]  = false;
            myNbDeg       = 2;
        }
        else if (mySurf is BoundedSurface || mySurf is RevolutionSurface //: b2 abv 18 Feb 98
                 || mySurf is global::Kerf.Geometry.OffsetSurface)
        { // rln S4135

            // rln S4135 //:r3
            myP3d[0] = myAdSur!.Value(su1, 0.5 * (sv1 + sv2));
            myFirstP2d[0].SetCoord(su1, sv2);
            myLastP2d[0].SetCoord(su1, sv1);

            myP3d[1] = myAdSur.Value(su2, 0.5 * (sv1 + sv2));
            myFirstP2d[1].SetCoord(su2, sv1);
            myLastP2d[1].SetCoord(su2, sv2);

            myP3d[2] = myAdSur.Value(0.5 * (su1 + su2), sv1);
            myFirstP2d[2].SetCoord(su1, sv1);
            myLastP2d[2].SetCoord(su2, sv1);

            myP3d[3] = myAdSur.Value(0.5 * (su1 + su2), sv2);
            myFirstP2d[3].SetCoord(su2, sv2);
            myLastP2d[3].SetCoord(su1, sv2);

            myFirstPar[0] = myFirstPar[1] = sv1;
            myLastPar[0]  = myLastPar[1]  = sv2;
            myUIsoDeg[0]  = myUIsoDeg[1]  = true;

            myFirstPar[2] = myFirstPar[3] = su1;
            myLastPar[2]  = myLastPar[3]  = su2;
            myUIsoDeg[2]  = myUIsoDeg[3]  = false;

            Point3D Corner1 = myAdSur.Value(su1, sv1);
            Point3D Corner2 = myAdSur.Value(su1, sv2);
            Point3D Corner3 = myAdSur.Value(su2, sv1);
            Point3D Corner4 = myAdSur.Value(su2, sv2);

            myPreci[0] = Max(Corner1.Distance(Corner2), Max(myP3d[0].Distance(Corner1), myP3d[0].Distance(Corner2)));
            myPreci[1] = Max(Corner3.Distance(Corner4), Max(myP3d[1].Distance(Corner3), myP3d[1].Distance(Corner4)));
            myPreci[2] = Max(Corner1.Distance(Corner3), Max(myP3d[2].Distance(Corner1), myP3d[2].Distance(Corner3)));
            myPreci[3] = Max(Corner2.Distance(Corner4), Max(myP3d[3].Distance(Corner2), myP3d[3].Distance(Corner4)));

            myNbDeg = 4;
        }

        SortSingularities();
    }

    /// <summary>
    /// Returns true if the surface has singularities for the given precision (singularities whose sizes are not
    /// greater than it).
    /// </summary>
    public bool HasSingularities(double preci)
    {
        return NbSingularities(preci) > 0;
    }

    /// <summary>
    /// Returns the number of singularities for the given precision (singularities whose sizes are not greater than
    /// it).
    /// </summary>
    public int NbSingularities(double preci)
    {
        if (myNbDeg < 0)
        {
            ComputeSingularities();
        }

        int Nb = 0;
        for (int i = 1; i <= myNbDeg; i++)
        {
            if (myPreci[i - 1] <= preci)
            {
                Nb++;
            }
        }

        return Nb;
    }

    /// <summary>
    /// Returns the singularity of rank <paramref name="num"/> (at most the number of possible singularities, not
    /// only those of a precision): the smallest precision with which its iso-line is degenerated, its 3D point (the
    /// middle of the iso-line), the first and last 2D points and parameters of the iso-line, and whether it is a U
    /// iso-line. Returns false, leaving the outputs unchanged, if <paramref name="num"/> is out of range.
    /// </summary>
    public bool Singularity(int num, ref double preci, ref Point3D P3d, ref Point2D firstP2d, ref Point2D lastP2d,
                            ref double firstpar, ref double lastpar, ref bool uisodeg)
    {
        // The fields are C arrays numbered from 0; num starts at 1
        if (myNbDeg < 0)
        {
            ComputeSingularities();
        }

        if (num < 1 || num > myNbDeg)
        {
            return false;
        }

        P3d      = myP3d[num - 1];
        preci    = myPreci[num - 1];
        firstP2d = myFirstP2d[num - 1];
        lastP2d  = myLastP2d[num - 1];
        firstpar = myFirstPar[num - 1];
        lastpar  = myLastPar[num - 1];
        uisodeg  = myUIsoDeg[num - 1];
        return true;
    }

    /// <summary>
    /// Returns true if a surface boundary is degenerated with <paramref name="preci"/> and the point is within
    /// <paramref name="preci"/> of its singular point.
    /// </summary>
    public bool IsDegenerated(in Point3D P3d, double preci)
    {
        if (myNbDeg < 0)
        {
            ComputeSingularities();
        }

        for (int i = 0; i < myNbDeg && myPreci[i] <= preci; i++)
        {
            myGap = myP3d[i].Distance(P3d);
            // rln S4135
            if (myGap <= preci)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// As <see cref="IsDegenerated(in Point3D, double)"/>, and returns the characteristics of the singularity nearest
    /// to the point among those matching (the outputs are unchanged when there is none).
    /// </summary>
    public bool DegeneratedValues(in Point3D P3d, double preci, ref Point2D firstP2d, ref Point2D lastP2d,
                                  ref double firstPar, ref double lastPar, bool forward = true)
    {
        _ = forward;
        if (myNbDeg < 0)
        {
            ComputeSingularities();
        }

        // #77 rln S4135: returning singularity which has minimum gap between singular point and input 3D point
        int    indMin = -1;
        double gapMin = RealLast();
        for (int i = 0; i < myNbDeg && myPreci[i] <= preci; i++)
        {
            myGap = myP3d[i].Distance(P3d);
            // rln S4135
            if (myGap <= preci)
            {
                if (gapMin > myGap)
                {
                    gapMin = myGap;
                    indMin = i;
                }
            }
        }

        if (indMin >= 0)
        {
            myGap    = gapMin;
            firstP2d = myFirstP2d[indMin];
            lastP2d  = myLastP2d[indMin];
            firstPar = myFirstPar[indMin];
            lastPar  = myLastPar[indMin];
            return true;
        }

        return false;
    }

    /// <summary>
    /// Projects a point on a singularity by completing <paramref name="result"/>: finds the iso-line degenerated with
    /// <paramref name="preci"/> whose singular point is within <paramref name="preci"/> of the point (or of the 3D
    /// point of <paramref name="result"/>), and sets the coordinate of <paramref name="result"/> along it from
    /// <paramref name="neighbour"/>.
    /// </summary>
    public bool ProjectDegenerated(in Point3D P3d, double preci, in Point2D neighbour, ref Point2D result)
    {
        if (myNbDeg < 0)
        {
            ComputeSingularities();
        }

        // added by rln on 03/12/97
        //: c1 abv 23 Feb 98: preci (3d) -> Resolution (2d)
        // #77 rln S4135
        int    indMin = -1;
        double gapMin = RealLast();
        for (int i = 0; i < myNbDeg && myPreci[i] <= preci; i++)
        {
            double gap2 = myP3d[i].SquareDistance(P3d);
            if (gap2 > preci * preci)
            {
                gap2 = Min(gap2, myP3d[i].SquareDistance(Value(result)));
            }

            // rln S4135
            if (gap2 <= preci * preci && gapMin > gap2)
            {
                gapMin = gap2;
                indMin = i;
            }
        }

        if (indMin < 0)
        {
            return false;
        }

        myGap = Math.Sqrt(gapMin);
        if (!myUIsoDeg[indMin])
        {
            result.SetX(neighbour.X());
        }
        else
        {
            result.SetY(neighbour.Y());
        }

        return true;
    }

    // pdn %12 11.02.99 PRO9234 entity 15402
    /// <summary>
    /// Checks whether the points at the start (<paramref name="direct"/>) or end of <paramref name="points"/> lie in
    /// a singularity, and if so sets their undetermined 2D coordinate from the nearest point that doesn't (evenly
    /// distributed if all do). Returns true if some points were adjusted.
    /// </summary>
    public bool ProjectDegenerated(int nbrPnt, NCollection_Sequence<Point3D> points, NCollection_Sequence<Point2D> pnt2d,
                                   double preci, bool direct)
    {
        if (myNbDeg < 0)
        {
            ComputeSingularities();
        }

        int step = (direct ? 1 : -1);
        // #77 rln S4135
        int    indMin = -1;
        double gapMin = RealLast(), prec2 = preci * preci;
        int    j      = (direct ? 1 : nbrPnt);
        for (int i = 0; i < myNbDeg && myPreci[i] <= preci; i++)
        {
            double gap2 = myP3d[i].SquareDistance(points[j]);
            if (gap2 > prec2)
            {
                gap2 = Min(gap2, myP3d[i].SquareDistance(Value(pnt2d[j])));
            }

            if (gap2 <= prec2 && gapMin > gap2)
            {
                gapMin = gap2;
                indMin = i;
            }
        }

        if (indMin < 0)
        {
            return false;
        }

        myGap = Math.Sqrt(gapMin);
        Point2D pk = new Point2D();

        int k;
        for (k = j + step; k <= nbrPnt && k >= 1; k += step)
        {
            pk = pnt2d[k];
            Point3D P1 = points[k];
            if (myP3d[indMin].SquareDistance(P1) > prec2 && myP3d[indMin].SquareDistance(Value(pk)) > prec2)
            {
                break;
            }
        }

        //: p8 abv 11 Mar 99: PRO7226 #489490: if whole pcurve is degenerate, distribute evenly
        if (k < 1 || k > nbrPnt)
        {
            double x1 = (myUIsoDeg[indMin] ? pnt2d[1].Y() : pnt2d[1].X());
            double x2 = (myUIsoDeg[indMin] ? pnt2d[nbrPnt].Y() : pnt2d[nbrPnt].X());
            for (j = 1; j <= nbrPnt; j++)
            {
                // szv#4:S4163:12Mar99 warning - possible div by zero
                double x = (x1 * (nbrPnt - j) + x2 * (j - 1)) / (nbrPnt - 1);
                if (!myUIsoDeg[indMin])
                {
                    pnt2d[j].SetX(x);
                }
                else
                {
                    pnt2d[j].SetY(x);
                }
            }

            return true;
        }

        for (j = k - step; j <= nbrPnt && j >= 1; j -= step)
        {
            if (!myUIsoDeg[indMin])
            {
                pnt2d[j].SetX(pk.X());
            }
            else
            {
                pnt2d[j].SetY(pk.Y());
            }
        }

        return true;
    }

    /// <summary>
    /// Returns true if the straight pcurve from p2d1 to p2d2 is degenerate: its 3D extent (ends and middle) is within
    /// <paramref name="tol"/>, and its 2D extent is more than <paramref name="ratio"/> times the resolution of that
    /// 3D extent. This detects singularities otherwise than ComputeSingularities.
    /// </summary>
    public bool IsDegenerated(in Point2D p2d1, in Point2D p2d2, double tol, double ratio)
    {
        Point3D p1    = Value(p2d1);
        Point3D p2    = Value(p2d2);
        Point3D pm    = Value(new Point2D(0.5 * (p2d1.XY() + p2d2.XY())));
        double max3d = Max(p1.Distance(p2), Max(pm.Distance(p1), pm.Distance(p2)));
        if (max3d > tol)
        {
            return false;
        }

        SurfaceAdaptor SA = Adaptor3d()!;
        double              RU = SA.UResolution(1.0);
        double              RV = SA.VResolution(1.0);

        if (RU < global::Kerf.Engine.Foundation.Precision.PConfusion() || RV < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        double du = Math.Abs(p2d1.X() - p2d2.X()) / RU;
        double dv = Math.Abs(p2d1.Y() - p2d2.Y()) / RV;
        max3d *= ratio;
        return du * du + dv * dv > max3d * max3d;
    }

    // The iso-curve, frozen (it is cached and handed out), or null where the surface can't make it
    private static Curve3D? ComputeIso(Surface surf, bool utype, double par)
    {
        Curve3D? iso;
        try
        {
            if (utype)
            {
                iso = surf.UIso(par);
            }
            else
            {
                iso = surf.VIso(par);
            }
        }
        catch (FailureException)
        {
            iso = null;
        }

        iso?.Freeze(); // OCCT.NET: a curve handed out (and cached) is frozen
        return iso;
    }

    /// <summary>Computes the bound iso-curves (protected against exceptions).</summary>
    public void ComputeBoundIsos()
    {
        if (myIsos)
        {
            return;
        }

        myIsos  = true;
        myIsoUF = ComputeIso(mySurf!, true, myUF);
        myIsoUL = ComputeIso(mySurf!, true, myUL);
        myIsoVF = ComputeIso(mySurf!, false, myVF);
        myIsoVL = ComputeIso(mySurf!, false, myVL);
    }

    /// <summary>Returns a U iso-curve (frozen; the bound ones are cached), or null if it can't be made.</summary>
    public Curve3D? UIso(double U)
    {
        if (U == myUF)
        {
            ComputeBoundIsos();
            return myIsoUF;
        }

        if (U == myUL)
        {
            ComputeBoundIsos();
            return myIsoUL;
        }

        return ComputeIso(mySurf!, true, U);
    }

    /// <summary>Returns a V iso-curve (frozen; the bound ones are cached), or null if it can't be made.</summary>
    public Curve3D? VIso(double V)
    {
        if (V == myVF)
        {
            ComputeBoundIsos();
            return myIsoVF;
        }

        if (V == myVL)
        {
            ComputeBoundIsos();
            return myIsoVL;
        }

        return ComputeIso(mySurf!, false, V);
    }

    /// <summary>
    /// Tells if the surface is spatially closed in U within <paramref name="preci"/> (Precision::Confusion() if
    /// negative): if Geom_Surface says so, else by the distance between its U bounds (poles of polynomial B-splines
    /// with full end multiplicities and of Bezier surfaces; values at the knots and their middles of the other
    /// B-splines; the ends of the basis curve of an extrusion; 101 points otherwise).
    /// </summary>
    public bool IsUClosed(double preci = -1)
    {
        double prec      = Max(preci, global::Kerf.Engine.Foundation.Precision.Confusion());
        double anUmidVal = -1.0;
        if (myUCloseVal < 0)
        {
            // To compute: minimal computations
            Bounds(out double uf, out double ul, out double vf, out double vl); // modified by rln on 12/11/97 mySurf-> is deleted
            RestrictBounds(ref uf, ref ul, ref vf, ref vl);
            myUDelt = Math.Abs(ul - uf) / 20; // modified by rln 11/11/97 instead of 10
                                              // because of the example when 10 was not enough
            if (mySurf!.IsUClosed)
            {
                myUCloseVal = 0.0;
                myUDelt     = 0.0;
                myGap       = 0.0;
                return true;
            }

            // Adapted computations
            // #67 rln S4135
            SurfaceAdaptor SurfAdapt = Adaptor3d()!;
            SurfaceKind surftype  = SurfAdapt.GetType();
            if (mySurf is TrimmedSurface)
            {
                surftype = OtherSurface;
            }

            switch (surftype)
            {
                case Plane:
                {
                    myUCloseVal = RealLast();
                    break;
                }
                case SurfaceOfExtrusion:
                { //: c8 abv 03 Mar 98: UKI60094 #753: process Geom_SurfaceOfLinearExtrusion
                    var        extr = (ExtrusionSurface)mySurf;
                    Curve3D crv  = extr.BasisCurve();
                    double     f    = crv.FirstParameter();
                    double     l    = crv.LastParameter();
                    //: r3 abv (smh) 30 Mar 99: protect against unexpected signals
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
                    {
                        Point3D p1 = crv.Value(f);
                        Point3D p2 = crv.Value(l);
                        myUCloseVal = p1.SquareDistance(p2);
                        Point3D pm = crv.Value((f + l) / 2.0);
                        anUmidVal = p1.SquareDistance(pm);
                    }
                    else
                    {
                        myUCloseVal = RealLast();
                    }

                    break;
                }
                case SurfaceKind.BSplineSurface:
                {
                    var    bs      = (global::Kerf.Geometry.BSplineSurface)mySurf;
                    int    nbup    = bs.UPoleCount;
                    double distmin = RealLast();
                    if (bs.IsUPeriodic)
                    {
                        myUCloseVal = 0;
                        myUDelt     = 0;
                    }
                    else if (nbup < 3)
                    { // modified by rln on 12/11/97
                        myUCloseVal = RealLast();
                    }
                    else if (bs.IsURational ||
                             // #6 rln 20/02/98 ProSTEP ug_exhaust-A.stp entity #18360 (Uclosed BSpline,
                             // but multiplicity of boundary knots != degree + 1)
                             bs.UMultiplicity(1) != bs.UDegree() + 1 || // #6 //:h4: #6 moved
                             bs.UMultiplicity(bs.UKnotCount) != bs.UDegree() + 1)
                    { // #6 //:h4
                        int    nbvk = bs.VKnotCount;
                        double v    = bs.VKnot(1);
                        Point3D p1   = SurfAdapt.Value(uf, v);
                        Point3D p2   = SurfAdapt.Value(ul, v);
                        myUCloseVal = p1.SquareDistance(p2);
                        Point3D pm = SurfAdapt.Value((uf + ul) / 2.0, v);
                        anUmidVal = p1.SquareDistance(pm);
                        distmin   = myUCloseVal;
                        for (int i = 2; i <= nbvk; i++)
                        {
                            v  = 0.5 * (bs.VKnot(i - 1) + bs.VKnot(i));
                            p1 = bs.Value(uf, v);
                            p2 = bs.Value(ul, v);
                            double aDist = p1.SquareDistance(p2);
                            if (aDist > myUCloseVal)
                            {
                                myUCloseVal = aDist;
                                pm          = bs.Value((uf + ul) / 2.0, v);
                                anUmidVal   = p1.SquareDistance(pm);
                            }
                            else
                            {
                                distmin = Min(distmin, aDist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        myUDelt = Min(myUDelt, 0.5 * SurfAdapt.UResolution(distmin)); // #4 smh
                    }
                    else
                    {
                        int nbvp = bs.VPoleCount;
                        myUCloseVal = bs.Pole(1, 1).SquareDistance(bs.Pole(nbup, 1));
                        anUmidVal   = bs.Pole(1, 1).SquareDistance(bs.Pole(nbup / 2 + 1, 1));
                        distmin     = myUCloseVal;
                        for (int i = 2; i <= nbvp; i++)
                        {
                            double aDist = bs.Pole(1, i).SquareDistance(bs.Pole(nbup, i));
                            if (aDist > myUCloseVal)
                            {
                                myUCloseVal = aDist;
                                anUmidVal   = bs.Pole(1, i).SquareDistance(bs.Pole(nbup / 2 + 1, i));
                            }
                            else
                            {
                                distmin = Min(distmin, aDist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        myUDelt = Min(myUDelt, 0.5 * SurfAdapt.UResolution(distmin)); // #4 smh
                    }

                    break;
                }
                case SurfaceKind.BezierSurface:
                {
                    var    bz      = (global::Kerf.Geometry.BezierSurface)mySurf;
                    int    nbup    = bz.UPoleCount;
                    double distmin = RealLast();
                    if (nbup < 3)
                    {
                        myUCloseVal = RealLast();
                    }
                    else
                    {
                        int nbvp = bz.VPoleCount;
                        myUCloseVal = bz.Pole(1, 1).SquareDistance(bz.Pole(nbup, 1));
                        anUmidVal   = bz.Pole(1, 1).SquareDistance(bz.Pole(nbup / 2 + 1, 1));
                        distmin     = myUCloseVal;
                        for (int i = 1; i <= nbvp; i++)
                        {
                            double aDist = bz.Pole(1, i).SquareDistance(bz.Pole(nbup, i));
                            if (aDist > myUCloseVal)
                            {
                                myUCloseVal = aDist;
                                anUmidVal   = bz.Pole(1, i).SquareDistance(bz.Pole(nbup / 2 + 1, i));
                            }
                            else
                            {
                                distmin = Min(distmin, aDist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        myUDelt = Min(myUDelt, 0.5 * SurfAdapt.UResolution(distmin)); // #4 smh
                    }

                    break;
                }
                default:
                { // Geom_RectangularTrimmedSurface and Geom_OffsetSurface
                    double distmin  = RealLast();
                    int    nbpoints = 101; // can be revised
                    Point3D p1       = SurfAdapt.Value(uf, vf);
                    Point3D p2       = SurfAdapt.Value(ul, vf);
                    myUCloseVal = p1.SquareDistance(p2);
                    Point3D pm = SurfAdapt.Value((uf + ul) / 2, vf);
                    anUmidVal = p1.SquareDistance(pm);
                    distmin   = myUCloseVal;
                    for (int i = 1; i < nbpoints; i++)
                    {
                        double vparam = vf + (vl - vf) * i / (nbpoints - 1);
                        p1 = SurfAdapt.Value(uf, vparam);
                        p2 = SurfAdapt.Value(ul, vparam);
                        double aDist = p1.SquareDistance(p2);
                        if (aDist > myUCloseVal)
                        {
                            myUCloseVal = aDist;
                            pm          = SurfAdapt.Value((uf + ul) / 2, vparam);
                            anUmidVal   = p1.SquareDistance(pm);
                        }
                        else
                        {
                            distmin = Min(distmin, aDist);
                        }
                    }

                    distmin = Math.Sqrt(distmin);
                    myUDelt = Min(myUDelt, 0.5 * SurfAdapt.UResolution(distmin)); // #4 smh
                    break;
                }
            } // switch

            myGap       = Math.Sqrt(myUCloseVal);
            myUCloseVal = myGap;
        }

        if (anUmidVal > 0.0 && myUCloseVal > Math.Sqrt(anUmidVal))
        {
            myUCloseVal = RealLast();
            return false;
        }

        return (myUCloseVal <= prec);
    }

    /// <summary>
    /// Tells if the surface is spatially closed in V within <paramref name="preci"/> (see
    /// <see cref="IsUClosed"/>; a surface of revolution by the ends of its basis curve).
    /// </summary>
    public bool IsVClosed(double preci = -1)
    {
        double prec     = Max(preci, global::Kerf.Engine.Foundation.Precision.Confusion());
        double aVmidVal = -1.0;
        if (myVCloseVal < 0)
        {
            // To compute: minimal computations
            Bounds(out double uf, out double ul, out double vf, out double vl); // modified by rln on 12/11/97 mySurf-> is deleted
            RestrictBounds(ref uf, ref ul, ref vf, ref vl);
            myVDelt = Math.Abs(vl - vf) / 20; // 2; rln S4135
                                              // because of the example when 10 was not enough
            if (mySurf!.IsVClosed)
            {
                myVCloseVal = 0.0;
                myVDelt     = 0.0;
                myGap       = 0.0;
                return true;
            }

            // Adapted computations
            // #67 rln S4135
            SurfaceAdaptor SurfAdapt = Adaptor3d()!;
            SurfaceKind surftype  = SurfAdapt.GetType();
            if (mySurf is TrimmedSurface)
            {
                surftype = OtherSurface;
            }

            switch (surftype)
            {
                case Plane:
                case SurfaceKind.Cone:
                case SurfaceKind.Cylinder:
                case SurfaceKind.Sphere:
                case SurfaceOfExtrusion:
                {
                    myVCloseVal = RealLast();
                    break;
                }
                case SurfaceOfRevolution:
                {
                    var        revol = (RevolutionSurface)mySurf;
                    Curve3D crv   = revol.BasisCurve();
                    Point3D     p1    = crv.Value(crv.FirstParameter());
                    Point3D     p2    = crv.Value(crv.LastParameter());
                    myVCloseVal = p1.SquareDistance(p2);
                    break;
                }
                case SurfaceKind.BSplineSurface:
                {
                    var    bs      = (global::Kerf.Geometry.BSplineSurface)mySurf;
                    int    nbvp    = bs.VPoleCount;
                    double distmin = RealLast();
                    if (bs.IsVPeriodic)
                    {
                        myVCloseVal = 0;
                        myVDelt     = 0;
                    }
                    else if (nbvp < 3)
                    { // modified by rln on 12/11/97
                        myVCloseVal = RealLast();
                    }
                    else if (bs.IsVRational || bs.VMultiplicity(1) != bs.VDegree() + 1 || // #6 //:h4
                             bs.VMultiplicity(bs.VKnotCount) != bs.VDegree() + 1)
                    { // #6 //:h4
                        int    nbuk = bs.UKnotCount;
                        double u    = bs.UKnot(1);
                        Point3D p1   = SurfAdapt.Value(u, vf);
                        Point3D p2   = SurfAdapt.Value(u, vl);
                        myVCloseVal = p1.SquareDistance(p2);
                        Point3D pm = SurfAdapt.Value(u, (vf + vl) / 2.0);
                        aVmidVal = p1.SquareDistance(pm);
                        distmin  = myVCloseVal;
                        for (int i = 2; i <= nbuk; i++)
                        {
                            u  = 0.5 * (bs.UKnot(i - 1) + bs.UKnot(i));
                            p1 = SurfAdapt.Value(u, vf);
                            p2 = SurfAdapt.Value(u, vl);
                            double aDist = p1.SquareDistance(p2);
                            if (aDist > myVCloseVal)
                            {
                                myVCloseVal = aDist;
                                pm          = SurfAdapt.Value(u, (vf + vl) / 2);
                                aVmidVal    = p1.SquareDistance(pm);
                            }
                            else
                            {
                                distmin = Min(distmin, aDist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        myVDelt = Min(myVDelt, 0.5 * SurfAdapt.VResolution(distmin)); // #4 smh
                    }
                    else
                    {
                        int nbup = bs.UPoleCount;
                        myVCloseVal = bs.Pole(1, 1).SquareDistance(bs.Pole(1, nbvp));
                        aVmidVal    = bs.Pole(1, 1).SquareDistance(bs.Pole(1, nbvp / 2 + 1));
                        distmin     = myVCloseVal;
                        for (int i = 2; i <= nbup; i++)
                        {
                            double aDist = bs.Pole(i, 1).SquareDistance(bs.Pole(i, nbvp));
                            if (aDist > myVCloseVal)
                            {
                                myVCloseVal = aDist;
                                aVmidVal    = bs.Pole(i, 1).SquareDistance(bs.Pole(i, nbvp / 2 + 1));
                            }
                            else
                            {
                                distmin = Min(distmin, aDist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        myVDelt = Min(myVDelt, 0.5 * SurfAdapt.VResolution(distmin)); // #4 smh
                    }

                    break;
                }
                case SurfaceKind.BezierSurface:
                {
                    var    bz      = (global::Kerf.Geometry.BezierSurface)mySurf;
                    int    nbvp    = bz.VPoleCount;
                    double distmin = RealLast();
                    if (nbvp < 3)
                    {
                        myVCloseVal = RealLast();
                    }
                    else
                    {
                        int nbup = bz.UPoleCount;
                        myVCloseVal = bz.Pole(1, 1).SquareDistance(bz.Pole(1, nbvp));
                        aVmidVal    = bz.Pole(1, 1).SquareDistance(bz.Pole(1, nbvp / 2 + 1));
                        distmin     = myVCloseVal;
                        for (int i = 2; i <= nbup; i++)
                        {
                            double aDist = bz.Pole(i, 1).SquareDistance(bz.Pole(i, nbvp));
                            if (aDist > myVCloseVal)
                            {
                                myVCloseVal = aDist;
                                aVmidVal    = bz.Pole(i, 1).SquareDistance(bz.Pole(i, nbvp / 2 + 1));
                            }
                            else
                            {
                                distmin = Min(distmin, aDist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        myVDelt = Min(myVDelt, 0.5 * SurfAdapt.VResolution(distmin)); // #4 smh
                    }

                    break;
                }
                default:
                { // Geom_RectangularTrimmedSurface and Geom_OffsetSurface
                    double distmin  = RealLast();
                    int    nbpoints = 101; // can be revised
                    Point3D p1       = SurfAdapt.Value(uf, vf);
                    Point3D p2       = SurfAdapt.Value(uf, vl);
                    Point3D pm       = SurfAdapt.Value(uf, (vf + vl) / 2);
                    myVCloseVal = p1.SquareDistance(p2);
                    aVmidVal    = p1.SquareDistance(pm);
                    distmin     = myVCloseVal;
                    for (int i = 1; i < nbpoints; i++)
                    {
                        double uparam = uf + (ul - uf) * i / (nbpoints - 1);
                        p1 = SurfAdapt.Value(uparam, vf);
                        p2 = SurfAdapt.Value(uparam, vl);
                        double aDist = p1.SquareDistance(p2);
                        if (aDist > myVCloseVal)
                        {
                            myVCloseVal = aDist;
                            pm          = SurfAdapt.Value(uparam, (vf + vl) / 2);
                            aVmidVal    = p1.SquareDistance(pm);
                        }
                        else
                        {
                            distmin = Min(distmin, aDist);
                        }
                    }

                    distmin = Math.Sqrt(distmin);
                    myVDelt = Min(myVDelt, 0.5 * SurfAdapt.VResolution(distmin)); // #4 smh
                    break;
                }
            } // switch

            myGap       = Math.Sqrt(myVCloseVal);
            myVCloseVal = myGap;
        }

        if (aVmidVal > 0.0 && myVCloseVal > Math.Sqrt(aVmidVal))
        {
            myVCloseVal = RealLast();
            return false;
        }

        return (myVCloseVal <= prec);
    }

    // Newton algo (S4030); returns 0 (failed), 1 or 2 (found, near a singular point)
    private int SurfaceNewton(in Point2D p2dPrev, in Point3D P3D, double preci, ref Point2D sol)
    {
        SurfaceAdaptor SurfAdapt = Adaptor3d()!;
        Bounds(out double uf, out double ul, out double vf, out double vl);
        double du = SurfAdapt.UResolution(preci);
        double dv = SurfAdapt.VResolution(preci);
        double UF = uf - du, UL = ul + du;
        double VF = vf - dv, VL = vl + dv;

        double Tol     = global::Kerf.Engine.Foundation.Precision.Confusion();
        double Tol2    = Tol * Tol; //, rs2p=1e10;
        double U       = p2dPrev.X(), V = p2dPrev.Y();
        var    rsfirst = new Vector3D(P3D.XYZ() - Value(U, V).XYZ()); // pdn
        for (int i = 0; i < 25; i++)
        {
            SurfAdapt.D2(U, V, out Point3D pnt, out Vector3D ru, out Vector3D rv, out Vector3D ruu, out Vector3D rvv, out Vector3D ruv);

            // normal
            double ru2  = ru * ru, rv2 = rv * rv;
            Vector3D n    = ru ^ rv;
            double nrm2 = n.SquareMagnitude();
            if (nrm2 < 1e-10 || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(nrm2))
            {
                break; // n == 0, use standard
            }

            // discriminant
            var    rs   = new Vector3D(P3D.XYZ() - Value(U, V).XYZ());
            double rSuu = (rs * ruu);
            double rSvv = (rs * rvv);
            double rSuv = (rs * ruv);
            double D    = -nrm2 + rv2 * rSuu + ru2 * rSvv - 2 * rSuv * (ru * rv) + rSuv * rSuv - rSuu * rSvv;
            if (Math.Abs(D) < 1e-10)
            {
                break; // bad case; use standard
            }

            // compute step
            double fract = 1.0 / D;
            du = (rs * ((n ^ rv) + ru * rSvv - rv * rSuv)) * fract;
            dv = (rs * ((ru ^ n) + rv * rSuu - ru * rSuv)) * fract;
            U += du;
            V += dv;
            if (U < UF || U > UL || V < VF || V > VL)
            {
                break;
            }

            // test the step by uv and deviation from the solution
            double aResolution = Max(1e-12, (U + V) * 10e-16);
            if (Math.Abs(du) + Math.Abs(dv) > aResolution)
            {
                continue; // Precision::PConfusion()  continue;
            }

            // pdn PRO10109 4517: protect against wrong result
            double rs2 = rs.SquareMagnitude();
            if (rs2 > rsfirst.SquareMagnitude())
            {
                break;
            }

            double rsn = rs * n;
            if (rs2 - rsn * rsn / nrm2 > Tol2)
            {
                break;
            }

            // OK, return the result
            sol.SetCoord(U, V);

            return (nrm2 < 0.01 * ru2 * rv2 ? 2 : 1); //: q6
        }

        return 0;
    }

    /// <summary>
    /// Projects a point on the surface as <see cref="ValueOfUV"/>, starting from the previous solution
    /// <paramref name="p2dPrev"/> by Newton's method. A solution farther than <paramref name="maxpreci"/> (when
    /// positive) from the point is bad, and <see cref="UVFromIso"/> is tried; if Newton fails, ValueOfUV is used.
    /// </summary>
    public Point2D NextValueOfUV(in Point2D p2dPrev, in Point3D P3D, double preci, double maxpreci = -1.0)
    {
        SurfaceAdaptor SurfAdapt = Adaptor3d()!;
        SurfaceKind surftype  = SurfAdapt.GetType();

        switch (surftype)
        {
            case SurfaceKind.BezierSurface:
            case SurfaceKind.BSplineSurface:
            case SurfaceOfExtrusion:
            case SurfaceOfRevolution:
            case SurfaceKind.OffsetSurface:
            {
                if (surftype == SurfaceKind.BSplineSurface)
                {
                    global::Kerf.Geometry.BSplineSurface aBSpline = SurfAdapt.BSpline();

                    // Check near to knot position ~ near to C0 points on U isoline.
                    if (SurfAdapt.UContinuity() == C0)
                    {
                        int aMinIndex = aBSpline.FirstUKnotIndex();
                        int aMaxIndex = aBSpline.LastUKnotIndex();
                        for (int anIdx = aMinIndex; anIdx <= aMaxIndex; ++anIdx)
                        {
                            double aKnot = aBSpline.UKnot(anIdx);
                            if (Math.Abs(aKnot - p2dPrev.X()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                return ValueOfUV(P3D, preci);
                            }
                        }
                    }

                    // Check near to knot position ~ near to C0 points on U isoline.
                    if (SurfAdapt.VContinuity() == C0)
                    {
                        int aMinIndex = aBSpline.FirstVKnotIndex();
                        int aMaxIndex = aBSpline.LastVKnotIndex();
                        for (int anIdx = aMinIndex; anIdx <= aMaxIndex; ++anIdx)
                        {
                            double aKnot = aBSpline.VKnot(anIdx);
                            if (Math.Abs(aKnot - p2dPrev.Y()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                return ValueOfUV(P3D, preci);
                            }
                        }
                    }
                }

                Point2D sol = new Point2D();
                int      res = SurfaceNewton(p2dPrev, P3D, preci, ref sol);
                if (res != 0)
                {
                    double gap = P3D.Distance(Value(sol));
                    if (res == 2 || //: q6 abv 19 Mar 99: protect against strange attractors
                        (maxpreci > 0.0 && gap - maxpreci > global::Kerf.Engine.Foundation.Precision.Confusion()))
                    { //: q1: check with maxpreci
                        double U = sol.X(), V = sol.Y();
                        myGap = UVFromIso(P3D, preci, ref U, ref V);
                        if (gap >= myGap)
                        {
                            return new Point2D(U, V);
                        }
                    }

                    myGap = gap;
                    return sol;
                }

                break;
            }
            default:
                break;
        }

        return ValueOfUV(P3D, preci);
    }

    /// <summary>
    /// Computes the parameters on the surface of the projection of a 3D point: analytically on elementary surfaces,
    /// else by Extrema_ExtPS improved by Newton and by projection on iso-lines (<see cref="UVFromIso"/>), which also
    /// handles points near the surface's boundaries and failures of Extrema. Sets <see cref="Gap"/> to the distance
    /// from the point to the solution (-1 and (0, 0) on surfaces it doesn't handle).
    /// </summary>
    public Point2D ValueOfUV(in Point3D P3D, double preci)
    {
        SurfaceAdaptor SurfAdapt = Adaptor3d()!;
        double              S         = 0.0, T = 0.0;
        myGap = -1.0;          // to be computed
        bool computed = true; // a priori

        Bounds(out double uf, out double ul, out double vf, out double vl); // modified by rln on 12/11/97 mySurf-> is deleted

        { //: c9 abv 3 Mar 98: UKI60107-1 #350: to prevent 'catch' from catching exception raising below it
            try
            { // ajout CKY 30-DEC-1997 (cf ProStep TR6 r_89-ug)
                SurfaceKind surftype = SurfAdapt.GetType();
                switch (surftype)
                {
                    case Plane:
                    {
                        Plane3D Plane = SurfAdapt.Plane();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Plane, P3D, out S, out T);
                        break;
                    }
                    case SurfaceKind.Cylinder:
                    {
                        Cylinder Cylinder = SurfAdapt.Cylinder();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cylinder, P3D, out S, out T);
                        S += ShapeAnalysis.AdjustByPeriod(S, 0.5 * (uf + ul), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.Cone:
                    {
                        Cone Cone = SurfAdapt.Cone();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cone, P3D, out S, out T);
                        S += ShapeAnalysis.AdjustByPeriod(S, 0.5 * (uf + ul), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.Sphere:
                    {
                        Sphere Sphere = SurfAdapt.Sphere();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Sphere, P3D, out S, out T);
                        S += ShapeAnalysis.AdjustByPeriod(S, 0.5 * (uf + ul), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.Torus:
                    {
                        Torus Torus = SurfAdapt.Torus();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(Torus, P3D, out S, out T);
                        S += ShapeAnalysis.AdjustByPeriod(S, 0.5 * (uf + ul), 2 * M_PI);
                        T += ShapeAnalysis.AdjustByPeriod(T, 0.5 * (vf + vl), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.BezierSurface:
                    case SurfaceKind.BSplineSurface:
                    case SurfaceOfExtrusion:
                    case SurfaceOfRevolution:
                    case SurfaceKind.OffsetSurface: //: d0 abv 3 Mar 98: UKI60107-1 #350
                    {
                        S = (uf + ul) / 2;
                        T = (vf + vl) / 2; // there will be at least something
                                           // pdn to fix hangs PRO17015
                        if ((surftype == SurfaceOfExtrusion) && global::Kerf.Engine.Foundation.Precision.IsInfinite(uf)
                            && global::Kerf.Engine.Foundation.Precision.IsInfinite(ul))
                        {
                            // conic case
                            var      prev     = new Point2D(S, T);
                            Point2D solution = new Point2D();
                            if (SurfaceNewton(prev, P3D, preci, ref solution) != 0)
                            {
                                return solution;
                            }

                            uf = -500;
                            ul = 500;
                        }

                        RestrictBounds(ref uf, ref ul, ref vf, ref vl);

                        //: 30 by abv 2.12.97: speed optimization
                        // code is taken from GeomAPI_ProjectPointOnSurf
                        if (!myExtOK)
                        {
                            //  Force the call to IsU-VClosed
                            if (myUCloseVal < 0)
                            {
                                IsUClosed();
                            }

                            if (myVCloseVal < 0)
                            {
                                IsVClosed();
                            }

                            double du = 0.0, dv = 0.0;
                            // extension of the surface range is limited to non-offset surfaces as the latter
                            // can throw exception (e.g. Geom_UndefinedValue) when computing value - see id23943
                            if (mySurf is not global::Kerf.Geometry.OffsetSurface)
                            {
                                // modified by rln during fixing CSR # BUC60035 entity #D231
                                du = Min(myUDelt, SurfAdapt.UResolution(preci));
                                dv = Min(myVDelt, SurfAdapt.VResolution(preci));
                            }

                            double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
                            myExtPS.SetFlag(ExtFlag.ExtFlag_MIN);
                            myExtPS.Initialize(SurfAdapt, uf - du, ul + du, vf - dv, vl + dv, Tol, Tol);
                            myExtOK = true;
                        }

                        myExtPS.Perform(P3D);
                        int nPSurf = (myExtPS.IsDone ? myExtPS.ExtCount : 0);

                        if (nPSurf > 0)
                        {
                            double dist2Min = myExtPS.SquareDistance(1);
                            int    indMin   = 1;
                            for (int sol = 2; sol <= nPSurf; sol++)
                            {
                                double dist2 = myExtPS.SquareDistance(sol);
                                if (dist2Min > dist2)
                                {
                                    dist2Min = dist2;
                                    indMin   = sol;
                                }
                            }

                            myExtPS.Point(indMin).Parameter(out S, out T);
                            // PTV 26.06.2002 WORKAROUND protect OCC486. Remove after fix bug.
                            // file CEA_cuve-V5.igs Entityes 244, 259, 847, 925
                            // if project point3D on SurfaceOfRevolution Extreme recompute 2d point, but
                            // returns an old distance from 3d to solution :-(
                            Point3D aCheckPnt = SurfAdapt.Value(S, T);
                            dist2Min = P3D.SquareDistance(aCheckPnt);
                            // end of WORKAROUND
                            double disSurf = Math.Sqrt(dist2Min); //, disCurv =1.e10;

                            // Test of bad projections on the boundaries:
                            double UU = S, VV = T, DistMinOnIso = RealLast(); // myGap;

                            // test added by rln on 08/12/97
                            bool possLockal = false; //: study S4030 (optimizing)
                            if (disSurf > preci)
                            {
                                var pp = new Point2D(UU, VV);
                                if (SurfaceNewton(pp, P3D, preci, ref pp) != 0)
                                { //: q2 abv 16 Mar 99: PRO7226 #412920
                                    double dist = P3D.Distance(Value(pp));
                                    if (dist < disSurf)
                                    {
                                        disSurf = dist;
                                        S       = UU = pp.X();
                                        T       = VV = pp.Y();
                                    }
                                }

                                if (disSurf < 10 * preci)
                                {
                                    if (mySurf!.Continuity() != C0)
                                    {
                                        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                                        SurfAdapt.D1(UU, VV, out Point3D pnt, out Vector3D D1U, out Vector3D D1V);
                                        Vector3D b    = D1U.Crossed(D1V);
                                        var    a    = new Vector3D(pnt, P3D);
                                        double ab   = a.Dot(b);
                                        double nrm2 = b.SquareMagnitude();
                                        if (nrm2 > 1e-10)
                                        {
                                            double dist = a.SquareMagnitude() - (ab * ab) / nrm2;
                                            possLockal = (dist < Tol * Tol);
                                        }
                                    }
                                }

                                if (!possLockal)
                                {
                                    DistMinOnIso = UVFromIso(P3D, preci, ref UU, ref VV);
                                }
                            }

                            if (disSurf > DistMinOnIso)
                            {
                                // take the parameters UU and VV
                                S     = UU;
                                T     = VV;
                                myGap = DistMinOnIso;
                            }
                            else
                            {
                                myGap = disSurf;
                            }
                        }
                        else
                        {
                            // try on the boundaries
                            double UU = S, VV = T;
                            myGap = UVFromIso(P3D, preci, ref UU, ref VV);
                            S     = UU;
                            T     = VV;
                        }
                    }
                        break;

                    default:
                        computed = false;
                        break;
                }
            } // end Try ValueOfUV (CKY 30-DEC-1997)
            catch (FailureException)
            {
                // No reason, but who knows. Better to return something wrong than to stop.
                S = (global::Kerf.Engine.Foundation.Precision.IsInfinite(uf)) ? 0 : (uf + ul) / 2.0;
                T = (global::Kerf.Engine.Foundation.Precision.IsInfinite(vf)) ? 0 : (vf + vl) / 2.0;
            }
        } //: c9

        // szv#4:S4163:12Mar99 waste raise
        if (computed)
        {
            if (myGap <= 0)
            {
                myGap = P3D.Distance(SurfAdapt.Value(S, T));
            }
        }
        else
        {
            myGap = -1.0;
            S     = 0.0;
            T     = 0.0;
        }

        return new Point2D(S, T);
    }

    /// <summary>
    /// Refines the parameters (U, V) of a point by projecting it on iso-lines: the surface's boundaries, the
    /// iso-lines through (U, V), then up to 5 alternating iso-lines through the new U and V. Returns the best distance
    /// between the point and the surface at the resulting (U, V), or a very large value on failure.
    /// </summary>
    public double UVFromIso(in Point3D P3d, double preci, ref double U, ref double V)
    {
        //  Projection that considers the isos as follows:
        //  the 4 boundaries, plus the isos in U and in V
        //  Often, one of the two is good...
        double theMin = RealLast();

        Point3D pntres;
        double Cf, Cl, UU, VV;

        //  Starting the search: point already found (?)
        UU = U;
        VV = V;
        Point3D depart = myAdSur!.Value(U, V);
        theMin = depart.Distance(P3d);

        if (theMin < preci / 10)
        {
            return theMin; // it was already OK
        }

        ComputeBoxes();
        if (myIsoUF is null || myIsoUL is null || myIsoVF is null || myIsoVL is null)
        {
            // no isolines
            // no more precise computation
            return theMin;
        }

        try
        { // RAJOUT
            // pdn Create BndBox containing point;
            var aPBox = new Box();
            aPBox.Set(P3d);

            // modified by rln on 04/12/97 in order to use these variables later
            bool        UV  = true;
            double      par = 0.0, other = 0.0, dist = 0.0;
            Curve3D? iso;
            var         anIsoCurve = new IsoCurve(Adaptor3d()!);
            for (int num = 0; num < 6; num++)
            {
                UV = (num < 3); // 0-1-2 : iso-U  3-4-5 : iso-V
                if (!(Adaptor3d()!.GetType() == SurfaceKind.OffsetSurface))
                {
                    Box? anIsoBox = null;
                    iso = null;
                    switch (num)
                    {
                        case 0:
                            par      = myUF;
                            iso      = myIsoUF;
                            anIsoBox = myBndUF;
                            break;
                        case 1:
                            par      = myUL;
                            iso      = myIsoUL;
                            anIsoBox = myBndUL;
                            break;
                        case 2:
                            par = U;
                            iso = UIso(U);
                            break;
                        case 3:
                            par      = myVF;
                            iso      = myIsoVF;
                            anIsoBox = myBndVF;
                            break;
                        case 4:
                            par      = myVL;
                            iso      = myIsoVL;
                            anIsoBox = myBndVL;
                            break;
                        case 5:
                            par = V;
                            iso = VIso(V);
                            break;
                        default:
                            break;
                    }

                    //    Go on it
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(par) && iso is not null)
                    {
                        if (anIsoBox.HasValue && anIsoBox.Value.Distance(aPBox) > theMin)
                        {
                            continue;
                        }

                        Cf = iso.FirstParameter();
                        Cl = iso.LastParameter();

                        RestrictBounds(ref Cf, ref Cl);
                        dist = new ShapeAnalysis_Curve().Project(iso, P3d, preci, out pntres, out other, Cf, Cl, false);
                        if (dist < theMin)
                        {
                            theMin = dist;
                            //: q6	if (UV) VV = other;  else UU = other;
                            //  Along an iso-U, compute the best V
                            UU = (UV ? par : other);
                            VV = (UV ? other : par); //: q6: uncommented
                        }
                    }
                }
                else
                {
                    CurveAdaptorBase? anAdaptor  = null;
                    var              aGeomCurve = new CurveAdaptor();

                    Box? anIsoBox = null;
                    switch (num)
                    {
                        case 0:
                            par = myUF;
                            aGeomCurve.Load(myIsoUF);
                            anAdaptor = aGeomCurve;
                            anIsoBox  = myBndUF;
                            break;
                        case 1:
                            par = myUL;
                            aGeomCurve.Load(myIsoUL);
                            anAdaptor = aGeomCurve;
                            anIsoBox  = myBndUL;
                            break;
                        case 2:
                            par = U;
                            anIsoCurve.Load(IsoU, U);
                            anAdaptor = anIsoCurve;
                            break;
                        case 3:
                            par = myVF;
                            aGeomCurve.Load(myIsoVF);
                            anAdaptor = aGeomCurve;
                            anIsoBox  = myBndVF;
                            break;
                        case 4:
                            par = myVL;
                            aGeomCurve.Load(myIsoVL);
                            anAdaptor = aGeomCurve;
                            anIsoBox  = myBndVL;
                            break;
                        case 5:
                            par = V;
                            anIsoCurve.Load(IsoV, V);
                            anAdaptor = anIsoCurve;
                            break;
                        default:
                            break;
                    }

                    if (anIsoBox.HasValue && anIsoBox.Value.Distance(aPBox) > theMin)
                    {
                        continue;
                    }

                    dist = new ShapeAnalysis_Curve().Project(anAdaptor!, P3d, preci, out pntres, out other);
                    if (dist < theMin)
                    {
                        theMin = dist;
                        UU     = (UV ? par : other);
                        VV     = (UV ? other : par); //: q6: uncommented
                    }
                }
            }

            // added by rln on 04/12/97 iterational process
            double PrevU    = U, PrevV = V;
            int    MaxIters = 5, Iters = 0;
            if (!(Adaptor3d()!.GetType() == SurfaceKind.OffsetSurface))
            {
                while (((PrevU != UU) || (PrevV != VV)) && (Iters < MaxIters) && (theMin > preci))
                {
                    PrevU = UU;
                    PrevV = VV;
                    if (UV)
                    {
                        par = UU;
                        iso = UIso(UU);
                    }
                    else
                    {
                        par = VV;
                        iso = VIso(VV);
                    }

                    if (iso is not null)
                    {
                        Cf = iso.FirstParameter();
                        Cl = iso.LastParameter();
                        RestrictBounds(ref Cf, ref Cl);
                        dist = new ShapeAnalysis_Curve().Project(iso, P3d, preci, out pntres, out other, Cf, Cl, false);
                        if (dist < theMin)
                        {
                            theMin = dist;
                            if (UV)
                            {
                                VV = other;
                            }
                            else
                            {
                                UU = other;
                            }
                        }
                    }

                    UV = !UV;
                    if (UV)
                    {
                        par = UU;
                        iso = UIso(UU);
                    }
                    else
                    {
                        par = VV;
                        iso = VIso(VV);
                    }

                    if (iso is not null)
                    {
                        Cf = iso.FirstParameter();
                        Cl = iso.LastParameter();
                        RestrictBounds(ref Cf, ref Cl);
                        dist = new ShapeAnalysis_Curve().Project(iso, P3d, preci, out pntres, out other, Cf, Cl, false);
                        if (dist < theMin)
                        {
                            theMin = dist;
                            if (UV)
                            {
                                VV = other;
                            }
                            else
                            {
                                UU = other;
                            }
                        }
                    }

                    UV = !UV;
                    Iters++;
                }
            }
            else
            {
                while (((PrevU != UU) || (PrevV != VV)) && (Iters < MaxIters) && (theMin > preci))
                {
                    PrevU = UU;
                    PrevV = VV;
                    if (UV)
                    {
                        par = UU;
                        anIsoCurve.Load(IsoU, UU);
                    }
                    else
                    {
                        par = VV;
                        anIsoCurve.Load(IsoV, VV);
                    }

                    Cf = anIsoCurve.FirstParameter();
                    Cl = anIsoCurve.LastParameter();
                    RestrictBounds(ref Cf, ref Cl);
                    dist = new ShapeAnalysis_Curve().Project(anIsoCurve, P3d, preci, out pntres, out other);
                    if (dist < theMin)
                    {
                        theMin = dist;
                        if (UV)
                        {
                            VV = other;
                        }
                        else
                        {
                            UU = other;
                        }
                    }

                    UV = !UV;
                    if (UV)
                    {
                        par = UU;
                        anIsoCurve.Load(IsoU, UU);
                    }
                    else
                    {
                        par = VV;
                        anIsoCurve.Load(IsoV, VV);
                    }

                    Cf = anIsoCurve.FirstParameter();
                    Cl = anIsoCurve.LastParameter();
                    RestrictBounds(ref Cf, ref Cl);
                    dist = new ShapeAnalysis_Curve().ProjectAct(anIsoCurve, P3d, preci, out pntres, out other);
                    if (dist < theMin)
                    {
                        theMin = dist;
                        if (UV)
                        {
                            VV = other;
                        }
                        else
                        {
                            UU = other;
                        }
                    }

                    UV = !UV;
                    Iters++;
                }
            }

            U = UU;
            V = VV;
        } // end try RAJOUT
        catch (FailureException)
        {
            theMin = RealLast(); // the starting theMin
        }

        return theMin;
    }

    private void SortSingularities()
    {
        for (int i = 0; i < myNbDeg - 1; i++)
        {
            double minPreci = myPreci[i];
            int    minIndex = i;
            for (int j = i + 1; j < myNbDeg; j++)
            {
                if (minPreci > myPreci[j])
                {
                    minPreci = myPreci[j];
                    minIndex = j;
                }
            }

            if (minIndex != i)
            {
                myPreci[minIndex] = myPreci[i];
                myPreci[i]        = minPreci;
                Point3D tmpP3d = myP3d[minIndex];
                myP3d[minIndex] = myP3d[i];
                myP3d[i]        = tmpP3d;
                Point2D tmpP2d = myFirstP2d[minIndex];
                myFirstP2d[minIndex] = myFirstP2d[i];
                myFirstP2d[i]        = tmpP2d;
                tmpP2d               = myLastP2d[minIndex];
                myLastP2d[minIndex]  = myLastP2d[i];
                myLastP2d[i]         = tmpP2d;
                double tmpPar = myFirstPar[minIndex];
                myFirstPar[minIndex] = myFirstPar[i];
                myFirstPar[i]        = tmpPar;
                tmpPar               = myLastPar[minIndex];
                myLastPar[minIndex]  = myLastPar[i];
                myLastPar[i]         = tmpPar;
                bool tmpUIsoDeg = myUIsoDeg[minIndex];
                myUIsoDeg[minIndex] = myUIsoDeg[i];
                myUIsoDeg[i]        = tmpUIsoDeg;
            }
        }
    }

    /// <summary>Returns the bounds of the surface (Geom_Surface's, buffered, or those set by SetDomain).</summary>
    public void Bounds(out double ufirst, out double ulast, out double vfirst, out double vlast)
    {
        ufirst = myUF;
        ulast  = myUL;
        vfirst = myVF;
        vlast  = myVL;
    }

    /// <summary>Returns the smallest precision for which the surface is U-closed.</summary>
    public double UCloseVal() => myUCloseVal;

    /// <summary>Returns the smallest precision for which the surface is V-closed.</summary>
    public double VCloseVal() => myVCloseVal;

    private void ComputeBoxes()
    {
        if (myIsoBoxes)
        {
            return;
        }

        myIsoBoxes = true;
        ComputeBoundIsos();
        if (myIsoUF is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(myIsoUF), global::Kerf.Engine.Foundation.Precision.Confusion(), ref myBndUF);
        }

        if (myIsoUL is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(myIsoUL), global::Kerf.Engine.Foundation.Precision.Confusion(), ref myBndUL);
        }

        if (myIsoVF is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(myIsoVF), global::Kerf.Engine.Foundation.Precision.Confusion(), ref myBndVF);
        }

        if (myIsoVL is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(myIsoVL), global::Kerf.Engine.Foundation.Precision.Confusion(), ref myBndVL);
        }
    }

    /// <summary>The bounding box of the iso-curve at the first U bound.</summary>
    public Box GetBoxUF()
    {
        ComputeBoxes();
        return myBndUF;
    }

    /// <summary>The bounding box of the iso-curve at the last U bound.</summary>
    public Box GetBoxUL()
    {
        ComputeBoxes();
        return myBndUL;
    }

    /// <summary>The bounding box of the iso-curve at the first V bound.</summary>
    public Box GetBoxVF()
    {
        ComputeBoxes();
        return myBndVF;
    }

    /// <summary>The bounding box of the iso-curve at the last V bound.</summary>
    public Box GetBoxVL()
    {
        ComputeBoxes();
        return myBndVL;
    }
}
