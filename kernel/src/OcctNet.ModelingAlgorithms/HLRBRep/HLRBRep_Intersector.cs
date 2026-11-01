// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.IntRes2d_Position;

namespace Occt;

/// <summary>
/// The intersections of the hidden-line algorithm: of a projected edge with itself, of two projected edges (each
/// shortened at its ends as asked), a simulated single crossing, and of a line of sight with a face's surface.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The edges' curves and the loaded surface are kept by reference, as OCCT keeps
/// pointers. The polyhedron of a non-quadric surface is built at the first line intersected and kept until the next
/// <see cref="Load"/>. OCCT's <c>PERF</c> counters (a compile-time option, off) are left out.
/// </remarks>
public sealed class HLRBRep_Intersector
{
    private IntRes2d_IntersectionPoint mySinglePoint;
    private int myTypePerform;
    private readonly HLRBRep_CInter myIntersector = new();
    private readonly HLRBRep_InterCSurf myCSIntersector = new();
    private HLRBRep_Surface? mySurface;
    private HLRBRep_ThePolyhedronOfInterCSurf? myPolyhedron;

    public HLRBRep_Intersector()
    {
        myTypePerform = 0;
        mySurface = null;
        myPolyhedron = null;
        // OCCT_DEBUG / PERF statistics left out.
        const int aMinNbHLRSamples = 4;
        myIntersector.SetMinNbSamples(aMinNbHLRSamples);
    }

    /// <summary>The self-intersection of the edge's projection, its ends cut by the fractions theDa1 and theDb1.</summary>
    public void Perform(HLRBRep_EdgeData theEdge1, double theDa1, double theDb1)
    {
        HLRBRep_Curve myC1 = theEdge1.Curve();

        myTypePerform = 1;

        double a, b, d, tol;
        float ta, tb;

        theEdge1.Status().Bounds(out a, out ta, out b, out tb);
        d = b - a;
        if (theDa1 != 0)
        {
            a = a + d * theDa1;
        }
        if (theDb1 != 0)
        {
            b = b - d * theDb1;
        }
        myC1.D0(a, out gp_Pnt2d pa);
        myC1.D0(b, out gp_Pnt2d pb);
        a = myC1.Parameter2d(a);
        b = myC1.Parameter2d(b);
        IntRes2d_Domain D1 = new(pa, a, (double)ta, pb, b, (double)tb);

        // modified by jgv, 18.04.2016 for OCC27341
        // tol = (double)(((HLRBRep_EdgeData*) A1)->Tolerance());
        tol = Precision.Confusion();

        myIntersector.Perform(myC1, D1, tol, tol);
    }

    /// <summary>
    /// The intersection of the projections of two edges, each shortened at its ends as asked (theDa, theDb); with
    /// theNoBound (the edges share a vertex), the cut grows until the domains' ends are far enough apart.
    /// </summary>
    public void Perform(int theNA, HLRBRep_EdgeData theEdge1, double theDa1, double theDb1, int theNB,
                        HLRBRep_EdgeData theEdge2, double theDa2, double theDb2, bool theEnBout)
    {
        HLRBRep_Curve myC1 = theEdge1.Curve();
        HLRBRep_Curve myC2 = theEdge2.Curve();

        myTypePerform = 1;

        gp_Pnt2d pa1, pb1, pa2, pb2;
        gp_Vec2d va1, vb1, va2, vb2;
        double a1, b1, a2, b2, d, dd, tol, tol1, tol2;
        float ta, tb;

        // modified by jgv, 18.04.2016 for OCC27341
        // tol1 = theEdge1->Tolerance();
        // tol2 = theEdge2->Tolerance();
        tol1 = Precision.Confusion();
        tol2 = Precision.Confusion();

        if (tol1 > tol2)
        {
            tol = tol1;
        }
        else
        {
            tol = tol2;
        }

        bool aPasBon;
        double aDecalagea1 = 100.0;
        double aDecalagea2 = 100.0;
        double aDecalageb1 = 100.0;
        double aDecalageb2 = 100.0;
        do
        {
            aPasBon = false;
            theEdge1.Status().Bounds(out a1, out ta, out b1, out tb); //--   -> Parametres 3d
            double mtol = tol;
            if (mtol < ta)
            {
                mtol = ta;
            }
            if (mtol < tb)
            {
                mtol = tb;
            }
            d = b1 - a1;

            double pdist = tol;
            if (pdist < 0.0000001)
            {
                pdist = 0.0000001;
            }

            if (theDa1 != 0)
            {
                //-- a = a + d * da1;
                myC1.D1(a1, out pa1, out va1);
                double qwe = va1.Magnitude();
                if (qwe > 1e-12)
                {
                    dd = pdist * aDecalagea1 / qwe;
                    if (dd < d * 0.4)
                    {
                        a1 += dd;
                    }
                    else
                    {
                        a1 += d * theDa1;
                        aDecalagea1 = -1;
                    }
                }
                else
                {
                    a1 += d * theDa1;
                    aDecalagea1 = -1;
                }
            }

            if (theDb1 != 0)
            {
                //-- b = b - d * db1;
                myC1.D1(b1, out pb1, out vb1);
                double qwe = vb1.Magnitude();
                if (qwe > 1e-12)
                {
                    dd = pdist * aDecalageb1 / qwe;
                    if (dd < d * 0.4)
                    {
                        b1 -= dd;
                    }
                    else
                    {
                        b1 -= d * theDb1;
                        aDecalageb1 = -1;
                    }
                }
                else
                {
                    b1 -= d * theDb1;
                    aDecalageb1 = -1;
                }
            }

            myC1.D0(a1, out pa1);
            myC1.D0(b1, out pb1);

            a1 = myC1.Parameter2d(a1);
            b1 = myC1.Parameter2d(b1);

            if (theEnBout)
            {
                ta = tb = -1.0f;
            }

            if (ta > tol)
            {
                ta = (float)tol;
            }
            if (tb > tol)
            {
                tb = (float)tol;
            }

            IntRes2d_Domain D1 = new(pa1, a1, (double)ta, pb1, b1, (double)tb);

            theEdge2.Status().Bounds(out a2, out ta, out b2, out tb);
            mtol = tol;
            if (mtol < ta)
            {
                mtol = ta;
            }
            if (mtol < tb)
            {
                mtol = tb;
            }

            d = b2 - a2;

            if (theDa2 != 0)
            {
                //-- a = a + d * da2;
                myC2.D1(a2, out pa2, out va2);
                double qwe = va2.Magnitude();
                if (qwe > 1e-12)
                {
                    dd = pdist * aDecalagea2 / qwe;
                    if (dd < d * 0.4)
                    {
                        a2 += dd;
                    }
                    else
                    {
                        a2 += d * theDa2;
                        aDecalagea2 = -1;
                    }
                }
                else
                {
                    a2 += d * theDa2;
                    aDecalagea2 = -1;
                }
            }

            if (theDb2 != 0)
            {
                //-- b = b - d * db2;
                myC2.D1(b2, out pb2, out vb2);
                double qwe = vb2.Magnitude();
                if (qwe > 1e-12)
                {
                    dd = pdist * aDecalageb2 / qwe;
                    if (dd < d * 0.4)
                    {
                        b2 -= dd;
                    }
                    else
                    {
                        b2 -= d * theDb2;
                        aDecalageb2 = -1;
                    }
                }
                else
                {
                    b2 -= d * theDb2;
                    aDecalageb2 = -1;
                }
            }

            myC2.D0(a2, out pa2);
            myC2.D0(b2, out pb2);

            a2 = myC2.Parameter2d(a2);
            b2 = myC2.Parameter2d(b2);

            if (theEnBout)
            {
                ta = tb = -1.0f;
            }

            if (ta > tol)
            {
                ta = (float)tol;
            }
            if (tb > tol)
            {
                tb = (float)tol;
            }

            IntRes2d_Domain D2 = new(pa2, a2, (double)ta, pb2, b2, (double)tb);

            if (theEnBout)
            {
                double a1a2 = (theDa1 != 0 || theDa2 != 0) ? pa1.Distance(pa2) : RealLast();
                double a1b2 = (theDa1 != 0 || theDb2 != 0) ? pa1.Distance(pb2) : RealLast();
                double b1a2 = (theDb1 != 0 || theDa2 != 0) ? pb1.Distance(pa2) : RealLast();
                double b1b2 = (theDb1 != 0 || theDb2 != 0) ? pb1.Distance(pb2) : RealLast();

                int cote = 1;
                double mindist = a1a2; //-- cas 1
                if (mindist > a1b2)
                {
                    mindist = a1b2;
                    cote = 2;
                }
                if (mindist > b1a2)
                {
                    mindist = b1a2;
                    cote = 3;
                }
                if (mindist > b1b2)
                {
                    mindist = b1b2;
                    cote = 4;
                }


                if (mindist < tol * 1000)
                {
                    aPasBon = true;
                    switch (cote)
                    {
                        case 1:
                        {
                            aDecalagea1 *= 2;
                            aDecalagea2 *= 2;
                            break;
                        }
                        case 2:
                        {
                            aDecalagea1 *= 2;
                            aDecalageb2 *= 2;
                            break;
                        }
                        case 3:
                        {
                            aDecalageb1 *= 2;
                            aDecalagea2 *= 2;
                            break;
                        }
                        default:
                        {
                            aDecalageb1 *= 2;
                            aDecalageb2 *= 2;
                            break;
                        }
                    }
                    if (aDecalagea1 < 0.0 || aDecalagea2 < 0.0 || aDecalageb1 < 0.0 || aDecalageb2 <= 0.0)
                    {
                        aPasBon = false;
                    }
                }
            }
            if (!aPasBon)
            {
                myIntersector.Perform(myC1, D1, myC2, D2, tol, tol);
            }
        } while (aPasBon);

        // PERF statistics left out.
    }

    /// <summary>Records a single crossing of the two edges at the parameters theU and theV (2D) without computing it.</summary>
    public void SimulateOnePoint(HLRBRep_EdgeData theEdge1, double theU, HLRBRep_EdgeData theEdge2, double theV)
    {
        HLRBRep_Curve myC1 = theEdge1.Curve();
        HLRBRep_Curve myC2 = theEdge2.Curve();

        double u3 = myC1.Parameter3d(theU);
        double v3 = myC2.Parameter3d(theV);
        myC1.D1(u3, out gp_Pnt2d P13, out gp_Vec2d T13);
        myC2.D1(v3, out gp_Pnt2d P23, out gp_Vec2d T23);

        IntRes2d_Transition Tr1 = new(), Tr2 = new();
        IntRes2d_Position Pos1, Pos2;
        Pos1 = Pos2 = IntRes2d_Middle;

        IntImpParGen.DetermineTransition(Pos1, ref T13, ref Tr1, Pos2, ref T23, ref Tr2, 0.0);
        myTypePerform = 0;
        mySinglePoint.SetValues(P13, theU, theV, Tr1, Tr2, false);
    }

    /// <summary>Loads the surface the next lines are intersected with; drops the polyhedron of the last one.</summary>
    public void Load(HLRBRep_Surface theSurface)
    {
        mySurface = theSurface;
        myPolyhedron?.Destroy();
        myPolyhedron = null;
    }

    /// <summary>The intersection of the line L with the loaded surface, up to the parameter P on the line.</summary>
    public void Perform(in gp_Lin L, double P)
    {
        myTypePerform = 2;
        GeomAbs_SurfaceType typ = HLRBRep_SurfaceTool.GetType(mySurface!);
        switch (typ)
        {
            case GeomAbs_Plane:
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
                myCSIntersector.Perform(L, mySurface!);
                break;
            default:
            {
                if (myPolyhedron is null)
                {
                    int nbsu, nbsv;
                    double u1, v1, u2, v2;
                    u1 = HLRBRep_SurfaceTool.FirstUParameter(mySurface!);
                    v1 = HLRBRep_SurfaceTool.FirstVParameter(mySurface!);
                    u2 = HLRBRep_SurfaceTool.LastUParameter(mySurface!);
                    v2 = HLRBRep_SurfaceTool.LastVParameter(mySurface!);
                    nbsu = HLRBRep_SurfaceTool.NbSamplesU(mySurface!, u1, u2);
                    nbsv = HLRBRep_SurfaceTool.NbSamplesV(mySurface!, v1, v2);
                    myPolyhedron = new HLRBRep_ThePolyhedronOfInterCSurf(mySurface!, nbsu, nbsv, u1, v1, u2, v2);
                }
                double x0, y0, z0, x1, y1, z1, pmin, pmax; //,pp;
                myPolyhedron.Bounding().Get(out x0, out y0, out z0, out x1, out y1, out z1);
                double p;
                p = ElCLib.Parameter(L, new gp_Pnt(x0, y0, z0));
                pmin = pmax = p;
                p = ElCLib.Parameter(L, new gp_Pnt(x0, y0, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = ElCLib.Parameter(L, new gp_Pnt(x1, y0, z0));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = ElCLib.Parameter(L, new gp_Pnt(x1, y0, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = ElCLib.Parameter(L, new gp_Pnt(x0, y1, z0));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = ElCLib.Parameter(L, new gp_Pnt(x0, y1, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = ElCLib.Parameter(L, new gp_Pnt(x1, y1, z0));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }

                p = ElCLib.Parameter(L, new gp_Pnt(x1, y1, z1));
                if (pmin > p)
                {
                    pmin = p;
                }
                if (pmax < p)
                {
                    pmax = p;
                }
                pmin -= 0.000001;
                pmax += 0.000001;

                if (pmin > P)
                {
                    pmin = pmax + 1;
                    pmax = pmax + 2;
                } //-- on va rejeter avec les boites
                else
                {
                    if (pmax > P)
                    {
                        pmax = P + 0.0000001;
                    }
                }
                HLRBRep_ThePolygonOfInterCSurf Polygon = new(L, pmin, pmax, 3);
                myCSIntersector.Perform(L, Polygon, mySurface!, myPolyhedron);

                break;
            }
        }
        // PERF statistics left out.
    }

    public bool IsDone()
    {
        if (myTypePerform == 1)
        {
            return myIntersector.IsDone();
        }
        else if (myTypePerform == 2)
        {
            return myCSIntersector.IsDone();
        }
        else
        {
            return (true);
        }
    }

    public int NbPoints()
    {
        if (myTypePerform == 43)
        {
            return (0);
        }
        if (myTypePerform == 1)
        {
            return myIntersector.NbPoints();
        }
        else if (myTypePerform == 2)
        {
            return myCSIntersector.NbPoints();
        }
        else
        {
            return (1);
        }
    }

    /// <summary>The point N of the 2D intersection (the simulated point after <see cref="SimulateOnePoint"/>).</summary>
    public IntRes2d_IntersectionPoint Point(int N)
    {
        if (myTypePerform == 0)
        {
            return (mySinglePoint);
        }
        else
        {
            return myIntersector.Point(N);
        }
    }

    /// <summary>The point N of the line/surface intersection.</summary>
    public IntCurveSurface_IntersectionPoint CSPoint(int N) => myCSIntersector.Point(N);

    public int NbSegments()
    {
        if (myTypePerform == 1)
        {
            return myIntersector.NbSegments();
        }
        else if (myTypePerform == 2)
        {
            return myCSIntersector.NbSegments();
        }
        else
        {
            return (0);
        }
    }

    public IntRes2d_IntersectionSegment Segment(int N) => myIntersector.Segment(N);

    public IntCurveSurface_IntersectionSegment CSSegment(int N) => myCSIntersector.Segment(N);

    /// <summary>Drops the polyhedron.</summary>
    public void Destroy()
    {
        myPolyhedron?.Destroy();
        myPolyhedron = null;
    }
}
