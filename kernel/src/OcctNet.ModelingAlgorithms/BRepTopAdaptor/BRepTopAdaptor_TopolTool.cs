// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_TopolTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_TopolTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The topological tool of a face for the intersection algorithms: its restrictions are the pcurves of its edges, its
/// vertices the edges' vertices, and points are classified with a <see cref="BRepTopAdaptor_FClass2d"/>.
/// </summary>
/// <remarks>
/// A working object (decision 101). The surface given to <see cref="Initialize(Adaptor3d_Surface)"/> must be a
/// <see cref="BRepAdaptor_Surface"/>, the curves <see cref="BRepAdaptor_Curve2d"/>s. The 2D classifier is built at
/// the first classification, with that call's tolerance, and kept (OCCT's <c>void*</c>).
/// </remarks>
public class BRepTopAdaptor_TopolTool : Adaptor3d_TopolTool
{
    private readonly TopExp_Explorer myVIterator = new();
    private TopoDS_Face myFace = new();
    private BRepTopAdaptor_FClass2d? myFClass2d;
    private BRepAdaptor_Curve2d? myCurve;
    private readonly NCollection_List<BRepAdaptor_Curve2d> myCurves = new();
    private NCollection_List<BRepAdaptor_Curve2d>.Iterator myCIterator = new();
    private double myU0;
    private double myV0;
    private double myDU;
    private double myDV;

    /// <summary>An empty tool.</summary>
    public BRepTopAdaptor_TopolTool()
    {
        myFClass2d = null;
        myU0 = 0.0;
        myV0 = 0.0;
        myDU = 0.0;
        myDV = 0.0;
        myNbSamplesU = -1;
    }

    /// <summary>The tool of the face of <paramref name="S"/> (a BRepAdaptor_Surface).</summary>
    public BRepTopAdaptor_TopolTool(Adaptor3d_Surface S)
    {
        myFClass2d = null;
        Initialize(S);
        // myS = S;
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as in OCCT.</summary>
    public override void Initialize()
    {
        throw new MethodNotImplementedException("BRepTopAdaptor_TopolTool::Initialize()");
    }

    /// <summary>Loads the face of <paramref name="S"/> (a BRepAdaptor_Surface) and the pcurves of its edges.</summary>
    /// <exception cref="ConstructionErrorException">The surface is not a BRepAdaptor_Surface.</exception>
    public override void Initialize(Adaptor3d_Surface S)
    {
        if (S is not BRepAdaptor_Surface brhs)
        {
            throw new ConstructionErrorException();
        }

        TopoDS_Shape s_wnt = brhs.Face();
        s_wnt.Orientation(TopAbs_FORWARD);
        myFace = TopoDS.Face(s_wnt);
        myFClass2d = null;
        myNbSamplesU = -1;
        myS = S;
        myCurves.Clear();
        var ex = new TopExp_Explorer(myFace, TopAbs_EDGE);
        for (; ex.More(); ex.Next())
        {
            var aCurve = new BRepAdaptor_Curve2d(TopoDS.Edge(ex.Current()), myFace);
            myCurves.Append(aCurve);
        }

        myCIterator = new NCollection_List<BRepAdaptor_Curve2d>.Iterator();
    }

    /// <summary>Loads the restriction <paramref name="C"/> (a BRepAdaptor_Curve2d) for the vertex iteration.</summary>
    /// <exception cref="ConstructionErrorException">The curve is not a BRepAdaptor_Curve2d.</exception>
    public override void Initialize(Adaptor2d_Curve2d C)
    {
        myCurve = C as BRepAdaptor_Curve2d;
        if (myCurve is null)
        {
            throw new ConstructionErrorException();
        }
    }

    /// <summary>Starts the iteration over the restrictions.</summary>
    public override void Init() => myCIterator.Initialize(myCurves);

    public override bool More() => myCIterator.More();

    public override void Next() => myCIterator.Next();

    /// <summary>The current restriction (a BRepAdaptor_Curve2d).</summary>
    public override Adaptor2d_Curve2d Value() => myCIterator.Value();

    /// <summary>The edge of the current restriction (a boxed TopoDS_Edge: OCCT's <c>void*</c> to it).</summary>
    // modified by NIZNHY-PKV Tue Mar 27 14:23:40 2001 f
    public override object? Edge()
    {
        BRepAdaptor_Curve2d aHCurve = myCIterator.Value();
        return aHCurve.Edge();
    }

    /// <summary>Starts the iteration over the vertices of the restriction loaded by <see cref="Initialize(Adaptor2d_Curve2d)"/>.</summary>
    public override void InitVertexIterator() => myVIterator.Init(myCurve!.Edge(), TopAbs_VERTEX);

    public override bool MoreVertex() => myVIterator.More();

    public override void NextVertex() => myVIterator.Next();

    /// <summary>The current vertex (a new BRepTopAdaptor_HVertex).</summary>
    public override Adaptor3d_HVertex Vertex() => new BRepTopAdaptor_HVertex(TopoDS.Vertex(myVIterator.Current()), myCurve!);

    /// <summary>
    /// The state of the point <paramref name="P"/> in the face (UNKNOWN if no face is loaded). The 2D classifier is
    /// built at the first call, with its tolerance <paramref name="Tol"/>.
    /// </summary>
    public override TopAbs_State Classify(in Point2D P, double Tol, bool RecadreOnPeriodic = true)
    {
        if (myFace.IsNull())
        {
            return TopAbs_UNKNOWN;
        }

        myFClass2d ??= new BRepTopAdaptor_FClass2d(myFace, Tol);

        return (myFClass2d.Perform(P, RecadreOnPeriodic));
    }

    /// <summary>True if the point <paramref name="P"/> is on the face's boundary within <paramref name="Tol"/>.</summary>
    public override bool IsThePointOn(in Point2D P, double Tol, bool RecadreOnPeriodic = true)
    {
        myFClass2d ??= new BRepTopAdaptor_FClass2d(myFace, Tol);

        return (TopAbs_ON == myFClass2d.TestOnRestriction(P, Tol, RecadreOnPeriodic));
    }

    /// <summary>Drops the 2D classifier (OCCT's Destroy, also called by its destructor).</summary>
    public void Destroy() => myFClass2d = null;

    /// <summary>The orientation of the edge of the arc <paramref name="C"/> (a BRepAdaptor_Curve2d).</summary>
    public override TopAbs_Orientation Orientation(Adaptor2d_Curve2d C)
    {
        var brhc = (BRepAdaptor_Curve2d)C;
        return brhc.Edge().Orientation();
    }

    /// <summary>The orientation of the vertex.</summary>
    public override TopAbs_Orientation Orientation(Adaptor3d_HVertex C) => base.Orientation(C);

    //-- ============================================================
    //-- methods  used for samples
    //-- ============================================================

    private static void Analyse(NCollection_Array2<Point3D> array2,
                                int nbup,
                                int nbvp,
                                ref int myNbSamplesU,
                                ref int myNbSamplesV)
    {
        Vector3D Vi = new(), Vip1 = new();
        int sh, nbch, i, j;

        sh = 1;
        nbch = 0;
        if (nbvp > 2)
        {
            for (i = 2; i < nbup; i++)
            {
                Point3D A = array2.Value(i, 1);
                Point3D B = array2.Value(i, 2);
                Point3D C = array2.Value(i, 3);
                Vi.SetCoord(C.X() - B.X() - B.X() + A.X(),
                            C.Y() - B.Y() - B.Y() + A.Y(),
                            C.Z() - B.Z() - B.Z() + A.Z());

                int locnbch = 0;

                for (j = 3; j < nbvp; j++)
                {
                    //-- test
                    Point3D Ax = array2.Value(i, j - 1);
                    Point3D Bx = array2.Value(i, j);
                    Point3D Cx = array2.Value(i, j + 1);
                    Vip1.SetCoord(Cx.X() - Bx.X() - Bx.X() + Ax.X(),
                                  Cx.Y() - Bx.Y() - Bx.Y() + Ax.Y(),
                                  Cx.Z() - Bx.Z() - Bx.Z() + Ax.Z());
                    double pd = Vi.Dot(Vip1);
                    Vi = Vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }

                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }

        myNbSamplesV = nbch + 5;

        nbch = 0;
        if (nbup > 2)
        {
            for (j = 2; j < nbvp; j++)
            {
                Point3D A = array2.Value(1, j);
                Point3D B = array2.Value(2, j);
                Point3D C = array2.Value(3, j);
                Vi.SetCoord(C.X() - B.X() - B.X() + A.X(),
                            C.Y() - B.Y() - B.Y() + A.Y(),
                            C.Z() - B.Z() - B.Z() + A.Z());

                int locnbch = 0;

                for (i = 3; i < nbup; i++)
                {
                    //-- test
                    Point3D Ax = array2.Value(i - 1, j);
                    Point3D Bx = array2.Value(i, j);
                    Point3D Cx = array2.Value(i + 1, j);
                    Vip1.SetCoord(Cx.X() - Bx.X() - Bx.X() + Ax.X(),
                                  Cx.Y() - Bx.Y() - Bx.Y() + Ax.Y(),
                                  Cx.Z() - Bx.Z() - Bx.Z() + Ax.Z());
                    double pd = Vi.Dot(Vip1);
                    Vi = Vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }

                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }

        myNbSamplesU = nbch + 5;
    }

    /// <summary>Computes the numbers of sample points in U and V and their spacing.</summary>
    public override void ComputeSamplePoints()
    {
        double uinf, usup, vinf, vsup;
        uinf = myS!.FirstUParameter();
        usup = myS.LastUParameter();
        vinf = myS.FirstVParameter();
        vsup = myS.LastVParameter();
        if (usup < uinf)
        {
            double temp = uinf;
            uinf = usup;
            usup = temp;
        }

        if (vsup < vinf)
        {
            double temp = vinf;
            vinf = vsup;
            vsup = temp;
        }

        if (uinf == RealFirst() && usup == RealLast())
        {
            uinf = -1.0e5;
            usup = 1.0e5;
        }
        else if (uinf == RealFirst())
        {
            uinf = usup - 2.0e5;
        }
        else if (usup == RealLast())
        {
            usup = uinf + 2.0e5;
        }

        if (vinf == RealFirst() && vsup == RealLast())
        {
            vinf = -1.0e5;
            vsup = 1.0e5;
        }
        else if (vinf == RealFirst())
        {
            vinf = vsup - 2.0e5;
        }
        else if (vsup == RealLast())
        {
            vsup = vinf + 2.0e5;
        }

        int nbsu, nbsv;
        SurfaceKind typS = myS.GetType();
        switch (typS)
        {
            case Plane:
            {
                nbsv = 2;
                nbsu = 2;
            }
                break;
            case BezierSurface:
            {
                nbsv = 3 + myS.NbVPoles();
                nbsu = 3 + myS.NbUPoles();
            }
                break;
            case BSplineSurface:
            {
                nbsv = myS.NbVKnots();
                nbsv *= myS.VDegree();
                if (nbsv < 4)
                {
                    nbsv = 4;
                }

                nbsu = myS.NbUKnots();
                nbsu *= myS.UDegree();
                if (nbsu < 4)
                {
                    nbsu = 4;
                }
            }
                break;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            {
                //-- Set 15 for 2pi
                //-- Not enough ->25 for 2pi
                nbsu = (int)(8 * (usup - uinf));
                nbsv = (int)(7 * (vsup - vinf));
                if (nbsu < 5)
                {
                    nbsu = 5;
                }

                if (nbsv < 5)
                {
                    nbsv = 5;
                }

                if (nbsu > 30)
                {
                    nbsu = 30; // modif HRT buc60462
                }

                if (nbsv > 15)
                {
                    nbsv = 15;
                }
                //-- printf("\n nbsu=%d nbsv=%d\n",nbsu,nbsv);
            }
                break;
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
            {
                nbsv = 15;
                nbsu = 25;
            }
                break;
            default:
            {
                nbsu = 10;
                nbsv = 10;
            }
                break;
        }

        //-- If the number of points is too great, analyze
        //--
        //--

        if (nbsu < 10)
        {
            nbsu = 10;
        }

        if (nbsv < 10)
        {
            nbsv = 10;
        }

        myNbSamplesU = nbsu;
        myNbSamplesV = nbsv;

        //-- printf("\n BRepTopAdaptor_TopolTool NbSu=%d NbSv=%d ",nbsu,nbsv);
        if (nbsu > 10 || nbsv > 10)
        {
            if (typS == BSplineSurface)
            {
                Geom_BSplineSurface Bspl = myS.BSpline();
                int nbup = Bspl.NbUPoles();
                int nbvp = Bspl.NbVPoles();
                NCollection_Array2<Point3D> array2 = Bspl.Poles();
                Analyse(array2, nbup, nbvp, ref myNbSamplesU, ref myNbSamplesV);
                nbsu = myNbSamplesU;
                nbsv = myNbSamplesV;
                //-- printf("\n Apres analyse BSPline  NbSu=%d NbSv=%d ",myNbSamplesU,myNbSamplesV);
            }
            else if (typS == BezierSurface)
            {
                Geom_BezierSurface Bez = myS.Bezier();
                int nbup = Bez.NbUPoles();
                int nbvp = Bez.NbVPoles();
                NCollection_Array2<Point3D> array2 = Bez.Poles();
                Analyse(array2, nbup, nbvp, ref myNbSamplesU, ref myNbSamplesV);
                nbsu = myNbSamplesU;
                nbsv = myNbSamplesV;
                //-- printf("\n Apres analyse Bezier  NbSu=%d NbSv=%d ",myNbSamplesU,myNbSamplesV);
            }
        }

        if (nbsu < 10)
        {
            nbsu = 10;
        }

        if (nbsv < 10)
        {
            nbsv = 10;
        }

        myNbSamplesU = nbsu;
        myNbSamplesV = nbsv;

        myU0 = uinf;
        myV0 = vinf;

        myDU = (usup - uinf) / (myNbSamplesU + 1);
        myDV = (vsup - vinf) / (myNbSamplesV + 1);
    }

    /// <summary>The number of sample points in U (computed at the first call).</summary>
    public override int NbSamplesU()
    {
        if (myNbSamplesU < 0)
        {
            ComputeSamplePoints();
        }

        return (myNbSamplesU);
    }

    /// <summary>The number of sample points in V (computed at the first call).</summary>
    public override int NbSamplesV()
    {
        if (myNbSamplesU < 0)
        {
            ComputeSamplePoints();
        }

        return (myNbSamplesV);
    }

    /// <summary>The number of sample points (computed at the first call).</summary>
    public override int NbSamples()
    {
        if (myNbSamplesU < 0)
        {
            ComputeSamplePoints();
        }

        return (myNbSamplesU * myNbSamplesV);
    }

    /// <summary>The sample point of index <paramref name="i"/>, in the parametric domain and in 3D.</summary>
    public override void SamplePoint(int i, out Point2D P2d, out Point3D P3d)
    {
        int iv = 1 + i / myNbSamplesU;
        int iu = 1 + i - (iv - 1) * myNbSamplesU;
        double u = myU0 + iu * myDU;
        double v = myV0 + iv * myDV;
        P2d = new Point2D(u, v);
        P3d = myS!.Value(u, v);
    }

    /// <summary>True if one of the bounds of the surface is infinite.</summary>
    public override bool DomainIsInfinite()
    {
        double uinf, usup, vinf, vsup;
        uinf = myS!.FirstUParameter();
        usup = myS.LastUParameter();
        vinf = myS.FirstVParameter();
        vsup = myS.LastVParameter();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uinf))
        {
            return (true);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(usup))
        {
            return (true);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vinf))
        {
            return (true);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vsup))
        {
            return (true);
        }

        return (false);
    }

    /// <summary>True: arcs and vertices have 3D representations.</summary>
    public override bool Has3d() => true;

    /// <summary>The 3D tolerance of the edge of the arc <paramref name="C"/>.</summary>
    /// <exception cref="DomainErrorException">The arc is not a BRepAdaptor_Curve2d with an edge.</exception>
    public override double Tol3d(Adaptor2d_Curve2d C)
    {
        if (C is not BRepAdaptor_Curve2d brhc)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: arc has no 3d representation");
        }

        TopoDS_Edge edge = brhc.Edge();
        if (edge.IsNull())
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: arc has no 3d representation");
        }

        return BRep_Tool.Tolerance(edge);
    }

    /// <summary>The 3D tolerance of the vertex <paramref name="V"/>.</summary>
    /// <exception cref="DomainErrorException">The vertex is not a BRepTopAdaptor_HVertex with a vertex.</exception>
    public override double Tol3d(Adaptor3d_HVertex V)
    {
        if (V is not BRepTopAdaptor_HVertex brhv)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        TopoDS_Vertex ver = brhv.Vertex();
        if (ver.IsNull())
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        return BRep_Tool.Tolerance(ver);
    }

    /// <summary>The 3D point of the vertex <paramref name="V"/>.</summary>
    /// <exception cref="DomainErrorException">The vertex is not a BRepTopAdaptor_HVertex with a vertex.</exception>
    public override Point3D Pnt(Adaptor3d_HVertex V)
    {
        if (V is not BRepTopAdaptor_HVertex brhv)
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        TopoDS_Vertex ver = brhv.Vertex();
        if (ver.IsNull())
        {
            throw new DomainErrorException("BRepTopAdaptor_TopolTool: vertex has no 3d representation");
        }

        return BRep_Tool.Pnt(ver);
    }
}
