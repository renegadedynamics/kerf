// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_TopolTool.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_TopolTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The default topological tool of the intersection and outline algorithms, based on the Umin, Vmin, Umax, Vmax
/// of a surface adaptor: the restrictions of the parametric domain are 2D lines, their ends are the vertices.
/// All methods may be redefined by a derived class (BRepTopAdaptor_TopolTool).
/// </summary>
/// <remarks>A working object (decision 101): <c>Initialize</c> and the iterators are public, as in OCCT.</remarks>
public class Adaptor3d_TopolTool : Standard_Transient
{
    private static double myInfinite => Precision.Infinite();

    // Absolute possible minimum of sample points (restriction of IntPolyh).
    private const int myMinPnts = 4;

    protected Adaptor3d_Surface? myS;
    protected int myNbSamplesU;
    protected int myNbSamplesV;
    protected NCollection_HArray1<double>? myUPars;
    protected NCollection_HArray1<double>? myVPars;

    private int nbRestr;
    private int idRestr;
    private double Uinf;
    private double Usup;
    private double Vinf;
    private double Vsup;
    private readonly Adaptor2d_Line2d?[] myRestr = new Adaptor2d_Line2d?[4];
    private int nbVtx;
    private int idVtx;
    private readonly Adaptor3d_HVertex?[] myVtx = new Adaptor3d_HVertex?[2];

    public Adaptor3d_TopolTool()
    {
        myNbSamplesU = -1;
        myNbSamplesV = -1;
        nbRestr = 0;
        idRestr = 0;
        Uinf = 0.0;
        Usup = 0.0;
        Vinf = 0.0;
        Vsup = 0.0;
        nbVtx = 0;
        idVtx = 0;
    }

    public Adaptor3d_TopolTool(Adaptor3d_Surface Surface)
    {
        // OCCT: myNbSamplesV, nbVtx and idVtx are left uninitialised here (C# zeroes them).
        Initialize(Surface);
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as in OCCT.</summary>
    public virtual void Initialize()
    {
        throw new Standard_NotImplemented("Adaptor3d_TopolTool::Initialize ()");
    }

    /// <summary>Takes the parametric domain of the surface and builds its restrictions.</summary>
    public virtual void Initialize(Adaptor3d_Surface S)
    {
        double pinf, psup, deltap;

        myNbSamplesU = -1;
        Uinf = S.FirstUParameter(); // where UIntervalFirst ??
        Vinf = S.FirstVParameter();
        Usup = S.LastUParameter();
        Vsup = S.LastVParameter();
        nbRestr = 0;
        idRestr = 0;

        bool Uinfinfinite = Precision.IsNegativeInfinite(Uinf);
        bool Usupinfinite = Precision.IsPositiveInfinite(Usup);
        bool Vinfinfinite = Precision.IsNegativeInfinite(Vinf);
        bool Vsupinfinite = Precision.IsPositiveInfinite(Vsup);

        if (!Vinfinfinite)
        {
            deltap = Min(Usup - Uinf, 2.0 * myInfinite);
            if (Uinf >= -myInfinite)
            {
                pinf = Uinf;
                psup = pinf + deltap;
            }
            else if (Usup <= myInfinite)
            {
                psup = Usup;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            myRestr[nbRestr] = new Adaptor2d_Line2d(new gp_Pnt2d(0.0, Vinf), new gp_Dir2d(gp_Dir2d.D.X), pinf, psup);
            nbRestr++;
        }

        if (!Usupinfinite)
        {
            deltap = Min(Vsup - Vinf, 2.0 * myInfinite);
            if (Vinf >= -myInfinite)
            {
                pinf = Vinf;
                psup = pinf + deltap;
            }
            else if (Vsup <= myInfinite)
            {
                psup = Vsup;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            myRestr[nbRestr] = new Adaptor2d_Line2d(new gp_Pnt2d(Usup, 0.0), new gp_Dir2d(gp_Dir2d.D.Y), pinf, psup);
            nbRestr++;
        }

        if (!Vsupinfinite)
        {
            deltap = Min(Usup - Uinf, 2.0 * myInfinite);
            if (-Usup >= -myInfinite)
            {
                pinf = -Usup;
                psup = pinf + deltap;
            }
            else if (-Uinf <= myInfinite)
            {
                psup = -Uinf;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            myRestr[nbRestr] = new Adaptor2d_Line2d(new gp_Pnt2d(0.0, Vsup), new gp_Dir2d(gp_Dir2d.D.NX), pinf, psup);
            nbRestr++;
        }

        if (!Uinfinfinite)
        {
            deltap = Min(Vsup - Vinf, 2.0 * myInfinite);
            if (-Vsup >= -myInfinite)
            {
                pinf = -Vsup;
                psup = pinf + deltap;
            }
            else if (-Vinf <= myInfinite)
            {
                psup = -Vinf;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            myRestr[nbRestr] = new Adaptor2d_Line2d(new gp_Pnt2d(Uinf, 0.0), new gp_Dir2d(gp_Dir2d.D.NY), pinf, psup);
            nbRestr++;
        }

        myS = S;

        if (nbRestr == 2 && S.GetType() == GeomAbs_Cone)
        {
            double U = 0.0, V = 0.0;
            GetConeApexParam(S.Cone(), out U, out V);

            deltap = Min(Usup - Uinf, 2.0 * myInfinite);
            if (Uinf >= -myInfinite)
            {
                pinf = Uinf;
                psup = pinf + deltap;
            }
            else if (Usup <= myInfinite)
            {
                psup = Usup;
                pinf = psup - deltap;
            }
            else
            {
                pinf = -myInfinite;
                psup = myInfinite;
            }

            myRestr[nbRestr] = new Adaptor2d_Line2d(new gp_Pnt2d(U, V), new gp_Dir2d(gp_Dir2d.D.X), pinf, psup);
            nbRestr++;
        }
    }

    /// <summary>Takes a restriction and builds its vertices (its finite ends).</summary>
    public virtual void Initialize(Adaptor2d_Curve2d Curve)
    {
        nbVtx = 0;
        idVtx = 0;
        double theUinf, theUsup;
        theUinf = Curve.FirstParameter();
        theUsup = Curve.LastParameter();
        //  if (!Precision::IsNegativeInfinite(theUinf)) {
        if (theUinf > -myInfinite)
        {
            myVtx[nbVtx] = new Adaptor3d_HVertex(Curve.Value(theUinf), TopAbs_FORWARD, 1.0e-8);
            nbVtx++;
        }
        //  if (!Precision::IsPositiveInfinite(theUsup)) {
        if (theUsup < myInfinite)
        {
            myVtx[nbVtx] = new Adaptor3d_HVertex(Curve.Value(theUsup), TopAbs_REVERSED, 1.0e-8);
            nbVtx++;
        }
    }

    /// <summary>Starts the iteration over the restrictions.</summary>
    public virtual void Init()
    {
        idRestr = 0;
    }

    public virtual bool More() => (idRestr < nbRestr);

    /// <summary>The current restriction.</summary>
    /// <exception cref="Standard_DomainError">The iteration is over.</exception>
    public virtual Adaptor2d_Curve2d Value()
    {
        if (idRestr >= nbRestr)
        {
            throw new Standard_DomainError();
        }
        return myRestr[idRestr]!;
    }

    public virtual void Next()
    {
        idRestr++;
    }

    /// <summary>Starts the iteration over the vertices of the restriction given to <c>Initialize(Adaptor2d_Curve2d)</c>.</summary>
    public virtual void InitVertexIterator()
    {
        idVtx = 0;
    }

    public virtual bool MoreVertex() => (idVtx < nbVtx);

    /// <summary>The current vertex.</summary>
    /// <exception cref="Standard_DomainError">The iteration is over.</exception>
    public virtual Adaptor3d_HVertex Vertex()
    {
        if (idVtx >= nbVtx)
        {
            throw new Standard_DomainError();
        }
        return myVtx[idVtx]!;
    }

    public virtual void NextVertex()
    {
        idVtx++;
    }

    /// <summary>Classifies a point of the parametric domain: IN, ON (within Tol of a finite bound) or OUT.</summary>
    /// <remarks>OCCT ignores <paramref name="ReacdreOnPeriodic"/>.</remarks>
    public virtual TopAbs_State Classify(in gp_Pnt2d P, double Tol, bool ReacdreOnPeriodic = true)
    {
        double U = P.X();
        double V = P.Y();

        if (nbRestr == 4)
        {
            if ((U < Uinf - Tol) || (U > Usup + Tol) || (V < Vinf - Tol) || (V > Vsup + Tol))
            {
                return TopAbs_OUT;
            }
            if ((Math.Abs(U - Uinf) <= Tol) || (Math.Abs(U - Usup) <= Tol) || (Math.Abs(V - Vinf) <= Tol)
                || (Math.Abs(V - Vsup) <= Tol))
            {
                return TopAbs_ON;
            }
            return TopAbs_IN;
        }
        else if (nbRestr == 0)
        {
            return TopAbs_IN;
        }
        else
        {
            bool dansu, dansv, surumin, surumax, survmin, survmax;
            if (Precision.IsNegativeInfinite(Uinf) && Precision.IsPositiveInfinite(Usup))
            {
                dansu = true;
                surumin = surumax = false;
            }
            else if (Precision.IsNegativeInfinite(Uinf))
            {
                surumin = false;
                // OCCT: `>=` here where the V branch below has `>` (docs/occt-issues.md).
                if (U >= Usup + Tol)
                {
                    dansu = false;
                    surumax = false;
                }
                else
                {
                    dansu = true;
                    surumax = false;
                    if (Math.Abs(U - Usup) <= Tol)
                    {
                        surumax = true;
                    }
                }
            }
            else if (Precision.IsPositiveInfinite(Usup))
            {
                surumax = false;
                if (U < Uinf - Tol)
                {
                    dansu = false;
                    surumin = false;
                }
                else
                {
                    dansu = true;
                    surumin = false;
                    if (Math.Abs(U - Uinf) <= Tol)
                    {
                        surumin = true;
                    }
                }
            }
            else
            {
                if ((U < Uinf - Tol) || (U > Usup + Tol))
                {
                    surumin = surumax = dansu = false;
                }
                else
                {
                    dansu = true;
                    surumin = surumax = false;
                    if (Math.Abs(U - Uinf) <= Tol)
                    {
                        surumin = true;
                    }
                    else if (Math.Abs(U - Usup) <= Tol)
                    {
                        surumax = true;
                    }
                }
            }

            if (Precision.IsNegativeInfinite(Vinf) && Precision.IsPositiveInfinite(Vsup))
            {
                dansv = true;
                survmin = survmax = false;
            }
            else if (Precision.IsNegativeInfinite(Vinf))
            {
                survmin = false;
                if (V > Vsup + Tol)
                {
                    dansv = false;
                    survmax = false;
                }
                else
                {
                    dansv = true;
                    survmax = false;
                    if (Math.Abs(V - Vsup) <= Tol)
                    {
                        survmax = true;
                    }
                }
            }
            else if (Precision.IsPositiveInfinite(Vsup))
            {
                survmax = false;
                if (V < Vinf - Tol)
                {
                    dansv = false;
                    survmin = false;
                }
                else
                {
                    dansv = true;
                    survmin = false;
                    if (Math.Abs(V - Vinf) <= Tol)
                    {
                        survmin = true;
                    }
                }
            }
            else
            {
                if ((V < Vinf - Tol) || (V > Vsup + Tol))
                {
                    survmin = survmax = dansv = false;
                }
                else
                {
                    dansv = true;
                    survmin = survmax = false;
                    if (Math.Abs(V - Vinf) <= Tol)
                    {
                        survmin = true;
                    }
                    else if (Math.Abs(V - Vsup) <= Tol)
                    {
                        survmax = true;
                    }
                }
            }

            if (!dansu || !dansv)
            {
                return TopAbs_OUT;
            }
            if (surumin || survmin || surumax || survmax)
            {
                return TopAbs_ON;
            }
            return TopAbs_IN;
        }
    }

    /// <summary>True if the point is on a finite bound of the domain (within Tol).</summary>
    /// <remarks>OCCT ignores <paramref name="ReacdreOnPeriodic"/>.</remarks>
    public virtual bool IsThePointOn(in gp_Pnt2d P, double Tol, bool ReacdreOnPeriodic = true)
    {
        double U = P.X();
        double V = P.Y();

        if (nbRestr == 4)
        {
            if ((U < Uinf - Tol) || (U > Usup + Tol) || (V < Vinf - Tol) || (V > Vsup + Tol))
            {
                return (false);
            }
            if ((Math.Abs(U - Uinf) <= Tol) || (Math.Abs(U - Usup) <= Tol) || (Math.Abs(V - Vinf) <= Tol)
                || (Math.Abs(V - Vsup) <= Tol))
            {
                return (true);
            }
            return (false);
        }
        else if (nbRestr == 0)
        {
            return (false);
        }
        else
        {
            bool dansu, dansv, surumin, surumax, survmin, survmax;
            if (Precision.IsNegativeInfinite(Uinf) && Precision.IsPositiveInfinite(Usup))
            {
                dansu = true;
                surumin = surumax = false;
            }
            else if (Precision.IsNegativeInfinite(Uinf))
            {
                surumin = false;
                // OCCT: `>=` here where the V branch below has `>` (docs/occt-issues.md).
                if (U >= Usup + Tol)
                {
                    dansu = false;
                    surumax = false;
                }
                else
                {
                    dansu = true;
                    surumax = false;
                    if (Math.Abs(U - Usup) <= Tol)
                    {
                        surumax = true;
                    }
                }
            }
            else if (Precision.IsPositiveInfinite(Usup))
            {
                surumax = false;
                if (U < Uinf - Tol)
                {
                    dansu = false;
                    surumin = false;
                }
                else
                {
                    dansu = true;
                    surumin = false;
                    if (Math.Abs(U - Uinf) <= Tol)
                    {
                        surumin = true;
                    }
                }
            }
            else
            {
                if ((U < Uinf - Tol) || (U > Usup + Tol))
                {
                    surumin = surumax = dansu = false;
                }
                else
                {
                    dansu = true;
                    surumin = surumax = false;
                    if (Math.Abs(U - Uinf) <= Tol)
                    {
                        surumin = true;
                    }
                    else if (Math.Abs(U - Usup) <= Tol)
                    {
                        surumax = true;
                    }
                }
            }

            if (Precision.IsNegativeInfinite(Vinf) && Precision.IsPositiveInfinite(Vsup))
            {
                dansv = true;
                survmin = survmax = false;
            }
            else if (Precision.IsNegativeInfinite(Vinf))
            {
                survmin = false;
                if (V > Vsup + Tol)
                {
                    dansv = false;
                    survmax = false;
                }
                else
                {
                    dansv = true;
                    survmax = false;
                    if (Math.Abs(V - Vsup) <= Tol)
                    {
                        survmax = true;
                    }
                }
            }
            else if (Precision.IsPositiveInfinite(Vsup))
            {
                survmax = false;
                if (V < Vinf - Tol)
                {
                    dansv = false;
                    survmin = false;
                }
                else
                {
                    dansv = true;
                    survmin = false;
                    if (Math.Abs(V - Vinf) <= Tol)
                    {
                        survmin = true;
                    }
                }
            }
            else
            {
                if ((V < Vinf - Tol) || (V > Vsup + Tol))
                {
                    survmin = survmax = dansv = false;
                }
                else
                {
                    dansv = true;
                    survmin = survmax = false;
                    if (Math.Abs(V - Vinf) <= Tol)
                    {
                        survmin = true;
                    }
                    else if (Math.Abs(V - Vsup) <= Tol)
                    {
                        survmax = true;
                    }
                }
            }

            if (!dansu || !dansv)
            {
                return (false);
            }
            if (surumin || survmin || surumax || survmax)
            {
                return (true);
            }
            return false;
        }
    }

    /// <summary>
    /// The orientation of an arc: FORWARD or REVERSED for a "real" limit of the surface, INTERNAL or EXTERNAL for
    /// an arc on the surface. Always FORWARD here.
    /// </summary>
    public virtual TopAbs_Orientation Orientation(Adaptor2d_Curve2d C) => TopAbs_FORWARD;

    /// <summary>The orientation of the vertex on the arc where it was found.</summary>
    public virtual TopAbs_Orientation Orientation(Adaptor3d_HVertex V) => V.Orientation();

    /// <summary>True if the vertices are the same, whatever their orientations.</summary>
    public virtual bool Identical(Adaptor3d_HVertex V1, Adaptor3d_HVertex V2) => V1.IsSame(V2);

    /// <summary>True if arcs and vertices have 3D representations (Tol3d and Pnt can be used). False here.</summary>
    public virtual bool Has3d() => false;

    /// <exception cref="Standard_DomainError">Always: no 3D representation.</exception>
    public virtual double Tol3d(Adaptor2d_Curve2d C) =>
        throw new Standard_DomainError("Adaptor3d_TopolTool: has no 3d representation");

    /// <exception cref="Standard_DomainError">Always: no 3D representation.</exception>
    public virtual double Tol3d(Adaptor3d_HVertex V) =>
        throw new Standard_DomainError("Adaptor3d_TopolTool: has no 3d representation");

    /// <exception cref="Standard_DomainError">Always: no 3D representation.</exception>
    public virtual gp_Pnt Pnt(Adaptor3d_HVertex V) =>
        throw new Standard_DomainError("Adaptor3d_TopolTool: has no 3d representation");

    //-- ============================================================
    //-- methods used for the samples
    //-- ============================================================

    private static void Analyse(NCollection_Array2<gp_Pnt> array2, int nbup, int nbvp, ref int myNbSamplesU,
                                ref int myNbSamplesV)
    {
        gp_Vec Vi = new gp_Vec(), Vip1 = new gp_Vec();
        int sh, nbch, i, j;

        sh = 1;
        nbch = 0;
        if (nbvp > 2)
        {
            for (i = 2; i < nbup; i++)
            {
                gp_Pnt A = array2.Value(i, 1);
                gp_Pnt B = array2.Value(i, 2);
                gp_Pnt C = array2.Value(i, 3);
                Vi.SetCoord(C.X() - B.X() - B.X() + A.X(),
                            C.Y() - B.Y() - B.Y() + A.Y(),
                            C.Z() - B.Z() - B.Z() + A.Z());
                int locnbch = 0;
                for (j = 3; j < nbvp; j++)
                { //-- try
                    gp_Pnt A1 = array2.Value(i, j - 1);
                    gp_Pnt B1 = array2.Value(i, j);
                    gp_Pnt C1 = array2.Value(i, j + 1);
                    Vip1.SetCoord(C1.X() - B1.X() - B1.X() + A1.X(),
                                  C1.Y() - B1.Y() - B1.Y() + A1.Y(),
                                  C1.Z() - B1.Z() - B1.Z() + A1.Z());
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
                gp_Pnt A = array2.Value(1, j);
                gp_Pnt B = array2.Value(2, j);
                gp_Pnt C = array2.Value(3, j);
                Vi.SetCoord(C.X() - B.X() - B.X() + A.X(),
                            C.Y() - B.Y() - B.Y() + A.Y(),
                            C.Z() - B.Z() - B.Z() + A.Z());
                int locnbch = 0;
                for (i = 3; i < nbup; i++)
                { //-- try
                    gp_Pnt A1 = array2.Value(i - 1, j);
                    gp_Pnt B1 = array2.Value(i, j);
                    gp_Pnt C1 = array2.Value(i + 1, j);
                    Vip1.SetCoord(C1.X() - B1.X() - B1.X() + A1.X(),
                                  C1.Y() - B1.Y() - B1.Y() + A1.Y(),
                                  C1.Z() - B1.Z() - B1.Z() + A1.Z());
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

    /// <summary>Computes the numbers of sample points of the intersection algorithms (NbSamplesU and NbSamplesV).</summary>
    public virtual void ComputeSamplePoints()
    {
        const int aMaxNbSample = 50;

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
        GeomAbs_SurfaceType typS = myS.GetType();
        switch (typS)
        {
            case GeomAbs_Plane:
            {
                nbsv = 2;
                nbsu = 2;
            }
                break;
            case GeomAbs_BezierSurface:
            {
                nbsv = 3 + myS.NbVPoles();
                nbsu = 3 + myS.NbUPoles();
            }
                break;
            case GeomAbs_BSplineSurface:
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
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
            case GeomAbs_SurfaceOfRevolution:
            case GeomAbs_SurfaceOfExtrusion:
            {
                nbsv = 15;
                nbsu = 15;
            }
                break;
            default:
            {
                nbsu = 10;
                nbsv = 10;
            }
                break;
        }

        //-- If the number of points is too great... analyze

        if (nbsu < 6)
        {
            nbsu = 6;
        }
        if (nbsv < 6)
        {
            nbsv = 6;
        }

        if (typS == GeomAbs_BSplineSurface)
        {
            if (nbsu > 8 || nbsv > 8)
            {
                Geom_BSplineSurface Bspl = myS.BSpline();
                NCollection_Array2<gp_Pnt> array2 = Bspl.InternalPoles();
                int nbup = array2.NbRows();
                int nbvp = array2.NbColumns();
                Analyse(array2, nbup, nbvp, ref nbsu, ref nbsv);
            }
            // Check anisotropy
            double anULen = (usup - uinf) / myS.UResolution(1.0);
            double anVLen = (vsup - vinf) / myS.VResolution(1.0);
            double aRatio = anULen / anVLen;
            if (aRatio >= 10.0)
            {
                nbsu *= 2;
                nbsu = Min(nbsu, aMaxNbSample);
            }
            else if (aRatio <= 0.1)
            {
                nbsv *= 2;
                nbsv = Min(nbsv, aMaxNbSample);
            }
        }
        else if (typS == GeomAbs_BezierSurface)
        {
            if (nbsu > 8 || nbsv > 8)
            {
                Geom_BezierSurface Bez = myS.Bezier();
                NCollection_Array2<gp_Pnt> array2 = Bez.InternalPoles();
                int nbup = array2.NbRows();
                int nbvp = array2.NbColumns();
                Analyse(array2, nbup, nbvp, ref nbsu, ref nbsv);
            }
        }

        myNbSamplesU = nbsu;
        myNbSamplesV = nbsv;
    }

    public virtual int NbSamplesU()
    {
        if (myNbSamplesU < 0)
        {
            ComputeSamplePoints();
        }
        return (myNbSamplesU);
    }

    public virtual int NbSamplesV()
    {
        if (myNbSamplesU < 0)
        {
            ComputeSamplePoints();
        }
        return (myNbSamplesV);
    }

    public virtual int NbSamples()
    {
        if (myNbSamplesU < 0)
        {
            ComputeSamplePoints();
        }
        return (myNbSamplesU * myNbSamplesV);
    }

    /// <summary>Copies the U parameters computed by <see cref="SamplePnts"/> into <paramref name="theArray"/> (OCCT's array assignment).</summary>
    public void UParameters(NCollection_Array1<double> theArray)
    {
        theArray.Assign(myUPars!.Array1());
    }

    /// <summary>Copies the V parameters computed by <see cref="SamplePnts"/> into <paramref name="theArray"/> (OCCT's array assignment).</summary>
    public void VParameters(NCollection_Array1<double> theArray)
    {
        theArray.Assign(myVPars!.Array1());
    }

    /// <summary>The sample point of index <paramref name="Index"/>, in the parametric domain and in 3D.</summary>
    public virtual void SamplePoint(int Index, out gp_Pnt2d P2d, out gp_Pnt P3d)
    {
        int i = Index;
        int iu, iv;
        double u, v;
        if (myUPars is null)
        {
            double myDU = (Usup - Uinf) / (myNbSamplesU + 1);
            double myDV = (Vsup - Vinf) / (myNbSamplesV + 1);
            iv = 1 + i / myNbSamplesU;
            iu = 1 + i - (iv - 1) * myNbSamplesU;
            u = Uinf + iu * myDU;
            v = Vinf + iv * myDV;
        }
        else
        {
            iv = (i - 1) / myNbSamplesU + 1;
            iu = (i - 1) % myNbSamplesU + 1;
            u = myUPars.Value(iu);
            v = myVPars!.Value(iv);
        }

        P2d = new gp_Pnt2d(u, v);
        P3d = myS!.Value(u, v);
    }

    /// <summary>True if one of the bounds of the domain is infinite.</summary>
    public virtual bool DomainIsInfinite()
    {
        if (Precision.IsNegativeInfinite(Uinf))
        {
            return (true);
        }
        if (Precision.IsPositiveInfinite(Usup))
        {
            return (true);
        }
        if (Precision.IsNegativeInfinite(Vinf))
        {
            return (true);
        }
        if (Precision.IsPositiveInfinite(Vsup))
        {
            return (true);
        }
        return (false);
    }

    /// <summary>The topological edge of a derived tool (OCCT's <c>void*</c>); null here.</summary>
    public virtual object? Edge() => null;

    /// <summary>
    /// Computes the sample parameters of the intersection algorithms: adaptively (<see cref="BSplSamplePnts"/>)
    /// for a B-spline surface; otherwise the numbers of <see cref="ComputeSamplePoints"/>, uniformly spaced.
    /// </summary>
    public virtual void SamplePnts(double theDefl, int theNUmin, int theNVmin)
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

        GeomAbs_SurfaceType typS = myS.GetType();
        if (typS == GeomAbs_BSplineSurface)
        {
            // Processing BSpline surface
            BSplSamplePnts(theDefl, theNUmin, theNVmin);
            return;
        }
        else
        {
            ComputeSamplePoints();
        }

        myUPars = new NCollection_HArray1<double>(1, myNbSamplesU);
        myVPars = new NCollection_HArray1<double>(1, myNbSamplesV);
        int i;
        double t, dt = (usup - uinf) / (myNbSamplesU - 1);
        myUPars.SetValue(1, uinf);
        myUPars.SetValue(myNbSamplesU, usup);
        for (i = 2, t = uinf + dt; i < myNbSamplesU; ++i, t += dt)
        {
            myUPars.SetValue(i, t);
        }

        dt = (vsup - vinf) / (myNbSamplesV - 1);
        myVPars.SetValue(1, vinf);
        myVPars.SetValue(myNbSamplesV, vsup);
        for (i = 2, t = vinf + dt; i < myNbSamplesV; ++i, t += dt)
        {
            myVPars.SetValue(i, t);
        }
    }

    /// <summary>
    /// Computes the sample parameters of a B-spline surface adaptively: the knots and degree subdivisions, thinned
    /// where the iso-curves stay within <paramref name="theDefl"/> of a chord.
    /// </summary>
    public virtual void BSplSamplePnts(double theDefl, int theNUmin, int theNVmin)
    {
        const int aMaxPnts = 1001;
        Geom_BSplineSurface aBS = myS!.BSpline();
        double uinf, usup, vinf, vsup;
        uinf = myS.FirstUParameter();
        usup = myS.LastUParameter();
        vinf = myS.FirstVParameter();
        vsup = myS.LastVParameter();

        int i, k, j = 1;
        double t1, t2, dt;
        int ui1 = aBS.FirstUKnotIndex();
        int ui2 = aBS.LastUKnotIndex();
        int vi1 = aBS.FirstVKnotIndex();
        int vi2 = aBS.LastVKnotIndex();

        for (i = ui1; i < ui2; ++i)
        {
            if (uinf >= aBS.UKnot(i) && uinf < aBS.UKnot(i + 1))
            {
                ui1 = i;
                break;
            }
        }

        for (i = ui2; i > ui1; --i)
        {
            if (usup <= aBS.UKnot(i) && usup > aBS.UKnot(i - 1))
            {
                ui2 = i;
                break;
            }
        }

        for (i = vi1; i < vi2; ++i)
        {
            if (vinf >= aBS.VKnot(i) && vinf < aBS.VKnot(i + 1))
            {
                vi1 = i;
                break;
            }
        }

        for (i = vi2; i > vi1; --i)
        {
            if (vsup <= aBS.VKnot(i) && vsup > aBS.VKnot(i - 1))
            {
                vi2 = i;
                break;
            }
        }

        int nbsu = ui2 - ui1 + 1;
        nbsu += (nbsu - 1) * (aBS.UDegree() - 1);
        int nbsv = vi2 - vi1 + 1;
        nbsv += (nbsv - 1) * (aBS.VDegree() - 1);
        bool bUuniform = false;
        bool bVuniform = false;

        // modified by NIZHNY-EMV Mon Jun 10 14:19:04 2013
        if (nbsu < theNUmin || nbsv < theNVmin)
        {
            int aNb;
            if (nbsu < nbsv)
            {
                aNb = (int)(nbsv * ((double)theNUmin) / ((double)nbsu));
                aNb = Min(aNb, 30);
                bVuniform = (aNb > nbsv) ? true : bVuniform;
                nbsv = bVuniform ? aNb : nbsv;
            }
            else
            {
                aNb = (int)(nbsu * ((double)theNVmin) / ((double)nbsv));
                aNb = Min(aNb, 30);
                bUuniform = (aNb > nbsu) ? true : bUuniform;
                nbsu = bUuniform ? aNb : nbsu;
            }
        }
        // modified by NIZHNY-EMV Mon Jun 10 14:19:05 2013

        if (nbsu < theNUmin)
        {
            nbsu = theNUmin;
            bUuniform = true;
        }
        else if (nbsu > aMaxPnts)
        {
            nbsu = aMaxPnts;
            bUuniform = true;
        }
        if (nbsv < theNVmin)
        {
            nbsv = theNVmin;
            bVuniform = true;
        }
        else if (nbsv > aMaxPnts)
        {
            nbsv = aMaxPnts;
            bVuniform = true;
        }

        NCollection_Array1<double> anUPars = new(1, nbsu);
        NCollection_Array1<bool> anUFlg = new(1, nbsu);
        NCollection_Array1<double> aVPars = new(1, nbsv);
        NCollection_Array1<bool> aVFlg = new(1, nbsv);

        // Filling of sample parameters
        if (bUuniform)
        {
            t1 = uinf;
            t2 = usup;
            dt = (t2 - t1) / (nbsu - 1);
            anUPars[1] = t1;
            anUFlg[1] = false;
            anUPars[nbsu] = t2;
            anUFlg[nbsu] = false;
            for (i = 2, t1 += dt; i < nbsu; ++i, t1 += dt)
            {
                anUPars[i] = t1;
                anUFlg[i] = false;
            }
        }
        else
        {
            int nbi = aBS.UDegree();
            k = 0;
            t1 = uinf;
            for (i = ui1 + 1; i <= ui2; ++i)
            {
                if (i == ui2)
                {
                    t2 = usup;
                }
                else
                {
                    t2 = aBS.UKnot(i);
                }
                dt = (t2 - t1) / nbi;
                j = 1;
                do
                {
                    ++k;
                    anUPars[k] = t1;
                    anUFlg[k] = false;
                    t1 += dt;
                } while (++j <= nbi);
                t1 = t2;
            }
            ++k;
            anUPars[k] = t1;
        }

        if (bVuniform)
        {
            t1 = vinf;
            t2 = vsup;
            dt = (t2 - t1) / (nbsv - 1);
            aVPars[1] = t1;
            aVFlg[1] = false;
            aVPars[nbsv] = t2;
            aVFlg[nbsv] = false;
            for (i = 2, t1 += dt; i < nbsv; ++i, t1 += dt)
            {
                aVPars[i] = t1;
                aVFlg[i] = false;
            }
        }
        else
        {
            int nbi = aBS.VDegree();
            k = 0;
            t1 = vinf;
            for (i = vi1 + 1; i <= vi2; ++i)
            {
                if (i == vi2)
                {
                    t2 = vsup;
                }
                else
                {
                    t2 = aBS.VKnot(i);
                }
                dt = (t2 - t1) / nbi;
                j = 1;
                do
                {
                    ++k;
                    aVPars[k] = t1;
                    aVFlg[k] = false;
                    t1 += dt;
                } while (++j <= nbi);
                t1 = t2;
            }
            ++k;
            aVPars[k] = t1;
        }

        // Analysis of deflection

        double aDefl2 = Max(theDefl * theDefl, 1.0e-9);
        double tol = Max(0.01 * aDefl2, 1.0e-9);
        int l;

        anUFlg[1] = true;
        anUFlg[nbsu] = true;
        for (i = 1; i <= nbsv; ++i)
        {
            t1 = aVPars[i];
            j = 1;
            bool bCont = true;
            while (j < nbsu - 1 && bCont)
            {
                if (anUFlg[j + 1])
                {
                    ++j;
                    continue;
                }

                t2 = anUPars[j];
                gp_Pnt p1 = myS.Value(t2, t1);
                for (k = j + 2; k <= nbsu; ++k)
                {
                    t2 = anUPars[k];
                    gp_Pnt p2 = myS.Value(t2, t1);

                    if (p1.SquareDistance(p2) <= tol)
                    {
                        continue;
                    }

                    gp_Lin lin = new(p1, new gp_Dir(new gp_Vec(p1, p2)));
                    bool ok = true;
                    for (l = j + 1; l < k; ++l)
                    {
                        if (anUFlg[l])
                        {
                            ok = false;
                            break;
                        }

                        gp_Pnt pp = myS.Value(anUPars[l], t1);
                        double d = lin.SquareDistance(pp);

                        if (d <= aDefl2)
                        {
                            continue;
                        }

                        ok = false;
                        break;
                    }

                    if (!ok)
                    {
                        j = k - 1;
                        anUFlg[j] = true;
                        break;
                    }

                    if (anUFlg[k])
                    {
                        j = k;
                        break;
                    }
                }

                if (k >= nbsu)
                {
                    bCont = false;
                }
            }
        }

        myNbSamplesU = 0;
        for (i = 1; i <= nbsu; i++)
        {
            if (anUFlg[i])
            {
                myNbSamplesU++;
            }
        }

        if (myNbSamplesU < myMinPnts)
        {
            if (myNbSamplesU == 2)
            {
                //"uniform" distribution;
                int nn = nbsu / myMinPnts;
                anUFlg[1 + nn] = true;
                anUFlg[nbsu - nn] = true;
                // OCCT: with nbsu < 4, nn is 0 and no flag is added, but the count below is still set to 4, so the
                // last entries of myUPars stay 0 (docs/occt-issues.md). j is the deflection loop's (1 or nbsu here).
            }
            else
            { // myNbSamplesU == 3
                // insert in bigger segment
                i = 2;
                while (!anUFlg[i++])
                {
                    ;
                }
                if (i < nbsu / 2)
                {
                    j = Min(i + (nbsu - i) / 2, nbsu - 1);
                }
                else
                {
                    j = Max(i / 2, 2);
                }
            }
            anUFlg[j] = true;
            myNbSamplesU = myMinPnts;
        }

        aVFlg[1] = true;
        aVFlg[nbsv] = true;
        for (i = 1; i <= nbsu; ++i)
        {
            t1 = anUPars[i];
            j = 1;
            bool bCont = true;
            while (j < nbsv - 1 && bCont)
            {
                if (aVFlg[j + 1])
                {
                    ++j;
                    continue;
                }

                t2 = aVPars[j];
                gp_Pnt p1 = myS.Value(t1, t2);
                for (k = j + 2; k <= nbsv; ++k)
                {
                    t2 = aVPars[k];
                    gp_Pnt p2 = myS.Value(t1, t2);

                    if (p1.SquareDistance(p2) <= tol)
                    {
                        continue;
                    }
                    gp_Lin lin = new(p1, new gp_Dir(new gp_Vec(p1, p2)));
                    bool ok = true;
                    for (l = j + 1; l < k; ++l)
                    {
                        if (aVFlg[l])
                        {
                            ok = false;
                            break;
                        }

                        gp_Pnt pp = myS.Value(t1, aVPars[l]);
                        double d = lin.SquareDistance(pp);

                        if (d <= aDefl2)
                        {
                            continue;
                        }

                        ok = false;
                        break;
                    }

                    if (!ok)
                    {
                        j = k - 1;
                        aVFlg[j] = true;
                        break;
                    }

                    if (aVFlg[k])
                    {
                        j = k;
                        break;
                    }
                }

                if (k >= nbsv)
                {
                    bCont = false;
                }
            }
        }

        myNbSamplesV = 0;
        for (i = 1; i <= nbsv; i++)
        {
            if (aVFlg[i])
            {
                myNbSamplesV++;
            }
        }

        if (myNbSamplesV < myMinPnts)
        {
            if (myNbSamplesV == 2)
            {
                //"uniform" distribution;
                int nn = nbsv / myMinPnts;
                aVFlg[1 + nn] = true;
                aVFlg[nbsv - nn] = true;
                myNbSamplesV = myMinPnts;
                // OCCT: as for U, with nbsv < 4 the count becomes 4 without new flags (docs/occt-issues.md).
            }
            else
            { // myNbSamplesU == 3
                // insert in bigger segment
                i = 2;
                while (!aVFlg[i++])
                {
                    ;
                }
                if (i < nbsv / 2)
                {
                    j = Min(i + (nbsv - i) / 2, nbsv - 1);
                }
                else
                {
                    j = Max(i / 2, 2);
                }
            }
            myNbSamplesV = myMinPnts;
            aVFlg[j] = true;
        }

        // modified by NIZNHY-PKV Fri Dec 16 10:05:01 2011f
        bool bFlag;

        // U
        bFlag = (myNbSamplesU < theNUmin);
        if (bFlag)
        {
            myNbSamplesU = nbsu;
        }

        myUPars = new NCollection_HArray1<double>(1, myNbSamplesU);

        for (j = 0, i = 1; i <= nbsu; ++i)
        {
            if (bFlag)
            {
                myUPars.SetValue(i, anUPars[i]);
            }
            else
            {
                if (anUFlg[i])
                {
                    ++j;
                    myUPars.SetValue(j, anUPars[i]);
                }
            }
        }

        // V
        bFlag = (myNbSamplesV < theNVmin);
        if (bFlag)
        {
            myNbSamplesV = nbsv;
        }

        myVPars = new NCollection_HArray1<double>(1, myNbSamplesV);

        for (j = 0, i = 1; i <= nbsv; ++i)
        {
            if (bFlag)
            {
                myVPars.SetValue(i, aVPars[i]);
            }
            else
            {
                if (aVFlg[i])
                {
                    ++j;
                    myVPars.SetValue(j, aVPars[i]);
                }
            }
        }
        // modified by NIZNHY-PKV Mon Dec 26 12:25:35 2011t
    }

    /// <summary>True if the sampling is uniform (any surface but a B-spline).</summary>
    public virtual bool IsUniformSampling()
    {
        GeomAbs_SurfaceType typS = myS!.GetType();

        return typS != GeomAbs_BSplineSurface;
    }

    /// <summary>Computes the parameters of the apex of a cone.</summary>
    public static void GetConeApexParam(in gp_Cone theC, out double theU, out double theV)
    {
        gp_Ax3 Pos = theC.Position();
        double Radius = theC.RefRadius();
        double SAngle = theC.SemiAngle();
        gp_Pnt P = theC.Apex();

        gp_Trsf T = new();
        T.SetTransformation(Pos);
        gp_Pnt Ploc = P.Transformed(T);

        if (Ploc.X() == 0.0 && Ploc.Y() == 0.0)
        {
            theU = 0.0;
        }
        else if (-Radius > Ploc.Z() * Math.Tan(SAngle))
        {
            // the point is at the `wrong` side of the apex
            theU = Math.Atan2(-Ploc.Y(), -Ploc.X());
        }
        else
        {
            theU = Math.Atan2(Ploc.Y(), Ploc.X());
        }

        if (theU < -1.0e-16)
        {
            theU += (M_PI + M_PI);
        }
        else if (theU < 0)
        {
            theU = 0;
        }

        theV =
            Math.Sin(SAngle) * (Ploc.X() * Math.Cos(theU) + Ploc.Y() * Math.Sin(theU) - Radius) + Math.Cos(SAngle) * Ploc.Z();
    }
}
