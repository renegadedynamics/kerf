// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The extrema of the distance between two surfaces: Extrema_ExtElSS for two planes, Extrema_GenExtSS (20 x 20
/// samples) otherwise. Extrema whose parameters fall outside the given ranges (beyond the tolerances) are dropped;
/// periodic parameters are first put into the period that starts at the range's start.
/// </summary>
/// <remarks>
/// The second surface is kept by reference, as OCCT keeps it: it must outlive the algorithm. For parallel planes
/// <see cref="NbExt"/> is 1 with the distance, and there are no points (<see cref="Points"/> throws, where OCCT's
/// release build reads past its empty sequences).
/// </remarks>
public class Extrema_ExtSS
{
    private Adaptor3d_Surface? myS2;
    private bool myDone;
    private bool myIsPar;
    private readonly Extrema_ExtElSS myExtElSS = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPOnS1 = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPOnS2 = new();
    private double myuinf1;
    private double myusup1;
    private double myvinf1;
    private double myvsup1;
    private double myuinf2;
    private double myusup2;
    private double myvinf2;
    private double myvsup2;
    private double mytolS1;
    private double mytolS2;
    private readonly NCollection_Sequence<double> mySqDist = new();
    private GeomAbs_SurfaceType myStype;

    /// <summary>An empty algorithm.</summary>
    public Extrema_ExtSS()
    {
        myS2 = null;
        myDone = false;
        myIsPar = false;
        myuinf1 = 0.0;
        myusup1 = 0.0;
        myvinf1 = 0.0;
        myvsup1 = 0.0;
        myuinf2 = 0.0;
        myusup2 = 0.0;
        myvinf2 = 0.0;
        myvsup2 = 0.0;
        mytolS1 = 0.0;
        mytolS2 = 0.0;
        myStype = GeomAbs_OtherSurface;
    }

    /// <summary>Computes the extrema between S1 and S2 over their whole ranges.</summary>
    public Extrema_ExtSS(Adaptor3d_Surface S1, Adaptor3d_Surface S2, double TolS1, double TolS2)
    {
        Initialize(S2, S2.FirstUParameter(), S2.LastUParameter(), S2.FirstVParameter(), S2.LastVParameter(), TolS2);

        Perform(S1, S1.FirstUParameter(), S1.LastUParameter(), S1.FirstVParameter(), S1.LastVParameter(), TolS1);
    }

    /// <summary>Computes the extrema between S1 and S2 over the given ranges.</summary>
    public Extrema_ExtSS(Adaptor3d_Surface S1, Adaptor3d_Surface S2, double Uinf1, double Usup1, double Vinf1,
        double Vsup1, double Uinf2, double Usup2, double Vinf2, double Vsup2, double TolS1, double TolS2)
    {
        Initialize(S2, Uinf2, Usup2, Vinf2, Vsup2, TolS2);
        Perform(S1, Uinf1, Usup1, Vinf1, Vsup1, TolS1);
    }

    /// <summary>Sets the second surface and its range.</summary>
    public void Initialize(Adaptor3d_Surface S2, double Uinf2, double Usup2, double Vinf2, double Vsup2, double TolS2)
    {
        myS2 = S2;
        myIsPar = false;
        myuinf2 = Uinf2;
        myusup2 = Usup2;
        myvinf2 = Vinf2;
        myvsup2 = Vsup2;
        mytolS2 = TolS2;
        myStype = S2.GetType();
    }

    /// <summary>Computes the extrema with S1 over the given range (the second surface must have been initialised).</summary>
    public void Perform(Adaptor3d_Surface S1, double Uinf1, double Usup1, double Vinf1, double Vsup1, double TolS1)
    {
        if (myS2 is null)
        {
            throw new StdFail_NotDone("Extrema_ExtSS: the second surface is not initialized");
        }
        myuinf1 = Uinf1;
        myusup1 = Usup1;
        myvinf1 = Vinf1;
        myvsup1 = Vsup1;
        mytolS1 = TolS1;
        myPOnS1.Clear();
        myPOnS2.Clear();
        mySqDist.Clear();
        int i;
        GeomAbs_SurfaceType myS1type = S1.GetType();
        const int NbU = 20, NbV = 20;

        switch (myS1type)
        {
            case GeomAbs_Plane:
            {
                switch (myStype)
                {
                    case GeomAbs_Plane:
                    {
                        myExtElSS.Perform(S1.Plane(), myS2.Plane());
                    }
                        break;
                    default:
                    {
                        Extrema_GenExtSS Ext = new(S1, myS2, NbU, NbV, mytolS1, mytolS2);
                        myDone = Ext.IsDone();
                        if (myDone)
                        {
                            int NbExt = Ext.NbExt();
                            double U1, V1, U2, V2;
                            Extrema_POnSurf PS1;
                            Extrema_POnSurf PS2;
                            for (i = 1; i <= NbExt; i++)
                            {
                                PS1 = Ext.PointOnS1(i);
                                PS2 = Ext.PointOnS2(i);
                                PS1.Parameter(out U1, out V1);
                                PS2.Parameter(out U2, out V2);
                                if (S1.IsUPeriodic())
                                {
                                    U1 = ElCLib.InPeriod(U1, myuinf1, myuinf1 + S1.UPeriod());
                                }
                                if (S1.IsVPeriodic())
                                {
                                    V1 = ElCLib.InPeriod(V1, myvinf1, myvinf1 + S1.VPeriod());
                                }
                                if (myS2.IsUPeriodic())
                                {
                                    U2 = ElCLib.InPeriod(U2, myuinf2, myuinf2 + myS2.UPeriod());
                                }
                                if (myS2.IsVPeriodic())
                                {
                                    V2 = ElCLib.InPeriod(V2, myvinf2, myvinf2 + myS2.VPeriod());
                                }

                                if ((myuinf1 - U1) <= mytolS1 && (U1 - myusup1) <= mytolS1
                                    && (myvinf1 - V1) <= mytolS1 && (V1 - myvsup1) <= mytolS1
                                    && (myuinf2 - U2) <= mytolS2 && (U2 - myusup2) <= mytolS2
                                    && (myvinf2 - V2) <= mytolS2 && (V2 - myvsup2) <= mytolS2)
                                {
                                    mySqDist.Append(Ext.SquareDistance(i));
                                    myPOnS1.Append(new Extrema_POnSurf(U1, V1, PS1.Value()));
                                    myPOnS2.Append(new Extrema_POnSurf(U2, V2, PS2.Value()));
                                }
                            }
                        }
                        return;
                    }
                }
                break;
            }
            default:
            {
                Extrema_GenExtSS Ext = new(S1, myS2, NbU, NbV, mytolS1, mytolS2);
                myDone = Ext.IsDone();
                if (myDone)
                {
                    int NbExt = Ext.NbExt();
                    double U1, V1, U2, V2;
                    Extrema_POnSurf PS1;
                    Extrema_POnSurf PS2;
                    for (i = 1; i <= NbExt; i++)
                    {
                        PS1 = Ext.PointOnS1(i);
                        PS2 = Ext.PointOnS2(i);
                        PS1.Parameter(out U1, out V1);
                        PS2.Parameter(out U2, out V2);
                        if (S1.IsUPeriodic())
                        {
                            U1 = ElCLib.InPeriod(U1, myuinf1, myuinf1 + S1.UPeriod());
                        }
                        if (S1.IsVPeriodic())
                        {
                            V1 = ElCLib.InPeriod(V1, myvinf1, myvinf1 + S1.VPeriod());
                        }
                        if (myS2.IsUPeriodic())
                        {
                            U2 = ElCLib.InPeriod(U2, myuinf2, myuinf2 + myS2.UPeriod());
                        }
                        if (myS2.IsVPeriodic())
                        {
                            V2 = ElCLib.InPeriod(V2, myvinf2, myvinf2 + myS2.VPeriod());
                        }

                        if ((myuinf1 - U1) <= mytolS1 && (U1 - myusup1) <= mytolS1 && (myvinf1 - V1) <= mytolS1
                            && (V1 - myvsup1) <= mytolS1 && (myuinf2 - U2) <= mytolS2 && (U2 - myusup2) <= mytolS2
                            && (myvinf2 - V2) <= mytolS2 && (V2 - myvsup2) <= mytolS2)
                        {
                            mySqDist.Append(Ext.SquareDistance(i));
                            myPOnS1.Append(new Extrema_POnSurf(U1, V1, PS1.Value()));
                            myPOnS2.Append(new Extrema_POnSurf(U2, V2, PS2.Value()));
                        }
                    }
                }
                return;
            }
        }

        myDone = myExtElSS.IsDone();
        if (myDone)
        {
            myIsPar = myExtElSS.IsParallel();
            if (myIsPar)
            {
                mySqDist.Append(myExtElSS.SquareDistance(1));
            }
            else
            {
                int NbExt = myExtElSS.NbExt();
                double U1, V1, U2, V2;
                for (i = 1; i <= NbExt; i++)
                {
                    myExtElSS.Points(i, out Extrema_POnSurf PS1, out Extrema_POnSurf PS2);
                    PS1.Parameter(out U1, out V1);
                    PS2.Parameter(out U2, out V2);
                    if ((myuinf1 - U1) <= mytolS1 && (U1 - myusup1) <= mytolS1 && (myvinf1 - V1) <= mytolS1
                        && (V1 - myvsup1) <= mytolS1 && (myuinf2 - U2) <= mytolS2 && (U2 - myusup2) <= mytolS2
                        && (myvinf2 - V2) <= mytolS2 && (V2 - myvsup2) <= mytolS2)
                    {
                        mySqDist.Append(myExtElSS.SquareDistance(i));
                        myPOnS1.Append(PS1);
                        myPOnS2.Append(PS2);
                    }
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the surfaces are parallel planes.</summary>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myIsPar;
    }

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int N)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        return mySqDist.Value(N);
    }

    /// <summary>The number of extrema (1 for parallel planes).</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return mySqDist.Length();
    }

    /// <summary>The points of the Nth extremum on S1 and S2.</summary>
    public void Points(int N, out Extrema_POnSurf P1, out Extrema_POnSurf P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        P1 = myPOnS1.Value(N);
        P2 = myPOnS2.Value(N);
    }
}
