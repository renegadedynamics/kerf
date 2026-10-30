// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The sections of the old pipe algorithm, for <see cref="GeomFill_AppSweep"/>: a circle of constant radius, a
/// constant section, or one evolving between two sections, moved along a path; or, with three adaptors, the arcs of
/// a pipe on an edge (the case <see cref="GeomFill_Pipe"/> uses).
/// </summary>
/// <remarks>
/// A working object. It works on copies of the caller's curves (or on new B-splines converted from them), and keeps
/// the caller's adaptors, as OCCT keeps the handles.
/// </remarks>
public sealed class GeomFill_SweepSectionGenerator : IAppBlend_SectionGenerator<int>
{
    private Geom_BSplineCurve? myPath;
    private Geom_BSplineCurve? myFirstSect;
    private Geom_BSplineCurve? myLastSect;
    private Adaptor3d_Curve? myAdpPath;
    private Adaptor3d_Curve? myAdpFirstSect;
    private Adaptor3d_Curve? myAdpLastSect;
    private gp_Ax1 myCircPathAxis = new gp_Ax1();
    private double myRadius;
    // OCCT's myIsDone is written by Init and Perform but never read; it is left out.
    private int myNbSections;
    private readonly NCollection_Sequence<gp_Trsf> myTrsfs = new();
    private int myType;
    private bool myPolynomial;

    public GeomFill_SweepSectionGenerator()
    {
        myRadius = 0.0;
        myNbSections = 0;
        myType = -1;
        myPolynomial = false;
    }

    /// <summary>A circle of radius Radius along Path.</summary>
    public GeomFill_SweepSectionGenerator(Geom_Curve Path, double Radius)
    {
        Init(Path, Radius);
    }

    /// <summary>The section FirstSect along Path.</summary>
    public GeomFill_SweepSectionGenerator(Geom_Curve Path, Geom_Curve FirstSect)
    {
        Init(Path, FirstSect);
    }

    /// <summary>A section evolving from FirstSect to LastSect along Path.</summary>
    public GeomFill_SweepSectionGenerator(Geom_Curve Path, Geom_Curve FirstSect, Geom_Curve LastSect)
    {
        Init(Path, FirstSect, LastSect);
    }

    /// <summary>The arcs of radius Radius from Curve1 to Curve2 around Path (a pipe on an edge).</summary>
    public GeomFill_SweepSectionGenerator(Adaptor3d_Curve Path, Adaptor3d_Curve Curve1, Adaptor3d_Curve Curve2,
                                          double Radius)
    {
        Init(Path, Curve1, Curve2, Radius);
    }

    public void Init(Geom_Curve Path, double Radius)
    {
        myRadius = Radius;
        GeomAdaptor_Curve ThePath = new GeomAdaptor_Curve(Path);

        if (ThePath.GetType() == GeomAbs_Circle)
        {
            myCircPathAxis = ThePath.Circle().Axis();
            myType = 4;
        }
        else
        {
            myType = 1;
        }
        if (Path is Geom_BSplineCurve)
        {
            myPath = (Geom_BSplineCurve)Path.Copy();
        }
        else
        {
            myPath = GeomConvert.CurveToBSplineCurve(Path);
        }
    }

    public void Init(Geom_Curve Path, Geom_Curve FirstSect)
    {
        myRadius = 0;
        GeomAdaptor_Curve ThePath = new GeomAdaptor_Curve(Path);

        if (ThePath.GetType() == GeomAbs_Circle)
        {
            myCircPathAxis = ThePath.Circle().Axis();
            myType = 5;
        }

        else
        {
            myType = 2;
        }

        if (Path is Geom_BSplineCurve)
        {
            myPath = (Geom_BSplineCurve)Path.Copy();
        }
        else
        {
            myPath = GeomConvert.CurveToBSplineCurve(Path);
        }
        if (FirstSect is Geom_BSplineCurve)
        {
            myFirstSect = (Geom_BSplineCurve)FirstSect.Copy();
        }
        else
        {
            // JAG
            myFirstSect = GeomConvert.CurveToBSplineCurve(FirstSect, Convert_QuasiAngular);
        }
        if (myFirstSect.IsPeriodic())
        {
            myFirstSect.SetNotPeriodic();
        }
    }

    public void Init(Geom_Curve Path, Geom_Curve FirstSect, Geom_Curve LastSect)
    {
        myRadius = 0;
        GeomAdaptor_Curve ThePath = new GeomAdaptor_Curve(Path);

        if (ThePath.GetType() == GeomAbs_Circle)
        {
            myCircPathAxis = ThePath.Circle().Axis();
            myType = 6;
        }
        else
        {
            myType = 3;
        }

        if (Path is Geom_BSplineCurve)
        {
            myPath = (Geom_BSplineCurve)Path.Copy();
        }
        else
        {
            myPath = GeomConvert.CurveToBSplineCurve(Path);
        }

        // JAG
        if (FirstSect is Geom_BSplineCurve)
        {
            myFirstSect = (Geom_BSplineCurve)FirstSect.Copy();
        }
        else
        {
            myFirstSect = GeomConvert.CurveToBSplineCurve(FirstSect, Convert_QuasiAngular);
        }
        if (LastSect is Geom_BSplineCurve)
        {
            myLastSect = (Geom_BSplineCurve)LastSect.Copy();
        }
        else
        {
            myLastSect = GeomConvert.CurveToBSplineCurve(LastSect, Convert_QuasiAngular);
        }

        if (myFirstSect.IsPeriodic())
        {
            myFirstSect.SetNotPeriodic();
        }
        if (myLastSect.IsPeriodic())
        {
            myLastSect.SetNotPeriodic();
        }

        // JAG

        GeomFill_Profiler Profil = new GeomFill_Profiler();
        Profil.AddCurve(myFirstSect);
        Profil.AddCurve(myLastSect);
        Profil.Perform(Precision.Confusion());

        myFirstSect = Profil.InternalCurve(1) as Geom_BSplineCurve;
        myLastSect = Profil.InternalCurve(2) as Geom_BSplineCurve;
    }

    public void Init(Adaptor3d_Curve Path, Adaptor3d_Curve Curve1, Adaptor3d_Curve Curve2, double Radius)
    {
        myRadius = Radius;
        myType = 0;

        Geom_Curve CC = GeomAdaptor.MakeCurve(Path)!;
        myPath = GeomConvert.CurveToBSplineCurve(CC);
        myAdpPath = Path;
        myAdpFirstSect = Curve1;
        myAdpLastSect = Curve2;
    }

    /// <summary>Computes the number of sections and their transformations.</summary>
    public void Perform(bool Polynomial = false)
    {
        myPolynomial = Polynomial;

        // eval myNbSections.
        int NSpans = myPath!.NbKnots() - 1;

        myNbSections = 21 * NSpans;

        double U;

        double U1 = myPath.FirstParameter();
        double U2 = myPath.LastParameter();

        GCPnts_QuasiUniformDeflection Samp = new GCPnts_QuasiUniformDeflection();
        // Calcul de la longueur approximative de la courbe
        GeomAdaptor_Curve AdpPath = new GeomAdaptor_Curve(myPath);
        gp_Pnt P1 = AdpPath.Value(U1);
        gp_Pnt P2 = AdpPath.Value((U1 + U2) / 2.0);
        gp_Pnt P3 = AdpPath.Value(U2);
        double Length = P1.Distance(P2) + P2.Distance(P3);
        double Fleche = 1.0e-5 * Length;
        Samp.Initialize(AdpPath, Fleche);

        if (Samp.IsDone() && (Samp.NbPoints() > myNbSections))
        {
            myNbSections = Samp.NbPoints();
        }
        // the transformations are calculate on differents points of <myPath>
        // corresponding to the path parameter uniformly reparted.
        double DeltaU = (U2 - U1) / (double)(myNbSections - 1);
        NCollection_Array1<double> Parameters = new(1, myNbSections);

        Parameters[1] = 0.0;
        for (int i = 2; i < myNbSections; i++)
        {
            Parameters[i] = (i - 1) * DeltaU;
        }
        Parameters[myNbSections] = U2 - U1;

        gp_Vec D1Ref, D1;
        gp_Pnt PRef, P;
        gp_Trsf TR = new gp_Trsf(), cumulTR = new gp_Trsf(), Trans = new gp_Trsf();

        myPath.D1(U1, out PRef, out D1Ref);

        if ((myType == 1) || (myType == 4))
        {
            // We create a circle with radius <myRadius>. This axis is create with
            // main direction <DRef> (first derivate vector of <myPath> on the first
            // point <PRef> ). This circle is, after transform to BSpline curve,
            // put in <myFirstSect>.

            gp_Ax2 CircleAxis = new gp_Ax2(PRef, new gp_Dir(D1Ref));
            Geom_TrimmedCurve Circ = new Geom_TrimmedCurve(new Geom_Circle(CircleAxis, myRadius), 0.0, 2.0 * M_PI);

            myFirstSect = GeomConvert.CurveToBSplineCurve(Circ, Convert_QuasiAngular);
        }

        if (myType <= 3 && myType >= 1)
        {
            for (int i = 2; i <= myNbSections; i++)
            {
                U = Parameters[i] + U1;
                if (i == myNbSections)
                {
                    U = U2;
                }

                myPath.D1(U, out P, out D1);

                // Eval the translation between the (i-1) section and the i-th.
                Trans.SetTranslation(PRef, P);

                gp_Trsf Rot = new gp_Trsf();
                if (!D1Ref.IsParallel(D1, Precision.Angular()))
                {
                    // Eval the Rotation between (i-1) section and the i-th.
                    Rot.SetRotation(new gp_Ax1(P, new gp_Dir(D1Ref ^ D1)), D1Ref.AngleWithRef(D1, D1Ref ^ D1));
                }
                else if (D1Ref.IsOpposite(D1, Precision.Angular()))
                {
                    // OCCT: TR is set only here (the braces put "TR = Rot * Trans" in the opposite-tangents branch), so
                    // the rotations and translations computed above are dropped (occt-issues.md).
                    // TR is the transformation between (i-1) section and the i-th.
                    TR = Rot * Trans;
                }
                // cumulTR is the transformation between <myFirstSec> and
                // the i-th section.
                cumulTR = TR * cumulTR;

                myTrsfs.Append(cumulTR);

                PRef = P;
                D1Ref = D1;
            }
        }
        else if (myType != 0)
        {
            for (int i = 2; i <= myNbSections; i++)
            {
                cumulTR.SetRotation(myCircPathAxis, Parameters[i]);
                myTrsfs.Append(cumulTR);
            }
        }

    }

    /// <summary>The shape of a section: its number of poles and knots, its degree, and the number of 2D poles (0).</summary>
    public void GetShape(out int NbPoles, out int NbKnots, out int Degree, out int NbPoles2d)
    {
        if (myType != 0)
        {
            NbPoles = myFirstSect!.NbPoles();
            NbKnots = myFirstSect.NbKnots();
            Degree = myFirstSect.Degree();
        }
        else
        { // myType == 0
            NbPoles = 7;
            NbKnots = 2;
            Degree = 6;
        }
        NbPoles2d = 0;
    }

    /// <summary>The knots of the sections.</summary>
    public void Knots(NCollection_Array1<double> TKnots)
    {
        if (myType != 0)
        {
            TKnots.Assign(myFirstSect!.InternalKnots());
        }
        else
        {
            TKnots[1] = 0.0;
            TKnots[2] = 1.0;
        }
    }

    /// <summary>The multiplicities of the sections.</summary>
    public void Mults(NCollection_Array1<int> TMults)
    {
        if (myType != 0)
        {
            TMults.Assign(myFirstSect!.InternalMultiplicities());
        }
        else
        {
            TMults[1] = TMults[2] = 7;
        }
    }

    /// <summary>The number of sections.</summary>
    public int NbSections() => myNbSections;

    /// <summary>
    /// The section P and its derivatives along the path; false where the derivatives aren't computed (between the
    /// ends, and for a pipe on an edge).
    /// </summary>
    public bool Section(int P,
                        NCollection_Array1<gp_Pnt> Poles,
                        NCollection_Array1<gp_Vec> DPoles,
                        NCollection_Array1<gp_Pnt2d> Poles2d,
                        NCollection_Array1<gp_Vec2d> DPoles2d,
                        NCollection_Array1<double> Weigths,
                        NCollection_Array1<double> DWeigths)
    {
        Section(P, Poles, Poles2d, Weigths);

        // pour les tuyaux sur aretes pour l'instant on ne calcule pas les derivees
        if (myType == 0)
        {
            return false; // a voir pour mieux.
        }

        // calcul des derivees sur la surface
        // on calcule les derivees en approximant le path au voisinage du point
        // P(u) par le cercle osculateur au path .

        // calcul du cercle osculateur.

        double U;
        if (P == 1)
        {
            U = myPath!.FirstParameter();
        }
        else if (P == myNbSections)
        {
            U = myPath!.LastParameter();
        }
        else
        {
            return false;
        }

        gp_Vec D1, D2;
        gp_Pnt Pt;

        myPath.D2(U, out Pt, out D1, out D2);
        double l = D1.Magnitude();

        if (l < Epsilon(1.0))
        {
            return false;
        }

        gp_Dir T = new gp_Dir(D1);
        double m = D2.Dot(new gp_Vec(T));
        gp_Vec D = D2 - m * new gp_Vec(T);
        double c = D.Magnitude() / (l * l);

        if (c < Epsilon(1.0))
        {
            // null curvature : equivalent to a translation of the section
            for (int i = 1; i <= myFirstSect!.NbPoles(); i++)
            {
                DPoles[i] = D1;
            }
        }
        else
        {
            gp_Dir N = new gp_Dir(D);
            gp_Pnt Q = Pt.Translated((1.0 / c) * new gp_Vec(N));
            double x, y;
            gp_Vec V;
            for (int i = 1; i <= myFirstSect!.NbPoles(); i++)
            {
                V = new gp_Vec(Q, Poles[i]);
                x = V * new gp_Vec(T);
                y = V * new gp_Vec(N);
                DPoles[i] = x * new gp_Vec(N) - y * new gp_Vec(T);
                if (DPoles[i].Magnitude() > Epsilon(1.0))
                {
                    DPoles[i].Normalize();
                    DPoles[i] *= Math.Sqrt(x * x + y * y);
                }
            }
        }

        for (int i = 1; i <= myFirstSect.NbPoles(); i++)
        {
            DWeigths[i] = 0.0;
        }

        return true;
    }

    /// <summary>The section P.</summary>
    public void Section(int P,
                        NCollection_Array1<gp_Pnt> Poles,
                        NCollection_Array1<gp_Pnt2d> Poles2d,
                        NCollection_Array1<double> Weigths)
    {
        if (myType != 0)
        {
            Poles.Assign(myFirstSect!.InternalPoles());
            Weigths.Assign(myFirstSect.InternalWeightsArray());
            gp_Trsf cumulTR;
            if (P > 1)
            {
                cumulTR = myTrsfs[P - 1];
                // <cumulTR> transform <myFirstSect> to the P ieme Section. In fact
                // each points of the array <poles> will be transformed.

                if ((myType == 3) || (myType == 6))
                {
                    for (int i = 1; i <= myFirstSect.NbPoles(); i++)
                    {
                        Poles[i].SetXYZ((myNbSections - P) * myFirstSect.Pole(i).XYZ()
                                        + (P - 1) * myLastSect!.Pole(i).XYZ());
                        Poles[i].SetXYZ(Poles[i].XYZ() / (myNbSections - 1));

                        Weigths[i] = (myNbSections - P) * myFirstSect.Weight(i) + (P - 1) * myLastSect.Weight(i);
                        Weigths[i] /= myNbSections - 1;
                    }
                }

                for (int i = 1; i <= Poles.Length(); i++)
                {
                    Poles[i].Transform(cumulTR);
                }
            }
        }
        else
        {
            double Coef = (P - 1.0) / (myNbSections - 1.0);
            double U = (1 - Coef) * myAdpPath!.FirstParameter() + Coef * myAdpPath.LastParameter();

            gp_Pnt PPath = myAdpPath.Value(U);

            double Alpha = U - myAdpPath.FirstParameter();
            Alpha /= myAdpPath.LastParameter() - myAdpPath.FirstParameter();

            double U1 = (1 - Alpha) * myAdpFirstSect!.FirstParameter() + Alpha * myAdpFirstSect.LastParameter();

            if (myAdpFirstSect.GetType() == GeomAbs_Line)
            {
                if (Precision.IsInfinite(myAdpFirstSect.FirstParameter())
                    || Precision.IsInfinite(myAdpFirstSect.LastParameter()))
                {
                    gp_Lin aLine = myAdpFirstSect.Line();
                    U1 = ElCLib.Parameter(aLine, PPath);
                }
            }
            gp_Pnt P1 = myAdpFirstSect.Value(U1);

            double U2 = (1 - Alpha) * myAdpLastSect!.FirstParameter() + Alpha * myAdpLastSect.LastParameter();

            if (myAdpLastSect.GetType() == GeomAbs_Line)
            {
                if (Precision.IsInfinite(myAdpLastSect.FirstParameter())
                    || Precision.IsInfinite(myAdpLastSect.LastParameter()))
                {
                    gp_Lin aLine = myAdpLastSect.Line();
                    U2 = ElCLib.Parameter(aLine, PPath);
                }
            }
            gp_Pnt P2 = myAdpLastSect.Value(U2);

            gp_Ax2 Axis = new gp_Ax2();
            double Angle;
            if (P1.Distance(P2) < Precision.Confusion())
            {
                Angle = 0.0;
            }
            else
            {
                Axis = new gp_Ax2(PPath, new gp_Dir(new gp_Vec(PPath, P1) ^ new gp_Vec(PPath, P2)),
                                  new gp_Dir(new gp_Vec(PPath, P1)));
                Angle = ElCLib.CircleParameter(Axis, P2);
            }
            if (Angle < Precision.Angular())
            {
                for (int i = 1; i <= Poles.Upper(); i++)
                {
                    Poles[i] = P1;
                    Weigths[i] = 1;
                }
            }
            else
            {
                Geom_Circle Circ = new Geom_Circle(Axis, myRadius);
                Geom_TrimmedCurve CT = new Geom_TrimmedCurve(Circ, 0.0, Angle);
                Geom_BSplineCurve BS;
                if (myPolynomial)
                {
                    BS = GeomConvert.CurveToBSplineCurve(CT, Convert_Polynomial);
                }
                else
                {
                    BS = GeomConvert.CurveToBSplineCurve(CT, Convert_QuasiAngular);
                }

                Poles.Assign(BS.InternalPoles());
                Weigths.Assign(BS.InternalWeightsArray());
            }
        }
    }

    /// <summary>The transformation from the first section to the section Index + 1.</summary>
    public gp_Trsf Transformation(int Index)
    {
        if (Index > myTrsfs.Length())
        {
            throw new Standard_RangeError("GeomFill_SweepSectionGenerator::Transformation");
        }

        return myTrsfs[Index];
    }

    /// <summary>The path parameter of the section P.</summary>
    public double Parameter(int P)
    {
        if (P == 1)
        {
            return myPath!.FirstParameter();
        }
        else if (P == myNbSections)
        {
            return myPath!.LastParameter();
        }
        else
        {
            double U1 = myPath!.FirstParameter();
            double U2 = myPath.LastParameter();
            double prm = ((myNbSections - P) * U1 + (P - 1) * U2) / (double)(myNbSections - 1);
            return prm;
        }
    }
}
