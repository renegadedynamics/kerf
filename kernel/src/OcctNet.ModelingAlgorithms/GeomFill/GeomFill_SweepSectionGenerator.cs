// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;

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
    private BSplineCurve3D? myPath;
    private BSplineCurve3D? myFirstSect;
    private BSplineCurve3D? myLastSect;
    private CurveAdaptorBase? myAdpPath;
    private CurveAdaptorBase? myAdpFirstSect;
    private CurveAdaptorBase? myAdpLastSect;
    private Axis3D myCircPathAxis = new Axis3D();
    private double myRadius;
    // OCCT's myIsDone is written by Init and Perform but never read; it is left out.
    private int myNbSections;
    private readonly NCollection_Sequence<Transform3D> myTrsfs = new();
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
    public GeomFill_SweepSectionGenerator(Curve3D Path, double Radius)
    {
        Init(Path, Radius);
    }

    /// <summary>The section FirstSect along Path.</summary>
    public GeomFill_SweepSectionGenerator(Curve3D Path, Curve3D FirstSect)
    {
        Init(Path, FirstSect);
    }

    /// <summary>A section evolving from FirstSect to LastSect along Path.</summary>
    public GeomFill_SweepSectionGenerator(Curve3D Path, Curve3D FirstSect, Curve3D LastSect)
    {
        Init(Path, FirstSect, LastSect);
    }

    /// <summary>The arcs of radius Radius from Curve1 to Curve2 around Path (a pipe on an edge).</summary>
    public GeomFill_SweepSectionGenerator(CurveAdaptorBase Path, CurveAdaptorBase Curve1, CurveAdaptorBase Curve2,
                                          double Radius)
    {
        Init(Path, Curve1, Curve2, Radius);
    }

    public void Init(Curve3D Path, double Radius)
    {
        myRadius = Radius;
        CurveAdaptor ThePath = new CurveAdaptor(Path);

        if (ThePath.GetType() == Circle)
        {
            myCircPathAxis = ThePath.Circle().Axis();
            myType = 4;
        }
        else
        {
            myType = 1;
        }
        if (Path is BSplineCurve3D)
        {
            myPath = (BSplineCurve3D)Path.Copy();
        }
        else
        {
            myPath = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(Path);
        }
    }

    public void Init(Curve3D Path, Curve3D FirstSect)
    {
        myRadius = 0;
        CurveAdaptor ThePath = new CurveAdaptor(Path);

        if (ThePath.GetType() == Circle)
        {
            myCircPathAxis = ThePath.Circle().Axis();
            myType = 5;
        }

        else
        {
            myType = 2;
        }

        if (Path is BSplineCurve3D)
        {
            myPath = (BSplineCurve3D)Path.Copy();
        }
        else
        {
            myPath = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(Path);
        }
        if (FirstSect is BSplineCurve3D)
        {
            myFirstSect = (BSplineCurve3D)FirstSect.Copy();
        }
        else
        {
            // JAG
            myFirstSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(FirstSect, QuasiAngular);
        }
        if (myFirstSect.IsPeriodic)
        {
            myFirstSect.SetNotPeriodic();
        }
    }

    public void Init(Curve3D Path, Curve3D FirstSect, Curve3D LastSect)
    {
        myRadius = 0;
        CurveAdaptor ThePath = new CurveAdaptor(Path);

        if (ThePath.GetType() == Circle)
        {
            myCircPathAxis = ThePath.Circle().Axis();
            myType = 6;
        }
        else
        {
            myType = 3;
        }

        if (Path is BSplineCurve3D)
        {
            myPath = (BSplineCurve3D)Path.Copy();
        }
        else
        {
            myPath = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(Path);
        }

        // JAG
        if (FirstSect is BSplineCurve3D)
        {
            myFirstSect = (BSplineCurve3D)FirstSect.Copy();
        }
        else
        {
            myFirstSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(FirstSect, QuasiAngular);
        }
        if (LastSect is BSplineCurve3D)
        {
            myLastSect = (BSplineCurve3D)LastSect.Copy();
        }
        else
        {
            myLastSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(LastSect, QuasiAngular);
        }

        if (myFirstSect.IsPeriodic)
        {
            myFirstSect.SetNotPeriodic();
        }
        if (myLastSect.IsPeriodic)
        {
            myLastSect.SetNotPeriodic();
        }

        // JAG

        GeomFill_Profiler Profil = new GeomFill_Profiler();
        Profil.AddCurve(myFirstSect);
        Profil.AddCurve(myLastSect);
        Profil.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());

        myFirstSect = Profil.InternalCurve(1) as BSplineCurve3D;
        myLastSect = Profil.InternalCurve(2) as BSplineCurve3D;
    }

    public void Init(CurveAdaptorBase Path, CurveAdaptorBase Curve1, CurveAdaptorBase Curve2, double Radius)
    {
        myRadius = Radius;
        myType = 0;

        Curve3D CC = global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeCurve(Path)!;
        myPath = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(CC);
        myAdpPath = Path;
        myAdpFirstSect = Curve1;
        myAdpLastSect = Curve2;
    }

    /// <summary>Computes the number of sections and their transformations.</summary>
    public void Perform(bool Polynomial = false)
    {
        myPolynomial = Polynomial;

        // eval myNbSections.
        int NSpans = myPath!.KnotCount - 1;

        myNbSections = 21 * NSpans;

        double U;

        double U1 = myPath.FirstParameter();
        double U2 = myPath.LastParameter();

        QuasiUniformDeflection Samp = new QuasiUniformDeflection();
        // Calcul de la longueur approximative de la courbe
        CurveAdaptor AdpPath = new CurveAdaptor(myPath);
        Point3D P1 = AdpPath.Value(U1);
        Point3D P2 = AdpPath.Value((U1 + U2) / 2.0);
        Point3D P3 = AdpPath.Value(U2);
        double Length = P1.Distance(P2) + P2.Distance(P3);
        double Fleche = 1.0e-5 * Length;
        Samp.Initialize(AdpPath, Fleche);

        if (Samp.IsDone && (Samp.PointCount > myNbSections))
        {
            myNbSections = Samp.PointCount;
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

        Vector3D D1Ref, D1;
        Point3D PRef, P;
        Transform3D TR = new Transform3D(), cumulTR = new Transform3D(), Trans = new Transform3D();

        myPath.D1(U1, out PRef, out D1Ref);

        if ((myType == 1) || (myType == 4))
        {
            // We create a circle with radius <myRadius>. This axis is create with
            // main direction <DRef> (first derivate vector of <myPath> on the first
            // point <PRef> ). This circle is, after transform to BSpline curve,
            // put in <myFirstSect>.

            Frame3D CircleAxis = new Frame3D(PRef, new Direction3D(D1Ref));
            TrimmedCurve3D Circ = new TrimmedCurve3D(new CircleCurve3D(CircleAxis, myRadius), 0.0, 2.0 * M_PI);

            myFirstSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(Circ, QuasiAngular);
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

                Transform3D Rot = new Transform3D();
                if (!D1Ref.IsParallel(D1, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // Eval the Rotation between (i-1) section and the i-th.
                    Rot.SetRotation(new Axis3D(P, new Direction3D(D1Ref ^ D1)), D1Ref.AngleWithRef(D1, D1Ref ^ D1));
                }
                else if (D1Ref.IsOpposite(D1, global::Kerf.Engine.Foundation.Precision.Angular()))
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
            NbPoles = myFirstSect!.PoleCount;
            NbKnots = myFirstSect.KnotCount;
            Degree = myFirstSect.Degree();
        }
        else
        { // myType == 0
            // OCCT: the quasi-angular shape even when Perform(Polynomial) asked for the polynomial conversion, whose
            // 8 poles Section then assigns (occt-issues.md).
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
                        NCollection_Array1<Point3D> Poles,
                        NCollection_Array1<Vector3D> DPoles,
                        NCollection_Array1<Point2D> Poles2d,
                        NCollection_Array1<Vector2D> DPoles2d,
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

        Vector3D D1, D2;
        Point3D Pt;

        myPath.D2(U, out Pt, out D1, out D2);
        double l = D1.Magnitude();

        if (l < Epsilon(1.0))
        {
            return false;
        }

        Direction3D T = new Direction3D(D1);
        double m = D2.Dot(new Vector3D(T));
        Vector3D D = D2 - m * new Vector3D(T);
        double c = D.Magnitude() / (l * l);

        if (c < Epsilon(1.0))
        {
            // null curvature : equivalent to a translation of the section
            for (int i = 1; i <= myFirstSect!.PoleCount; i++)
            {
                DPoles[i] = D1;
            }
        }
        else
        {
            Direction3D N = new Direction3D(D);
            Point3D Q = Pt.Translated((1.0 / c) * new Vector3D(N));
            double x, y;
            Vector3D V;
            for (int i = 1; i <= myFirstSect!.PoleCount; i++)
            {
                V = new Vector3D(Q, Poles[i]);
                x = V * new Vector3D(T);
                y = V * new Vector3D(N);
                DPoles[i] = x * new Vector3D(N) - y * new Vector3D(T);
                if (DPoles[i].Magnitude() > Epsilon(1.0))
                {
                    DPoles[i].Normalize();
                    DPoles[i] *= Math.Sqrt(x * x + y * y);
                }
            }
        }

        for (int i = 1; i <= myFirstSect.PoleCount; i++)
        {
            DWeigths[i] = 0.0;
        }

        return true;
    }

    /// <summary>The section P.</summary>
    public void Section(int P,
                        NCollection_Array1<Point3D> Poles,
                        NCollection_Array1<Point2D> Poles2d,
                        NCollection_Array1<double> Weigths)
    {
        if (myType != 0)
        {
            Poles.Assign(myFirstSect!.InternalPoles());
            Weigths.Assign(myFirstSect.InternalWeightsArray());
            Transform3D cumulTR;
            if (P > 1)
            {
                cumulTR = myTrsfs[P - 1];
                // <cumulTR> transform <myFirstSect> to the P ieme Section. In fact
                // each points of the array <poles> will be transformed.

                if ((myType == 3) || (myType == 6))
                {
                    for (int i = 1; i <= myFirstSect.PoleCount; i++)
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

            Point3D PPath = myAdpPath.Value(U);

            double Alpha = U - myAdpPath.FirstParameter();
            Alpha /= myAdpPath.LastParameter() - myAdpPath.FirstParameter();

            double U1 = (1 - Alpha) * myAdpFirstSect!.FirstParameter() + Alpha * myAdpFirstSect.LastParameter();

            if (myAdpFirstSect.GetType() == Line)
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myAdpFirstSect.FirstParameter())
                    || global::Kerf.Engine.Foundation.Precision.IsInfinite(myAdpFirstSect.LastParameter()))
                {
                    Line3D aLine = myAdpFirstSect.Line();
                    U1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLine, PPath);
                }
            }
            Point3D P1 = myAdpFirstSect.Value(U1);

            double U2 = (1 - Alpha) * myAdpLastSect!.FirstParameter() + Alpha * myAdpLastSect.LastParameter();

            if (myAdpLastSect.GetType() == Line)
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myAdpLastSect.FirstParameter())
                    || global::Kerf.Engine.Foundation.Precision.IsInfinite(myAdpLastSect.LastParameter()))
                {
                    Line3D aLine = myAdpLastSect.Line();
                    U2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLine, PPath);
                }
            }
            Point3D P2 = myAdpLastSect.Value(U2);

            Frame3D Axis = new Frame3D();
            double Angle;
            if (P1.Distance(P2) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                Angle = 0.0;
            }
            else
            {
                Axis = new Frame3D(PPath, new Direction3D(new Vector3D(PPath, P1) ^ new Vector3D(PPath, P2)),
                                  new Direction3D(new Vector3D(PPath, P1)));
                Angle = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(Axis, P2);
            }
            if (Angle < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                for (int i = 1; i <= Poles.Upper(); i++)
                {
                    Poles[i] = P1;
                    Weigths[i] = 1;
                }
            }
            else
            {
                CircleCurve3D Circ = new CircleCurve3D(Axis, myRadius);
                TrimmedCurve3D CT = new TrimmedCurve3D(Circ, 0.0, Angle);
                BSplineCurve3D BS;
                if (myPolynomial)
                {
                    BS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(CT, Polynomial);
                }
                else
                {
                    BS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(CT, QuasiAngular);
                }

                Poles.Assign(BS.InternalPoles());
                Weigths.Assign(BS.InternalWeightsArray());
            }
        }
    }

    /// <summary>The transformation from the first section to the section Index + 1.</summary>
    public Transform3D Transformation(int Index)
    {
        if (Index > myTrsfs.Length())
        {
            throw new RangeErrorException("GeomFill_SweepSectionGenerator::Transformation");
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
