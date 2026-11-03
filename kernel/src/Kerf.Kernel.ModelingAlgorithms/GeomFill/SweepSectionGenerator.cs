// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SweepSectionGenerator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The sections of the old pipe algorithm, for <see cref="AppSweep"/>: a circle of constant radius, a
/// constant section, or one evolving between two sections, moved along a path; or, with three adaptors, the arcs of
/// a pipe on an edge (the case <see cref="SurfacePipe"/> uses).
/// </summary>
/// <remarks>
/// A working object. It works on copies of the caller's curves (or on new B-splines converted from them), and keeps
/// the caller's adaptors, as OCCT keeps the handles.
/// </remarks>
public sealed class SweepSectionGenerator : ISectionGenerator<int>
{
    private BSplineCurve3D? _path;
    private BSplineCurve3D? _firstSect;
    private BSplineCurve3D? _lastSect;
    private CurveAdaptorBase? _adpPath;
    private CurveAdaptorBase? _adpFirstSect;
    private CurveAdaptorBase? _adpLastSect;
    private Axis3D _circPathAxis = new Axis3D();
    private double _radius;
    // OCCT's myIsDone is written by Init and Perform but never read; it is left out.
    private int _nbSections;
    private readonly NCollection_Sequence<Transform3D> _trsfs = new();
    private int _type;
    private bool _polynomial;

    public SweepSectionGenerator()
    {
        _radius = 0.0;
        _nbSections = 0;
        _type = -1;
        _polynomial = false;
    }

    /// <summary>A circle of radius Radius along Path.</summary>
    public SweepSectionGenerator(Curve3D path, double radius)
    {
        Init(path, radius);
    }

    /// <summary>The section FirstSect along Path.</summary>
    public SweepSectionGenerator(Curve3D path, Curve3D firstSect)
    {
        Init(path, firstSect);
    }

    /// <summary>A section evolving from FirstSect to LastSect along Path.</summary>
    public SweepSectionGenerator(Curve3D path, Curve3D firstSect, Curve3D lastSect)
    {
        Init(path, firstSect, lastSect);
    }

    /// <summary>The arcs of radius Radius from Curve1 to Curve2 around Path (a pipe on an edge).</summary>
    public SweepSectionGenerator(CurveAdaptorBase path, CurveAdaptorBase curve1, CurveAdaptorBase curve2,
                                          double radius)
    {
        Init(path, curve1, curve2, radius);
    }

    public void Init(Curve3D path, double radius)
    {
        _radius = radius;
        CurveAdaptor thePath = new CurveAdaptor(path);

        if (thePath.GetType() == Circle)
        {
            _circPathAxis = thePath.Circle().Axis();
            _type = 4;
        }
        else
        {
            _type = 1;
        }
        if (path is BSplineCurve3D)
        {
            _path = (BSplineCurve3D)path.Copy();
        }
        else
        {
            _path = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(path);
        }
    }

    public void Init(Curve3D path, Curve3D firstSect)
    {
        _radius = 0;
        CurveAdaptor thePath = new CurveAdaptor(path);

        if (thePath.GetType() == Circle)
        {
            _circPathAxis = thePath.Circle().Axis();
            _type = 5;
        }

        else
        {
            _type = 2;
        }

        if (path is BSplineCurve3D)
        {
            _path = (BSplineCurve3D)path.Copy();
        }
        else
        {
            _path = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(path);
        }
        if (firstSect is BSplineCurve3D)
        {
            _firstSect = (BSplineCurve3D)firstSect.Copy();
        }
        else
        {
            // JAG
            _firstSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(firstSect, QuasiAngular);
        }
        if (_firstSect.IsPeriodic)
        {
            _firstSect.SetNotPeriodic();
        }
    }

    public void Init(Curve3D path, Curve3D firstSect, Curve3D lastSect)
    {
        _radius = 0;
        CurveAdaptor thePath = new CurveAdaptor(path);

        if (thePath.GetType() == Circle)
        {
            _circPathAxis = thePath.Circle().Axis();
            _type = 6;
        }
        else
        {
            _type = 3;
        }

        if (path is BSplineCurve3D)
        {
            _path = (BSplineCurve3D)path.Copy();
        }
        else
        {
            _path = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(path);
        }

        // JAG
        if (firstSect is BSplineCurve3D)
        {
            _firstSect = (BSplineCurve3D)firstSect.Copy();
        }
        else
        {
            _firstSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(firstSect, QuasiAngular);
        }
        if (lastSect is BSplineCurve3D)
        {
            _lastSect = (BSplineCurve3D)lastSect.Copy();
        }
        else
        {
            _lastSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(lastSect, QuasiAngular);
        }

        if (_firstSect.IsPeriodic)
        {
            _firstSect.SetNotPeriodic();
        }
        if (_lastSect.IsPeriodic)
        {
            _lastSect.SetNotPeriodic();
        }

        // JAG

        Profiler profil = new Profiler();
        profil.AddCurve(_firstSect);
        profil.AddCurve(_lastSect);
        profil.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());

        _firstSect = profil.InternalCurve(1) as BSplineCurve3D;
        _lastSect = profil.InternalCurve(2) as BSplineCurve3D;
    }

    public void Init(CurveAdaptorBase path, CurveAdaptorBase curve1, CurveAdaptorBase curve2, double radius)
    {
        _radius = radius;
        _type = 0;

        Curve3D cc = global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeCurve(path)!;
        _path = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(cc);
        _adpPath = path;
        _adpFirstSect = curve1;
        _adpLastSect = curve2;
    }

    /// <summary>Computes the number of sections and their transformations.</summary>
    public void Perform(bool polynomial = false)
    {
        _polynomial = polynomial;

        // eval myNbSections.
        int nSpans = _path!.KnotCount - 1;

        _nbSections = 21 * nSpans;

        double u;

        double u1 = _path.FirstParameter();
        double u2 = _path.LastParameter();

        QuasiUniformDeflection samp = new QuasiUniformDeflection();
        // Calcul de la longueur approximative de la courbe
        CurveAdaptor adpPath = new CurveAdaptor(_path);
        Point3D p1 = adpPath.Value(u1);
        Point3D p2 = adpPath.Value((u1 + u2) / 2.0);
        Point3D p3 = adpPath.Value(u2);
        double length = p1.Distance(p2) + p2.Distance(p3);
        double fleche = 1.0e-5 * length;
        samp.Initialize(adpPath, fleche);

        if (samp.IsDone && (samp.PointCount > _nbSections))
        {
            _nbSections = samp.PointCount;
        }
        // the transformations are calculate on differents points of <myPath>
        // corresponding to the path parameter uniformly reparted.
        double deltaU = (u2 - u1) / (double)(_nbSections - 1);
        NCollection_Array1<double> parameters = new(1, _nbSections);

        parameters[1] = 0.0;
        for (int i = 2; i < _nbSections; i++)
        {
            parameters[i] = (i - 1) * deltaU;
        }
        parameters[_nbSections] = u2 - u1;

        Vector3D d1Ref, d1;
        Point3D pRef, p;
        Transform3D tr = new Transform3D(), cumulTR = new Transform3D(), trans = new Transform3D();

        _path.D1(u1, out pRef, out d1Ref);

        if ((_type == 1) || (_type == 4))
        {
            // We create a circle with radius <myRadius>. This axis is create with
            // main direction <DRef> (first derivate vector of <myPath> on the first
            // point <PRef> ). This circle is, after transform to BSpline curve,
            // put in <myFirstSect>.

            Frame3D circleAxis = new Frame3D(pRef, new Direction3D(d1Ref));
            TrimmedCurve3D circ = new TrimmedCurve3D(new CircleCurve3D(circleAxis, _radius), 0.0, 2.0 * M_PI);

            _firstSect = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(circ, QuasiAngular);
        }

        if (_type <= 3 && _type >= 1)
        {
            for (int i = 2; i <= _nbSections; i++)
            {
                u = parameters[i] + u1;
                if (i == _nbSections)
                {
                    u = u2;
                }

                _path.D1(u, out p, out d1);

                // Eval the translation between the (i-1) section and the i-th.
                trans = trans.WithTranslation(pRef, p);

                Transform3D rot = new Transform3D();
                if (!d1Ref.IsParallel(d1, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // Eval the Rotation between (i-1) section and the i-th.
                    rot = rot.WithRotation(new Axis3D(p, new Direction3D(d1Ref ^ d1)), d1Ref.AngleWithRef(d1, d1Ref ^ d1));
                }
                else if (d1Ref.IsOpposite(d1, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // OCCT: TR is set only here (the braces put "TR = Rot * Trans" in the opposite-tangents branch), so
                    // the rotations and translations computed above are dropped (occt-issues.md).
                    // TR is the transformation between (i-1) section and the i-th.
                    tr = rot * trans;
                }
                // cumulTR is the transformation between <myFirstSec> and
                // the i-th section.
                cumulTR = tr * cumulTR;

                _trsfs.Append(cumulTR);

                pRef = p;
                d1Ref = d1;
            }
        }
        else if (_type != 0)
        {
            for (int i = 2; i <= _nbSections; i++)
            {
                cumulTR = cumulTR.WithRotation(_circPathAxis, parameters[i]);
                _trsfs.Append(cumulTR);
            }
        }

    }

    /// <summary>The shape of a section: its number of poles and knots, its degree, and the number of 2D poles (0).</summary>
    public void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d)
    {
        if (_type != 0)
        {
            nbPoles = _firstSect!.PoleCount;
            nbKnots = _firstSect.KnotCount;
            degree = _firstSect.Degree();
        }
        else
        { // myType == 0
            // OCCT: the quasi-angular shape even when Perform(Polynomial) asked for the polynomial conversion, whose
            // 8 poles Section then assigns (occt-issues.md).
            nbPoles = 7;
            nbKnots = 2;
            degree = 6;
        }
        nbPoles2d = 0;
    }

    /// <summary>The knots of the sections.</summary>
    public void Knots(NCollection_Array1<double> tKnots)
    {
        if (_type != 0)
        {
            tKnots.Assign(_firstSect!.InternalKnots());
        }
        else
        {
            tKnots[1] = 0.0;
            tKnots[2] = 1.0;
        }
    }

    /// <summary>The multiplicities of the sections.</summary>
    public void Mults(NCollection_Array1<int> tMults)
    {
        if (_type != 0)
        {
            tMults.Assign(_firstSect!.InternalMultiplicities());
        }
        else
        {
            tMults[1] = tMults[2] = 7;
        }
    }

    /// <summary>The number of sections.</summary>
    public int SectionCount => _nbSections;

    /// <summary>
    /// The section P and its derivatives along the path; false where the derivatives aren't computed (between the
    /// ends, and for a pipe on an edge).
    /// </summary>
    public bool Section(int p,
                        NCollection_Array1<Point3D> poles,
                        NCollection_Array1<Vector3D> dPoles,
                        NCollection_Array1<Point2D> poles2d,
                        NCollection_Array1<Vector2D> dPoles2d,
                        NCollection_Array1<double> weigths,
                        NCollection_Array1<double> dWeigths)
    {
        Section(p, poles, poles2d, weigths);

        // pour les tuyaux sur aretes pour l'instant on ne calcule pas les derivees
        if (_type == 0)
        {
            return false; // a voir pour mieux.
        }

        // calcul des derivees sur la surface
        // on calcule les derivees en approximant le path au voisinage du point
        // P(u) par le cercle osculateur au path .

        // calcul du cercle osculateur.

        double u;
        if (p == 1)
        {
            u = _path!.FirstParameter();
        }
        else if (p == _nbSections)
        {
            u = _path!.LastParameter();
        }
        else
        {
            return false;
        }

        Vector3D d1, d2;
        Point3D pt;

        _path.D2(u, out pt, out d1, out d2);
        double l = d1.Magnitude();

        if (l < Epsilon(1.0))
        {
            return false;
        }

        Direction3D t = new Direction3D(d1);
        double m = d2.Dot(new Vector3D(t));
        Vector3D d = d2 - m * new Vector3D(t);
        double c = d.Magnitude() / (l * l);

        if (c < Epsilon(1.0))
        {
            // null curvature : equivalent to a translation of the section
            for (int i = 1; i <= _firstSect!.PoleCount; i++)
            {
                dPoles[i] = d1;
            }
        }
        else
        {
            Direction3D n = new Direction3D(d);
            Point3D q = pt.Translated((1.0 / c) * new Vector3D(n));
            double x, y;
            Vector3D v;
            for (int i = 1; i <= _firstSect!.PoleCount; i++)
            {
                v = new Vector3D(q, poles[i]);
                x = v * new Vector3D(t);
                y = v * new Vector3D(n);
                dPoles[i] = x * new Vector3D(n) - y * new Vector3D(t);
                if (dPoles[i].Magnitude() > Epsilon(1.0))
                {
                    dPoles[i] = dPoles[i].Normalized();
                    dPoles[i] *= Math.Sqrt(x * x + y * y);
                }
            }
        }

        for (int i = 1; i <= _firstSect.PoleCount; i++)
        {
            dWeigths[i] = 0.0;
        }

        return true;
    }

    /// <summary>The section P.</summary>
    public void Section(int p,
                        NCollection_Array1<Point3D> poles,
                        NCollection_Array1<Point2D> poles2d,
                        NCollection_Array1<double> weigths)
    {
        if (_type != 0)
        {
            poles.Assign(_firstSect!.InternalPoles());
            weigths.Assign(_firstSect.InternalWeightsArray());
            Transform3D cumulTR;
            if (p > 1)
            {
                cumulTR = _trsfs[p - 1];
                // <cumulTR> transform <myFirstSect> to the P ieme Section. In fact
                // each points of the array <poles> will be transformed.

                if ((_type == 3) || (_type == 6))
                {
                    for (int i = 1; i <= _firstSect.PoleCount; i++)
                    {
                        poles[i] = poles[i].WithXYZ((_nbSections - p) * _firstSect.Pole(i).XYZ()
                                        + (p - 1) * _lastSect!.Pole(i).XYZ());
                        poles[i] = poles[i].WithXYZ(poles[i].XYZ() / (_nbSections - 1));

                        weigths[i] = (_nbSections - p) * _firstSect.Weight(i) + (p - 1) * _lastSect.Weight(i);
                        weigths[i] /= _nbSections - 1;
                    }
                }

                for (int i = 1; i <= poles.Length(); i++)
                {
                    poles[i] = poles[i].Transformed(cumulTR);
                }
            }
        }
        else
        {
            double coef = (p - 1.0) / (_nbSections - 1.0);
            double u = (1 - coef) * _adpPath!.FirstParameter() + coef * _adpPath.LastParameter();

            Point3D pPath = _adpPath.Value(u);

            double alpha = u - _adpPath.FirstParameter();
            alpha /= _adpPath.LastParameter() - _adpPath.FirstParameter();

            double u1 = (1 - alpha) * _adpFirstSect!.FirstParameter() + alpha * _adpFirstSect.LastParameter();

            if (_adpFirstSect.GetType() == CurveKind.Line)
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_adpFirstSect.FirstParameter())
                    || global::Kerf.Engine.Foundation.Precision.IsInfinite(_adpFirstSect.LastParameter()))
                {
                    Line3D line = _adpFirstSect.Line();
                    u1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, pPath);
                }
            }
            Point3D p1 = _adpFirstSect.Value(u1);

            double u2 = (1 - alpha) * _adpLastSect!.FirstParameter() + alpha * _adpLastSect.LastParameter();

            if (_adpLastSect.GetType() == CurveKind.Line)
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(_adpLastSect.FirstParameter())
                    || global::Kerf.Engine.Foundation.Precision.IsInfinite(_adpLastSect.LastParameter()))
                {
                    Line3D line = _adpLastSect.Line();
                    u2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, pPath);
                }
            }
            Point3D p2 = _adpLastSect.Value(u2);

            Frame3D axis = new Frame3D();
            double angle;
            if (p1.Distance(p2) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                angle = 0.0;
            }
            else
            {
                axis = new Frame3D(pPath, new Direction3D(new Vector3D(pPath, p1) ^ new Vector3D(pPath, p2)),
                                  new Direction3D(new Vector3D(pPath, p1)));
                angle = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(axis, p2);
            }
            if (angle < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                for (int i = 1; i <= poles.Upper(); i++)
                {
                    poles[i] = p1;
                    weigths[i] = 1;
                }
            }
            else
            {
                CircleCurve3D circ = new CircleCurve3D(axis, _radius);
                TrimmedCurve3D ct = new TrimmedCurve3D(circ, 0.0, angle);
                BSplineCurve3D bs;
                if (_polynomial)
                {
                    bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(ct, Polynomial);
                }
                else
                {
                    bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(ct, QuasiAngular);
                }

                poles.Assign(bs.InternalPoles());
                weigths.Assign(bs.InternalWeightsArray());
            }
        }
    }

    /// <summary>The transformation from the first section to the section Index + 1.</summary>
    public Transform3D Transformation(int index)
    {
        if (index > _trsfs.Length())
        {
            throw new RangeErrorException("GeomFill_SweepSectionGenerator::Transformation");
        }

        return _trsfs[index];
    }

    /// <summary>The path parameter of the section P.</summary>
    public double Parameter(int p)
    {
        if (p == 1)
        {
            return _path!.FirstParameter();
        }
        else if (p == _nbSections)
        {
            return _path!.LastParameter();
        }
        else
        {
            double u1 = _path!.FirstParameter();
            double u2 = _path.LastParameter();
            double prm = ((_nbSections - p) * u1 + (p - 1) * u2) / (double)(_nbSections - 1);
            return prm;
        }
    }
}
