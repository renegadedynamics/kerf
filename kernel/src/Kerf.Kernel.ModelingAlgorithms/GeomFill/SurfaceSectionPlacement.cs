// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionPlacement.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SectionPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// Places a section (a curve or a point) on a sweep's path: finds the parameter on the path and on the section
/// where they meet best, and the transformation that takes the section into the location law's frame there.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's location law and section as OCCT keeps the handles and
/// only evaluates them (it adapts the section with an adaptor of its own). <see cref="Section"/> and
/// <see cref="ModifiedSection"/> return new, unfrozen curves.
/// </remarks>
public sealed class SurfaceSectionPlacement
{
    private bool _done;
    private bool _isplan;
    private Axis3D _theAxe;
    private readonly double _gabarit;
    private SurfaceLocationLaw _law;
    private readonly CurveAdaptor _adpSection;
    private readonly Curve3D? _section;
    private double _secParam;
    private double _pathParam;
    private double _dist;
    private double _angleMax;
    private readonly CurvePointExtrema _ext;
    private readonly bool _isPoint;
    private readonly Point3D _point;

    private static void Tangente(CurveAdaptorBase path, double param, out Point3D p, out Vector3D tang)
    {
        path.D1(param, out p, out tang);
        double norm = tang.Magnitude();

        for (int ii = 2; (ii < 12) && (norm < global::Kerf.Engine.Foundation.Precision.Confusion()); ii++)
        {
            tang = path.DN(param, ii);
            norm = tang.Magnitude();
        }

        if (norm > 100 * Constants.Resolution())
        {
            tang /= norm;
        }
    }

    private static double Penalite(double angle, double dist)
    {
        double penal;

        if (dist < 1)
        {
            penal = Math.Sqrt(dist);
        }
        else if (dist < 2)
        {
            penal = Math.Pow(dist, 2);
        }
        else
        {
            penal = dist + 2;
        }

        if (angle > 1.0e-3)
        {
            penal += 1.0 / angle - 2.0 / M_PI;
        }
        else
        {
            penal += 1.0e3;
        }

        return penal;
    }

    private static double EvalAngle(in Vector3D v1, in Vector3D v2)
    {
        double angle;
        angle = v1.Angle(v2);
        if (angle > M_PI / 2)
        {
            angle = M_PI - angle;
        }
        return angle;
    }

    // Examines an extremum to update Dist and Param.
    private static void DistMini(CurvePointExtrema ext, CurveAdaptorBase c, ref double dist, ref double param)
    {
        double dist1, dist2;
        int ii;
        double Dist2 = RealLast();

        ext.TrimmedSquareDistances(out dist1, out dist2, out _, out _);
        if ((dist1 < Dist2) || (dist2 < Dist2))
        {
            if (dist1 < dist2)
            {
                Dist2 = dist1;
                param = c.FirstParameter();
            }
            else
            {
                Dist2 = dist2;
                param = c.LastParameter();
            }
        }

        if (ext.IsDone)
        {
            for (ii = 1; ii <= ext.ExtCount; ii++)
            {
                if (ext.SquareDistance(ii) < Dist2)
                {
                    Dist2 = ext.SquareDistance(ii);
                    param = ext.Point(ii).Parameter();
                }
            }
        }
        dist = Math.Sqrt(Dist2);
    }

    /// <summary>Prepares the placement of Section (a Geom_Curve or a Geom_CartesianPoint) along the location law L.</summary>
    public SurfaceSectionPlacement(SurfaceLocationLaw l, Geometry3D section)
    {
        _law = l;
        _secParam = 0.0;
        _pathParam = 0.0;
        _dist = RealLast();
        _angleMax = 0.0;
        _adpSection = new CurveAdaptor();
        _ext = new CurvePointExtrema();
        _theAxe = new Axis3D();

        _done = false;
        _isplan = false;
        _isPoint = false;

        if (section is CartesianPoint point && section.GetType() == typeof(CartesianPoint))
        {
            _isPoint = true;
            _point = point.Pnt();
            _isplan = true;
        }
        else
        {
            Curve3D curveSection = (Curve3D)section;
            _adpSection.Load(curveSection);
            _section = curveSection;
        }

        int i, j, nbPoles = 0;
        double xmin, ymin, zmin, xmax, ymax, zmax;

        // Boite d'encombrement de la section pour en deduire le gabarit
        Box box = new Box();
        if (_isPoint)
        {
            box.Add(_point);
        }
        else
        {
            Add3dCurve.Add(_adpSection, 1.0e-4, ref box);
        }
        box.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

        double dx = xmax - xmin;
        double dy = ymax - ymin;
        double dz = zmax - zmin;
        _gabarit = Math.Sqrt(dx * dx + dy * dy + dz * dz) / 2.0;

        _gabarit += global::Kerf.Engine.Foundation.Precision.Confusion(); // Cas des toute petite

        // Initialisation de TheAxe pour les cas singulier
        if (!_isPoint)
        {
            Tangente(_adpSection, (_adpSection.FirstParameter() + _adpSection.LastParameter()) / 2, out Point3D p,
                     out Vector3D v);
            _theAxe = _theAxe.WithLocation(p);
            _theAxe = _theAxe.WithDirection(new Direction3D(v));

            // y a t'il un Plan moyen ?
            CurveKind theType = _adpSection.GetType();
            switch (theType)
            {
                case Circle:
                    _isplan = true;
                    _theAxe = _adpSection.Circle().Axis();
                    break;
                case Ellipse:
                    _isplan = true;
                    _theAxe = _adpSection.Ellipse().Axis();
                    break;
                case Hyperbola:
                    _isplan = true;
                    _theAxe = _adpSection.Hyperbola().Axis();
                    break;
                case Parabola:
                    _isplan = true;
                    _theAxe = _adpSection.Parabola().Axis();
                    break;
                case Line:
                    nbPoles = 0; // Pas de Plan !!
                    break;
                case BezierCurve:
                case BSplineCurve:
                    nbPoles = _adpSection.PoleCount;
                    break;
                default:
                    nbPoles = 21;
                    break;
            }

            if (!_isplan && nbPoles > 2)
            {
                // Calcul d'un plan moyen.
                NCollection_HArray1<Point3D> pnts;
                double first = _adpSection.FirstParameter();
                double last = _adpSection.LastParameter();
                if (_adpSection.IsPeriodic)
                {
                    // Correct boundaries to avoid mistake of LocateU
                    Curve3D curve = _adpSection.Curve()!;
                    if (curve.GetType() == typeof(TrimmedCurve3D))
                    {
                        curve = ((TrimmedCurve3D)curve).BasisCurve();
                    }
                    double ufirst = curve.FirstParameter();
                    double period = curve.Period();
                    double u1 = ufirst + Math.Floor((first - ufirst) / period) * period;
                    double u2 = u1 + period;
                    if (Math.Abs(first - u1) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        first = u1;
                    }
                    if (Math.Abs(last - u2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        last = u2;
                    }
                }
                double t, delta;
                if (_adpSection.GetType() == BSplineCurve)
                {
                    BSplineCurve3D bc = (BSplineCurve3D)_adpSection.Curve()!;
                    int i1, i2, i3, i4;
                    bc.LocateU(first, global::Kerf.Engine.Foundation.Precision.Confusion(), out i1, out i2);
                    bc.LocateU(last, global::Kerf.Engine.Foundation.Precision.Confusion(), out i3, out i4);
                    int nbKnots = i3 - i2 + 1;

                    int nbLocalPnts = 10;
                    int nbPnts = (nbKnots - 1) * nbLocalPnts;
                    if (nbPnts < 0)
                    {
                        nbPnts = 0;
                    }
                    if (i1 != i2)
                    {
                        nbPnts += nbLocalPnts;
                    }
                    if (i3 != i4 && first < bc.Knot(i3))
                    {
                        nbPnts += nbLocalPnts;
                    }
                    if (!_adpSection.IsClosed)
                    {
                        nbPnts++;
                    }
                    pnts = new NCollection_HArray1<Point3D>(1, nbPnts);
                    int nb = 1;
                    if (i1 != i2)
                    {
                        double locallast = (bc.Knot(i2) < last) ? bc.Knot(i2) : last;
                        delta = (locallast - first) / nbLocalPnts;
                        for (j = 0; j < nbLocalPnts; j++)
                        {
                            t = first + j * delta;
                            pnts.SetValue(nb++, _adpSection.Value(t));
                        }
                    }
                    for (i = i2; i < i3; i++)
                    {
                        t = bc.Knot(i);
                        delta = (bc.Knot(i + 1) - t) / nbLocalPnts;
                        for (j = 0; j < nbLocalPnts; j++)
                        {
                            pnts.SetValue(nb++, _adpSection.Value(t));
                            t += delta;
                        }
                    }
                    if (i3 != i4 && first < bc.Knot(i3))
                    {
                        t = bc.Knot(i3);
                        delta = (last - t) / nbLocalPnts;
                        for (j = 0; j < nbLocalPnts; j++)
                        {
                            pnts.SetValue(nb++, _adpSection.Value(t));
                            t += delta;
                        }
                    }
                    if (!_adpSection.IsClosed)
                    {
                        pnts.SetValue(nb, _adpSection.Value(last));
                    }
                }
                else // other type
                {
                    int nbPnts = nbPoles - 1;
                    if (!_adpSection.IsClosed)
                    {
                        nbPnts++;
                    }
                    pnts = new NCollection_HArray1<Point3D>(1, nbPnts);
                    delta = (last - first) / (nbPoles - 1);
                    for (i = 0; i < nbPoles - 1; i++)
                    {
                        t = first + i * delta;
                        pnts.SetValue(i + 1, _adpSection.Value(t));
                    }
                    if (!_adpSection.IsClosed)
                    {
                        pnts.SetValue(nbPnts, _adpSection.Value(last));
                    }
                }

                global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(pnts, out Frame3D axe, out bool issing, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (!issing)
                {
                    _isplan = true;
                    _theAxe = _theAxe.WithLocation(axe.Location());
                    _theAxe = _theAxe.WithDirection(axe.Direction());
                }
            }

            _ext.Initialize(_adpSection, _adpSection.FirstParameter(), _adpSection.LastParameter(),
                             global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    /// <summary>Changes the location law.</summary>
    public void SetLocation(SurfaceLocationLaw l)
    {
        _law = l;
    }

    /// <summary>Computes the placement on the curve of the location law.</summary>
    public void Perform(double tol)
    {
        CurveAdaptorBase path;
        path = _law.GetCurve()!;
        Perform(path, tol);
    }

    /// <summary>Computes the placement on Path.</summary>
    public void Perform(CurveAdaptorBase path, double tol)
    {
        double intTol = 1.0e-5;
        double distCenter = global::Kerf.Engine.Foundation.Precision.Infinite();

        if (_isPoint)
        {
            CurvePointExtrema projector = new CurvePointExtrema(_point, path, global::Kerf.Engine.Foundation.Precision.Confusion());
            DistMini(projector, path, ref _dist, ref _pathParam);
            _angleMax = M_PI / 2;
        }
        else
        {
            _pathParam = path.FirstParameter();
            _secParam = _adpSection.FirstParameter();

            double distaux = 0.0, taux = 0.0, alpha;
            Point3D ponPath, ponSec, p;
            Vector3D vRef = new Vector3D(), dp1;
            vRef = vRef.WithXYZ(_theAxe.Direction().XYZ());

            Tangente(path, _pathParam, out ponPath, out dp1);
            ponSec = _adpSection.Value(_secParam);
            _dist = ponPath.Distance(ponSec);
            if (_dist > tol)
            { // On Cherche un meilleur point sur la section
                _ext.Perform(ponPath);
                if (_ext.IsDone)
                {
                    DistMini(_ext, _adpSection, ref _dist, ref _secParam);
                    ponSec = _adpSection.Value(_secParam);
                }
            }
            _angleMax = EvalAngle(vRef, dp1);
            if (_isplan)
            {
                _angleMax = M_PI / 2 - _angleMax;
            }

            bool trouve = false;
            int ii;

            if (_isplan)
            {
                // (1.1) Distances Point-Plan
                double distPlan;
                Vector3D v1 = new Vector3D(ponPath, _theAxe.Location());
                distPlan = Math.Abs(v1.Dot(vRef));
                if (distPlan <= intTol)
                {
                    distCenter = v1.Magnitude();
                }

                Point3D plast = path.Value(path.LastParameter());
                v1 = v1.WithXYZ(_theAxe.Location().XYZ() - plast.XYZ());
                distPlan = Math.Abs(v1.Dot(vRef));
                if (distPlan <= intTol)
                {
                    double dist = v1.Magnitude();
                    if (dist < distCenter)
                    {
                        distCenter = dist;
                        ponPath = plast;
                        _pathParam = path.LastParameter();
                    }
                }

                // (1.2) Intersection Plan-courbe
                CoordinateSystem axe = new CoordinateSystem(_theAxe.Location(), _theAxe.Direction());
                PlaneSurface plan = new PlaneSurface(axe);
                SurfaceAdaptor adplan = new SurfaceAdaptor(plan);
                HInter intersector = new HInter();
                intersector.Perform(path, adplan);
                if (intersector.IsDone)
                {
                    double w;
                    double dist;
                    for (ii = 1; ii <= intersector.PointCount; ii++)
                    {
                        w = intersector.Point(ii).W();
                        p = path.Value(w);
                        dist = p.Distance(_theAxe.Location());
                        if (dist < distCenter)
                        {
                            distCenter = dist;
                            ponPath = p;
                            _pathParam = w;
                        }
                    }
                }
                if (!intersector.IsDone || intersector.PointCount == 0)
                {
                    // Comparing the distances from the path's endpoints to the best matching plane of the
                    // profile.
                    Point3D firstPoint = path.Value(path.FirstParameter());
                    Point3D lastPoint = path.Value(path.LastParameter());
                    Plane3D plane = plan.Pln();
                    double firstDistance = plane.SquareDistance(firstPoint);
                    double lastDistance = plane.SquareDistance(lastPoint);

                    if (((Math.Abs(firstDistance) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                         && Math.Abs(lastDistance) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                        || firstDistance < lastDistance)
                    {
                        _pathParam = path.FirstParameter();
                    }
                    else
                    {
                        _pathParam = path.LastParameter();
                        Tangente(path, _pathParam, out ponPath, out dp1);
                        ponSec = _adpSection.Value(_secParam);
                        _dist = ponPath.Distance(ponSec);
                        if (_dist > tol)
                        { // On Cherche un meilleur point sur la section
                            _ext.Perform(ponPath);
                            if (_ext.IsDone)
                            {
                                DistMini(_ext, _adpSection, ref _dist, ref _secParam);
                                ponSec = _adpSection.Value(_secParam);
                            }
                        }
                        _angleMax = EvalAngle(vRef, dp1);
                        _angleMax = M_PI / 2 - _angleMax;
                    }
                }
                // OCCT keeps an older version of this block commented out; it is not ported.
            }

            // Cas General
            if (!_isplan)
            {
                // (2.1) Distance avec les extremites ...
                _ext.Perform(ponPath);
                if (_ext.IsDone)
                {
                    DistMini(_ext, _adpSection, ref distaux, ref taux);
                    if (distaux < _dist)
                    {
                        _dist = distaux;
                        _secParam = taux;
                    }
                }
                trouve = (_dist <= tol);
                if (!trouve)
                {
                    Tangente(path, path.LastParameter(), out p, out dp1);
                    alpha = EvalAngle(vRef, dp1);
                    _ext.Perform(p);
                    if (_ext.IsDone)
                    {
                        if (_ext.IsDone)
                        {
                            DistMini(_ext, _adpSection, ref distaux, ref taux);
                            if (Choix(distaux, alpha))
                            {
                                _dist = distaux;
                                _secParam = taux;
                                _angleMax = alpha;
                                ponPath = p;
                                _pathParam = path.LastParameter();
                            }
                        }
                    }
                    trouve = (_dist <= tol);
                }

                // (2.2) Distance courbe-courbe
                if (!trouve)
                {
                    ExtCC ext = new ExtCC(path, _adpSection, path.FirstParameter(), path.LastParameter(),
                                                          _adpSection.FirstParameter(), _adpSection.LastParameter(),
                                                          path.Resolution(tol / 100), _adpSection.Resolution(tol / 100));
                    if (ext.IsDone && !ext.IsParallel)
                    {
                        for (ii = 1; ii <= ext.ExtCount; ii++)
                        {
                            distaux = Math.Sqrt(ext.SquareDistance(ii));
                            ext.Points(ii, out POnCurv p1, out POnCurv p2);
                            Tangente(path, p1.Parameter(), out p, out dp1);
                            alpha = EvalAngle(vRef, dp1);
                            if (Choix(distaux, alpha))
                            {
                                trouve = true;
                                _dist = distaux;
                                _pathParam = p1.Parameter();
                                _secParam = p2.Parameter();
                                ponSec = p2.Value();
                                ponPath = p;
                                _angleMax = alpha;
                            }
                        }
                    }
                    if (!trouve)
                    {
                        // Si l'on a toujours rien, on essai une distance point/path
                        // c'est la derniere chance.
                        CurvePointExtrema pExt = new CurvePointExtrema();
                        pExt.Initialize(path, path.FirstParameter(), path.LastParameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
                        pExt.Perform(ponSec);
                        if (pExt.IsDone)
                        {
                            // modified for OCC13595  Tue Oct 17 15:00:08 2006.BEGIN
                            DistMini(pExt, path, ref distaux, ref taux);
                            // modified for OCC13595  Tue Oct 17 15:00:11 2006.END
                            Tangente(path, taux, out p, out dp1);
                            alpha = EvalAngle(vRef, dp1);
                            if (Choix(distaux, alpha))
                            {
                                _dist = distaux;
                                ponPath = p;
                                _angleMax = alpha;
                                _pathParam = taux;
                            }
                        }
                    }
                }
            }
        }

        _done = true;
    }

    /// <summary>Computes the placement for the given parameter on the path.</summary>
    public void Perform(double param, double tol)
    {
        _done = true;
        CurveAdaptorBase path;
        path = _law.GetCurve()!;

        _pathParam = param;
        if (_isPoint)
        {
            Point3D ponPath = path.Value(_pathParam);
            _dist = ponPath.Distance(_point);
            _angleMax = M_PI / 2;
        }
        else
        {
            _secParam = _adpSection.FirstParameter();

            Point3D ponPath, ponSec;
            Vector3D vRef = new Vector3D(), dp1;
            vRef = vRef.WithXYZ(_theAxe.Direction().XYZ());

            Tangente(path, _pathParam, out ponPath, out dp1);
            ponSec = _adpSection.Value(_secParam);
            _dist = ponPath.Distance(ponSec);
            if (_dist > tol)
            { // On Cherche un meilleur point sur la section
                _ext.Perform(ponPath);
                if (_ext.IsDone)
                {
                    DistMini(_ext, _adpSection, ref _dist, ref _secParam);
                    ponSec = _adpSection.Value(_secParam);
                }
            }
            _angleMax = EvalAngle(vRef, dp1);
            if (_isplan)
            {
                _angleMax = M_PI / 2 - _angleMax;
            }
        }

        _done = true;
    }

    public bool IsDone => _done;

    public double ParameterOnPath() => _pathParam;

    public double ParameterOnSection() => _secParam;

    public double Distance() => _dist;

    public double Angle() => _angleMax;

    /// <summary>
    /// The transformation from the section's coordinates to the location law's frame at the path parameter.
    /// WithTranslation forces the contact of the section and the path; WithCorrection rotates a planar section so
    /// that its normal is the law's tangent.
    /// </summary>
    public Transform3D Transformation(bool withTranslation, bool withCorrection = false)
    {
        Vector3D v = new Vector3D();
        Matrix3x3 m = new Matrix3x3();
        Direction3D dn = new Direction3D(), d = new Direction3D();
        // modified by NIZHNY-MKK  Fri Oct 17 15:27:07 2003
        Point3D p = new Point3D(0.0, 0.0, 0.0), pSection = new Point3D(0.0, 0.0, 0.0);

        // Calcul des reperes
        _law.D0(_pathParam, ref m, ref v);

        p = p.WithXYZ(v.XYZ());
        d = d.WithXYZ(m.Column(3));
        dn = dn.WithXYZ(m.Column(1));
        CoordinateSystem paxe = new CoordinateSystem(p, d, dn);

        if (withTranslation || withCorrection)
        {
            if (_isPoint)
            {
                pSection = _point;
            }
            else
            {
                pSection = _section!.Value(_secParam);
            }
        }
        // modified by NIZHNY-MKK  Wed Oct  8 15:03:19 2003.BEGIN
        Transform3D rot = new Transform3D();

        if (withCorrection && !_isPoint)
        {
            double angle;

            if (!_isplan)
            {
                throw new FailureException("Illegal usage: can't rotate non-planar profile");
            }

            Direction3D profileNormal = _theAxe.Direction();
            Direction3D spineStartDir = paxe.Direction();
            if (!profileNormal.IsParallel(spineStartDir, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                Direction3D dirAxeOfRotation = profileNormal ^ spineStartDir;
                angle = profileNormal.AngleWithRef(spineStartDir, dirAxeOfRotation);
                Axis3D axeOfRotation = new Axis3D(_theAxe.Location(), dirAxeOfRotation);
                rot = rot.WithRotation(axeOfRotation, angle);
            }
            pSection = pSection.Transformed(rot);
        }
        // modified by NIZHNY-MKK  Wed Oct  8 15:03:21 2003.END

        if (withTranslation)
        {
            Xyz coord = default;
            coord.SetLinearForm(-1, pSection.XYZ(), v.XYZ());
            p = new Point3D(coord);
        }
        else
        {
            p = p.WithCoord(0.0, 0.0, 0.0);
        }

        CoordinateSystem saxe = new CoordinateSystem(p, Constants.DZ(), Constants.DX());

        // Transfo
        Transform3D tf = new Transform3D().WithTransformation(saxe, paxe);

        if (withCorrection)
        {
            tf *= rot;
        }

        return tf;
    }

    /// <summary>
    /// The section in the frame of the location law (a new curve); WithTranslation forces the contact of the
    /// section and the path.
    /// </summary>
    public Curve3D Section(bool withTranslation)
    {
        Curve3D theSection = (Curve3D)_section!.Copy();
        theSection.Transform(Transformation(withTranslation, false));
        return theSection;
    }

    /// <summary>
    /// The section in the frame of the location law, rotated so that its normal is the law's (a new curve);
    /// WithTranslation forces the contact of the section and the path.
    /// </summary>
    public Curve3D ModifiedSection(bool withTranslation)
    {
        Curve3D theSection = (Curve3D)_section!.Copy();
        theSection.Transform(Transformation(withTranslation, true));
        return theSection;
    }

    // OCCT's private SectionAxis is defined but never called; it is internal here so that the port keeps it.
    internal void SectionAxis(in Matrix3x3 m, ref Vector3D t, ref Vector3D n, ref Vector3D bn)
    {
        double eps = 1.0e-10;
        Direction3D d;
        Vector3D pathNormal = new Vector3D();
        CLProps cp = new CLProps(_section!, _secParam, 2, eps);
        if (cp.IsTangentDefined())
        {
            cp.Tangent(out d);
            t = t.WithXYZ(d.XYZ());
            t = t.Normalized();
            if (cp.Curvature() > eps)
            {
                cp.Normal(out d);
                n = n.WithXYZ(d.XYZ());
            }
            else
            {
                // Cas ambigu, on essai de recuperer la normal a la trajectoire
                // Reste le probleme des points d'inflexions, qui n'est pas
                // bien traiter par LProp (pas de recuperation de la normal) :
                // A voir...
                pathNormal = pathNormal.WithXYZ(m.Column(1));
                pathNormal = pathNormal.Normalized();
                bn = t ^ pathNormal;
                if (bn.Magnitude() > eps)
                {
                    bn = bn.Normalized();
                }
                n = bn ^ t;
            }
        }
        else
        { // Cas indefinie, on prend le triedre
            // complet sur la trajectoire
            t = t.WithXYZ(m.Column(3));
            n = n.WithXYZ(m.Column(2));
        }
        bn = t ^ n;
    }

    // Whether the pair (dist, angle) is better than the current one.
    private bool Choix(double dist, double angle)
    {
        double evoldist, evolangle;
        evoldist = dist - _dist;
        evolangle = angle - _angleMax;

        // (1) Si la gain en distance est > que le gabarit, on prend
        if (evoldist < -_gabarit)
        {
            return true;
        }

        //  (2) si l'ecart en distance est de l'ordre du gabarit
        if (Math.Abs(evoldist) < _gabarit)
        {
            //  (2.1) si le gain en angle est important on garde
            if (evolangle > 0.5)
            {
                return true;
            }

            //  (2.2) si la variation d'angle est moderee on evalue
            //  une fonction de penalite
            if (Penalite(angle, dist / _gabarit) < Penalite(_angleMax, _dist / _gabarit))
            {
                return true;
            }
        }

        return false;
    }
}
