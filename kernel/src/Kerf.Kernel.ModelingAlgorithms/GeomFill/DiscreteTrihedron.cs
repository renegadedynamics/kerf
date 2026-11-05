// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DiscreteTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DiscreteTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A discrete trihedron law: trihedrons at sampled parameters, each the previous one rotated onto the new tangent
/// (a rotation-minimizing frame). Lines and conics use the Frenet law. Its derivatives are zero.
/// </summary>
internal class DiscreteTrihedron : TrihedronLaw
{
    private static readonly double TolConf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private Point3D _point;
    private readonly NCollection_HSequence<Frame3D> _trihedrons;
    private readonly NCollection_HSequence<double> _knots;
    private readonly Frenet _frenet;
    private bool _useFrenet;

    public DiscreteTrihedron()
    {
        _useFrenet = false;
        _frenet = new Frenet();
        _knots = new NCollection_HSequence<double>();
        _trihedrons = new NCollection_HSequence<Frame3D>();
    }

    public override TrihedronLaw Copy()
    {
        DiscreteTrihedron copy = new DiscreteTrihedron();
        if (_curve is not null)
        {
            copy.SetCurve(_curve);
        }
        return copy;
    }

    /// <summary>Initializes the curve; returns true when the Frenet law is used (lines and conics).</summary>
    public override bool SetCurve(CurveAdaptorBase c)
    {
        base.SetCurve(c);
        if (c is not null)
        {
            CurveKind type;
            type = c.GetType();
            switch (type)
            {
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                case Line:
                {
                    // No problem
                    _useFrenet = true;
                    _frenet.SetCurve(c);
                    break;
                }
                default:
                {
                    _useFrenet = false;
                    // We have to fill <myKnots> and <myTrihedrons>
                    Init();
                    break;
                }
            }
        }
        return _useFrenet;
    }

    /// <summary>Computes the sampled trihedrons.</summary>
    /// <remarks>
    /// OCCT appends to the knots and trihedrons without clearing them, so a second <see cref="SetCurve"/> on a
    /// free-form curve keeps the first curve's samples (occt-issues.md); kept.
    /// </remarks>
    public void Init()
    {
        int nbIntervals = _trimmed!.NbIntervals(CN);
        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, nbIntervals + 1);
        _trimmed.Intervals(knots, CN);

        int nbSamples = 10;

        int i, j;
        // OCCT: the sequences are not cleared first (a second SetCurve appends to the previous samples)
        for (i = 1; i <= nbIntervals; i++)
        {
            double delta = (knots[i + 1] - knots[i]) / nbSamples;
            for (j = 0; j < nbSamples; j++)
            {
                double param = knots[i] + j * delta;
                _knots.Append(param);
            }
        }
        _knots.Append(knots[nbIntervals + 1]);

        Point3D origin = new Point3D(0.0, 0.0, 0.0), subPnt;
        Direction3D tangDir;
        double norm;
        for (i = 1; i <= _knots.Length(); i++)
        {
            double param = _knots.Value(i);
            _trimmed.D1(param, out Point3D pnt, out Vector3D tangent);
            norm = tangent.Magnitude;
            if (norm < TolConf)
            {
                double subdelta = (_knots.Value(i + 1) - _knots.Value(i)) / nbSamples;
                if (subdelta < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    subdelta = _knots.Value(i + 1) - _knots.Value(i);
                }
                subPnt = _trimmed.Value(param + subdelta);
                tangent = tangent.WithXYZ(subPnt.XYZ() - pnt.XYZ());
            }

            tangDir = new Direction3D(tangent); // normalize;
            tangent = new Vector3D(tangDir);
            if (i == 1) // first point
            {
                Frame3D firstAxis = new Frame3D(origin, tangDir);
                _trihedrons.Append(firstAxis);
            }
            else
            {
                Frame3D lastAxis = _trihedrons.Value(_trihedrons.Length());
                Vector3D lastTangent = new Vector3D(lastAxis.Direction);
                Vector3D axisOfRotation = lastTangent ^ tangent;
                if (axisOfRotation.Magnitude <= Constants.Resolution()) // tangents are equal or opposite
                {
                    double scalarProduct = lastTangent * tangent;
                    if (scalarProduct > 0.0)
                    { // tangents are equal
                        _trihedrons.Append(lastAxis);
                    }
                    else // tangents are opposite
                    {
                        double newParam = (_knots.Value(i - 1) + _knots.Value(i)) / 2.0;
                        if (newParam - _knots.Value(i - 1) < Constants.Resolution())
                        {
                            throw new ConstructionErrorException(
                                "GeomFill_DiscreteTrihedron : impassable singularities on path curve");
                        }
                        _knots.InsertBefore(i, newParam);
                        i--;
                    }
                }
                else // good value of angle
                {
                    double theAngle = lastTangent.AngleWithRef(tangent, axisOfRotation);
                    Axis3D theAxisOfRotation = new Axis3D(origin, new Direction3D(axisOfRotation));
                    Frame3D newAxis = lastAxis.Rotated(theAxisOfRotation, theAngle);
                    newAxis = newAxis.WithDirection(tangDir); // to prevent accumulation of floating computations error
                    _trihedrons.Append(newAxis);
                }
            }
        }
    }

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        if (_useFrenet)
        {
            _frenet.D0(param, ref tangent, ref normal, ref biNormal);
        }
        else
        {
            // Locate <Param> in the sequence <myKnots>
            int index = -1;
            double tolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();
            int nbSamples = 10;
            Point3D origin = new Point3D(0.0, 0.0, 0.0);

            int i;
            int i1, i2;
            i1 = 1;
            i2 = _knots.Length();
            for (;;)
            {
                i = (i1 + i2) / 2;
                if (param <= _knots.Value(i))
                {
                    i2 = i;
                }
                else
                {
                    i1 = i;
                }
                if (i2 - i1 <= 1)
                {
                    break;
                }
            }
            index = i1;
            if (Math.Abs(param - _knots.Value(i2)) < tolPar)
            {
                index = i2;
            }

            double prevParam = _knots.Value(index);
            Frame3D prevAxis = _trihedrons.Value(index);
            Frame3D theAxis;
            if (Math.Abs(param - prevParam) < tolPar)
            {
                theAxis = prevAxis;
            }
            else //<Param> is between knots
            {
                _trimmed!.D1(param, out _point, out tangent);
                double norm = tangent.Magnitude;
                if (norm < TolConf)
                {
                    double subdelta = (_knots.Value(index + 1) - param) / nbSamples;
                    if (subdelta < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        subdelta = _knots.Value(index + 1) - param;
                    }
                    Point3D subPnt = _trimmed.Value(param + subdelta);
                    tangent = tangent.WithXYZ(subPnt.XYZ() - _point.XYZ());
                }

                Direction3D tangDir = new Direction3D(tangent); // normalize;
                tangent = new Vector3D(tangDir);
                Vector3D prevTangent = new Vector3D(prevAxis.Direction);
                Vector3D axisOfRotation = prevTangent ^ tangent;
                if (axisOfRotation.Magnitude <= Constants.Resolution()) // tangents are equal
                {
                    // we assume that tangents can not be opposite
                    theAxis = prevAxis;
                }
                else // good value of angle
                {
                    double theAngle = prevTangent.AngleWithRef(tangent, axisOfRotation);
                    Axis3D theAxisOfRotation = new Axis3D(origin, new Direction3D(axisOfRotation));
                    theAxis = prevAxis.Rotated(theAxisOfRotation, theAngle);
                }
                theAxis = theAxis.WithDirection(tangDir); // to prevent accumulation of floating computations error
            } // end of else (Param is between knots)

            tangent = new Vector3D(theAxis.Direction);
            normal = new Vector3D(theAxis.XDirection);
            biNormal = new Vector3D(theAxis.YDirection);
        }
        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        if (_useFrenet)
        {
            _frenet.D1(param, ref tangent, ref dTangent, ref normal, ref dNormal, ref biNormal, ref dBiNormal);
        }
        else
        {
            D0(param, ref tangent, ref normal, ref biNormal);

            dTangent = dTangent.WithCoord(0.0, 0.0, 0.0);
            dNormal = dNormal.WithCoord(0.0, 0.0, 0.0);
            dBiNormal = dBiNormal.WithCoord(0.0, 0.0, 0.0);
        }
        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        if (_useFrenet)
        {
            _frenet.D2(param, ref tangent, ref dTangent, ref d2Tangent, ref normal, ref dNormal, ref d2Normal,
                        ref biNormal, ref dBiNormal, ref d2BiNormal);
        }
        else
        {
            D0(param, ref tangent, ref normal, ref biNormal);

            dTangent = dTangent.WithCoord(0.0, 0.0, 0.0);
            dNormal = dNormal.WithCoord(0.0, 0.0, 0.0);
            dBiNormal = dBiNormal.WithCoord(0.0, 0.0, 0.0);
            d2Tangent = d2Tangent.WithCoord(0.0, 0.0, 0.0);
            d2Normal = d2Normal.WithCoord(0.0, 0.0, 0.0);
            d2BiNormal = d2BiNormal.WithCoord(0.0, 0.0, 0.0);
        }
        return true;
    }

    public override int NbIntervals(Continuity s) => _trimmed!.NbIntervals(CN);

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _trimmed!.Intervals(t, CN);
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        int num = 20; // order of digitalization
        Vector3D t = new Vector3D(), n = new Vector3D(), bn = new Vector3D();
        aTangent = new Vector3D(0, 0, 0);
        aNormal = new Vector3D(0, 0, 0);
        aBiNormal = new Vector3D(0, 0, 0);
        double step = (_trimmed!.LastParameter() - _trimmed.FirstParameter()) / num;
        double param;
        for (int i = 0; i <= num; i++)
        {
            param = _trimmed.FirstParameter() + i * step;
            if (param > _trimmed.LastParameter())
            {
                param = _trimmed.LastParameter();
            }
            D0(param, ref t, ref n, ref bn);
            aTangent += t;
            aNormal += n;
            aBiNormal += bn;
        }
        aTangent /= num + 1;
        aNormal /= num + 1;

        aTangent = aTangent.Normalized();
        aBiNormal = aTangent.Crossed(aNormal).Normalized();
        aNormal = aBiNormal.Crossed(aTangent);
    }

    public override bool IsConstant => _curve!.GetType() == Line;

    /// <summary>Returns true.</summary>
    public override bool IsOnlyBy3dCurve => true;
}
