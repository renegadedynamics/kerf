// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelicoidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelicoidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A circular helicoid: the ruled surface swept by a line rotating uniformly about an axis while translating
/// along it, <c>S(u,v) = O + v*cos(u)*XDir + v*sin(u)*YDir + (P*u/(2*Pi))*ZDir</c>, with pitch P != 0 (axial
/// advance per 2*Pi turn). u and v in (-inf, +inf); neither periodic nor closed; CN. Reversal, transformation and
/// iso curves are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class CircularHelicoidSurface : ElementarySurface
{
    private double _pitch; // Axial advance per 2*Pi turn

    /// <summary>Creates a circular helicoid.</summary>
    /// <exception cref="ConstructionErrorException">thePitch == 0.</exception>
    public CircularHelicoidSurface(in CoordinateSystem position, double pitch)
    {
        _pitch = pitch;
        if (pitch == 0.0)
        {
            throw new ConstructionErrorException("GeomEval_CircularHelicoidSurface: pitch must be != 0");
        }
        _pos = position;
    }

    /// <summary>The pitch.</summary>
    public double Pitch() => _pitch;

    /// <exception cref="ConstructionErrorException">thePitch == 0.</exception>
    internal void SetPitch(double pitch)
    {
        CheckMutable();
        if (pitch == 0.0)
        {
            throw new ConstructionErrorException(
                "GeomEval_CircularHelicoidSurface::SetPitch: pitch must be != 0");
        }
        _pitch = pitch;
    }

    /// <summary>A copy with the pitch set to thePitch.</summary>
    /// <exception cref="ConstructionErrorException">thePitch == 0.</exception>
    public CircularHelicoidSurface WithPitch(double pitch)
    {
        CircularHelicoidSurface copy = (CircularHelicoidSurface)Copy();
        copy.SetPitch(pitch);
        return copy;
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_CircularHelicoidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_CircularHelicoidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_CircularHelicoidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double v)
    {
        throw new MethodNotImplementedException("GeomEval_CircularHelicoidSurface::VReversedParameter");
    }

    /// <summary>Infinite bounds for both parameters.</summary>
    internal override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        u2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        v1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        v2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    public override bool IsUClosed => false;

    public override bool IsVClosed => false;

    public override bool IsUPeriodic => false;

    public override bool IsVPeriodic => false;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D UIso(double u)
    {
        throw new MethodNotImplementedException("GeomEval_CircularHelicoidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D VIso(double v)
    {
        throw new MethodNotImplementedException("GeomEval_CircularHelicoidSurface::VIso");
    }

    internal override Point3D EvalD0(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        double zComp = _pitch * u / (2.0 * M_PI);

        return new Point3D(o + v * cosU * xd + v * sinU * yd + zComp * zd);
    }

    internal override ResD1 EvalD1(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        double zComp = _pitch * u / (2.0 * M_PI);
        double zRate = _pitch / (2.0 * M_PI);

        ResD1 result;
        result.Point = new Point3D(o + v * cosU * xd + v * sinU * yd + zComp * zd);

        // dS/du = -v*sin(u)*XDir + v*cos(u)*YDir + P/(2*Pi)*ZDir
        result.D1U = new Vector3D(v * (-sinU) * xd + v * cosU * yd + zRate * zd);

        // dS/dv = cos(u)*XDir + sin(u)*YDir
        result.D1V = new Vector3D(cosU * xd + sinU * yd);

        return result;
    }

    internal override ResD2 EvalD2(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        double zComp = _pitch * u / (2.0 * M_PI);
        double zRate = _pitch / (2.0 * M_PI);

        ResD2 result;
        result.Point = new Point3D(o + v * cosU * xd + v * sinU * yd + zComp * zd);
        result.D1U = new Vector3D(v * (-sinU) * xd + v * cosU * yd + zRate * zd);
        result.D1V = new Vector3D(cosU * xd + sinU * yd);

        // d2S/dudu = -v*cos(u)*XDir - v*sin(u)*YDir
        result.D2U = new Vector3D(v * (-cosU) * xd + v * (-sinU) * yd);

        // d2S/dvdv = 0 (ruled surface)
        result.D2V = new Vector3D(0.0, 0.0, 0.0);

        // d2S/dudv = -sin(u)*XDir + cos(u)*YDir
        result.D2UV = new Vector3D((-sinU) * xd + cosU * yd);

        return result;
    }

    internal override ResD3 EvalD3(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        double zComp = _pitch * u / (2.0 * M_PI);
        double zRate = _pitch / (2.0 * M_PI);

        ResD3 result;
        result.Point = new Point3D(o + v * cosU * xd + v * sinU * yd + zComp * zd);
        result.D1U = new Vector3D(v * (-sinU) * xd + v * cosU * yd + zRate * zd);
        result.D1V = new Vector3D(cosU * xd + sinU * yd);
        result.D2U = new Vector3D(v * (-cosU) * xd + v * (-sinU) * yd);
        result.D2V = new Vector3D(0.0, 0.0, 0.0);
        result.D2UV = new Vector3D((-sinU) * xd + cosU * yd);

        // d3S/du3 = v*sin(u)*XDir - v*cos(u)*YDir
        result.D3U = new Vector3D(v * sinU * xd + v * (-cosU) * yd);

        // d3S/dv3 = 0
        result.D3V = new Vector3D(0.0, 0.0, 0.0);

        // d3S/du2dv = -cos(u)*XDir - sin(u)*YDir
        result.D3UUV = new Vector3D((-cosU) * xd + (-sinU) * yd);

        // d3S/dudv2 = 0
        result.D3UVV = new Vector3D(0.0, 0.0, 0.0);

        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException(
                "GeomEval_CircularHelicoidSurface::EvalDN: invalid derivative order");
        }

        // S(u,v) = O + v*cos(u)*XDir + v*sin(u)*YDir + P*u/(2*Pi)*ZDir
        // The v-dependence is linear, so Nv >= 2 => 0.
        // The u-dependence of the Z term is linear, so Nu >= 2 on Z => 0.
        if (nv >= 2)
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }

        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        // d^Nu/du^Nu[cos(u)] = cos(u + Nu*Pi/2)
        // d^Nu/du^Nu[sin(u)] = sin(u + Nu*Pi/2)
        double phaseU = nu * M_PI / 2.0;

        if (nv == 0)
        {
            // d^Nu/du^Nu of [v*cos(u)*XD + v*sin(u)*YD + P*u/(2*Pi)*ZD]
            double coeffX0 = v * Math.Cos(u + phaseU);
            double coeffY0 = v * Math.Sin(u + phaseU);
            Xyz result = coeffX0 * xd + coeffY0 * yd;

            // Z term: d^Nu/du^Nu[P*u/(2*Pi)] = P/(2*Pi) if Nu=1, 0 if Nu>=2
            if (nu == 1)
            {
                result += (_pitch / (2.0 * M_PI)) * zd;
            }
            return new Vector3D(result);
        }

        // Nv == 1: d/dv of d^Nu/du^Nu [v*cos(u)*XD + v*sin(u)*YD]
        // = d^Nu/du^Nu[cos(u)]*XD + d^Nu/du^Nu[sin(u)]*YD
        double coeffX = Math.Cos(u + phaseU);
        double coeffY = Math.Sin(u + phaseU);
        return new Vector3D(coeffX * xd + coeffY * yd);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_CircularHelicoidSurface::Transform");
    }

    public override Geometry3D Copy() => new CircularHelicoidSurface(_pos, _pitch);
}
