// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GenLocateExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocEPCOfLocateExtPC.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distance between a point and a 3D curve nearest a starting parameter: the root of
/// <see cref="PcfOfEpcOfExtPC"/> found by math_FunctionRoot from U0, kept if the function is below 1e-7
/// there.
/// </summary>
/// <remarks>
/// The 3D instantiation of OCCT's template Extrema_GenLocateExtPC (Extrema_LocEPCOfLocateExtPC). An algorithm object
/// (decision 117). Keeps a reference to the curve, as OCCT does.
/// </remarks>
internal class LocEpcOfLocateExtPC
{
    private bool _done;
    private double _mytolU;
    private double _myumin;
    private double _myusup;
    private readonly PcfOfEpcOfExtPC _f = new();

    public LocEpcOfLocateExtPC()
    {
        _done = false;
        _mytolU = 0.0;
        _myumin = 0.0;
        _myusup = 0.0;
    }

    /// <summary>The extremum between theP and theC nearest theU0, on the curve's whole range.</summary>
    public LocEpcOfLocateExtPC(in Point3D p, CurveAdaptorBase c, double u0, double tolU)
    {
        Initialize(c, CurveTool.FirstParameter(c), CurveTool.LastParameter(c), tolU);
        Perform(p, u0);
    }

    /// <summary>The extremum between theP and theC nearest theU0, on [theUmin, theUsup].</summary>
    public LocEpcOfLocateExtPC(in Point3D p, CurveAdaptorBase c, double u0, double umin,
        double usup, double tolU)
    {
        Initialize(c, umin, usup, tolU);
        Perform(p, u0);
    }

    /// <summary>Sets the curve, the range and the parametric tolerance.</summary>
    public void Initialize(CurveAdaptorBase c, double umin, double usup, double tolU)
    {
        _done = false;
        _f.Initialize(c);
        _myumin = umin;
        _myusup = usup;
        _mytolU = tolU;
    }

    /// <summary>Searches the extremum for theP from the parameter theU0.</summary>
    public void Perform(in Point3D p, double u0)
    {
        _f.SetPoint(p);
        FunctionRoot s = new(_f, u0, _mytolU, _myumin, _myusup);
        _done = s.IsDone;
        if (_done)
        {
            double uu;
            POnCurv pp = Point();
            uu = pp.Parameter();
            if (_f.Value(uu, out double ff))
            {
                if (Math.Abs(ff) >= 1.0e-07)
                {
                    _done = false;
                }
            }
            else
            {
                _done = false;
            }
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone => _done;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _f.SquareDistance(1);
    }

    /// <summary>True if the extremum is a minimum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsMin
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _f.IsMin(1);
        }
    }

    /// <summary>The point of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public POnCurv Point()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _f.Point(1);
    }
}
