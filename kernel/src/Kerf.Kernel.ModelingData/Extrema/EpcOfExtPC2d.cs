// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GGenExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_EPCOfExtPC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_EPCOfELPCOfLocateExtPC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extremum distances between a point and a 2D curve on a parameter range: the roots of
/// <see cref="PcfOfEpcOfExtPC2d"/> found by math_FunctionRoots on a sampling of the range.
/// </summary>
/// <remarks>
/// The 2D instantiation of OCCT's template Extrema_GGenExtPC, which OCCT names Extrema_EPCOfExtPC2d and
/// Extrema_EPCOfELPCOfLocateExtPC2d (one type). An algorithm object (decision 117): initialize it, then
/// <c>Perform</c>. Keeps a reference to the curve, as OCCT does.
/// </remarks>
internal class EpcOfExtPC2d
{
    private bool _done;
    private int _mynbsample;
    private double _myumin;
    private double _myusup;
    private double _mytolu;
    private double _mytolF;
    private readonly PcfOfEpcOfExtPC2d _f;

    public EpcOfExtPC2d()
    {
        _done = false;
        _mynbsample = 0;
        _myumin = 0.0;
        _myusup = 0.0;
        _mytolu = 0.0;
        _mytolF = 0.0;
        _f = new PcfOfEpcOfExtPC2d();
    }

    /// <summary>All the extrema between theP and theC, from theNbSample samples of the curve's whole range.</summary>
    public EpcOfExtPC2d(in Point2D p, Curve2DAdaptorBase c, int nbSample, double tolU, double tolF)
    {
        _f = new PcfOfEpcOfExtPC2d(p, c);
        Initialize(c, nbSample, tolU, tolF);
        Perform(p);
    }

    /// <summary>All the extrema between theP and theC on [theUmin, theUsup], from theNbSample samples.</summary>
    public EpcOfExtPC2d(in Point2D p, Curve2DAdaptorBase c, int nbSample, double umin, double usup,
        double tolU, double tolF)
    {
        _f = new PcfOfEpcOfExtPC2d(p, c);
        Initialize(c, nbSample, umin, usup, tolU, tolF);
        Perform(p);
    }

    /// <summary>Sets the curve, the number of samples and the tolerances, over the curve's whole range.</summary>
    public void Initialize(Curve2DAdaptorBase c, int nbU, double tolU, double tolF)
    {
        _mynbsample = nbU;
        _mytolu = tolU;
        _mytolF = tolF;
        _f.Initialize(c);
        _myumin = Curve2dTool.FirstParameter(c);
        _myusup = Curve2dTool.LastParameter(c);
    }

    /// <summary>Sets the curve, the number of samples, the range and the tolerances.</summary>
    public void Initialize(Curve2DAdaptorBase c, int nbU, double umin, double usup, double tolU,
        double tolF)
    {
        _mynbsample = nbU;
        _mytolu = tolU;
        _mytolF = tolF;
        _f.Initialize(c);
        _myumin = umin;
        _myusup = usup;
    }

    /// <summary>Sets only the number of samples, the range and the tolerances.</summary>
    public void Initialize(int nbU, double umin, double usup, double tolU, double tolF)
    {
        _mynbsample = nbU;
        _mytolu = tolU;
        _mytolF = tolF;
        _myumin = umin;
        _myusup = usup;
    }

    /// <summary>Sets the curve of the function.</summary>
    public void Initialize(Curve2DAdaptorBase c) => _f.Initialize(c);

    /// <summary>Searches the extrema for the point theP.</summary>
    public void Perform(in Point2D p)
    {
        _f.SetPoint(p);
        _f.SubIntervalInitialize(_myumin, _myusup);
        _done = false;

        FunctionRoots s = new(_f, _myumin, _myusup, _mynbsample, _mytolu, _mytolF, _mytolF);
        if (!s.IsDone || s.IsAllNull)
        {
            return;
        }

        _done = true;
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _f.ExtCount;
        }
    }

    /// <summary>The Nth extremum square distance.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _f.SquareDistance(n);
    }

    /// <summary>True if the Nth extremum distance is a minimum.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public bool IsMin(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _f.IsMin(n);
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    /// <exception cref="OutOfRangeException">theN is not in 1..NbExt().</exception>
    public POnCurv2d Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _f.Point(n);
    }
}
