// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CurveShapeIntersector.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CurveShapeIntersector.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CurveShapeIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// The intersection between an axis or a circle and the faces of a shape, the points sorted in increasing parameter
/// along the axis or the circle. The transitions take the faces' orientations into account.
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shape; the points' faces are the shape's own.</remarks>
internal sealed class LocalCurveShapeIntersector
{
    private bool _done;
    private readonly NCollection_Sequence<PntFace> _points = new();

    public LocalCurveShapeIntersector()
    {
        _done = false;
    }

    public LocalCurveShapeIntersector(in Axis3D axis, in Shape s)
    {
        Init(axis, s);
    }

    public LocalCurveShapeIntersector(in Circle3D c, in Shape s)
    {
        Init(c, s);
    }

    public void Init(in Axis3D axis, in Shape s)
    {
        _done = false;
        _points.Clear();
        if (s.IsNull)
        {
            return;
        }
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        CurveShapeIntersector theInt = new();
        theInt.Init(s, new Line3D(axis), tol);
        Perform(theInt, _points);
        _done = true;
    }

    public void Init(in Circle3D c, in Shape s)
    {
        _done = false;
        _points.Clear();
        if (s.IsNull)
        {
            return;
        }
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        CircleCurve3D gc = new(c);
        CurveAdaptor ac = new(gc, 0.0, 2.0 * Math.PI);

        CurveShapeIntersector theInt = new();
        theInt.Init(s, ac, tol);

        Perform(theInt, _points);
        _done = true;
    }

    public bool IsDone => _done;

    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _points.Length();
        }
    }

    public PntFace Point(int i)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _points.Value(i);
    }

    /// <summary>
    /// Searches the first intersection point located after the parameter <paramref name="From"/> whose orientation
    /// is not EXTERNAL; see <see cref="CSIntersector"/>. The outputs are left unchanged where OCCT leaves them.
    /// </summary>
    public bool LocalizeAfter(double From, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, fmeps = From - eps;
        int i, ifirst, nbpoints = _points.Length();
        for (ifirst = 1; ifirst <= nbpoints; ifirst++)
        {
            if (_points.Value(ifirst).Parameter() >= fmeps)
            {
                break;
            }
        }
        bool retVal = false;
        if (ifirst <= nbpoints)
        {
            i = ifirst;
            indFrom = ifirst;
            bool found = false;
            while (!found)
            {
                or = _points.Value(i).Orientation();
                param = _points.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (_points.Value(i).Parameter() - param <= eps)
                    {
                        if (or != External && or != _points.Value(i).Orientation())
                        {
                            or = External;
                        }
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (or == External)
                {
                    found = (i > nbpoints);
                    indFrom = i;
                }
                else
                { // on a une intersection franche
                    indTo = i - 1;
                    found = true;
                    retVal = true;
                }
            }
        }

        return retVal;
    }

    /// <summary>
    /// Searches the first intersection point located before the parameter <paramref name="From"/> whose orientation
    /// is not EXTERNAL.
    /// </summary>
    public bool LocalizeBefore(double From, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, fpeps = From + eps;
        int i, ifirst, nbpoints = _points.Length();
        for (ifirst = nbpoints; ifirst >= 1; ifirst--)
        {
            if (_points.Value(ifirst).Parameter() <= fpeps)
            {
                break;
            }
        }
        bool retVal = false;
        if (ifirst >= 1)
        {
            i = ifirst;
            indTo = ifirst;
            bool found = false;
            while (!found)
            {
                or = _points.Value(i).Orientation();
                param = _points.Value(i).Parameter();
                i = i - 1;
                while (i >= 1)
                {
                    if (param - _points.Value(i).Parameter() <= eps)
                    {
                        if (or != External && or != _points.Value(i).Orientation())
                        {
                            or = External;
                        }
                        i--;
                    }
                    else
                    {
                        break;
                    }
                }
                if (or == External)
                {
                    found = (i < 1);
                    indTo = i;
                }
                else
                { // on a une intersection franche
                    indFrom = i + 1;
                    found = true;
                    retVal = true;
                }
            }
        }

        return retVal;
    }

    /// <summary>Searches the first intersection point after the index <paramref name="fromInd"/> whose orientation is not EXTERNAL.</summary>
    public bool LocalizeAfter(int fromInd, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        int nbpoints = _points.Length();
        if (fromInd >= nbpoints)
        {
            return false;
        }

        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, fmeps;
        int i, ifirst;
        if (fromInd >= 1)
        {
            fmeps = _points.Value(fromInd).Parameter() - eps;
            for (ifirst = fromInd + 1; ifirst <= nbpoints; ifirst++)
            {
                if (_points.Value(ifirst).Parameter() >= fmeps)
                {
                    break;
                }
            }
        }
        else
        {
            ifirst = 1;
        }

        bool retVal = false;
        if (ifirst <= nbpoints)
        {
            i = ifirst;
            indFrom = ifirst;
            bool found = false;
            while (!found)
            {
                or = _points.Value(i).Orientation();
                param = _points.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (_points.Value(i).Parameter() - param <= eps)
                    {
                        if (or != External && or != _points.Value(i).Orientation())
                        {
                            or = External;
                        }
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (or == External)
                {
                    found = (i > nbpoints);
                    indFrom = i;
                }
                else
                { // on a une intersection franche
                    indTo = i - 1;
                    found = true;
                    retVal = true;
                }
            }
        }
        return retVal;
    }

    /// <summary>Searches the first intersection point before the index <paramref name="fromInd"/> whose orientation is not EXTERNAL.</summary>
    public bool LocalizeBefore(int fromInd, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        int nbpoints = _points.Length();
        if (fromInd <= 1)
        {
            return false;
        }

        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double param, fpeps;
        int i, ifirst;
        if (fromInd <= nbpoints)
        {
            fpeps = _points.Value(fromInd).Parameter() + eps;
            for (ifirst = fromInd - 1; ifirst >= 1; ifirst--)
            {
                if (_points.Value(ifirst).Parameter() <= fpeps)
                {
                    break;
                }
            }
        }
        else
        {
            ifirst = nbpoints;
        }

        bool retVal = false;
        if (ifirst >= 1)
        {
            i = ifirst;
            indTo = ifirst;
            bool found = false;
            while (!found)
            {
                or = _points.Value(i).Orientation();
                param = _points.Value(i).Parameter();
                i = i - 1;
                while (i >= 1)
                {
                    if (param - _points.Value(i).Parameter() <= eps)
                    {
                        if (or != External && or != _points.Value(i).Orientation())
                        {
                            or = External;
                        }
                        i--;
                    }
                    else
                    {
                        break;
                    }
                }
                if (or == External)
                {
                    found = (i < 1);
                    indTo = i;
                }
                else
                { // on a une intersection franche
                    indFrom = i + 1;
                    found = true;
                    retVal = true;
                }
            }
        }
        return retVal;
    }

    private static void Perform(CurveShapeIntersector theInt, NCollection_Sequence<PntFace> points)
    {
        double param, paramu, paramv;
        int i, nbpoints = 0;

        Orientation theor = Forward, orface;

        while (theInt.More())
        {
            Point3D thept = theInt.Pnt();
            Face theface = theInt.Face();
            orface = theface.Orientation;
            param = theInt.W();
            paramu = theInt.U();
            paramv = theInt.V();

            switch (theInt.Transition())
            {
                case TransitionOnCurve.In:
                    if (orface == Forward)
                    {
                        theor = Forward;
                    }
                    else if (orface == Reversed)
                    {
                        theor = Reversed;
                    }
                    else
                    {
                        theor = External;
                    }
                    break;
                case TransitionOnCurve.Out:
                    if (orface == Forward)
                    {
                        theor = Reversed;
                    }
                    else if (orface == Reversed)
                    {
                        theor = Forward;
                    }
                    else
                    {
                        theor = External;
                    }
                    break;
                case TransitionOnCurve.Tangent:
                    theor = External;
                    break;
            }

            PntFace newpt = new(thept, theface, theor, param, paramu, paramv);

            for (i = 1; i <= nbpoints; i++)
            {
                if (points.Value(i).Parameter() - param > 0.0)
                {
                    break;
                }
            }
            if (i <= nbpoints)
            {
                points.InsertBefore(i, newpt);
            }
            else
            {
                points.Append(newpt);
            }
            nbpoints++;
            theInt.Next();
        }
    }
}
