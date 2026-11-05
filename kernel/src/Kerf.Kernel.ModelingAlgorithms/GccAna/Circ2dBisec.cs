// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// The bisecting curves of two 2D circles: the loci of the points equidistant from both. Depending on their relative
/// position, ellipses, hyperbolas, circles or lines.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); as in OCCT, each solution is built when it is
/// asked for.
/// </remarks>
internal sealed class Circ2dBisec
{
    private bool _wellDone;
    private int _nbrSol;
    private int _intersection;
    private bool _sameradius;
    private Circle2D _circle1;
    private Circle2D _circle2;

    /// <summary>Constructs the bisecting curves of Circ1 and Circ2.</summary>
    public Circ2dBisec(in Circle2D circ1, in Circle2D circ2)
    {
        //=========================================================================
        //  Initialization of fields :                                            +
        //            - circle1  (Circle : first argument)                        +
        //            - circle2  (Line   : second argument)                       +
        //            - intersection (Integer showing the smallest position of    +
        //                            two circles correspondingly to each other)  +
        //            - sameradius   (Boolean showing if the two circles have     +
        //                            the same radius or not)                     +
        //            - NbrSol   (Integer showing the number of solutions)        +
        //            - WellDone (Boolean showing success or failure of the algo) +
        //=========================================================================

        _wellDone = false;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        double r1 = circ1.Radius;
        double r2 = circ2.Radius;
        _sameradius = Math.Abs(r1 - r2) <= tol;
        if (r1 < r2)
        {
            _circle1 = circ2;
            _circle2 = circ1;
            r1 = _circle1.Radius;
            r2 = _circle2.Radius;
        }
        else
        {
            _circle1 = circ1;
            _circle2 = circ2;
        }
        double dist = _circle2.Location.DistanceTo(_circle1.Location);
        if (r1 - dist - r2 > tol)
        {
            _intersection = 0;
            _nbrSol = 2;
            _wellDone = true;
        }
        else if (Math.Abs(r1 - dist - r2) <= tol)
        {
            _intersection = 1;
            if (_sameradius)
            {
                _nbrSol = 0;
                _wellDone = true;
            }
            else
            {
                _nbrSol = 2;
                _wellDone = true;
            }
        }
        else if ((dist + r2 - r1 > tol) && (r1 - dist + r2 > tol))
        {
            _intersection = 2;
            if (_sameradius)
            {
                _nbrSol = 2;
                _wellDone = true;
            }
            else
            {
                _nbrSol = 3;
                _wellDone = true;
            }
        }
        else if (Math.Abs(r1 - dist + r2) <= tol)
        {
            _intersection = 3;
            if (_sameradius)
            {
                _nbrSol = 2;
                _wellDone = true;
            }
            else
            {
                _nbrSol = 3;
                _wellDone = true;
            }
        }
        else
        {
            _intersection = 4;
            if (_sameradius)
            {
                _nbrSol = 3;
                _wellDone = true;
            }
            else
            {
                _nbrSol = 4;
                _wellDone = true;
            }
        }
    }

    //=========================================================================
    //  Processing.                                                           +
    //  Return the coordinates of centers of circles circle1 and circle2      +
    //  (xcencir1, ycencir1, xcencir2, ycencir2).                             +
    //  Also return the radiuses of two circles R1 and R2.                    +
    //=========================================================================

    /// <summary>
    /// The solution number Index. As in OCCT, it is null for the index combinations OCCT's cases don't cover.
    /// </summary>
    public ConstraintBisec ThisSolution(int index)
    {
        double tol = 1.0e-14;
        ConstraintBisec? bissol = null;

        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            double xcencir1 = _circle1.Location.X;
            double ycencir1 = _circle1.Location.Y;
            double xcencir2 = _circle2.Location.X;
            double ycencir2 = _circle2.Location.Y;
            double dist = _circle1.Location.DistanceTo(_circle2.Location);

            Point2D pcen = new((xcencir1 + xcencir2) / 2.0, (ycencir1 + ycencir2) / 2.0);
            Direction2D dircen = new(), medcen = new();
            if (dist > tol)
            {
                dircen = dircen.WithCoord(xcencir2 - xcencir1, ycencir2 - ycencir1);
                medcen = medcen.WithCoord(ycencir2 - ycencir1, xcencir1 - xcencir2);
            }
            Direction2D dirx = new(Direction2D.D.X);
            Axis2D acenx = new(pcen, dirx);
            Axis2D acencen = new(pcen, dircen);

            double r1 = _circle1.Radius;
            double r2 = _circle2.Radius;

            if ((_nbrSol == 1) && (_intersection == 0))
            {
                double r;
                if (index == 1)
                {
                    r = (r1 + r2) / 2.0;
                }
                else
                {
                    r = (r1 - r2) / 2.0;
                }
                Circle2D c = new(acenx, r);
                bissol = new BCirc(c);
            }
            else if ((_nbrSol == 2) && (_intersection == 1))
            {
                if (index == 1)
                {
                    Ellipse2D e = new(acencen,
                                       (r1 + r2) / 2.0,
                                       Math.Sqrt((r1 * r1 + r2 * r2 - dist * dist) / 4.0 + r1 * r2 / 2.0));
                    bissol = new BElips(e);
                }
                else if (index == 2)
                {
                    Line2D l = new(_circle1.Location, dircen);
                    bissol = new BLine(l);
                }
            }
            else if ((_nbrSol == 2) && (_intersection == 0))
            {
                if (index == 1)
                {
                    if (Math.Abs(xcencir2 - xcencir1) < tol && Math.Abs(ycencir2 - ycencir1) < tol)
                    {
                        Circle2D c = new(acenx, (r1 + r2) / 2.0);
                        bissol = new BCirc(c);
                    }
                    else
                    {
                        Ellipse2D e = new(acencen,
                                           (r1 + r2) / 2.0,
                                           Math.Sqrt((r1 * r1 + r2 * r2 - dist * dist) / 4.0 + r1 * r2 / 2.0));
                        bissol = new BElips(e);
                    }
                }
                else if (index == 2)
                {
                    if (Math.Abs(xcencir2 - xcencir1) < tol && Math.Abs(ycencir2 - ycencir1) < tol)
                    {
                        Circle2D c = new(acencen, (r1 - r2) / 2.0);
                        bissol = new BCirc(c);
                    }
                    else
                    {
                        Ellipse2D e = new(acencen,
                                           (r1 - r2) / 2.0,
                                           Math.Sqrt((r1 * r1 + r2 * r2 - dist * dist) / 4.0 - r1 * r2 / 2.0));
                        bissol = new BElips(e);
                    }
                }
            }
            else if (_intersection == 2)
            {
                if (_sameradius)
                {
                    if (index == 1)
                    {
                        Line2D l = new(pcen, medcen);
                        bissol = new BLine(l);
                    }
                    else if (index == 2)
                    {
                        Ellipse2D e = new(acencen, r1, Math.Sqrt(r1 * r1 - dist * dist / 4.0));
                        bissol = new BElips(e);
                    }
                }
                else
                {
                    if (index == 1)
                    {
                        Hyperbola2D h1;
                        h1 = new Hyperbola2D(acencen,
                                           (r1 - r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 - r2) * (r1 - r2)) / 2.0);
                        bissol = new BHyper(h1);
                    }
                    else if (index == 2)
                    {
                        Hyperbola2D h1 = new(acencen,
                                           (r1 - r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 - r2) * (r1 - r2)) / 2.0);
                        bissol = new BHyper(h1.OtherBranch());
                    }
                    else if (index == 3)
                    {
                        Ellipse2D e = new(acencen,
                                           (r1 + r2) / 2.0,
                                           Math.Sqrt((r1 * r1 + r2 * r2 - dist * dist) / 4.0 + r1 * r2 / 2.0));
                        bissol = new BElips(e);
                    }
                }
            }
            else if (_intersection == 3)
            {
                if (_sameradius)
                {
                    if (index == 1)
                    {
                        Line2D l = new(pcen, dircen);
                        bissol = new BLine(l);
                    }
                    else if (index == 2)
                    {
                        Line2D l = new(pcen, medcen);
                        bissol = new BLine(l);
                    }
                }
                else
                {
                    if (index == 1)
                    {
                        Line2D l = new(pcen, dircen);
                        bissol = new BLine(l);
                    }
                    else if (index == 2)
                    {
                        Hyperbola2D h1 = new(acencen,
                                           (r1 - r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 - r2) * (r1 - r2)) / 2.0);
                        bissol = new BHyper(h1);
                    }
                    else if (index == 3)
                    {
                        Hyperbola2D h1 = new(acencen,
                                           (r1 - r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 - r2) * (r1 - r2)) / 2.0);
                        bissol = new BHyper(h1.OtherBranch());
                    }
                }
            }
            else if (_intersection == 4)
            {
                if (_sameradius)
                {
                    if (index == 1)
                    {
                        Line2D l = new(pcen, medcen);
                        bissol = new BLine(l);
                    }
                    else if (index == 2)
                    {
                        Hyperbola2D h1 = new(acencen, r1, Math.Sqrt(dist * dist - 4 * r1 * r1) / 2.0);
                        bissol = new BHyper(h1);
                    }
                    else if (index == 3)
                    {
                        Hyperbola2D h1 = new(acencen, r1, Math.Sqrt(dist * dist - 4 * r1 * r1) / 2.0);
                        bissol = new BHyper(h1.OtherBranch());
                    }
                }
                else
                {
                    if (index == 1)
                    {
                        Hyperbola2D h1 = new(acencen,
                                           (r1 - r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 - r2) * (r1 - r2)) / 2.0);
                        bissol = new BHyper(h1);
                    }
                    else if (index == 2)
                    {
                        Hyperbola2D h1 = new(acencen,
                                           (r1 - r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 - r2) * (r1 - r2)) / 2.0);
                        bissol = new BHyper(h1.OtherBranch());
                    }
                    else if (index == 3)
                    {
                        Hyperbola2D h1 = new(acencen,
                                           (r1 + r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 + r2) * (r1 + r2)) / 2.0);
                        bissol = new BHyper(h1);
                    }
                    else if (index == 4)
                    {
                        Hyperbola2D h1 = new(acencen,
                                           (r1 + r2) / 2.0,
                                           Math.Sqrt(dist * dist - (r1 + r2) * (r1 + r2)) / 2.0);
                        bissol = new BHyper(h1.OtherBranch());
                    }
                }
            }
        }
        return bissol!;
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of bisecting curves.</summary>
    public int SolutionCount
    {
        get
        {
            if (!_wellDone)
            {
                throw new NotDoneException();
            }
            return _nbrSol;
        }
    }
}
