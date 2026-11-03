// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Solid.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Solid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The checks of a solid on its own (<see cref="Minimum"/>): shells sharing faces
/// (<see cref="InvalidImbricationOfShells"/>), parts that aren't shells and aren't internal, or only internal
/// shells (<see cref="BadOrientationOfSubshape"/>), closed shells outside one another
/// (<see cref="SubshapeNotInShape"/>) and more than one closed shell that isn't a hole
/// (<see cref="EnclosedRegion"/>).
/// </summary>
/// <remarks>
/// Each closed shell is classified in a solid of its own, built for the check with the internal builder and never
/// handed out; the checked solid is only read.
/// </remarks>
public class SolidCheck : ResultCheck
{
    // OCCT's BRepCheck_ToolSolid (a class local to the .cxx). Its BRepCheck_HSC, a handle holding the classifier, is
    // the classifier itself.
    private sealed class BRepCheck_ToolSolid
    {
        private bool _isHole;
        private Point3D _pnt;
        private double _pntTol;
        private Solid _solid;
        private SolidClassifier? _hsc;

        public BRepCheck_ToolSolid()
        {
            _isHole = false;
            _pntTol = global::Kerf.Engine.Foundation.Precision.Confusion();
            _pnt = new Point3D(-1.0, -1.0, -1.0);
            _solid = new Solid();
        }

        public void SetSolid(in Solid aZ) => _solid = aZ;

        public Solid Solid() => _solid;

        public bool IsHole => _isHole;

        public Point3D InnerPoint() => _pnt;

        public double CheckTol() => _pntTol;

        public bool IsOut(BRepCheck_ToolSolid aOther)
        {
            bool bFlag;
            Containment state;
            //
            SolidClassifier sc = _hsc!;
            //
            sc.Perform(aOther.InnerPoint(), aOther.CheckTol());
            state = sc.State();
            bFlag = (state == Out);
            //
            return bFlag;
        }

        public void Init()
        {
            double t, par_T;
            var exp = new Explorer();
            //
            // 0.myHSC
            _hsc = new SolidClassifier();
            //
            SolidClassifier sc = _hsc;
            // 1. Load
            sc.Load(_solid);
            //
            // 2. myIsHole
            sc.PerformInfinitePoint(RealSmall());
            _isHole = (sc.State() == In);
            //
            // 3. myPnt
            par_T = 0.43213918; // 10*e^(-PI)
            exp.Init(_solid, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                if (!Tool.Degenerated(e))
                {
                    Curve3D c3D = Tool.Curve(e, out double t1, out double t2)!;
                    t = (1.0 - par_T) * t1 + par_T * t2;
                    _pnt = c3D.Value(t);
                    _pntTol = Tool.Tolerance(e);
                    break;
                }
            }
        }
    }

    /// <summary>Checks the solid on its own (<see cref="Minimum"/>).</summary>
    public SolidCheck(in Solid s)
    {
        Init(s);
    }

    /// <summary>Nothing more than the minimum.</summary>
    public override void Blind()
    {
        if (!_blind)
        {
            // nothing more than in the minimum
            _blind = true;
        }
    }

    /// <summary>Nothing: a solid has no checks in a context.</summary>
    public override void InContext(in Shape s)
    {
    }

    /// <summary>
    /// Checks the solid per se: shells that overlap, detached non-internal parts, and the closed non-internal shells
    /// (outside one another, or more than one enclosing a region).
    /// </summary>
    public override void Minimum()
    {
        if (_min)
        {
            return;
        }

        _min = true;
        //
        bool bFound, bIsHole, bFlag;
        int i, j, nbVTS, nbVTS1, iCntSh, iCntShInt;
        Solid z = new();
        var it = new Iterator();
        var itF = new Iterator();
        var bb = new Builder();
        var exp = new Explorer();
        var mss = new HashSet<Shape>(ShapeHasher.Instance);
        Orientation or;
        var vts = new List<BRepCheck_ToolSolid>();

        var newList = new NCollection_List<StatusCheck>();
        NCollection_List<StatusCheck> lst = _map.SetValueRef(_shape, newList);
        lst.Append(StatusCheck.NoError);
        //
        //-------------------------------------------------
        // 1. InvalidImbricationOfShells
        bFound = false;
        exp.Init(_shape, ShapeType.Face);
        for (; !bFound && exp.More(); exp.Next())
        {
            Shape f = exp.Current();
            if (!mss.Add(f))
            {
                BRepCheckCheck.Add(lst, StatusCheck.InvalidImbricationOfShells);
                bFound = !bFound;
            }
        }

        //
        //-------------------------------------------------
        // 2.
        //    - Too many growths,
        //    - There is smt of the solid that is out of solid
        iCntSh = 0;
        iCntShInt = 0;
        it.Initialize(_shape);
        for (; it.More(); it.Next())
        {
            Shape sx = it.Value();
            //
            if (sx.Type != ShapeType.Shell)
            {
                or = sx.Orientation;
                if (or != Internal)
                {
                    BRepCheckCheck.Add(lst, StatusCheck.BadOrientationOfSubshape);
                }

                continue;
            }

            //
            Shell sh = ShapeCasts.Shell(sx);
            //
            // Skip internal shells
            bFound = false;
            itF.Initialize(sh);
            for (; !bFound && itF.More(); itF.Next())
            {
                Shape f = itF.Value();
                or = f.Orientation;
                if (or == Internal)
                {
                    bFound = !bFound;
                }
            }

            if (bFound)
            {
                ++iCntShInt;
                continue;
            }

            //
            ++iCntSh;
            //
            // Skip not closed shells
            if (!Tool.IsClosed(sh))
            {
                continue;
            }

            //
            // OCCT.NET: a new unfrozen solid holding the (frozen) shell, never handed out
            bb.MakeSolid(ref z);
            bb.Add(z, sh);
            //
            var ts = new BRepCheck_ToolSolid();
            //
            ts.SetSolid(z);
            vts.Add(ts);
        } // for (; aIt.More(); aIt.Next()) {

        //
        if (iCntSh == 0 && iCntShInt != 0)
        {
            // all shells in the solid are internal
            BRepCheckCheck.Add(lst, StatusCheck.BadOrientationOfSubshape);
        }

        //
        nbVTS = vts.Count;
        if (nbVTS < 2)
        {
            return;
        }

        //
        nbVTS1 = 0;
        for (i = 0; i < nbVTS; ++i)
        {
            BRepCheck_ToolSolid ts = vts[i];
            //
            ts.Init();
            bIsHole = ts.IsHole;
            if (!bIsHole)
            {
                ++nbVTS1;
                if (nbVTS1 > 1)
                {
                    // Too many growths
                    BRepCheckCheck.Add(lst, StatusCheck.EnclosedRegion);
                    break;
                }
            }
        }

        //
        // OCCT: after the break above, the remaining tools are never initialised, and IsOut below dereferences
        // their null classifier (the analyzer doesn't catch exceptions from Minimum)
        bFound = false;
        nbVTS1 = nbVTS - 1;
        for (i = 0; !bFound && i < nbVTS1; ++i)
        {
            BRepCheck_ToolSolid tSi = vts[i];
            //
            for (j = i + 1; !bFound && j < nbVTS; ++j)
            {
                BRepCheck_ToolSolid tSj = vts[j];
                //
                bFlag = tSi.IsOut(tSj);
                if (bFlag)
                {
                    // smt of solid is out of solid
                    BRepCheckCheck.Add(lst, StatusCheck.SubshapeNotInShape);
                    bFound = !bFound;
                }
            }
        }

        //
        // myMin = true;
    }
}
