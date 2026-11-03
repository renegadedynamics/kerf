// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TypeOfContact.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Construction.Filling;

/// <summary>Whether a sweep along an auxiliary spine keeps the profile in contact with it.</summary>
public enum TypeOfContact
{
    /// <summary>No contact is kept.</summary>
    NoContact,

    /// <summary>The profile is translated to touch the auxiliary spine.</summary>
    Contact,

    /// <summary>The profile is translated and scaled to touch the auxiliary spine on its border.</summary>
    ContactOnBorder,
}
